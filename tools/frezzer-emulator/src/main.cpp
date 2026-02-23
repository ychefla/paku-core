/**
 * @file main.cpp
 * @brief Frezzer PRO BLE Emulator — ESP32 firmware
 *
 * Emulates a Frezzer PRO 65L 12/24V compressor fridge (Alpicool OEM platform).
 * Acts as a BLE GATT peripheral (server), advertising as "WT-0001" with the
 * exact same service/characteristic layout as the real hardware.
 *
 * Works with:
 *   - paku-core (ESP32 BLE client) — test the full connection + data flow
 *   - Official Frezzer / Alpicool app — verify our protocol implementation
 *   - nRF Connect / LightBlue — raw GATT inspection
 *
 * Alpicool protocol (from klightspeed/BrassMonkeyFridgeMonitor, MIT):
 *   Service   0x1234
 *   Write     0x1235  client → fridge  (commands)
 *   Notify    0x1236  fridge → client  (status responses)
 *   Frame:    FE FE | lenByte | cmd | body... | cs_hi cs_lo
 *   Checksum: 16-bit big-endian sum of all preceding bytes
 *
 * Serial commands (115200 baud):
 *   +/-       raise/lower current temperature 1 °C
 *   t<n>      set target temp (e.g. "t-10", "t3")
 *   m0        MAX_COOL mode
 *   m1        ECO mode
 *   mf        power off (off mode)
 *   l         toggle panel lock
 *   v<n>      set battery voltage (e.g. "v12.4")
 *   e<0-3>    inject error (0=none,1=temp_sensor,2=compressor,3=low_bat)
 *   s         print current status frame (hex)
 *   ?         print this help
 */

#include <Arduino.h>
#include <NimBLEDevice.h>

// ── Alpicool GATT UUIDs ───────────────────────────────────────────────────────
static const NimBLEUUID SERVICE_UUID      ("00001234-0000-1000-8000-00805f9b34fb");
static const NimBLEUUID WRITE_CHAR_UUID   ("00001235-0000-1000-8000-00805f9b34fb");
static const NimBLEUUID NOTIFY_CHAR_UUID  ("00001236-0000-1000-8000-00805f9b34fb");

static const char* DEVICE_NAME = "WT-0001";

// ── BLE handles ───────────────────────────────────────────────────────────────
static NimBLEServer*         pServer       = nullptr;
static NimBLECharacteristic* pNotifyChar   = nullptr;
static bool                  clientConnected = false;

// ── Alpicool command codes ─────────────────────────────────────────────────────
static constexpr uint8_t CMD_BIND             = 0x00;
static constexpr uint8_t CMD_QUERY            = 0x01;
static constexpr uint8_t CMD_SET              = 0x02;
static constexpr uint8_t CMD_RESET            = 0x04;
static constexpr uint8_t CMD_SET_LEFT_TARGET  = 0x05;
static constexpr uint8_t CMD_SET_RIGHT_TARGET = 0x06;

// ── Simulated fridge state ────────────────────────────────────────────────────
struct FridgeState {
    float    currentTemp    =  5.0f;   // °C — starts at room temperature
    float    targetTemp     = -8.0f;   // °C — setpoint
    float    batteryVoltage = 12.4f;   // V
    uint8_t  batPercent     = 80;
    bool     powered        = true;    // false = off
    bool     eco            = false;   // ECO mode (true) vs MAX (false)
    bool     locked         = false;   // panel lock
    uint8_t  batSaverMode   = 0;       // 0=Low,1=Mid,2=High
    uint8_t  errorCode      = 0;       // 0=none
    bool     compressorOn   = false;   // derived
};

static FridgeState fridge;
static unsigned long lastTickMs = 0;

// ── Protocol helpers ──────────────────────────────────────────────────────────

static uint16_t alpicoolChecksum(const uint8_t* data, size_t len) {
    uint16_t sum = 0;
    for (size_t i = 0; i < len; i++) sum += data[i];
    return sum;
}

/**
 * @brief Build the 24-byte QUERY response frame.
 *
 * Body layout (18 bytes, body[0] = frame[4]):
 *  [0]  locked      bool
 *  [1]  poweredOn   bool
 *  [2]  runMode     0=Max, 1=Eco
 *  [3]  batSaver    0=Low,1=Mid,2=High
 *  [4]  leftTarget  signed int8 °C
 *  [5]  tempMax     signed int8 (°C, +10 default)
 *  [6]  tempMin     signed int8 (°C, -25 default)
 *  [7]  leftRetDiff uint8 (hysteresis °C)
 *  [8]  startDelay  uint8 (minutes)
 *  [9]  unit        0=Celsius, 1=Fahrenheit
 * [10] leftTCHot    signed int8 (temp correction)
 * [11] leftTCMid    signed int8
 * [12] leftTCCold   signed int8
 * [13] leftTCHalt   signed int8
 * [14] leftCurrent  signed int8 °C  ← actual current temperature
 * [15] batPercent   uint8 (0-100, 0x7F=unknown)
 * [16] batVolInt    uint8 (integer volts)
 * [17] batVolDec    uint8 (decimal × 10, e.g. 4 → 12.4 V)
 */
static size_t buildQueryResponse(const FridgeState& s, uint8_t* buf, size_t bufLen) {
    // Frame = FE FE 15 01 [18 body] cs_hi cs_lo = 24 bytes
    static constexpr size_t FRAME_LEN = 24;
    if (bufLen < FRAME_LEN) return 0;

    int8_t targetI8  = (int8_t)constrain((int)s.targetTemp,  -25, 20);
    int8_t currentI8 = (int8_t)constrain((int)s.currentTemp, -40, 50);
    uint8_t batVolInt = (uint8_t)(int)s.batteryVoltage;
    uint8_t batVolDec = (uint8_t)((s.batteryVoltage - batVolInt) * 10.0f + 0.5f);

    buf[0]  = 0xFE;
    buf[1]  = 0xFE;
    buf[2]  = 21;           // lenByte = 18(body) + 1(cmd) + 2(cs) = 21
    buf[3]  = CMD_QUERY;    // cmd = 0x01

    // body starts at buf[4]
    buf[4]  = s.locked  ? 1 : 0;            // [0] locked
    buf[5]  = s.powered ? 1 : 0;            // [1] poweredOn
    buf[6]  = s.eco     ? 1 : 0;            // [2] runMode
    buf[7]  = s.batSaverMode;               // [3] batSaver
    buf[8]  = (uint8_t)targetI8;            // [4] leftTarget
    buf[9]  = (uint8_t)(int8_t)10;          // [5] tempMax   +10 °C
    buf[10] = (uint8_t)(int8_t)(-25);       // [6] tempMin   -25 °C
    buf[11] = 2;                             // [7] leftRetDiff  2 °C
    buf[12] = 0;                             // [8] startDelay   0 min
    buf[13] = 0;                             // [9] unit   Celsius
    buf[14] = 0;                             // [10] leftTCHot
    buf[15] = 0;                             // [11] leftTCMid
    buf[16] = 0;                             // [12] leftTCCold
    buf[17] = 0;                             // [13] leftTCHalt
    buf[18] = (uint8_t)currentI8;           // [14] leftCurrent ← actual temp
    buf[19] = s.batPercent;                 // [15] batPercent
    buf[20] = batVolInt;                    // [16] batVolInt
    buf[21] = batVolDec;                    // [17] batVolDec

    // Checksum over bytes 0..21 (everything except the 2 cs bytes)
    uint16_t cs = alpicoolChecksum(buf, FRAME_LEN - 2);
    buf[22] = (uint8_t)(cs >> 8);
    buf[23] = (uint8_t)(cs & 0xFF);

    return FRAME_LEN;
}

/**
 * @brief Build a 6-byte ACK echo frame for commands that don't return data.
 *        Frame: FE FE 03 <cmd> cs_hi cs_lo
 */
static size_t buildAck(uint8_t cmd, uint8_t* buf, size_t bufLen) {
    if (bufLen < 6) return 0;
    buf[0] = 0xFE;  buf[1] = 0xFE;
    buf[2] = 0x03;  buf[3] = cmd;
    uint16_t cs = alpicoolChecksum(buf, 4);
    buf[4] = (uint8_t)(cs >> 8);
    buf[5] = (uint8_t)(cs & 0xFF);
    return 6;
}

/**
 * @brief Validate and parse an incoming Alpicool command frame.
 * @return true if frame is valid, sets cmd and body pointers.
 */
static bool parseFrame(const uint8_t* data, size_t len,
                       uint8_t& cmd, const uint8_t*& body, size_t& bodyLen) {
    if (!data || len < 6) return false;
    if (data[0] != 0xFE || data[1] != 0xFE) return false;

    uint8_t pktLen     = data[2];
    size_t  totalBytes = (size_t)pktLen + 3;
    if (len < totalBytes) return false;

    // Verify checksum (last 2 bytes)
    uint16_t gotCs   = ((uint16_t)data[totalBytes - 2] << 8) | data[totalBytes - 1];
    uint16_t calcCs  = alpicoolChecksum(data, totalBytes - 2);
    if (gotCs != calcCs) {
        Serial.printf("  [WARN] checksum mismatch: got %04X, calc %04X\n", gotCs, calcCs);
        // still process — some app versions omit real checksum
    }

    cmd     = data[3];
    body    = &data[4];
    bodyLen = totalBytes - 6;   // frame total - header(2) - lenByte(1) - cmd(1) - cs(2)
    return true;
}

// ── Thermal simulation ────────────────────────────────────────────────────────

static void tickFridge(unsigned long nowMs) {
    float dt = (nowMs - lastTickMs) / 1000.0f;   // seconds
    lastTickMs = nowMs;
    if (dt <= 0 || dt > 10.0f) return;

    if (!fridge.powered) {
        // Drift toward ambient (+20 °C) slowly
        fridge.currentTemp += (20.0f - fridge.currentTemp) * 0.015f * dt;
        fridge.compressorOn = false;
        return;
    }

    const float hysteresis = 2.0f;
    if (fridge.currentTemp > fridge.targetTemp + hysteresis) {
        fridge.compressorOn = true;
    } else if (fridge.currentTemp <= fridge.targetTemp) {
        fridge.compressorOn = false;
    }

    if (fridge.compressorOn) {
        float rate = fridge.eco ? 0.3f : 0.6f;   // ECO slower, MAX faster
        fridge.currentTemp -= rate * dt;
    } else {
        // Natural warm-up from insulation leakage
        fridge.currentTemp += (20.0f - fridge.currentTemp) * 0.004f * dt;
    }

    // Keep within physical range
    fridge.currentTemp = constrain(fridge.currentTemp, -30.0f, 40.0f);

    // Slow battery drain simulation
    fridge.batteryVoltage = max(10.5f, fridge.batteryVoltage - 0.00008f * dt);
    float normalized = (fridge.batteryVoltage - 10.5f) / (14.4f - 10.5f);
    fridge.batPercent = (uint8_t)constrain((int)(normalized * 100), 0, 100);
}

// ── Notify current state to connected client ──────────────────────────────────

static void sendStatusNotify() {
    if (!clientConnected || !pNotifyChar) return;
    uint8_t frame[24];
    size_t len = buildQueryResponse(fridge, frame, sizeof(frame));
    if (len > 0) {
        pNotifyChar->setValue(frame, len);
        pNotifyChar->notify();
    }
}

// ── BLE callbacks ─────────────────────────────────────────────────────────────

class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pSrv, ble_gap_conn_desc* desc) override {
        clientConnected = true;
        Serial.printf("\n[BLE] Client connected: %s\n",
                      NimBLEAddress(desc->peer_ota_addr).toString().c_str());
        // Stop advertising while connected (optional, improves stability)
        NimBLEDevice::getAdvertising()->stop();
    }

    void onDisconnect(NimBLEServer* pSrv) override {
        clientConnected = false;
        Serial.println("[BLE] Client disconnected — resuming advertisement");
        NimBLEDevice::getAdvertising()->start();
    }
};

class WriteCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pChar, ble_gap_conn_desc* desc) override {
        NimBLEAttValue val = pChar->getValue();
        const uint8_t* data = val.data();
        size_t          len  = val.length();

        Serial.printf("[BLE] Write (%zu bytes): ", len);
        for (size_t i = 0; i < len; i++) Serial.printf("%02X ", data[i]);
        Serial.println();

        uint8_t        cmd     = 0;
        const uint8_t* body    = nullptr;
        size_t         bodyLen = 0;

        if (!parseFrame(data, len, cmd, body, bodyLen)) {
            Serial.println("  [ERR] Invalid frame — ignored");
            return;
        }

        uint8_t resp[32];
        size_t  respLen = 0;

        switch (cmd) {

            case CMD_QUERY:
                Serial.println("  → QUERY");
                respLen = buildQueryResponse(fridge, resp, sizeof(resp));
                break;

            case CMD_SET_LEFT_TARGET:
                if (bodyLen >= 1) {
                    int8_t t = (int8_t)body[0];
                    fridge.targetTemp = (float)t;
                    Serial.printf("  → SET_LEFT_TARGET: %d °C\n", t);
                }
                respLen = buildAck(cmd, resp, sizeof(resp));
                break;

            case CMD_SET_RIGHT_TARGET:
                if (bodyLen >= 1) {
                    int8_t t = (int8_t)body[0];
                    Serial.printf("  → SET_RIGHT_TARGET (zone 2 ignored): %d °C\n", t);
                }
                respLen = buildAck(cmd, resp, sizeof(resp));
                break;

            case CMD_SET:
                // Full settings payload — same body layout as query response body
                Serial.printf("  → SET (%zu body bytes)\n", bodyLen);
                if (bodyLen >= 18) {
                    fridge.locked    = (body[0] != 0);
                    fridge.powered   = (body[1] != 0);
                    fridge.eco       = (body[2] == 1);
                    fridge.targetTemp = (float)(int8_t)body[4];
                    Serial.printf("     powered=%d  eco=%d  target=%.0f°C  locked=%d\n",
                                  fridge.powered, fridge.eco, fridge.targetTemp, fridge.locked);
                }
                respLen = buildAck(cmd, resp, sizeof(resp));
                break;

            case CMD_BIND:
                Serial.println("  → BIND (press physical button to confirm)");
                respLen = buildAck(cmd, resp, sizeof(resp));
                break;

            case CMD_RESET:
                Serial.println("  → FACTORY RESET");
                fridge = FridgeState{};
                respLen = buildAck(cmd, resp, sizeof(resp));
                break;

            default:
                Serial.printf("  → Unknown cmd 0x%02X\n", cmd);
                break;
        }

        if (respLen > 0 && pNotifyChar) {
            Serial.printf("[BLE] Notify (%zu bytes): ", respLen);
            for (size_t i = 0; i < respLen; i++) Serial.printf("%02X ", resp[i]);
            Serial.println();
            pNotifyChar->setValue(resp, respLen);
            pNotifyChar->notify();
        }
    }
};

// ── Serial command interface ──────────────────────────────────────────────────

static void printHelp() {
    Serial.println("─────────────────────────────────────────────");
    Serial.println(" Frezzer PRO Emulator — serial commands");
    Serial.println("  +/-         current temp ±1 °C");
    Serial.println("  t<n>        target temp  (e.g. t-10, t3)");
    Serial.println("  m0          MAX_COOL mode");
    Serial.println("  m1          ECO mode");
    Serial.println("  mf          power OFF");
    Serial.println("  l           toggle panel lock");
    Serial.println("  v<n>        battery voltage (e.g. v11.8)");
    Serial.println("  e<0-3>      error code (0=none)");
    Serial.println("  s           print current status frame (hex)");
    Serial.println("  ?           this help");
    Serial.println("─────────────────────────────────────────────");
}

static void printStatus() {
    uint8_t frame[24];
    size_t len = buildQueryResponse(fridge, frame, sizeof(frame));
    Serial.printf("  current=%.1f°C  target=%.0f°C  mode=%s  comp=%s\n",
                  fridge.currentTemp, fridge.targetTemp,
                  fridge.powered ? (fridge.eco ? "ECO" : "MAX") : "OFF",
                  fridge.compressorOn ? "ON" : "off");
    Serial.printf("  bat=%.1fV (%d%%)  locked=%d  err=%d\n",
                  fridge.batteryVoltage, fridge.batPercent,
                  fridge.locked, fridge.errorCode);
    if (len > 0) {
        Serial.printf("  Frame[%zu]: ", len);
        for (size_t i = 0; i < len; i++) Serial.printf("%02X ", frame[i]);
        Serial.println();
    }
}

static String serialBuf;

static void handleSerial() {
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\r') continue;
        if (c == '\n') {
            serialBuf.trim();
            if (serialBuf.length() == 0) { serialBuf = ""; return; }

            String cmd = serialBuf;
            serialBuf = "";

            if (cmd == "+") {
                fridge.currentTemp += 1.0f;
                Serial.printf("  current → %.1f°C\n", fridge.currentTemp);
            } else if (cmd == "-") {
                fridge.currentTemp -= 1.0f;
                Serial.printf("  current → %.1f°C\n", fridge.currentTemp);
            } else if (cmd.startsWith("t")) {
                float v = cmd.substring(1).toFloat();
                if (v >= -25 && v <= 20) {
                    fridge.targetTemp = v;
                    Serial.printf("  target → %.0f°C\n", v);
                } else {
                    Serial.println("  Range: -25 to +20");
                }
            } else if (cmd == "m0") {
                fridge.powered = true;  fridge.eco = false;
                Serial.println("  mode → MAX_COOL");
            } else if (cmd == "m1") {
                fridge.powered = true;  fridge.eco = true;
                Serial.println("  mode → ECO");
            } else if (cmd == "mf") {
                fridge.powered = false;
                Serial.println("  mode → OFF");
            } else if (cmd == "l") {
                fridge.locked = !fridge.locked;
                Serial.printf("  locked → %s\n", fridge.locked ? "ON" : "OFF");
            } else if (cmd.startsWith("v")) {
                float v = cmd.substring(1).toFloat();
                if (v >= 9 && v <= 30) {
                    fridge.batteryVoltage = v;
                    Serial.printf("  voltage → %.1fV\n", v);
                } else {
                    Serial.println("  Range: 9–30 V");
                }
            } else if (cmd.startsWith("e")) {
                int e = cmd.substring(1).toInt();
                fridge.errorCode = (uint8_t)constrain(e, 0, 3);
                const char* names[] = {"none", "temp_sensor", "compressor", "low_bat"};
                Serial.printf("  error → %s\n", names[fridge.errorCode]);
            } else if (cmd == "s") {
                printStatus();
            } else if (cmd == "?") {
                printHelp();
            } else {
                Serial.printf("  Unknown: %s — type ? for help\n", cmd.c_str());
            }
            return;
        }
        serialBuf += c;
    }
}

// ── Periodic status print ─────────────────────────────────────────────────────

static unsigned long lastPrintMs    = 0;
static unsigned long lastNotifyMs   = 0;
static constexpr unsigned long PRINT_INTERVAL_MS  = 5000;
static constexpr unsigned long NOTIFY_INTERVAL_MS = 2000;  // Alpicool app polls 2 s

// ── Setup ─────────────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println("\n\n  Frezzer PRO BLE Emulator");
    Serial.println("  ─────────────────────────");
    Serial.printf ("  Device name : %s\n", DEVICE_NAME);
    Serial.println("  Service     : 1234");
    Serial.println("  Write char  : 1235");
    Serial.println("  Notify char : 1236");
    Serial.printf ("  Target temp : %.0f °C\n", fridge.targetTemp);
    Serial.printf ("  Start temp  : %.1f °C\n", fridge.currentTemp);
    Serial.println();

    NimBLEDevice::init(DEVICE_NAME);
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);  // max TX power for range

    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

    NimBLEService* pService = pServer->createService(SERVICE_UUID);

    // Write characteristic — client sends commands here
    NimBLECharacteristic* pWriteChar = pService->createCharacteristic(
        WRITE_CHAR_UUID,
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
    );
    pWriteChar->setCallbacks(new WriteCallbacks());

    // Notify characteristic — fridge sends status back
    pNotifyChar = pService->createCharacteristic(
        NOTIFY_CHAR_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
    );
    // Pre-populate with current state (clients may read before any QUERY)
    uint8_t initFrame[24];
    size_t initLen = buildQueryResponse(fridge, initFrame, sizeof(initFrame));
    pNotifyChar->setValue(initFrame, initLen);

    pService->start();

    // Advertising — must include local name "WT-0001" and the service UUID
    NimBLEAdvertising* pAdv = NimBLEDevice::getAdvertising();
    pAdv->addServiceUUID(SERVICE_UUID);
    pAdv->setScanResponse(true);
    pAdv->setName(DEVICE_NAME);           // Ensures name in scan response/adv
    pAdv->start();

    Serial.println("  BLE advertising started — waiting for connections...");
    printHelp();

    lastTickMs   = millis();
    lastPrintMs  = millis();
    lastNotifyMs = millis();
}

// ── Loop ──────────────────────────────────────────────────────────────────────

void loop() {
    unsigned long now = millis();

    // Simulate thermal dynamics
    tickFridge(now);

    // Send periodic notify to connected client (mirrors real fridge behavior)
    if (clientConnected && (now - lastNotifyMs >= NOTIFY_INTERVAL_MS)) {
        sendStatusNotify();
        lastNotifyMs = now;
    }

    // Serial status print
    if (now - lastPrintMs >= PRINT_INTERVAL_MS) {
        Serial.printf("[EMU] temp=%.1f°C  target=%.0f°C  mode=%s  comp=%s  bat=%.1fV (%d%%)  conn=%s\n",
                      fridge.currentTemp, fridge.targetTemp,
                      !fridge.powered ? "OFF" : (fridge.eco ? "ECO" : "MAX"),
                      fridge.compressorOn ? "ON " : "off",
                      fridge.batteryVoltage, fridge.batPercent,
                      clientConnected ? "yes" : "no");
        lastPrintMs = now;
    }

    // Handle serial input
    handleSerial();

    delay(50);
}
