#include <Arduino.h>
#include <RF24.h>
#include <SPI.h>
#include <esp_system.h>
#include <WiFi.h>

// --- NRF24 pins ---
#define PIN_CE   22
#define PIN_CSN  21
#define PIN_SCK  18
#define PIN_MOSI 23
#define PIN_MISO 19
#define PIN_IRQ  27
#define RF24_SPI_SPEED_HZ 1000000

#ifndef SNIFFER_FIXED_CHANNEL
#define SNIFFER_FIXED_CHANNEL -1
#endif

#ifndef SNIFFER_USE_IRQ
#define SNIFFER_USE_IRQ 0
#endif

#ifndef SNIFFER_IRQ_EDGE
#define SNIFFER_IRQ_EDGE 0
#endif

#ifndef SNIFFER_CHANNEL_SWITCH_MS
#define SNIFFER_CHANNEL_SWITCH_MS 200
#endif

#define SNIFFER_INVALID_LOG_MS 1000
#define SNIFFER_MAX_PKTS_PER_LOOP 48
#define SNIFFER_POLL_DELAY_US 100
#define SNIFFER_IRQ_EMPTY_BACKOFF_MS 20
#define MILIGHT_CCT_REQ_TYPE 0x45

#ifndef SNIFFER_FILTER_DUPLICATES
#define SNIFFER_FILTER_DUPLICATES 0
#endif

#define SNIFFER_STATS_PRINT_MS 5000
#define SNIFFER_MAX_STATS 48

// ---- MiLight V2 RF Encoding ----
// From esp8266_milight_hub V2RFEncoding.cpp
#define V2_OFFSET_JUMP_START 0x54

static const uint8_t V2_OFFSETS[8][4] = {
  {0x45, 0x1F, 0x14, 0x5C}, // byte1: request type
  {0x2B, 0xC9, 0xE3, 0x11}, // byte2: id1
  {0x6D, 0x5F, 0x8A, 0x2B}, // byte3: id2
  {0xAF, 0x03, 0x1D, 0xF3}, // byte4: command
  {0x1A, 0xE2, 0xF0, 0xD1}, // byte5: argument
  {0x04, 0xD8, 0x71, 0x42}, // byte6: sequence
  {0xAF, 0x04, 0xDD, 0x07}, // byte7: group
  {0x61, 0x13, 0x38, 0x64}, // byte8: checksum
};

// Returns offset for byte index (1-based) given key and optional jump start
static uint8_t v2Offset(uint8_t byteIdx, uint8_t key, uint8_t jumpStart) {
  uint8_t base = V2_OFFSETS[byteIdx - 1][key % 4];
  if (jumpStart > 0 && key >= jumpStart && key < jumpStart + 0x80) {
    base += 0x80;
  }
  return base;
}

static uint8_t v2XorKey(uint8_t key) {
  const uint8_t shift = (key & 0x0F) < 0x04 ? 0 : 1;
  const uint8_t x = (((key & 0xF0) >> 4) + shift + 6) % 8;
  const uint8_t msn = (((4 + x) ^ 1) & 0x0F) << 4;
  const uint8_t lsn = ((((key & 0x0F) + 4) ^ 2) & 0x0F);
  return msn | lsn;
}

// Decode packet in-place (bytes 1-8, byte 0 is the key byte)
// NOTE: byte 8 (checksum) uses jumpStart=0, matching v2Encode which uses V2_OFFSETS[7] directly.
static void v2Decode(uint8_t *p) {
  uint8_t key = v2XorKey(p[0]);
  for (int i = 1; i <= 8; i++) {
    uint8_t off = v2Offset(i, p[0], i == 8 ? 0 : V2_OFFSET_JUMP_START);
    p[i] = (p[i] - off) ^ key;
  }
}
// ----------------------------------

// CCT syncword address (5 bytes, same as transmit side)
// Derived from syncword0=0x050A, syncword3=0x55AA, preamble=0xAA, trailer=0x05
static const uint8_t CCT_ADDRESS[5] = {0xAA, 0x5A, 0x05, 0x0A, 0x55};

// CCT channels (2.404, 2.439, 2.474 GHz)
static const uint8_t CCT_CHANNELS[3] = {4, 39, 74};

static RF24 radio(PIN_CE, PIN_CSN, RF24_SPI_SPEED_HZ);
static bool radio_ok = false;
static bool irq_ok = true;
static volatile bool irq_fired = false;
static uint8_t cur_ch = 0;
static uint32_t last_channel_switch = 0;
static uint32_t pkts_received = 0;
static uint32_t invalid_count = 0;
static uint8_t last_raw[10] = {0};
static uint32_t dup_count = 0;
static uint32_t unique_count = 0;
static uint32_t last_invalid_log = 0;
static uint32_t last_stats_print = 0;
static uint32_t last_heartbeat = 0;
#define SNIFFER_HEARTBEAT_MS 10000

struct ActionStat {
    uint8_t req_type;
    uint8_t id1;
    uint8_t id2;
    uint8_t group;
    uint8_t cmd;
    uint8_t arg;
    uint32_t count;
    uint32_t first_ms;
    uint32_t last_ms;
};

static ActionStat stats[SNIFFER_MAX_STATS] = {};
static uint8_t stats_used = 0;

static uint8_t reverseBits(uint8_t b) {
    b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
    b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
    b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
    return b;
}

static void printRawHex(const uint8_t *raw, size_t len) {
    for (size_t i = 0; i < len; i++) {
        if (raw[i] < 0x10) Serial.print("0");
        Serial.print(raw[i], HEX);
    }
}

static const char* actionName(uint8_t command) {
    switch (command) {
        case 0x11: return "GRP-ON";
        case 0x12: return "GRP-ON";
        case 0x1C: return "BRIGHT";
        case 0x19: return "TEMP";
        default:   return "?";
    }
}

static void updateStats(uint8_t req_type, uint8_t id1, uint8_t id2, uint8_t group,
                        uint8_t cmd, uint8_t arg, uint32_t now_ms) {
    for (uint8_t i = 0; i < stats_used; i++) {
        ActionStat &s = stats[i];
        if (s.req_type == req_type && s.id1 == id1 && s.id2 == id2 &&
            s.group == group && s.cmd == cmd && s.arg == arg) {
            s.count++;
            s.last_ms = now_ms;
            return;
        }
    }

    if (stats_used >= SNIFFER_MAX_STATS) return;

    ActionStat &n = stats[stats_used++];
    n.req_type = req_type;
    n.id1 = id1;
    n.id2 = id2;
    n.group = group;
    n.cmd = cmd;
    n.arg = arg;
    n.count = 1;
    n.first_ms = now_ms;
    n.last_ms = now_ms;
}

static void printStats(uint32_t now_ms) {
    Serial.printf("MAP,total_unique=%u,uptime_ms=%lu\n", stats_used, now_ms);
    for (uint8_t i = 0; i < stats_used; i++) {
        const ActionStat &s = stats[i];
        Serial.printf("MAP,%u,type=0x%02X,id=%02X%02X,grp=%u,cmd=0x%02X(%s),arg=%u,count=%lu,first_ms=%lu,last_ms=%lu\n",
                      i,
                      s.req_type,
                      s.id1,
                      s.id2,
                      s.group,
                      s.cmd,
                      actionName(s.cmd),
                      s.arg,
                      s.count,
                      s.first_ms,
                      s.last_ms);
    }
}

static uint8_t configuredChannel() {
#if SNIFFER_FIXED_CHANNEL >= 0
    return (uint8_t)SNIFFER_FIXED_CHANNEL;
#else
    return CCT_CHANNELS[cur_ch];
#endif
}

static void tuneToCurrentChannel() {
    radio.setChannel(configuredChannel());
    radio.flush_rx();
    radio.startListening();
}

static void IRAM_ATTR onRadioIrq() {
    irq_fired = true;
}

void setup() {
    WiFi.mode(WIFI_OFF);
    btStop();
    Serial.begin(115200);
    delay(500);
    Serial.println("=== MiLight CCT SNIFFER ===");
    Serial.flush();

    pinMode(PIN_CE, OUTPUT);
    digitalWrite(PIN_CE, LOW);
    pinMode(PIN_CSN, OUTPUT);
    digitalWrite(PIN_CSN, HIGH);
    pinMode(PIN_MISO, INPUT_PULLUP);
#if SNIFFER_USE_IRQ
    pinMode(PIN_IRQ, INPUT_PULLUP);
    Serial.printf("IRQ mode: nRF24 IRQ -> GPIO%d\n", PIN_IRQ);
    Serial.printf("IRQ initial level: %s\n", digitalRead(PIN_IRQ) == LOW ? "LOW" : "HIGH");
#endif

#ifdef SNIFFER_DIAG_NO_SPI
    Serial.println("DIAG: stopped before SPI.begin().");
    Serial.flush();
    return;
#endif

    SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CSN);

#ifdef SNIFFER_DIAG_SPI_ONLY
    Serial.println("DIAG: SPI.begin() done, RF24 not started.");
    Serial.flush();
    return;
#endif

#ifdef SNIFFER_DIAG_RF24_BEGIN
    Serial.println("DIAG: before radio.begin().");
    Serial.flush();
    bool begin_ok = radio.begin();
    bool chip_ok = radio.isChipConnected();
    Serial.printf("DIAG: radio.begin=%s isChipConnected=%s\n",
                  begin_ok ? "YES" : "NO",
                  chip_ok ? "YES" : "NO");
    Serial.flush();
    return;
#endif

    if (!radio.begin() || !radio.isChipConnected()) {
        Serial.println("FAIL: NRF24 not responding - check wiring/power.");
        Serial.println("Sniffer idle until a working radio is connected and ESP is reset.");
        Serial.flush();
        return;
    }
    radio_ok = true;

    radio.setAutoAck(false);
    radio.setDataRate(RF24_1MBPS);
    radio.setPALevel(RF24_PA_LOW);
    radio.setAddressWidth(5);
    radio.setPayloadSize(10);       // 1 length byte + 9 payload bytes (actual data from remote)
    radio.disableCRC();             // CRC handled in PL1167 wrapper (already stripped by HW)
    radio.openReadingPipe(1, CCT_ADDRESS);
#if SNIFFER_USE_IRQ
    radio.maskIRQ(true, true, false); // only RX_DR drives IRQ low
#endif
    tuneToCurrentChannel();

#if SNIFFER_USE_IRQ
    radio.clearStatusFlags();
    delay(5);
    Serial.printf("IRQ after clear: %s\n", digitalRead(PIN_IRQ) == LOW ? "LOW" : "HIGH");
    if (digitalRead(PIN_IRQ) == LOW) {
        Serial.println("FAIL: IRQ stuck LOW. Check nRF24 IRQ wiring/pin label.");
        Serial.println("No SPI polling will be done in this state.");
        Serial.flush();
        irq_ok = false;
        return;
    }
#if SNIFFER_IRQ_EDGE
    attachInterrupt(digitalPinToInterrupt(PIN_IRQ), onRadioIrq, FALLING);
    Serial.println("IRQ edge mode enabled.");
#endif
#endif

    last_channel_switch = millis();
}

void loop() {
    if (!radio_ok || !irq_ok) {
        delay(1000);
        return;
    }

#if SNIFFER_USE_IRQ
#if SNIFFER_IRQ_EDGE
    if (!irq_fired) {
        delay(1);
        return;
    }
    irq_fired = false;
#else
    if (digitalRead(PIN_IRQ) != LOW) {
#if SNIFFER_FIXED_CHANNEL < 0
        if (millis() - last_channel_switch > SNIFFER_CHANNEL_SWITCH_MS) {
            cur_ch = (cur_ch + 1) % 3;
            radio.stopListening();
            tuneToCurrentChannel();
            last_channel_switch = millis();
        }
#endif
        delay(1);
        return;
    }
#endif
#else
#if SNIFFER_FIXED_CHANNEL < 0
    // Switch channel on a fixed cadence and discard stale RX FIFO contents.
    if (millis() - last_channel_switch > SNIFFER_CHANNEL_SWITCH_MS) {
        cur_ch = (cur_ch + 1) % 3;
        radio.stopListening();
        tuneToCurrentChannel();
        last_channel_switch = millis();
    }
#endif
#endif

    uint32_t now_hb = millis();
    if (now_hb - last_heartbeat >= SNIFFER_HEARTBEAT_MS) {
        Serial.printf("[ALIVE] CH%d pkts=%lu unique=%lu\n",
                      configuredChannel(), pkts_received, unique_count);
        last_heartbeat = now_hb;
    }

    uint8_t drained = 0;
    while (radio.available() && drained < SNIFFER_MAX_PKTS_PER_LOOP) {
        drained++;
        uint8_t raw[10] = {0};
        radio.read(raw, 10);
        pkts_received++;

#if SNIFFER_FILTER_DUPLICATES
        // Optional duplicate filter - default disabled to avoid dropping valid repeats.
        if (memcmp(raw, last_raw, 10) == 0) {
            dup_count++;
            continue;
        }
#endif
        memcpy(last_raw, raw, 10);
        dup_count = 0;
        unique_count++;

        // Generated MiLight frames start with a bit-reversed payload length byte.
        uint8_t len_byte = reverseBits(raw[0]);

        if (len_byte != 0x09) {
            invalid_count++;
            uint32_t now = millis();
            if (now - last_invalid_log >= SNIFFER_INVALID_LOG_MS) {
                Serial.printf("[CH%2d] invalid=%lu last_len=0x%02X raw=",
                              configuredChannel(), invalid_count, len_byte);
                printRawHex(raw, 10);
                Serial.println();
                last_invalid_log = now;
            }
            continue;
        }

        // Bit-reverse the 9 payload bytes (NRF24+PL1167 layer)
        uint8_t p[9];
        for (int i = 0; i < 9; i++) p[i] = reverseBits(raw[i + 1]);

        // Apply V2 RF decoding (MiLight application layer)
        v2Decode(p);

        // After V2 decode:
        // p[0] = raw key byte (NOT decoded, used as V2 seed)
        // p[1] = request type  (0x5A = CCT V2 command)
        // p[2] = device ID high
        // p[3] = device ID low
        // p[4] = command
        // p[5] = argument/value
        // p[6] = sequence counter
        // p[7] = group/zone (0=all, 1-4=groups)
        // p[8] = checksum
        uint8_t key_byte = p[0];
        uint8_t req_type = p[1];
        uint8_t id1      = p[2];
        uint8_t id2      = p[3];
        uint8_t command  = p[4];
        uint8_t argument = p[5];
        uint8_t sequence = p[6];
        uint8_t group    = p[7];
        uint8_t chk_recv = p[8];

        // Checksum validate: sum of decoded bytes 0-6 should equal checksum
        uint8_t chk_calc = v2XorKey(p[0]);
        for (int i = 1; i <= 7; i++) chk_calc += p[i];
        chk_calc += 2;

        // Drop decoded noise unless it matches the V2 CCT request type and checksum.
        if (req_type != MILIGHT_CCT_REQ_TYPE || chk_calc != chk_recv) {
            invalid_count++;
            uint32_t now = millis();
            if (now - last_invalid_log >= SNIFFER_INVALID_LOG_MS) {
                Serial.printf("[DECODE-FAIL] CH%d type=0x%02X chk_calc=0x%02X chk_recv=0x%02X raw=",
                              configuredChannel(), req_type, chk_calc, chk_recv);
                printRawHex(raw, 10);
                Serial.println();
                last_invalid_log = now;
            }
            continue;
        }

        const char* action = actionName(command);
        uint32_t now_ms = millis();
        updateStats(req_type, id1, id2, group, command, argument, now_ms);

        // Print raw bytes (for replay test)
        Serial.printf("RAW: ");
        for (int i = 0; i < 10; i++) Serial.printf("%02X ", raw[i]);
        Serial.printf("| [CH%2d] #%lu | type=0x%02X id=%02X%02X grp=%d cmd=0x%02X(%s) arg=%d seq=%d\n",
                      configuredChannel(), unique_count,
                      req_type, id1, id2, group, command, action, argument, sequence);
        Serial.flush();

        Serial.printf("CSV,%lu,%d,", millis(), configuredChannel());
        printRawHex(raw, 10);
        Serial.printf(",0x%02X,%02X%02X,%d,0x%02X,%d,%d\n",
                      req_type, id1, id2, group, command, argument, sequence);
        Serial.flush();

        if (now_ms - last_stats_print >= SNIFFER_STATS_PRINT_MS) {
            printStats(now_ms);
            last_stats_print = now_ms;
        }
    }

#if SNIFFER_USE_IRQ
    if (drained == 0) {
        radio.clearStatusFlags();
        delay(SNIFFER_IRQ_EMPTY_BACKOFF_MS);
        return;
    }
#endif

    delayMicroseconds(SNIFFER_POLL_DELAY_US);
}
