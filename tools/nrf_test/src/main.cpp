#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>

// LilyGo T-Display S3 — P2 connector wiring (matches production pin_config.h)
// GPIO2 as CSN is safe: NRF24 CS idles HIGH = correct strapping level on GPIO2
#define PIN_CE   1   // GPIO1  → NRF24 CE
#define PIN_CSN  2   // GPIO2  → NRF24 CSN (production assignment)
#define PIN_SCK  10  // GPIO10 → NRF24 SCK
#define PIN_MOSI 11  // GPIO11 → NRF24 MOSI (MO)
#define PIN_MISO 12  // GPIO12 → NRF24 MISO (MI)

RF24 radio(PIN_CE, PIN_CSN);

void setup() {
    Serial.begin(115200);

    // Wait for NRF24 power-on stabilisation (adapter regulator needs ~100ms)
    delay(2000);
    Serial.println("[nrf] NRF24L01+ test starting...");
    Serial.printf("[nrf] CE=%d CSN=%d SCK=%d MOSI=%d MISO=%d\n",
                  PIN_CE, PIN_CSN, PIN_SCK, PIN_MOSI, PIN_MISO);

    // --- GPIO raw state BEFORE SPI init ---
    Serial.println("[nrf] --- GPIO state before SPI.begin() ---");
    // Set all as plain inputs (no pull) and read
    const int pins[] = {PIN_CE, PIN_CSN, PIN_SCK, PIN_MOSI, PIN_MISO};
    const char* names[] = {"CE(1)", "CSN(2)", "SCK(10)", "MOSI(11)", "MISO(12)"};
    for (int i = 0; i < 5; i++) {
        pinMode(pins[i], INPUT);
        Serial.printf("[nrf]   %-10s floating: %d\n", names[i], digitalRead(pins[i]));
    }
    // Check MISO with explicit pull-up — if it reads 1, nothing is pulling it down externally
    pinMode(PIN_MISO, INPUT_PULLUP);
    delay(1);
    Serial.printf("[nrf]   MISO(12) with pull-up: %d (0=ext pulldown/short, 1=floating)\n",
                  digitalRead(PIN_MISO));
    pinMode(PIN_MISO, INPUT);

    // --- BIT-BANG SPI PROBE (before SPI.begin — no ESP32 SPI peripheral involved) ---
    // This definitively tests the NRF24 over raw GPIO toggling.
    // Sends NOP (0xFF) → STATUS register is always the first byte returned.
    // Default STATUS = 0x0E. Any value other than 0xFF/0x00 means NRF24 is alive.
    Serial.println("[nrf] --- Bit-bang SPI probe (raw GPIO, no SPI peripheral) ---");
    pinMode(PIN_CE,  OUTPUT); digitalWrite(PIN_CE,  LOW);
    pinMode(PIN_CSN, OUTPUT); digitalWrite(PIN_CSN, HIGH);
    pinMode(PIN_SCK, OUTPUT); digitalWrite(PIN_SCK, LOW);
    pinMode(PIN_MOSI,OUTPUT); digitalWrite(PIN_MOSI,LOW);
    pinMode(PIN_MISO,INPUT);
    delay(5);

    // Bit-bang helper: sends one byte MSB-first, returns received byte
    auto bb_xfer = [](uint8_t out) -> uint8_t {
        uint8_t in = 0;
        for (int i = 7; i >= 0; i--) {
            digitalWrite(PIN_MOSI, (out >> i) & 1);
            delayMicroseconds(5);
            digitalWrite(PIN_SCK, HIGH);
            delayMicroseconds(5);
            in = (in << 1) | digitalRead(PIN_MISO);
            digitalWrite(PIN_SCK, LOW);
            delayMicroseconds(5);
        }
        return in;
    };

    // Send NOP (0xFF) — STATUS is always the first byte back
    digitalWrite(PIN_CSN, LOW); delayMicroseconds(10);
    uint8_t bb_nop = bb_xfer(0xFF);
    digitalWrite(PIN_CSN, HIGH); delayMicroseconds(10);
    Serial.printf("[nrf] BB NOP    → STATUS = 0x%02X\n", bb_nop);

    // Read CONFIG register: command 0x00 (R_REGISTER | addr 0x00)
    // First byte back = STATUS, second byte back = CONFIG value
    digitalWrite(PIN_CSN, LOW); delayMicroseconds(10);
    uint8_t bb_stat2 = bb_xfer(0x00);   // command byte → STATUS returned
    uint8_t bb_cfg   = bb_xfer(0x00);   // dummy → CONFIG register value returned
    digitalWrite(PIN_CSN, HIGH);
    Serial.printf("[nrf] BB R_REG(0x00) → STATUS=0x%02X, CONFIG=0x%02X\n", bb_stat2, bb_cfg);

    if (bb_nop == 0xFF) {
        Serial.println("[nrf] >>> BB: MISO floating (0xFF) — NRF24 MISO not driving");
    } else if (bb_nop == 0x00) {
        Serial.println("[nrf] >>> BB: MISO stuck LOW (0x00)");
    } else {
        Serial.printf("[nrf] >>> BB: NRF24 IS responding! STATUS=0x%02X (expect 0x0E)\n", bb_nop);
    }
    Serial.println("[nrf] ---------------------------------");

    // SPI.begin() called ONCE — repeated calls corrupt USB CDC on ESP32-S3
    SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CSN);
    delay(100);

    Serial.println("[nrf] Calling radio.begin()...");
    if (!radio.begin(&SPI)) {
        Serial.println("[nrf] FAIL: radio.begin() returned false");
        Serial.printf("[nrf] isChipConnected: %s\n", radio.isChipConnected() ? "YES" : "NO");
        return;
    }

    Serial.println("[nrf] radio.begin() OK");
    Serial.printf("[nrf] isChipConnected: %s\n", radio.isChipConnected() ? "YES" : "NO");
    radio.setPALevel(RF24_PA_LOW);
    radio.setDataRate(RF24_250KBPS);
    radio.setChannel(76);
    Serial.printf("[nrf] Channel:  %d\n", radio.getChannel());
    Serial.printf("[nrf] PA level: %d\n", radio.getPALevel());
    Serial.printf("[nrf] Data rate: %d\n", radio.getDataRate());
    Serial.printf("[nrf] CRC length: %d\n", radio.getCRCLength());
    radio.printPrettyDetails();
    Serial.println("[nrf] >>> PASS: NRF24L01+ connected and responding!");
}

void loop() {
    delay(5000);
    Serial.printf("[nrf] alive, isChipConnected=%s\n",
                  radio.isChipConnected() ? "YES" : "NO");
}



