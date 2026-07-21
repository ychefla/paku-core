#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>

#define PIN_CE   1
#define PIN_CSN  2
#define PIN_SCK  10
#define PIN_MOSI 11
#define PIN_MISO 12

RF24 radio(PIN_CE, PIN_CSN);
bool radio_ok = false;

void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("[nrf] boot");
    Serial.flush();
}

void loop() {
    static bool done = false;

    if (!done) {
        Serial.println("[nrf] --- GPIO state ---");
        const int pins[] = {PIN_CE, PIN_CSN, PIN_SCK, PIN_MOSI, PIN_MISO};
        const char* names[] = {"CE(1)", "CSN(2)", "SCK(10)", "MOSI(11)", "MISO(12)"};
        for (int i = 0; i < 5; i++) {
            pinMode(pins[i], INPUT);
            Serial.printf("[nrf]   %-10s = %d\n", names[i], digitalRead(pins[i]));
        }
        pinMode(PIN_MISO, INPUT_PULLUP); delay(1);
        Serial.printf("[nrf]   MISO+pullup = %d\n", digitalRead(PIN_MISO));
        pinMode(PIN_MISO, INPUT);

        // Bit-bang probe
        Serial.println("[nrf] --- bit-bang SPI probe ---");
        pinMode(PIN_CE,  OUTPUT); digitalWrite(PIN_CE,  LOW);
        pinMode(PIN_CSN, OUTPUT); digitalWrite(PIN_CSN, HIGH);
        pinMode(PIN_SCK, OUTPUT); digitalWrite(PIN_SCK, LOW);
        pinMode(PIN_MOSI,OUTPUT); digitalWrite(PIN_MOSI,LOW);
        pinMode(PIN_MISO,INPUT);
        delay(5);

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

        digitalWrite(PIN_CSN, LOW); delayMicroseconds(10);
        uint8_t status = bb_xfer(0xFF);
        digitalWrite(PIN_CSN, HIGH); delayMicroseconds(10);

        digitalWrite(PIN_CSN, LOW); delayMicroseconds(10);
        uint8_t st2 = bb_xfer(0x00);
        uint8_t cfg = bb_xfer(0x00);
        digitalWrite(PIN_CSN, HIGH);

        Serial.printf("[nrf] BB NOP STATUS=0x%02X\n", status);
        Serial.printf("[nrf] BB CONFIG STATUS=0x%02X VALUE=0x%02X\n", st2, cfg);
        if (status == 0xFF)      Serial.println("[nrf] >>> MISO floating — chip not responding");
        else if (status == 0x00) Serial.println("[nrf] >>> MISO stuck LOW");
        else                     Serial.printf("[nrf] >>> CHIP ALIVE (expect 0x0E)\n");

        Serial.flush();
        done = true;
    }

    Serial.println("[nrf] alive");
    Serial.flush();
    delay(3000);
}
