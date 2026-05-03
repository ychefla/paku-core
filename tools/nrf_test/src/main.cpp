#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>

// VSPI pins: SCK=18, MISO=19, MOSI=23, CE=22, CSN=21
#define PIN_CE  22
#define PIN_CSN 21

RF24 radio(PIN_CE, PIN_CSN);

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("[nrf] NRF24L01+ test starting...");

    if (!radio.begin()) {
        Serial.println("[nrf] FAIL: radio.begin() returned false - check wiring!");
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
    Serial.printf("[nrf] alive, isChipConnected=%s\n", radio.isChipConnected() ? "YES" : "NO");
}
