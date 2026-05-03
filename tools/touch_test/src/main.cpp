#include <Arduino.h>
#define PIN_TOUCH 32

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("[touch] TTP223 test ready. Touch the sensor!");
    pinMode(PIN_TOUCH, INPUT);
}

static bool last = false;
void loop() {
    bool cur = digitalRead(PIN_TOUCH);
    if (cur != last) {
        last = cur;
        Serial.printf("[touch] %s  (millis=%lu)\n", cur ? "TOUCHED" : "released", millis());
    }
    delay(10);
}
