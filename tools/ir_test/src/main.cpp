/**
 * ir_test/src/main.cpp
 *
 * Tests MaxxFan IR TX (GPIO25) and IR RX (GPIO27) independently.
 *
 * Every 3 seconds:
 *   1. Announces "Sending IR frame..." on serial
 *   2. Sends a MaxxFan "fan on, 50% speed, exhaust" command via GPIO25
 *   3. Prints "TX done"
 *
 * Continuously (between TX bursts):
 *   - Monitors GPIO27 for falling/rising edges using interrupts
 *   - Captures pulse durations in microseconds
 *   - After each burst of pulses (gap > 10 ms), prints the captured pulses
 *
 * Expected on serial if wiring is correct:
 *   - RX pulses appear within ~20 ms of "Sending IR frame..."
 *   - Pulse widths ~800 µs (MaxxFan bit period)
 *   - Preamble pulses ~400 µs mark + ~400 µs space (approx)
 *
 * If RX never shows pulses:
 *   - Check IR LED is actually emitting (use phone camera to verify)
 *   - Check VS1838B output is at 3.3V idle (active LOW when receiving)
 *   - Make sure IR LED and receiver are pointed at each other (30–50 cm)
 */

#include <Arduino.h>
#include "maxxfan_ir.h"

#define PIN_IR_TX  25
#define PIN_IR_RX  27

// ---- RX pulse capture -------------------------------------------------------
#define MAX_PULSES 200

static volatile uint32_t pulse_times[MAX_PULSES];
static volatile uint8_t  pulse_levels[MAX_PULSES];  // 0=LOW, 1=HIGH
static volatile int      pulse_count = 0;
static volatile uint32_t last_edge_us = 0;
static volatile bool     capture_active = false;

void IRAM_ATTR ir_rx_isr() {
    uint32_t now = micros();
    if (pulse_count < MAX_PULSES) {
        pulse_times[pulse_count] = now - last_edge_us;
        pulse_levels[pulse_count] = digitalRead(PIN_IR_RX);
        pulse_count++;
    }
    last_edge_us = now;
    capture_active = true;
}

// ---- TX state ---------------------------------------------------------------
static MaxxFanState fan_state;

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== MaxxFan IR TX/RX Test ===");
    Serial.printf("  TX pin: GPIO%d\n", PIN_IR_TX);
    Serial.printf("  RX pin: GPIO%d\n\n", PIN_IR_RX);

    // TX init
    maxxfan_ir_init(PIN_IR_TX);

    // Set up a test state: fan on, manual mode, 50% speed, exhaust
    fan_state.fan_on   = true;
    fan_state.exhaust  = true;
    fan_state.lid_open = true;
    fan_state.auto_mode = false;
    fan_state.speed    = 50;

    // RX init — VS1838B is active-LOW, use pullup to avoid floating
    pinMode(PIN_IR_RX, INPUT_PULLUP);
    Serial.printf("RX idle level: %s (should be HIGH for VS1838B)\n",
                  digitalRead(PIN_IR_RX) ? "HIGH" : "LOW");
    attachInterrupt(digitalPinToInterrupt(PIN_IR_RX), ir_rx_isr, CHANGE);

    Serial.println("Setup done. Sending first frame in 1 s...\n");
    delay(1000);
}

static uint32_t last_tx_ms = 0;
static uint32_t last_print_ms = 0;

void loop() {
    uint32_t now_ms = millis();

    // ---------- TX: every 3 seconds ----------
    if (now_ms - last_tx_ms >= 3000) {
        last_tx_ms = now_ms;

        // Reset RX capture before TX
        noInterrupts();
        pulse_count   = 0;
        capture_active = false;
        interrupts();

        Serial.println("--- Sending IR frame (fan on, 50%, exhaust) ---");
        maxxfan_ir_send(fan_state);
        Serial.println("TX done.");
    }

    // ---------- RX: print after 80 ms gap (frame should arrive within 50 ms) ---
    if (capture_active && (now_ms - last_tx_ms) >= 80 && now_ms - last_print_ms >= 500) {
        last_print_ms = now_ms;

        noInterrupts();
        int count = pulse_count;
        interrupts();

        if (count == 0) {
            Serial.println("RX: no pulses captured after TX");
        } else {
            Serial.printf("RX: captured %d edges\n", count);
            for (int i = 0; i < count && i < 60; i++) {
                Serial.printf("  [%3d] %s  %5lu us\n",
                    i,
                    pulse_levels[i] ? "HIGH" : "LOW ",
                    (unsigned long)pulse_times[i]);
            }
            if (count > 60) Serial.printf("  ... (%d more)\n", count - 60);

            // Simple sanity: look for ~800 µs pulses (MaxxFan bit period)
            int hits = 0;
            for (int i = 1; i < count; i++) {
                if (pulse_times[i] >= 600 && pulse_times[i] <= 1000) hits++;
            }
            Serial.printf("RX sanity: %d pulses in 600–1000 µs range (MaxxFan bit period ~800 µs)\n", hits);
            if (hits >= 10) {
                Serial.println(">>> PASS: IR loopback looks correct!");
            } else if (count > 5) {
                Serial.println(">>> PARTIAL: pulses detected but widths unexpected.");
                Serial.println("    Check beam alignment and VS1838B orientation.");
            } else {
                Serial.println(">>> FAIL: too few pulses. Check wiring.");
            }
        }
        Serial.println();

        noInterrupts();
        capture_active = false;
        interrupts();
    }
}
