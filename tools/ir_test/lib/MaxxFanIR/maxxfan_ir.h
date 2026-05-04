/**
 * @file maxxfan_ir.h
 * @brief MaxxFan Deluxe IR control library for ESP32
 * 
 * This library implements the MaxxFan Deluxe IR protocol using the ESP32 RMT peripheral.
 * Protocol uses 38 kHz IR carrier with custom RS232-like encoding.
 * 
 * Ported from: https://github.com/brown-studios/esphome-maxxfan-protocol (MIT license)
 * Protocol docs: https://github.com/skypeachblue/maxxfan-reversing
 */

#pragma once

#include <Arduino.h>

/**
 * @brief MaxxFan state structure
 * 
 * Represents the current state of the MaxxFan device.
 * This is the "assumed" state since IR is one-directional (no feedback).
 */
struct MaxxFanState {
    bool fan_on = false;           ///< Fan power: true=on, false=off
    bool exhaust = true;           ///< Direction: true=exhaust, false=intake
    bool lid_open = false;         ///< Cover/lid: true=open, false=closed
    bool auto_mode = false;        ///< Mode: true=auto (thermostat), false=manual
    uint8_t speed = 0;             ///< Fan speed 0-100 (steps of 10)
    uint8_t auto_temp_f = 70;      ///< Thermostat setpoint in °F (for auto mode)
    bool warn = false;             ///< Warning beep: true=beep enabled, false=silent
    bool special = false;          ///< Special mode (auto/ceiling mode)
};

/**
 * @brief Initialize the MaxxFan IR transmitter
 * 
 * Configures the ESP32 RMT peripheral for 38 kHz IR transmission on the specified GPIO.
 * Must be called once during setup before calling maxxfan_ir_send().
 * 
 * @param gpio_pin GPIO pin connected to IR LED (e.g., GPIO 3)
 */
void maxxfan_ir_init(uint8_t gpio_pin);

/**
 * @brief Send IR command to MaxxFan
 * 
 * Encodes the current state into a 16-byte IR packet and transmits it.
 * Updates the internal assumed state.
 * 
 * @param state Fan state to transmit
 */
void maxxfan_ir_send(const MaxxFanState& state);

/**
 * @brief Get current assumed fan state
 * 
 * Returns a reference to the internal state tracker.
 * Since IR is one-directional, this reflects the last transmitted state.
 * 
 * @return Reference to current assumed state
 */
MaxxFanState& maxxfan_ir_get_state();
