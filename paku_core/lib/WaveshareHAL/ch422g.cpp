/**
 * @file ch422g.cpp
 * @brief CH422G I2C IO-expander driver implementation.
 *
 * Register map (inferred from Waveshare demo code – not verified against
 * an official CH422G datasheet):
 *   0x24  –  OC output register (active when OC mode enabled)
 *   0x46  –  mode/config ("set") register
 *   0x70  –  push-pull output register (active when IO-OE mode enabled)
 *
 * To use the EXIO pins as push-pull outputs the sequence is:
 *   1. Write 0x01 to register 0x46   → enable IO-OE (push-pull mode)
 *   2. Write bitmask to register 0x70 → set pin levels
 */
#include "ch422g.h"

/// CH422G internal registers (addresses on the I2C bus)
static constexpr uint8_t CH422G_REG_SET   = 0x46;  // mode / config
static constexpr uint8_t CH422G_REG_OC    = 0x24;  // open-collector output
static constexpr uint8_t CH422G_REG_IO    = 0x70;  // push-pull output

CH422G::CH422G(TwoWire &wire, uint8_t addr)
    : _wire(wire), _addr(addr), _outputState(0x00) {}

bool CH422G::begin() {
    // Enable push-pull output mode (IO-OE = 1)
    if (!_write(CH422G_REG_SET, 0x01)) return false;

    // Start with all outputs LOW
    _outputState = 0x00;
    return _write(CH422G_REG_IO, _outputState);
}

void CH422G::digitalWrite(uint8_t pin, uint8_t level) {
    if (pin > 7) return;
    if (level) {
        _outputState |= (1 << pin);
    } else {
        _outputState &= ~(1 << pin);
    }
    _write(CH422G_REG_IO, _outputState);
}

void CH422G::writeOutputs(uint8_t value) {
    _outputState = value;
    _write(CH422G_REG_IO, _outputState);
}

/* ---------- private ---------- */

bool CH422G::_write(uint8_t regAddr, uint8_t data) {
    // The CH422G uses a slightly unusual scheme where the register
    // address IS the I2C device address (bit-shifted).  Each register
    // has its own 7-bit address on the bus rather than using a sub-register
    // byte.  So we shift regAddr >> 1 to form the 7-bit I2C address.
    _wire.beginTransmission(regAddr >> 1);
    _wire.write(data);
    return (_wire.endTransmission() == 0);
}
