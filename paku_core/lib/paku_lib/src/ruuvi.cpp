/**
 * @file ruuvi.cpp
 * @brief Ruuvi tag message parsing implementation
 */
#include "ruuvi.h"

/**
 * RAWv2 format (data format 5):
 * Offset 0: Format (0x05)
 * Offset 1-2: Temperature (0.005°C resolution, signed)
 * Offset 3-4: Humidity (0.0025% resolution, unsigned)
 * Offset 5-6: Pressure (1 Pa resolution, unsigned, subtract 50000)
 * Offset 7-8: Acceleration X (mG resolution, signed)
 * Offset 9-10: Acceleration Y (mG resolution, signed)
 * Offset 11-12: Acceleration Z (mG resolution, signed)
 * Offset 13-14: Power info (bits 0-4: TX power, bits 5-15: battery voltage)
 * Offset 15: Movement counter
 * Offset 16-17: Measurement sequence
 * Offset 18-23: MAC address (optional)
 */

bool isValidRuuviV5(const uint8_t* data, size_t length) {
    if (data == nullptr || length < 18) {
        return false;
    }
    
    // Check format identifier
    return data[0] == 0x05;
}

RuuviData parseRuuviDataV5(const uint8_t* data, size_t length) {
    RuuviData result = {};
    result.valid = false;
    
    if (!isValidRuuviV5(data, length)) {
        return result;
    }
    
    // Temperature: bytes 1-2, 0.005°C resolution, signed
    int16_t tempRaw = static_cast<int16_t>((data[1] << 8) | data[2]);
    if (tempRaw == static_cast<int16_t>(0x8000)) {
        // Invalid value
        result.temperature = 0.0f;
    } else {
        result.temperature = tempRaw * 0.005f;
    }
    
    // Humidity: bytes 3-4, 0.0025% resolution, unsigned
    uint16_t humidRaw = static_cast<uint16_t>((data[3] << 8) | data[4]);
    if (humidRaw == 0xFFFF) {
        result.humidity = 0.0f;
    } else {
        result.humidity = humidRaw * 0.0025f;
    }
    
    // Pressure: bytes 5-6, 1 Pa resolution, offset by 50000
    uint16_t pressRaw = static_cast<uint16_t>((data[5] << 8) | data[6]);
    if (pressRaw == 0xFFFF) {
        result.pressure = 0.0f;
    } else {
        result.pressure = static_cast<float>(pressRaw) + 50000.0f;
    }
    
    // Acceleration X: bytes 7-8, mG resolution, signed
    int16_t accelXRaw = static_cast<int16_t>((data[7] << 8) | data[8]);
    if (accelXRaw == static_cast<int16_t>(0x8000)) {
        result.accelerationX = 0.0f;
    } else {
        result.accelerationX = accelXRaw / 1000.0f;  // Convert mG to G
    }
    
    // Acceleration Y: bytes 9-10, mG resolution, signed
    int16_t accelYRaw = static_cast<int16_t>((data[9] << 8) | data[10]);
    if (accelYRaw == static_cast<int16_t>(0x8000)) {
        result.accelerationY = 0.0f;
    } else {
        result.accelerationY = accelYRaw / 1000.0f;
    }
    
    // Acceleration Z: bytes 11-12, mG resolution, signed
    int16_t accelZRaw = static_cast<int16_t>((data[11] << 8) | data[12]);
    if (accelZRaw == static_cast<int16_t>(0x8000)) {
        result.accelerationZ = 0.0f;
    } else {
        result.accelerationZ = accelZRaw / 1000.0f;
    }
    
    // Power info: bytes 13-14
    // Bits 0-4: TX power = -40 + (value * 2) dBm
    // Bits 5-15: Battery voltage = (value + 1600) mV
    uint16_t powerRaw = static_cast<uint16_t>((data[13] << 8) | data[14]);
    uint8_t txPowerRaw = powerRaw & 0x1F;
    uint16_t batteryRaw = (powerRaw >> 5) & 0x7FF;
    
    if (txPowerRaw == 0x1F) {
        result.txPower = 0;
    } else {
        result.txPower = static_cast<int8_t>(-40 + (txPowerRaw * 2));
    }
    
    if (batteryRaw == 0x7FF) {
        result.batteryVoltage = 0.0f;
    } else {
        result.batteryVoltage = (batteryRaw + 1600) / 1000.0f;  // Convert mV to V
    }
    
    // Movement counter: byte 15
    result.movementCounter = data[15];
    
    // Measurement sequence: bytes 16-17
    result.measurementSequence = static_cast<uint16_t>((data[16] << 8) | data[17]);
    
    result.valid = true;
    return result;
}
