/**
 * @file ruuvi.h
 * @brief Ruuvi tag message parsing and handling
 * 
 * This module provides structures and functions for parsing
 * Ruuvi tag BLE advertisement data.
 */
#pragma once

#include <cstdint>
#include <cstddef>

/**
 * @brief Ruuvi tag sensor data format 5 (RAWv2)
 */
struct RuuviData {
    float temperature;      // Temperature in Celsius
    float humidity;         // Humidity in %
    float pressure;         // Pressure in Pa
    float accelerationX;    // Acceleration X in g
    float accelerationY;    // Acceleration Y in g
    float accelerationZ;    // Acceleration Z in g
    float batteryVoltage;   // Battery voltage in V
    int8_t txPower;         // TX power in dBm
    uint8_t movementCounter; // Movement counter
    uint16_t measurementSequence; // Measurement sequence number
    bool valid;             // Whether the data is valid
};

/**
 * @brief Parses Ruuvi RAWv2 (format 5) data from BLE advertisement
 * 
 * @param data Raw advertisement data (24 bytes expected)
 * @param length Length of the data
 * @return RuuviData Parsed sensor data
 */
RuuviData parseRuuviDataV5(const uint8_t* data, size_t length);

/**
 * @brief Validates if the data is a valid Ruuvi RAWv2 format
 * 
 * @param data Raw advertisement data
 * @param length Length of the data
 * @return true if valid Ruuvi format 5 data
 * @return false otherwise
 */
bool isValidRuuviV5(const uint8_t* data, size_t length);
