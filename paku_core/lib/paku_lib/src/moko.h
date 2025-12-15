/**
 * @file moko.h
 * @brief MoKo sensor message parsing and handling
 * 
 * This module provides structures and functions for parsing
 * MoKo BeaconX Pro BLE advertisement data.
 * 
 * BeaconX Pro Frame Types (parsed):
 * - 0x10 0x06: Temperature and Humidity data (4 bytes: temp + humid)
 * - 0x10 0x02: Device info (battery level, firmware, etc.)
 * - 0x16: Service data format (alternative temp/humid format)
 * 
 * Frame Types (rejected):
 * - 0x12: iBeacon/other beacon types
 * - 0x09: Device name
 * 
 * Expected broadcast configuration: "Temperature and Humidity" + "Device info" modes enabled
 */
#pragma once

#include <cstdint>
#include <cstddef>

/**
 * @brief MoKo sensor data structure
 * 
 * Supports MoKo H2, H3, H4 beacon sensors with temperature, humidity,
 * accelerometer, and battery information.
 */
struct MoKoData {
    float temperature;      // Temperature in Celsius
    float humidity;         // Humidity in %
    float pressure;         // Pressure in Pa (if supported)
    float accelerationX;    // Acceleration X in g
    float accelerationY;    // Acceleration Y in g
    float accelerationZ;    // Acceleration Z in g
    float batteryVoltage;   // Battery voltage in V
    uint8_t batteryPercent; // Battery percentage 0-100
    int8_t rssi;            // Signal strength in dBm
    uint16_t frameCounter;  // Frame counter
    bool valid;             // Whether the data is valid
};

/**
 * @brief Parses MoKo sensor data from BLE advertisement
 * 
 * MoKo sensors use various formats depending on the model.
 * This parser supports the common format used by H2/H3/H4 series.
 * 
 * @param data Raw advertisement data
 * @param length Length of the data
 * @return MoKoData Parsed sensor data
 */
MoKoData parseMoKoData(const uint8_t* data, size_t length);

/**
 * @brief Validates if the data is a valid MoKo format
 * 
 * @param data Raw advertisement data
 * @param length Length of the data
 * @return true if valid MoKo data
 * @return false otherwise
 */
bool isValidMoKoData(const uint8_t* data, size_t length);

/**
 * @brief Detects MoKo sensor model from advertisement data
 * 
 * @param data Raw advertisement data
 * @param length Length of the data
 * @return Model identifier (0 = unknown, 1 = H2, 2 = H3, 3 = H4)
 */
uint8_t detectMoKoModel(const uint8_t* data, size_t length);
