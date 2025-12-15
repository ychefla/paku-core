/**
 * @file moko_scanner.h
 * @brief BLE scanner for MoKo sensor data collection
 * 
 * This module provides interfaces for scanning and collecting
 * data from MoKo BLE sensors (H2/H3/H4 series) and managing sensor registrations.
 */
#pragma once

#include <cstdint>
#include <cstddef>
#include "moko.h"

/**
 * @brief Maximum number of MoKo sensors that can be tracked
 */
#define MAX_MOKO_SENSORS 8

/**
 * @brief MoKo manufacturer ID (0x004C for some models, varies by model)
 * Note: MoKo sensors may use different manufacturer IDs depending on the model.
 * Common values: 0x004C (Apple iBeacon compatible), 0x0590, or custom IDs.
 */
#define MOKO_MANUFACTURER_ID_PRIMARY 0x004C
#define MOKO_MANUFACTURER_ID_SECONDARY 0x0590

/**
 * @brief Timeout in milliseconds for considering a sensor as stale
 */
#define MOKO_STALE_TIMEOUT_MS 300000  // 5 minutes

/**
 * @brief Structure to track a single MoKo sensor
 */
struct MoKoSensor {
    uint8_t macAddress[6];      // MAC address of the sensor
    char macString[18];         // MAC address as string "AA:BB:CC:DD:EE:FF"
    char location[32];          // Location name (e.g., "cabin", "kitchen")
    MoKoData lastData;          // Most recent sensor data
    unsigned long lastSeen;     // Timestamp when last seen (millis)
    bool registered;            // Whether this sensor is registered/configured
    bool hasData;               // Whether we have valid data from this sensor
    uint8_t model;              // Model identifier (0=unknown, 1=H2, 2=H3, 3=H4)
};

/**
 * @brief Structure to hold historical data entry from sensor memory
 */
struct MoKoHistoryEntry {
    MoKoData data;              // Sensor data
    unsigned long timestamp;    // Timestamp of reading
    bool valid;                 // Whether entry is valid
};

/**
 * @brief Container for scan results
 */
struct MoKoScanResult {
    uint8_t sensorCount;        // Number of sensors found in scan
    bool success;               // Whether scan completed successfully
};

/**
 * @brief Initializes the MoKo scanner
 * 
 * @return true if initialization successful
 */
bool initMoKoScanner(void);

/**
 * @brief Registers a known MoKo sensor by MAC address
 * 
 * @param macAddress MAC address as string (e.g., "AA:BB:CC:DD:EE:FF")
 * @param location Location identifier for this sensor
 * @return true if registered successfully
 */
bool registerMoKoSensor(const char* macAddress, const char* location);

/**
 * @brief Clears all registered sensors
 */
void clearRegisteredSensors(void);

/**
 * @brief Gets the number of registered sensors
 * 
 * @return Number of registered sensors
 */
uint8_t getRegisteredSensorCount(void);

/**
 * @brief Gets a registered sensor by index
 * 
 * @param index Sensor index (0 to getRegisteredSensorCount()-1)
 * @return Pointer to MoKoSensor or nullptr if invalid index
 */
const MoKoSensor* getRegisteredSensor(uint8_t index);

/**
 * @brief Gets a registered sensor by MAC address
 * 
 * @param macAddress MAC address as string
 * @return Pointer to MoKoSensor or nullptr if not found
 */
const MoKoSensor* findRegisteredSensorByMac(const char* macAddress);

/**
 * @brief Updates sensor data from BLE advertisement
 * 
 * This function is called when a BLE scan receives MoKo
 * manufacturer data. It parses the data and updates the
 * corresponding sensor entry.
 * 
 * @param macAddress MAC address of the sensor
 * @param data Raw manufacturer data
 * @param length Length of the data
 * @param currentTime Current timestamp (millis)
 * @return true if data was successfully parsed and stored
 */
bool updateMoKoSensorData(const uint8_t* macAddress, const uint8_t* data, 
                          size_t length, unsigned long currentTime);

/**
 * @brief Checks if a sensor has recent data (not stale)
 * 
 * @param sensor Pointer to the sensor
 * @param currentTime Current timestamp (millis)
 * @return true if data is fresh (within MOKO_STALE_TIMEOUT_MS)
 */
bool isSensorDataFresh(const MoKoSensor* sensor, unsigned long currentTime);

/**
 * @brief Gets all sensors with fresh (non-stale) data
 * 
 * @param sensors Output array of sensor pointers
 * @param maxSensors Maximum number of sensors to return
 * @param currentTime Current timestamp (millis)
 * @return Number of fresh sensors returned
 */
uint8_t getFreshSensors(const MoKoSensor** sensors, uint8_t maxSensors, unsigned long currentTime);

/**
 * @brief Converts MAC address bytes to string
 * 
 * @param mac Input MAC address (6 bytes)
 * @param buffer Output string buffer (must be at least 18 bytes)
 */
void mokoMacToString(const uint8_t* mac, char* buffer);

/**
 * @brief Converts MAC address string to bytes
 * 
 * @param macString Input string "AA:BB:CC:DD:EE:FF"
 * @param mac Output array (6 bytes)
 * @return true if parsed successfully
 */
bool mokoStringToMac(const char* macString, uint8_t* mac);

/**
 * @brief Checks if data is MoKo manufacturer data
 * 
 * @param manufacturerId Manufacturer ID from BLE advertisement
 * @return true if this is MoKo data
 */
bool isMoKoManufacturer(uint16_t manufacturerId);

/**
 * @brief Checks if device name indicates MoKo sensor
 * 
 * MoKo sensors often use names like "MK_H2", "MK_H3", "MK_H4", etc.
 * 
 * @param deviceName BLE device name
 * @return true if name pattern matches MoKo sensors
 */
bool isMoKoDeviceName(const char* deviceName);

#ifdef UNIT_TEST
/**
 * @brief Resets scanner state for testing
 */
void resetMoKoScannerForTesting(void);

/**
 * @brief Gets internal sensor array for testing
 */
MoKoSensor* getInternalSensorsForTesting(void);
#endif
