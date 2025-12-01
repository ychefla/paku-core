/**
 * @file ruuvi_scanner.h
 * @brief BLE scanner for RuuviTag data collection
 * 
 * This module provides interfaces for scanning and collecting
 * data from RuuviTag BLE sensors and managing tag registrations.
 */
#pragma once

#include <cstdint>
#include <cstddef>
#include "ruuvi.h"

/**
 * @brief Maximum number of Ruuvi tags that can be tracked
 */
#define MAX_RUUVI_TAGS 8

/**
 * @brief Ruuvi manufacturer ID (0x0499 for Ruuvi Innovations)
 */
#define RUUVI_MANUFACTURER_ID 0x0499

/**
 * @brief Timeout in milliseconds for considering a tag as stale
 */
#define RUUVI_STALE_TIMEOUT_MS 300000  // 5 minutes

/**
 * @brief Structure to track a single RuuviTag
 */
struct RuuviTag {
    uint8_t macAddress[6];      // MAC address of the tag
    char macString[18];         // MAC address as string "AA:BB:CC:DD:EE:FF"
    char location[32];          // Location name (e.g., "cabin", "kitchen")
    RuuviData lastData;         // Most recent sensor data
    unsigned long lastSeen;     // Timestamp when last seen (millis)
    bool registered;            // Whether this tag is registered/configured
    bool hasData;               // Whether we have valid data from this tag
};

/**
 * @brief Structure to hold historical data entry from tag memory
 */
struct RuuviHistoryEntry {
    RuuviData data;             // Sensor data
    unsigned long timestamp;    // Timestamp of reading
    bool valid;                 // Whether entry is valid
};

/**
 * @brief Container for scan results
 */
struct RuuviScanResult {
    uint8_t tagCount;           // Number of tags found in scan
    bool success;               // Whether scan completed successfully
};

/**
 * @brief Initializes the Ruuvi scanner
 * 
 * @return true if initialization successful
 */
bool initRuuviScanner(void);

/**
 * @brief Registers a known RuuviTag by MAC address
 * 
 * @param macAddress MAC address as string (e.g., "AA:BB:CC:DD:EE:FF")
 * @param location Location identifier for this tag
 * @return true if registered successfully
 */
bool registerRuuviTag(const char* macAddress, const char* location);

/**
 * @brief Clears all registered tags
 */
void clearRegisteredTags(void);

/**
 * @brief Gets the number of registered tags
 * 
 * @return Number of registered tags
 */
uint8_t getRegisteredTagCount(void);

/**
 * @brief Gets a registered tag by index
 * 
 * @param index Tag index (0 to getRegisteredTagCount()-1)
 * @return Pointer to RuuviTag or nullptr if invalid index
 */
const RuuviTag* getRegisteredTag(uint8_t index);

/**
 * @brief Gets a registered tag by MAC address
 * 
 * @param macAddress MAC address as string
 * @return Pointer to RuuviTag or nullptr if not found
 */
const RuuviTag* findRegisteredTagByMac(const char* macAddress);

/**
 * @brief Updates tag data from BLE advertisement
 * 
 * This function is called when a BLE scan receives Ruuvi
 * manufacturer data. It parses the data and updates the
 * corresponding tag entry.
 * 
 * @param macAddress MAC address of the tag
 * @param data Raw manufacturer data
 * @param length Length of the data
 * @param currentTime Current timestamp (millis)
 * @return true if data was successfully parsed and stored
 */
bool updateRuuviTagData(const uint8_t* macAddress, const uint8_t* data, 
                        size_t length, unsigned long currentTime);

/**
 * @brief Checks if a tag has recent data (not stale)
 * 
 * @param tag Pointer to tag
 * @param currentTime Current timestamp (millis)
 * @return true if tag has recent data
 */
bool isTagDataFresh(const RuuviTag* tag, unsigned long currentTime);

/**
 * @brief Gets all tags with fresh data
 * 
 * @param tags Array to store tag pointers
 * @param maxTags Maximum number of tags to return
 * @param currentTime Current timestamp (millis)
 * @return Number of tags with fresh data
 */
uint8_t getFreshTags(const RuuviTag** tags, uint8_t maxTags, unsigned long currentTime);

/**
 * @brief Converts MAC address bytes to string format
 * 
 * @param mac MAC address bytes (6 bytes)
 * @param buffer Output buffer (at least 18 bytes)
 */
void macToString(const uint8_t* mac, char* buffer);

/**
 * @brief Parses MAC address string to bytes
 * 
 * @param macString MAC string in format "AA:BB:CC:DD:EE:FF"
 * @param mac Output array (6 bytes)
 * @return true if parsed successfully
 */
bool stringToMac(const char* macString, uint8_t* mac);

/**
 * @brief Placeholder for downloading historical data from tag memory
 * 
 * @note RuuviTags do not support direct memory download via BLE in
 *       standard configurations. This is a placeholder for future
 *       GATT-based implementations or compatible devices.
 * 
 * @param tag Pointer to the tag
 * @param entries Output array for history entries
 * @param maxEntries Maximum entries to return
 * @return Number of entries retrieved (0 for placeholder)
 */
uint8_t downloadTagHistory(const RuuviTag* tag, RuuviHistoryEntry* entries, 
                           uint8_t maxEntries);

/**
 * @brief Checks if data is Ruuvi manufacturer data
 * 
 * @param manufacturerId Manufacturer ID from BLE advertisement
 * @return true if this is Ruuvi data
 */
bool isRuuviManufacturer(uint16_t manufacturerId);

#ifdef UNIT_TEST
/**
 * @brief Resets internal state for testing
 */
void resetRuuviScannerForTesting(void);

/**
 * @brief Gets internal tags array for testing
 */
RuuviTag* getInternalTagsForTesting(void);
#endif
