/**
 * @file frezzer.h
 * @brief Frezzer PRO compressor fridge BLE data structures and parsing
 * 
 * This module provides structures and functions for parsing and handling
 * Frezzer PRO 65L 12/24V compressor fridge BLE data.
 * 
 * @note The Frezzer PRO uses a proprietary BLE protocol. The UUIDs and
 *       data formats in this file are based on reverse engineering and
 *       may need adjustment for your specific device/firmware version.
 */
#pragma once

#include <cstdint>
#include <cstddef>

/**
 * @brief Maximum number of Frezzer devices that can be tracked
 */
#define MAX_FREZZER_DEVICES 4

/**
 * @brief Frezzer device name prefix for BLE scanning
 */
#define FREZZER_DEVICE_NAME_PREFIX "FREZZER"

/**
 * @brief Timeout in milliseconds for considering device data as stale
 */
#define FREZZER_STALE_TIMEOUT_MS 300000  // 5 minutes

/**
 * @brief Connection timeout in milliseconds
 */
#define FREZZER_CONNECT_TIMEOUT_MS 10000  // 10 seconds

/**
 * @brief Frezzer operating mode enumeration
 */
enum class FrezzerMode : uint8_t {
    OFF = 0,          ///< Fridge is off
    FRIDGE = 1,       ///< Fridge mode (typically 0-8°C)
    FREEZER = 2,      ///< Freezer mode (typically -18 to -22°C)
    ECO = 3,          ///< Eco/power saving mode
    MAX_COOL = 4,     ///< Maximum cooling mode
    UNKNOWN = 255     ///< Unknown mode
};

/**
 * @brief Frezzer compressor state enumeration
 */
enum class FrezzerCompressorState : uint8_t {
    OFF = 0,          ///< Compressor off
    RUNNING = 1,      ///< Compressor running
    STANDBY = 2,      ///< Compressor in standby (waiting)
    ERROR = 3,        ///< Compressor error
    UNKNOWN = 255     ///< Unknown state
};

/**
 * @brief Frezzer error codes
 */
enum class FrezzerError : uint8_t {
    NONE = 0,             ///< No error
    TEMP_SENSOR = 1,      ///< Temperature sensor error
    COMPRESSOR = 2,       ///< Compressor error
    LOW_VOLTAGE = 3,      ///< Low battery voltage
    HIGH_VOLTAGE = 4,     ///< High battery voltage
    OVERTEMP = 5,         ///< Overtemperature protection
    COMMUNICATION = 6,    ///< BLE communication error
    UNKNOWN = 255         ///< Unknown error
};

/**
 * @brief Frezzer device sensor data
 */
struct FrezzerData {
    float currentTemp;             ///< Current internal temperature in Celsius
    float targetTemp;              ///< Target temperature setting in Celsius
    float batteryVoltage;          ///< Battery/power supply voltage
    FrezzerMode mode;              ///< Current operating mode
    FrezzerCompressorState compressor; ///< Compressor state
    FrezzerError error;            ///< Current error code
    bool lidOpen;                  ///< Lid/door open sensor status
    bool lowVoltageProtection;     ///< Low voltage protection active
    uint8_t powerLevel;            ///< Power consumption level (0-100%)
    bool valid;                    ///< Whether the data is valid
};

/**
 * @brief Frezzer command types for control
 */
enum class FrezzerCommand : uint8_t {
    SET_TARGET_TEMP = 0x01,   ///< Set target temperature
    SET_MODE = 0x02,          ///< Set operating mode
    POWER_ON = 0x03,          ///< Turn on the fridge
    POWER_OFF = 0x04,         ///< Turn off the fridge
    REQUEST_STATUS = 0x05,    ///< Request current status
    SET_LOW_VOLTAGE_CUTOFF = 0x06  ///< Set low voltage protection threshold
};

/**
 * @brief Result codes for Frezzer operations
 */
enum class FrezzerResult {
    SUCCESS = 0,
    NOT_CONNECTED = 1,
    CONNECT_FAILED = 2,
    WRITE_FAILED = 3,
    READ_FAILED = 4,
    TIMEOUT = 5,
    INVALID_PARAM = 6,
    DEVICE_NOT_FOUND = 7,
    SERVICE_NOT_FOUND = 8,
    CHARACTERISTIC_NOT_FOUND = 9,
    ALREADY_CONNECTED = 10,
    BUSY = 11,
    UNKNOWN_ERROR = 255
};

/**
 * @brief Structure to track a single Frezzer device
 */
struct FrezzerDevice {
    uint8_t macAddress[6];      ///< MAC address of the device
    char macString[18];         ///< MAC address as string "AA:BB:CC:DD:EE:FF"
    char deviceName[32];        ///< Device name from BLE scan
    char location[32];          ///< User-assigned location name
    FrezzerData lastData;       ///< Most recent sensor data
    unsigned long lastSeen;     ///< Timestamp when last seen (millis)
    bool registered;            ///< Whether this device is registered/configured
    bool hasData;               ///< Whether we have valid data from this device
    bool connected;             ///< Whether currently connected via GATT
};

/**
 * @brief Parses raw BLE data into FrezzerData structure
 * 
 * @param data Raw BLE characteristic data
 * @param length Length of the data
 * @return FrezzerData Parsed sensor data
 */
FrezzerData parseFrezzerData(const uint8_t* data, size_t length);

/**
 * @brief Validates if the data appears to be valid Frezzer data
 * 
 * @param data Raw BLE data
 * @param length Length of the data
 * @return true if data appears valid
 */
bool isValidFrezzerData(const uint8_t* data, size_t length);

/**
 * @brief Converts Frezzer mode enum to string
 * 
 * @param mode The mode to convert
 * @return const char* String representation
 */
const char* frezzerModeToString(FrezzerMode mode);

/**
 * @brief Converts Frezzer compressor state to string
 * 
 * @param state The compressor state to convert
 * @return const char* String representation
 */
const char* frezzerCompressorStateToString(FrezzerCompressorState state);

/**
 * @brief Converts Frezzer error code to string
 * 
 * @param error The error code to convert
 * @return const char* String representation
 */
const char* frezzerErrorToString(FrezzerError error);

/**
 * @brief Converts FrezzerResult to human-readable string
 * 
 * @param result The result code
 * @return const char* String representation
 */
const char* frezzerResultToString(FrezzerResult result);

/**
 * @brief Checks if a BLE device name matches Frezzer naming pattern
 * 
 * @param deviceName The device name from BLE scan
 * @return true if this appears to be a Frezzer device
 */
bool isFrezzerDevice(const char* deviceName);

/**
 * @brief Builds a command payload for sending to the Frezzer
 * 
 * @param cmd Command type
 * @param param Command parameter (interpretation depends on command)
 * @param buffer Output buffer for the command (at least 8 bytes)
 * @param bufferSize Size of the output buffer
 * @return size_t Number of bytes written, 0 on error
 */
size_t buildFrezzerCommand(FrezzerCommand cmd, int16_t param, 
                            uint8_t* buffer, size_t bufferSize);

#ifdef UNIT_TEST
/**
 * @brief Reset internal state for testing
 */
void resetFrezzerForTesting(void);
#endif
