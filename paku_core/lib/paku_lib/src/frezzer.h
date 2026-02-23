/**
 * @file frezzer.h
 * @brief Frezzer PRO compressor fridge BLE data structures and parsing
 *
 * This module provides structures and functions for parsing and handling
 * Frezzer PRO 65L 12/24V compressor fridge BLE data.
 *
 * @note The Frezzer PRO uses the Alpicool OEM BLE protocol, confirmed via
 *       its BLE device name "WT-0001". The protocol is documented by
 *       klightspeed/BrassMonkeyFridgeMonitor (MIT) and implemented in
 *       Gruni22/alpicool_ha_ble (Home Assistant integration).
 *       Frame format: FE FE | len | cmd | body... | checksum(2, big-endian)
 *       Service 0x1234, write char 0x1235, notify char 0x1236.
 */
#pragma once

#include <cstdint>
#include <cstddef>

/**
 * @brief Maximum number of Frezzer devices that can be tracked
 */
#define MAX_FREZZER_DEVICES 4

/**
 * @brief Known BLE device names for Alpicool OEM fridge platform.
 *
 * The Frezzer PRO advertises as "WT-0001". Other Alpicool-compatible
 * brands use names starting with A1-, AK1-, AK2-, or AK3-.
 * Use isFrezzerDevice() for matching logic.
 */
#define FREZZER_DEVICE_NAME_WT  "WT-0001"
#define FREZZER_DEVICE_NAME_A1  "A1-"
#define FREZZER_DEVICE_NAME_AK1 "AK1-"
#define FREZZER_DEVICE_NAME_AK2 "AK2-"
#define FREZZER_DEVICE_NAME_AK3 "AK3-"

/**
 * @brief Query interval in milliseconds (Alpicool app polls every 2 s)
 */
#define FREZZER_QUERY_INTERVAL_MS 2000

/**
 * @brief Timeout in milliseconds for considering device data as stale
 */
#define FREZZER_STALE_TIMEOUT_MS 300000  // 5 minutes

/**
 * @brief Connection timeout in milliseconds
 */
#define FREZZER_CONNECT_TIMEOUT_MS 10000  // 10 seconds

/**
 * @brief Frezzer operating mode (derived from Alpicool poweredOn + runMode fields)
 *
 * Alpicool protocol: poweredOn (bool) + runMode (0=Max, 1=Eco).
 */
enum class FrezzerMode : uint8_t {
    OFF      = 0,    ///< Fridge powered off (poweredOn=false)
    MAX_COOL = 1,    ///< Maximum cooling (poweredOn=true, runMode=0)
    ECO      = 2,    ///< Eco/power-saving mode (poweredOn=true, runMode=1)
    UNKNOWN  = 255   ///< Unknown/parse error
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
 * @brief Frezzer device sensor data (Alpicool query response, single-zone)
 *
 * All temperature values are in Celsius (signed int8 in wire format).
 * battery_voltage = batVolInt + batVolDec / 10.0f
 */
struct FrezzerData {
    float currentTemp;                 ///< Current internal temperature (°C)
    float targetTemp;                  ///< Target temperature setpoint (°C)
    float batteryVoltage;              ///< Supply voltage (V)
    FrezzerMode mode;                  ///< Combined power+run mode
    FrezzerCompressorState compressor; ///< Inferred compressor state
    FrezzerError error;                ///< Error state
    bool locked;                       ///< Control panel locked
    uint8_t batPercent;                ///< Battery charge % (0-100, 0x7F=unknown)
    uint8_t batSaverMode;              ///< Low-voltage cutout level (0=Low,1=Mid,2=High)
    bool valid;                        ///< Whether data was parsed successfully
};

/**
 * @brief Alpicool protocol command codes (from klightspeed/BrassMonkeyFridgeMonitor)
 *
 * Frame: FE FE | (len+2) | cmd [param...] | checksum_hi checksum_lo
 */
enum class FrezzerCommand : uint8_t {
    BIND             = 0x00, ///< Bind (fridge shows "APP", user presses button)
    QUERY            = 0x01, ///< Query all status — fridge responds on notify char
    SET              = 0x02, ///< Set all parameters (full settings payload)
    RESET            = 0x04, ///< Factory reset
    SET_LEFT_TARGET  = 0x05, ///< Set unit 1 (single/left zone) target temp (int8 °C)
    SET_RIGHT_TARGET = 0x06  ///< Set unit 2 (right zone) target temp (int8 °C)
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
