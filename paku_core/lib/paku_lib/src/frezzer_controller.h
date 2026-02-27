/**
 * @file frezzer_controller.h
 * @brief Frezzer PRO compressor fridge BLE GATT controller
 *
 * This module provides interfaces for connecting to and controlling
 * Frezzer PRO 65L 12/24V compressor fridges via BLE GATT.
 *
 * @note Protocol confirmed: the Frezzer PRO (BLE name "WT-0001") uses the
 *       Alpicool OEM platform. UUIDs and protocol documented by
 *       klightspeed/BrassMonkeyFridgeMonitor (MIT license).
 */
#pragma once

#include <cstdint>
#include <cstddef>
#include "frezzer.h"

/**
 * @brief Alpicool GATT service UUID (16-bit 0x1234 in 128-bit form)
 */
#define FREZZER_SERVICE_UUID      "00001234-0000-1000-8000-00805f9b34fb"

/**
 * @brief Write characteristic UUID (0x1235) — send commands to the fridge
 */
#define FREZZER_WRITE_CHAR_UUID   "00001235-0000-1000-8000-00805f9b34fb"

/**
 * @brief Notify characteristic UUID (0x1236) — receive status from the fridge
 */
#define FREZZER_NOTIFY_CHAR_UUID  "00001236-0000-1000-8000-00805f9b34fb"

/**
 * @brief Maximum time to wait for BLE scan results (ms)
 */
#define FREZZER_SCAN_TIMEOUT_MS 10000

/**
 * @brief Interval between QUERY polls when connected (ms).
 * Alpicool app polls every 2 seconds.
 */
#define FREZZER_STATUS_INTERVAL_MS 2000

/**
 * @brief Callback type for Frezzer status updates
 * 
 * @param device Pointer to the device that sent the update
 * @param data Current device data
 */
typedef void (*FrezzerStatusCallback)(const FrezzerDevice* device, const FrezzerData* data);

/**
 * @brief Callback type for Frezzer connection state changes
 * 
 * @param device Pointer to the device
 * @param connected true if connected, false if disconnected
 */
typedef void (*FrezzerConnectionCallback)(const FrezzerDevice* device, bool connected);

/**
 * @brief Initializes the Frezzer controller
 * 
 * Must be called after BLEDevice::init() in the main application.
 * 
 * @return true if initialization successful
 */
bool initFrezzerController(void);

/**
 * @brief Registers a known Frezzer device by MAC address
 * 
 * @param macAddress MAC address as string (e.g., "AA:BB:CC:DD:EE:FF")
 * @param location Location identifier for this device (e.g., "van_fridge")
 * @return true if registered successfully
 */
bool registerFrezzerDevice(const char* macAddress, const char* location);

/**
 * @brief Clears all registered Frezzer devices
 */
void clearFrezzerDevices(void);

/**
 * @brief Gets the number of registered Frezzer devices
 * 
 * @return Number of registered devices
 */
uint8_t getFrezzerDeviceCount(void);

/**
 * @brief Gets a registered Frezzer device by index
 * 
 * @param index Device index (0 to getFrezzerDeviceCount()-1)
 * @return Pointer to FrezzerDevice or nullptr if invalid index
 */
const FrezzerDevice* getFrezzerDevice(uint8_t index);

/**
 * @brief Finds a registered Frezzer device by MAC address
 * 
 * @param macAddress MAC address as string
 * @return Pointer to FrezzerDevice or nullptr if not found
 */
const FrezzerDevice* findFrezzerDeviceByMac(const char* macAddress);

/**
 * @brief Updates Frezzer device data from BLE scan advertisement
 * 
 * Called during BLE scanning to process any Frezzer advertisement data.
 * Note: Most Frezzer data comes from GATT connections, not advertisements.
 * 
 * @param macAddress MAC address of the device
 * @param deviceName Device name from advertisement
 * @param rssi Signal strength
 * @param currentTime Current timestamp (millis)
 * @return true if device was found/registered
 */
bool updateFrezzerFromScan(const uint8_t* macAddress, const char* deviceName,
                            int rssi, unsigned long currentTime);

/**
 * @brief Connects to a Frezzer device via BLE GATT
 * 
 * This is a blocking operation that may take several seconds.
 * 
 * @param device Pointer to the device to connect to
 * @return FrezzerResult SUCCESS or error code
 */
FrezzerResult connectFrezzer(FrezzerDevice* device);

/**
 * @brief Disconnects from a Frezzer device
 * 
 * @param device Pointer to the device to disconnect
 * @return FrezzerResult SUCCESS or error code
 */
FrezzerResult disconnectFrezzer(FrezzerDevice* device);

/**
 * @brief Checks if currently connected to a Frezzer device
 * 
 * @param device Pointer to the device to check
 * @return true if connected
 */
bool isFrezzerConnected(const FrezzerDevice* device);

/**
 * @brief Reads current status from connected Frezzer device
 * 
 * Updates the device's lastData field with current status.
 * 
 * @param device Pointer to the device to read from
 * @return FrezzerResult SUCCESS or error code
 */
FrezzerResult readFrezzerStatus(FrezzerDevice* device);

/**
 * @brief Sets the target temperature on a connected Frezzer device
 * 
 * @param device Pointer to the device
 * @param tempCelsius Target temperature in Celsius (-25 to 20)
 * @return FrezzerResult SUCCESS or error code
 */
FrezzerResult setFrezzerTargetTemp(FrezzerDevice* device, float tempCelsius);

/**
 * @brief Sets the operating mode on a connected Frezzer device
 * 
 * @param device Pointer to the device
 * @param mode Desired operating mode
 * @return FrezzerResult SUCCESS or error code
 */
FrezzerResult setFrezzerMode(FrezzerDevice* device, FrezzerMode mode);

/**
 * @brief Turns the Frezzer device on
 * 
 * @param device Pointer to the device
 * @return FrezzerResult SUCCESS or error code
 */
FrezzerResult turnFrezzerOn(FrezzerDevice* device);

/**
 * @brief Turns the Frezzer device off
 * 
 * @param device Pointer to the device
 * @return FrezzerResult SUCCESS or error code
 */
FrezzerResult turnFrezzerOff(FrezzerDevice* device);

/**
 * @brief Registers a callback for status updates
 * 
 * @param callback Function to call when status updates are received
 */
void setFrezzerStatusCallback(FrezzerStatusCallback callback);

/**
 * @brief Registers a callback for connection state changes
 * 
 * @param callback Function to call when connection state changes
 */
void setFrezzerConnectionCallback(FrezzerConnectionCallback callback);

/**
 * @brief Process pending Frezzer operations
 * 
 * Should be called periodically from the main loop or a task
 * to handle connection maintenance, status polling, and notifications.
 * 
 * @param currentTime Current timestamp (millis)
 */
void processFrezzerTasks(unsigned long currentTime);

/**
 * @brief Gets all devices with fresh data
 * 
 * @param devices Array to store device pointers
 * @param maxDevices Maximum number of devices to return
 * @param currentTime Current timestamp (millis)
 * @return Number of devices with fresh data
 */
uint8_t getFreshFrezzerDevices(const FrezzerDevice** devices, uint8_t maxDevices,
                                unsigned long currentTime);

/**
 * @brief Checks if a device has recent data (not stale)
 * 
 * @param device Pointer to device
 * @param currentTime Current timestamp (millis)
 * @return true if device has recent data
 */
bool isFrezzerDataFresh(const FrezzerDevice* device, unsigned long currentTime);

/**
 * @brief MAC address utilities - convert bytes to string
 * 
 * @param mac MAC address bytes (6 bytes)
 * @param buffer Output buffer (at least 18 bytes)
 */
void frezzerMacToString(const uint8_t* mac, char* buffer);

/**
 * @brief MAC address utilities - convert string to bytes
 * 
 * @param macString MAC string in format "AA:BB:CC:DD:EE:FF"
 * @param mac Output array (6 bytes)
 * @return true if parsed successfully
 */
bool frezzerStringToMac(const char* macString, uint8_t* mac);

#ifdef UNIT_TEST
/**
 * @brief Resets internal state for testing
 */
void resetFrezzerControllerForTesting(void);

/**
 * @brief Gets internal devices array for testing
 */
FrezzerDevice* getInternalFrezzerDevicesForTesting(void);
#endif
