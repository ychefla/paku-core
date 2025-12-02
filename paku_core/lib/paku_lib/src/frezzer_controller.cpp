/**
 * @file frezzer_controller.cpp
 * @brief Frezzer PRO compressor fridge BLE GATT controller implementation
 */
#include "frezzer_controller.h"
#include "frezzer.h"
#include <cstring>
#include <cstdio>

// Internal storage for registered devices
static FrezzerDevice registeredDevices[MAX_FREZZER_DEVICES];
static uint8_t deviceCount = 0;
static bool initialized = false;

// Callbacks
static FrezzerStatusCallback statusCallback = nullptr;
static FrezzerConnectionCallback connectionCallback = nullptr;

// Last status poll time for each device
static unsigned long lastStatusPoll[MAX_FREZZER_DEVICES] = {0};

bool initFrezzerController(void) {
    clearFrezzerDevices();
    initialized = true;
    return true;
}

void clearFrezzerDevices(void) {
    memset(registeredDevices, 0, sizeof(registeredDevices));
    deviceCount = 0;
    for (uint8_t i = 0; i < MAX_FREZZER_DEVICES; i++) {
        lastStatusPoll[i] = 0;
    }
}

bool registerFrezzerDevice(const char* macAddress, const char* location) {
    if (macAddress == nullptr || location == nullptr) {
        return false;
    }
    
    if (deviceCount >= MAX_FREZZER_DEVICES) {
        return false;
    }
    
    // Check if already registered
    if (findFrezzerDeviceByMac(macAddress) != nullptr) {
        return false;  // Already registered
    }
    
    FrezzerDevice* device = &registeredDevices[deviceCount];
    
    // Parse MAC address
    if (!frezzerStringToMac(macAddress, device->macAddress)) {
        return false;
    }
    
    // Copy MAC string
    strncpy(device->macString, macAddress, sizeof(device->macString) - 1);
    device->macString[sizeof(device->macString) - 1] = '\0';
    
    // Copy location
    strncpy(device->location, location, sizeof(device->location) - 1);
    device->location[sizeof(device->location) - 1] = '\0';
    
    // Initialize state
    device->deviceName[0] = '\0';
    device->registered = true;
    device->hasData = false;
    device->connected = false;
    device->lastSeen = 0;
    memset(&device->lastData, 0, sizeof(device->lastData));
    device->lastData.mode = FrezzerMode::UNKNOWN;
    device->lastData.compressor = FrezzerCompressorState::UNKNOWN;
    device->lastData.error = FrezzerError::NONE;
    
    deviceCount++;
    return true;
}

uint8_t getFrezzerDeviceCount(void) {
    return deviceCount;
}

const FrezzerDevice* getFrezzerDevice(uint8_t index) {
    if (index >= deviceCount) {
        return nullptr;
    }
    return &registeredDevices[index];
}

const FrezzerDevice* findFrezzerDeviceByMac(const char* macAddress) {
    if (macAddress == nullptr) {
        return nullptr;
    }
    
    for (uint8_t i = 0; i < deviceCount; i++) {
        // Case-insensitive comparison
        bool match = true;
        const char* a = macAddress;
        const char* b = registeredDevices[i].macString;
        while (*a && *b) {
            char ca = (*a >= 'a' && *a <= 'z') ? (*a - 32) : *a;
            char cb = (*b >= 'a' && *b <= 'z') ? (*b - 32) : *b;
            if (ca != cb) {
                match = false;
                break;
            }
            a++;
            b++;
        }
        if (match && *a == *b) {
            return &registeredDevices[i];
        }
    }
    return nullptr;
}

bool updateFrezzerFromScan(const uint8_t* macAddress, const char* deviceName,
                            int rssi, unsigned long currentTime) {
    if (macAddress == nullptr) {
        return false;
    }
    
    // Check if device name matches Frezzer pattern
    if (deviceName != nullptr && !isFrezzerDevice(deviceName)) {
        return false;
    }
    
    // Convert MAC to string for lookup
    char macStr[18];
    frezzerMacToString(macAddress, macStr);
    
    // Find registered device
    FrezzerDevice* device = nullptr;
    for (uint8_t i = 0; i < deviceCount; i++) {
        if (strcmp(registeredDevices[i].macString, macStr) == 0) {
            device = &registeredDevices[i];
            break;
        }
    }
    
    // If not registered but we have space and it looks like a Frezzer, auto-register
    if (device == nullptr && deviceCount < MAX_FREZZER_DEVICES && 
        deviceName != nullptr && isFrezzerDevice(deviceName)) {
        device = &registeredDevices[deviceCount];
        memcpy(device->macAddress, macAddress, 6);
        strncpy(device->macString, macStr, sizeof(device->macString) - 1);
        device->macString[sizeof(device->macString) - 1] = '\0';
        snprintf(device->location, sizeof(device->location), "fridge_%d", deviceCount);
        device->registered = false;  // Mark as auto-discovered
        device->hasData = false;
        device->connected = false;
        deviceCount++;
    }
    
    if (device == nullptr) {
        return false;
    }
    
    // Update device name if provided
    if (deviceName != nullptr) {
        strncpy(device->deviceName, deviceName, sizeof(device->deviceName) - 1);
        device->deviceName[sizeof(device->deviceName) - 1] = '\0';
    }
    
    device->lastSeen = currentTime;
    
    return true;
}

// Note: The actual BLE GATT connection implementation would require
// the ESP32 BLE client API. These are placeholder implementations
// that would be replaced with actual BLE code when running on hardware.

FrezzerResult connectFrezzer(FrezzerDevice* device) {
    if (device == nullptr) {
        return FrezzerResult::INVALID_PARAM;
    }
    
    if (device->connected) {
        return FrezzerResult::ALREADY_CONNECTED;
    }
    
    // Placeholder: Actual implementation would use BLEClient to connect
    // BLEClient* pClient = BLEDevice::createClient();
    // pClient->connect(device->macAddress);
    // ... discover services and characteristics ...
    
    // For now, mark as connected (simulation for testing)
#ifdef UNIT_TEST
    device->connected = true;
    if (connectionCallback) {
        connectionCallback(device, true);
    }
    return FrezzerResult::SUCCESS;
#else
    // In real implementation, this would return success after connection
    return FrezzerResult::NOT_CONNECTED;
#endif
}

FrezzerResult disconnectFrezzer(FrezzerDevice* device) {
    if (device == nullptr) {
        return FrezzerResult::INVALID_PARAM;
    }
    
    if (!device->connected) {
        return FrezzerResult::NOT_CONNECTED;
    }
    
    // Placeholder: Actual implementation would disconnect the BLE client
    
    device->connected = false;
    
    if (connectionCallback) {
        connectionCallback(device, false);
    }
    
    return FrezzerResult::SUCCESS;
}

bool isFrezzerConnected(const FrezzerDevice* device) {
    if (device == nullptr) {
        return false;
    }
    return device->connected;
}

FrezzerResult readFrezzerStatus(FrezzerDevice* device) {
    if (device == nullptr) {
        return FrezzerResult::INVALID_PARAM;
    }
    
    if (!device->connected) {
        return FrezzerResult::NOT_CONNECTED;
    }
    
    // Placeholder: Actual implementation would read from BLE characteristic
    // pRemoteCharacteristic->readValue();
    // Parse the data and update device->lastData
    
#ifdef UNIT_TEST
    // Simulate reading status for testing
    device->lastData.valid = true;
    device->lastData.currentTemp = -5.0f;
    device->lastData.targetTemp = -8.0f;
    device->lastData.batteryVoltage = 12.4f;
    device->lastData.mode = FrezzerMode::FRIDGE;
    device->lastData.compressor = FrezzerCompressorState::RUNNING;
    device->lastData.error = FrezzerError::NONE;
    device->lastData.lidOpen = false;
    device->lastData.lowVoltageProtection = false;
    device->lastData.powerLevel = 50;
    device->hasData = true;
    
    if (statusCallback) {
        statusCallback(device, &device->lastData);
    }
    
    return FrezzerResult::SUCCESS;
#else
    return FrezzerResult::NOT_CONNECTED;
#endif
}

FrezzerResult setFrezzerTargetTemp(FrezzerDevice* device, float tempCelsius) {
    if (device == nullptr) {
        return FrezzerResult::INVALID_PARAM;
    }
    
    // Validate temperature range
    if (tempCelsius < -25.0f || tempCelsius > 20.0f) {
        return FrezzerResult::INVALID_PARAM;
    }
    
    if (!device->connected) {
        return FrezzerResult::NOT_CONNECTED;
    }
    
    // Build command
    uint8_t cmdBuffer[8];
    int16_t tempParam = (int16_t)(tempCelsius * 10);  // 0.1°C resolution
    size_t cmdLen = buildFrezzerCommand(FrezzerCommand::SET_TARGET_TEMP, 
                                         tempParam, cmdBuffer, sizeof(cmdBuffer));
    
    if (cmdLen == 0) {
        return FrezzerResult::UNKNOWN_ERROR;
    }
    
    // Placeholder: Actual implementation would write to BLE characteristic
    // pRemoteCharacteristic->writeValue(cmdBuffer, cmdLen);
    
#ifdef UNIT_TEST
    device->lastData.targetTemp = tempCelsius;
    return FrezzerResult::SUCCESS;
#else
    return FrezzerResult::NOT_CONNECTED;
#endif
}

FrezzerResult setFrezzerMode(FrezzerDevice* device, FrezzerMode mode) {
    if (device == nullptr) {
        return FrezzerResult::INVALID_PARAM;
    }
    
    if (mode == FrezzerMode::UNKNOWN) {
        return FrezzerResult::INVALID_PARAM;
    }
    
    if (!device->connected) {
        return FrezzerResult::NOT_CONNECTED;
    }
    
    // Build command
    uint8_t cmdBuffer[8];
    size_t cmdLen = buildFrezzerCommand(FrezzerCommand::SET_MODE,
                                         static_cast<int16_t>(mode),
                                         cmdBuffer, sizeof(cmdBuffer));
    
    if (cmdLen == 0) {
        return FrezzerResult::UNKNOWN_ERROR;
    }
    
#ifdef UNIT_TEST
    device->lastData.mode = mode;
    return FrezzerResult::SUCCESS;
#else
    return FrezzerResult::NOT_CONNECTED;
#endif
}

FrezzerResult turnFrezzerOn(FrezzerDevice* device) {
    if (device == nullptr) {
        return FrezzerResult::INVALID_PARAM;
    }
    
    if (!device->connected) {
        return FrezzerResult::NOT_CONNECTED;
    }
    
    uint8_t cmdBuffer[8];
    size_t cmdLen = buildFrezzerCommand(FrezzerCommand::POWER_ON, 0,
                                         cmdBuffer, sizeof(cmdBuffer));
    
    if (cmdLen == 0) {
        return FrezzerResult::UNKNOWN_ERROR;
    }
    
#ifdef UNIT_TEST
    device->lastData.mode = FrezzerMode::FRIDGE;
    return FrezzerResult::SUCCESS;
#else
    return FrezzerResult::NOT_CONNECTED;
#endif
}

FrezzerResult turnFrezzerOff(FrezzerDevice* device) {
    if (device == nullptr) {
        return FrezzerResult::INVALID_PARAM;
    }
    
    if (!device->connected) {
        return FrezzerResult::NOT_CONNECTED;
    }
    
    uint8_t cmdBuffer[8];
    size_t cmdLen = buildFrezzerCommand(FrezzerCommand::POWER_OFF, 0,
                                         cmdBuffer, sizeof(cmdBuffer));
    
    if (cmdLen == 0) {
        return FrezzerResult::UNKNOWN_ERROR;
    }
    
#ifdef UNIT_TEST
    device->lastData.mode = FrezzerMode::OFF;
    return FrezzerResult::SUCCESS;
#else
    return FrezzerResult::NOT_CONNECTED;
#endif
}

void setFrezzerStatusCallback(FrezzerStatusCallback callback) {
    statusCallback = callback;
}

void setFrezzerConnectionCallback(FrezzerConnectionCallback callback) {
    connectionCallback = callback;
}

void processFrezzerTasks(unsigned long currentTime) {
    // Poll status for connected devices at regular intervals
    for (uint8_t i = 0; i < deviceCount; i++) {
        FrezzerDevice* device = &registeredDevices[i];
        
        if (device->connected) {
            // Check if it's time to poll status
            if (currentTime - lastStatusPoll[i] >= FREZZER_STATUS_INTERVAL_MS) {
                readFrezzerStatus(device);
                lastStatusPoll[i] = currentTime;
            }
        }
    }
}

uint8_t getFreshFrezzerDevices(const FrezzerDevice** devices, uint8_t maxDevices,
                                unsigned long currentTime) {
    if (devices == nullptr || maxDevices == 0) {
        return 0;
    }
    
    uint8_t count = 0;
    for (uint8_t i = 0; i < deviceCount && count < maxDevices; i++) {
        if (isFrezzerDataFresh(&registeredDevices[i], currentTime)) {
            devices[count++] = &registeredDevices[i];
        }
    }
    
    return count;
}

bool isFrezzerDataFresh(const FrezzerDevice* device, unsigned long currentTime) {
    if (device == nullptr || !device->hasData) {
        return false;
    }
    
    // Handle potential overflow of millis()
    unsigned long elapsed;
    if (currentTime >= device->lastSeen) {
        elapsed = currentTime - device->lastSeen;
    } else {
        // Overflow occurred
        elapsed = (0xFFFFFFFF - device->lastSeen) + currentTime + 1;
    }
    
    return elapsed < FREZZER_STALE_TIMEOUT_MS;
}

void frezzerMacToString(const uint8_t* mac, char* buffer) {
    if (mac == nullptr || buffer == nullptr) {
        if (buffer != nullptr) {
            buffer[0] = '\0';
        }
        return;
    }
    
    snprintf(buffer, 18, "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

bool frezzerStringToMac(const char* macString, uint8_t* mac) {
    if (macString == nullptr || mac == nullptr) {
        return false;
    }
    
    unsigned int values[6];
    int parsed = sscanf(macString, "%02X:%02X:%02X:%02X:%02X:%02X",
                        &values[0], &values[1], &values[2],
                        &values[3], &values[4], &values[5]);
    
    if (parsed != 6) {
        // Try lowercase
        parsed = sscanf(macString, "%02x:%02x:%02x:%02x:%02x:%02x",
                        &values[0], &values[1], &values[2],
                        &values[3], &values[4], &values[5]);
    }
    
    if (parsed != 6) {
        return false;
    }
    
    for (int i = 0; i < 6; i++) {
        if (values[i] > 255) {
            return false;
        }
        mac[i] = static_cast<uint8_t>(values[i]);
    }
    
    return true;
}

#ifdef UNIT_TEST
void resetFrezzerControllerForTesting(void) {
    clearFrezzerDevices();
    statusCallback = nullptr;
    connectionCallback = nullptr;
    initialized = false;
}

FrezzerDevice* getInternalFrezzerDevicesForTesting(void) {
    return registeredDevices;
}
#endif
