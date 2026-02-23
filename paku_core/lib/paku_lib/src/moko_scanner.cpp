/**
 * @file moko_scanner.cpp
 * @brief BLE scanner for MoKo sensor data collection implementation
 */
#include "moko_scanner.h"
#include "moko.h"
#include <cstring>
#include <cstdio>

// Internal storage for registered sensors
static MoKoSensor registeredSensors[MAX_MOKO_SENSORS];
static uint8_t sensorCount = 0;
static bool initialized = false;

bool initMoKoScanner(void) {
    clearRegisteredSensors();
    initialized = true;
    return true;
}

void clearRegisteredSensors(void) {
    memset(registeredSensors, 0, sizeof(registeredSensors));
    sensorCount = 0;
}

bool registerMoKoSensor(const char* macAddress, const char* location) {
    if (!initialized) {
        initMoKoScanner();
    }
    
    if (sensorCount >= MAX_MOKO_SENSORS) {
        return false;  // Registry full
    }
    
    if (macAddress == nullptr || location == nullptr) {
        return false;
    }
    
    // Check if already registered
    for (uint8_t i = 0; i < sensorCount; i++) {
        if (strcmp(registeredSensors[i].macString, macAddress) == 0) {
            return false;  // Already registered
        }
    }
    
    // Parse MAC address
    uint8_t mac[6];
    if (!mokoStringToMac(macAddress, mac)) {
        return false;  // Invalid MAC format
    }
    
    // Add new sensor
    MoKoSensor* sensor = &registeredSensors[sensorCount];
    memcpy(sensor->macAddress, mac, 6);
    strncpy(sensor->macString, macAddress, sizeof(sensor->macString) - 1);
    sensor->macString[sizeof(sensor->macString) - 1] = '\0';
    strncpy(sensor->location, location, sizeof(sensor->location) - 1);
    sensor->location[sizeof(sensor->location) - 1] = '\0';
    sensor->registered = true;
    sensor->hasData = false;
    sensor->lastSeen = 0;
    sensor->model = 0;  // Unknown initially
    
    sensorCount++;
    return true;
}

uint8_t getRegisteredSensorCount(void) {
    return sensorCount;
}

const MoKoSensor* getRegisteredSensor(uint8_t index) {
    if (index >= sensorCount) {
        return nullptr;
    }
    return &registeredSensors[index];
}

const MoKoSensor* findRegisteredSensorByMac(const char* macAddress) {
    if (macAddress == nullptr) {
        return nullptr;
    }
    
    for (uint8_t i = 0; i < sensorCount; i++) {
        if (strcmp(registeredSensors[i].macString, macAddress) == 0) {
            return &registeredSensors[i];
        }
    }
    
    return nullptr;
}

void mokoMacToString(const uint8_t* mac, char* buffer) {
    if (mac == nullptr || buffer == nullptr) {
        return;
    }
    
    snprintf(buffer, 18, "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

bool mokoStringToMac(const char* macString, uint8_t* mac) {
    if (macString == nullptr || mac == nullptr) {
        return false;
    }
    
    // Parse MAC address string (format: "AA:BB:CC:DD:EE:FF" or "aa:bb:cc:dd:ee:ff")
    int values[6];
    int parsed = sscanf(macString, "%02x:%02x:%02x:%02x:%02x:%02x",
                        &values[0], &values[1], &values[2],
                        &values[3], &values[4], &values[5]);
    
    if (parsed != 6) {
        return false;
    }
    
    for (int i = 0; i < 6; i++) {
        mac[i] = static_cast<uint8_t>(values[i]);
    }
    
    return true;
}

bool updateMoKoSensorData(const uint8_t* macAddress, const uint8_t* data, 
                          size_t length, unsigned long currentTime) {
    if (!initialized) {
        initMoKoScanner();
    }
    
    if (macAddress == nullptr || data == nullptr) {
        return false;
    }
    
    // Convert MAC to string for lookup
    char macString[18];
    mokoMacToString(macAddress, macString);
    
    // Find or create sensor entry
    MoKoSensor* sensor = nullptr;
    
    // First, try to find existing registered sensor
    for (uint8_t i = 0; i < sensorCount; i++) {
        if (memcmp(registeredSensors[i].macAddress, macAddress, 6) == 0) {
            sensor = &registeredSensors[i];
            break;
        }
    }
    
    // If not found and auto-discovery is enabled, add new sensor
    if (sensor == nullptr && sensorCount < MAX_MOKO_SENSORS) {
        sensor = &registeredSensors[sensorCount];
        memcpy(sensor->macAddress, macAddress, 6);
        mokoMacToString(macAddress, sensor->macString);
        snprintf(sensor->location, sizeof(sensor->location), "auto_%02X%02X",
                 macAddress[4], macAddress[5]);
        sensor->registered = false;  // Auto-discovered, not pre-registered
        sensorCount++;
    }
    
    if (sensor == nullptr) {
        return false;  // No space for new sensor
    }
    
    // Parse MoKo data
    MoKoData parsedData = parseMoKoData(data, length);
    
    if (!parsedData.valid) {
        return false;
    }
    
    // Update sensor data
    sensor->lastData = parsedData;
    sensor->lastSeen = currentTime;
    sensor->hasData = true;
    sensor->model = detectMoKoModel(data, length);
    
    return true;
}

bool isSensorDataFresh(const MoKoSensor* sensor, unsigned long currentTime) {
    if (sensor == nullptr || !sensor->hasData) {
        return false;
    }
    
    unsigned long timeSinceLastSeen = currentTime - sensor->lastSeen;
    return timeSinceLastSeen < MOKO_STALE_TIMEOUT_MS;
}

uint8_t getFreshSensors(const MoKoSensor** sensors, uint8_t maxSensors, unsigned long currentTime) {
    if (sensors == nullptr) {
        return 0;
    }
    
    uint8_t count = 0;
    for (uint8_t i = 0; i < sensorCount && count < maxSensors; i++) {
        if (isSensorDataFresh(&registeredSensors[i], currentTime)) {
            sensors[count++] = &registeredSensors[i];
        }
    }
    
    return count;
}

bool isMoKoManufacturer(uint16_t manufacturerId) {
    return manufacturerId == MOKO_MANUFACTURER_ID_PRIMARY || 
           manufacturerId == MOKO_MANUFACTURER_ID_SECONDARY;
}

bool isMoKoDeviceName(const char* deviceName) {
    if (deviceName == nullptr) {
        return false;
    }
    
    // Check for common MoKo naming patterns
    // MK_H2, MK_H3, MK_H4, MKiBeacon, MOKO, etc.
    return (strstr(deviceName, "MK_") != nullptr ||
            strstr(deviceName, "MKiBeacon") != nullptr ||
            strstr(deviceName, "MOKO") != nullptr ||
            strstr(deviceName, "Moko") != nullptr);
}

#ifdef UNIT_TEST
void resetMoKoScannerForTesting(void) {
    clearRegisteredSensors();
    initialized = false;
}

MoKoSensor* getInternalSensorsForTesting(void) {
    return registeredSensors;
}
#endif
