/**
 * @file ruuvi_scanner.cpp
 * @brief BLE scanner for RuuviTag data collection implementation
 */
#include "ruuvi_scanner.h"
#include "ruuvi.h"
#include <cstring>
#include <cstdio>

// Internal storage for registered tags
static RuuviTag registeredTags[MAX_RUUVI_TAGS];
static uint8_t tagCount = 0;
static bool initialized = false;
static RuuviWhitelistMode whitelistMode = RuuviWhitelistMode::AUTO_DISCOVER;

bool initRuuviScanner(void) {
    clearRegisteredTags();
    initialized = true;
    return true;
}

void clearRegisteredTags(void) {
    memset(registeredTags, 0, sizeof(registeredTags));
    tagCount = 0;
}

void setWhitelistMode(RuuviWhitelistMode mode) {
    whitelistMode = mode;
}

RuuviWhitelistMode getWhitelistMode(void) {
    return whitelistMode;
}

bool removeRuuviTag(const char* macAddress) {
    if (macAddress == nullptr) {
        return false;
    }

    for (uint8_t i = 0; i < tagCount; i++) {
        // Case-insensitive MAC comparison (reuse existing pattern)
        bool match = true;
        const char* a = macAddress;
        const char* b = registeredTags[i].macString;
        while (*a && *b) {
            char ca = (*a >= 'a' && *a <= 'z') ? (*a - 32) : *a;
            char cb = (*b >= 'a' && *b <= 'z') ? (*b - 32) : *b;
            if (ca != cb) { match = false; break; }
            a++; b++;
        }
        if (match && *a == *b) {
            // Shift remaining tags down
            for (uint8_t j = i; j < tagCount - 1; j++) {
                registeredTags[j] = registeredTags[j + 1];
            }
            memset(&registeredTags[tagCount - 1], 0, sizeof(RuuviTag));
            tagCount--;
            return true;
        }
    }
    return false;
}

bool registerRuuviTag(const char* macAddress, const char* location) {
    if (macAddress == nullptr || location == nullptr) {
        return false;
    }
    
    if (tagCount >= MAX_RUUVI_TAGS) {
        return false;
    }
    
    // Check if already registered
    if (findRegisteredTagByMac(macAddress) != nullptr) {
        return false;  // Already registered
    }
    
    RuuviTag* tag = &registeredTags[tagCount];
    
    // Parse MAC address
    if (!stringToMac(macAddress, tag->macAddress)) {
        return false;
    }
    
    // Copy MAC string
    strncpy(tag->macString, macAddress, sizeof(tag->macString) - 1);
    tag->macString[sizeof(tag->macString) - 1] = '\0';
    
    // Copy location
    strncpy(tag->location, location, sizeof(tag->location) - 1);
    tag->location[sizeof(tag->location) - 1] = '\0';
    
    // Initialize state
    tag->registered = true;
    tag->hasData = false;
    tag->lastSeen = 0;
    memset(&tag->lastData, 0, sizeof(tag->lastData));
    
    tagCount++;
    return true;
}

uint8_t getRegisteredTagCount(void) {
    return tagCount;
}

const RuuviTag* getRegisteredTag(uint8_t index) {
    if (index >= tagCount) {
        return nullptr;
    }
    return &registeredTags[index];
}

const RuuviTag* findRegisteredTagByMac(const char* macAddress) {
    if (macAddress == nullptr) {
        return nullptr;
    }
    
    for (uint8_t i = 0; i < tagCount; i++) {
        // Case-insensitive comparison
        bool match = true;
        const char* a = macAddress;
        const char* b = registeredTags[i].macString;
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
        if (match && *a == *b) {  // Both reached end
            return &registeredTags[i];
        }
    }
    return nullptr;
}

void macToString(const uint8_t* mac, char* buffer) {
    if (mac == nullptr || buffer == nullptr) {
        if (buffer != nullptr) {
            buffer[0] = '\0';
        }
        return;
    }
    
    snprintf(buffer, 18, "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

bool stringToMac(const char* macString, uint8_t* mac) {
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

bool updateRuuviTagData(const uint8_t* macAddress, const uint8_t* data, 
                        size_t length, unsigned long currentTime) {
    if (macAddress == nullptr || data == nullptr || length == 0) {
        return false;
    }
    
    // Convert MAC to string for lookup
    char macStr[18];
    macToString(macAddress, macStr);
    
    // Find registered tag
    RuuviTag* tag = nullptr;
    for (uint8_t i = 0; i < tagCount; i++) {
        if (strcmp(registeredTags[i].macString, macStr) == 0) {
            tag = &registeredTags[i];
            break;
        }
    }
    
    // If not registered, behavior depends on whitelist mode:
    // - WHITELIST mode: reject unknown tags
    // - AUTO_DISCOVER mode: auto-register if space available
    if (tag == nullptr) {
        if (whitelistMode == RuuviWhitelistMode::WHITELIST) {
            return false;  // Tag not whitelisted, ignore
        }
        if (tagCount < MAX_RUUVI_TAGS) {
            tag = &registeredTags[tagCount];
            memcpy(tag->macAddress, macAddress, 6);
            strncpy(tag->macString, macStr, sizeof(tag->macString) - 1);
            tag->macString[sizeof(tag->macString) - 1] = '\0';
            snprintf(tag->location, sizeof(tag->location), "tag_%d", tagCount);
            tag->registered = false;  // Mark as auto-discovered
            tag->hasData = false;
            tagCount++;
        }
    }
    
    if (tag == nullptr) {
        return false;  // No space for new tag
    }
    
    // Parse Ruuvi data
    RuuviData parsed = parseRuuviDataV5(data, length);
    if (!parsed.valid) {
        return false;
    }
    
    // Update tag
    tag->lastData = parsed;
    tag->lastSeen = currentTime;
    tag->hasData = true;
    
    return true;
}

bool isTagDataFresh(const RuuviTag* tag, unsigned long currentTime) {
    if (tag == nullptr || !tag->hasData) {
        return false;
    }
    
    // Handle potential overflow of millis()
    unsigned long elapsed;
    if (currentTime >= tag->lastSeen) {
        elapsed = currentTime - tag->lastSeen;
    } else {
        // Overflow occurred
        elapsed = (0xFFFFFFFF - tag->lastSeen) + currentTime + 1;
    }
    
    return elapsed < RUUVI_STALE_TIMEOUT_MS;
}

uint8_t getFreshTags(const RuuviTag** tags, uint8_t maxTags, unsigned long currentTime) {
    if (tags == nullptr || maxTags == 0) {
        return 0;
    }
    
    uint8_t count = 0;
    for (uint8_t i = 0; i < tagCount && count < maxTags; i++) {
        if (isTagDataFresh(&registeredTags[i], currentTime)) {
            tags[count++] = &registeredTags[i];
        }
    }
    
    return count;
}

uint8_t downloadTagHistory(const RuuviTag* tag, RuuviHistoryEntry* entries, 
                           uint8_t maxEntries) {
    // Placeholder implementation
    // RuuviTags broadcast data via BLE advertisements, they don't support
    // direct memory download in standard configurations.
    // This function is a placeholder for future GATT-based implementations
    // or for compatible devices that support history download.
    
    (void)tag;
    (void)entries;
    (void)maxEntries;
    
    // Return 0 to indicate no history data available
    // Future implementation could:
    // 1. Connect to tag via GATT
    // 2. Request history data from tag memory
    // 3. Parse and return historical entries
    return 0;
}

bool isRuuviManufacturer(uint16_t manufacturerId) {
    return manufacturerId == RUUVI_MANUFACTURER_ID;
}

#ifdef UNIT_TEST
void resetRuuviScannerForTesting(void) {
    clearRegisteredTags();
    initialized = false;
}

RuuviTag* getInternalTagsForTesting(void) {
    return registeredTags;
}
#endif
