/**
 * @file test_ruuvi_scanner.cpp
 * @brief Unit tests for Ruuvi scanner module
 */
#include <unity.h>

#ifndef UNIT_TEST
#define UNIT_TEST
#endif
#include "ruuvi_scanner.h"

void setUp(void) {
    resetRuuviScannerForTesting();
}

void tearDown(void) {
    resetRuuviScannerForTesting();
}

/**
 * Test initialization
 */
void test_initRuuviScanner(void) {
    bool result = initRuuviScanner();
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL_UINT8(0, getRegisteredTagCount());
}

/**
 * Test MAC string conversion
 */
void test_macToString(void) {
    uint8_t mac[6] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
    char buffer[18];
    
    macToString(mac, buffer);
    
    TEST_ASSERT_EQUAL_STRING("AA:BB:CC:DD:EE:FF", buffer);
}

/**
 * Test MAC string to bytes
 */
void test_stringToMac(void) {
    uint8_t mac[6];
    
    bool result = stringToMac("AA:BB:CC:DD:EE:FF", mac);
    
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL_UINT8(0xAA, mac[0]);
    TEST_ASSERT_EQUAL_UINT8(0xBB, mac[1]);
    TEST_ASSERT_EQUAL_UINT8(0xCC, mac[2]);
    TEST_ASSERT_EQUAL_UINT8(0xDD, mac[3]);
    TEST_ASSERT_EQUAL_UINT8(0xEE, mac[4]);
    TEST_ASSERT_EQUAL_UINT8(0xFF, mac[5]);
}

/**
 * Test MAC string lowercase
 */
void test_stringToMac_lowercase(void) {
    uint8_t mac[6];
    
    bool result = stringToMac("aa:bb:cc:dd:ee:ff", mac);
    
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL_UINT8(0xAA, mac[0]);
    TEST_ASSERT_EQUAL_UINT8(0xFF, mac[5]);
}

/**
 * Test invalid MAC string
 */
void test_stringToMac_invalid(void) {
    uint8_t mac[6];
    
    TEST_ASSERT_FALSE(stringToMac("invalid", mac));
    TEST_ASSERT_FALSE(stringToMac("AA:BB:CC", mac));
    TEST_ASSERT_FALSE(stringToMac(nullptr, mac));
}

/**
 * Test registering a tag
 */
void test_registerRuuviTag(void) {
    bool result = registerRuuviTag("AA:BB:CC:DD:EE:FF", "cabin");
    
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL_UINT8(1, getRegisteredTagCount());
    
    const RuuviTag* tag = getRegisteredTag(0);
    TEST_ASSERT_NOT_NULL(tag);
    TEST_ASSERT_EQUAL_STRING("AA:BB:CC:DD:EE:FF", tag->macString);
    TEST_ASSERT_EQUAL_STRING("cabin", tag->location);
    TEST_ASSERT_TRUE(tag->registered);
    TEST_ASSERT_FALSE(tag->hasData);
}

/**
 * Test registering multiple tags
 */
void test_registerRuuviTag_multiple(void) {
    TEST_ASSERT_TRUE(registerRuuviTag("AA:BB:CC:DD:EE:01", "cabin"));
    TEST_ASSERT_TRUE(registerRuuviTag("AA:BB:CC:DD:EE:02", "kitchen"));
    TEST_ASSERT_TRUE(registerRuuviTag("AA:BB:CC:DD:EE:03", "lounge"));
    
    TEST_ASSERT_EQUAL_UINT8(3, getRegisteredTagCount());
    
    TEST_ASSERT_EQUAL_STRING("cabin", getRegisteredTag(0)->location);
    TEST_ASSERT_EQUAL_STRING("kitchen", getRegisteredTag(1)->location);
    TEST_ASSERT_EQUAL_STRING("lounge", getRegisteredTag(2)->location);
}

/**
 * Test registering duplicate tag
 */
void test_registerRuuviTag_duplicate(void) {
    TEST_ASSERT_TRUE(registerRuuviTag("AA:BB:CC:DD:EE:FF", "cabin"));
    TEST_ASSERT_FALSE(registerRuuviTag("AA:BB:CC:DD:EE:FF", "kitchen"));
    
    TEST_ASSERT_EQUAL_UINT8(1, getRegisteredTagCount());
}

/**
 * Test finding tag by MAC
 */
void test_findRegisteredTagByMac(void) {
    registerRuuviTag("AA:BB:CC:DD:EE:01", "cabin");
    registerRuuviTag("AA:BB:CC:DD:EE:02", "kitchen");
    
    const RuuviTag* tag = findRegisteredTagByMac("AA:BB:CC:DD:EE:02");
    TEST_ASSERT_NOT_NULL(tag);
    TEST_ASSERT_EQUAL_STRING("kitchen", tag->location);
    
    // Case insensitive
    tag = findRegisteredTagByMac("aa:bb:cc:dd:ee:01");
    TEST_ASSERT_NOT_NULL(tag);
    TEST_ASSERT_EQUAL_STRING("cabin", tag->location);
}

/**
 * Test finding non-existent tag
 */
void test_findRegisteredTagByMac_notFound(void) {
    registerRuuviTag("AA:BB:CC:DD:EE:01", "cabin");
    
    const RuuviTag* tag = findRegisteredTagByMac("11:22:33:44:55:66");
    TEST_ASSERT_NULL(tag);
}

/**
 * Test updating tag data
 */
void test_updateRuuviTagData(void) {
    registerRuuviTag("AA:BB:CC:DD:EE:FF", "cabin");
    
    // Ruuvi RAWv2 test data
    uint8_t macAddr[6] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
    uint8_t ruuviData[] = {
        0x05,              // Format 5
        0x0A, 0xAA,        // Temperature: 13.65°C
        0x9A, 0x64,        // Humidity: 98.81%
        0xC3, 0x7A,        // Pressure: 100042 Pa
        0x00, 0x04,        // Accel X
        0xFF, 0xFC,        // Accel Y
        0x03, 0xE8,        // Accel Z
        0xB0, 0x34,        // Power info
        0x42,              // Movement
        0x00, 0xAB         // Sequence
    };
    
    bool result = updateRuuviTagData(macAddr, ruuviData, sizeof(ruuviData), 1000);
    TEST_ASSERT_TRUE(result);
    
    const RuuviTag* tag = getRegisteredTag(0);
    TEST_ASSERT_TRUE(tag->hasData);
    TEST_ASSERT_EQUAL_UINT32(1000, tag->lastSeen);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 13.65f, tag->lastData.temperature);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 98.81f, tag->lastData.humidity);
}

/**
 * Test auto-discovery of unknown tags
 */
void test_updateRuuviTagData_autoDiscover(void) {
    // No tags registered yet
    TEST_ASSERT_EQUAL_UINT8(0, getRegisteredTagCount());
    
    uint8_t macAddr[6] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66};
    uint8_t ruuviData[] = {
        0x05, 0x0A, 0xAA, 0x9A, 0x64, 0xC3, 0x7A,
        0x00, 0x04, 0xFF, 0xFC, 0x03, 0xE8,
        0xB0, 0x34, 0x42, 0x00, 0xAB
    };
    
    bool result = updateRuuviTagData(macAddr, ruuviData, sizeof(ruuviData), 1000);
    TEST_ASSERT_TRUE(result);
    
    // Should have auto-registered
    TEST_ASSERT_EQUAL_UINT8(1, getRegisteredTagCount());
    
    const RuuviTag* tag = getRegisteredTag(0);
    TEST_ASSERT_EQUAL_STRING("11:22:33:44:55:66", tag->macString);
    TEST_ASSERT_FALSE(tag->registered);  // Not explicitly registered
    TEST_ASSERT_TRUE(tag->hasData);
}

/**
 * Test stale data detection
 */
void test_isTagDataFresh(void) {
    registerRuuviTag("AA:BB:CC:DD:EE:FF", "cabin");
    
    uint8_t macAddr[6] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
    uint8_t ruuviData[] = {
        0x05, 0x0A, 0xAA, 0x9A, 0x64, 0xC3, 0x7A,
        0x00, 0x04, 0xFF, 0xFC, 0x03, 0xE8,
        0xB0, 0x34, 0x42, 0x00, 0xAB
    };
    
    updateRuuviTagData(macAddr, ruuviData, sizeof(ruuviData), 1000);
    
    const RuuviTag* tag = getRegisteredTag(0);
    
    // Fresh at 2000 (1 second later)
    TEST_ASSERT_TRUE(isTagDataFresh(tag, 2000));
    
    // Stale at 1000 + RUUVI_STALE_TIMEOUT_MS + 1
    TEST_ASSERT_FALSE(isTagDataFresh(tag, 1000 + RUUVI_STALE_TIMEOUT_MS + 1));
}

/**
 * Test getting fresh tags
 */
void test_getFreshTags(void) {
    registerRuuviTag("AA:BB:CC:DD:EE:01", "cabin");
    registerRuuviTag("AA:BB:CC:DD:EE:02", "kitchen");
    registerRuuviTag("AA:BB:CC:DD:EE:03", "lounge");
    
    uint8_t ruuviData[] = {
        0x05, 0x0A, 0xAA, 0x9A, 0x64, 0xC3, 0x7A,
        0x00, 0x04, 0xFF, 0xFC, 0x03, 0xE8,
        0xB0, 0x34, 0x42, 0x00, 0xAB
    };
    
    // Update first and third tags
    uint8_t mac1[6] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x01};
    uint8_t mac3[6] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x03};
    
    updateRuuviTagData(mac1, ruuviData, sizeof(ruuviData), 1000);
    updateRuuviTagData(mac3, ruuviData, sizeof(ruuviData), 1000);
    
    const RuuviTag* freshTags[8];
    uint8_t count = getFreshTags(freshTags, 8, 2000);
    
    TEST_ASSERT_EQUAL_UINT8(2, count);
}

/**
 * Test Ruuvi manufacturer ID check
 */
void test_isRuuviManufacturer(void) {
    TEST_ASSERT_TRUE(isRuuviManufacturer(0x0499));
    TEST_ASSERT_FALSE(isRuuviManufacturer(0x0000));
    TEST_ASSERT_FALSE(isRuuviManufacturer(0x004C));  // Apple
}

/**
 * Test download history placeholder
 */
void test_downloadTagHistory_placeholder(void) {
    registerRuuviTag("AA:BB:CC:DD:EE:FF", "cabin");
    const RuuviTag* tag = getRegisteredTag(0);
    
    RuuviHistoryEntry entries[10];
    uint8_t count = downloadTagHistory(tag, entries, 10);
    
    // Should return 0 as it's a placeholder
    TEST_ASSERT_EQUAL_UINT8(0, count);
}

/**
 * Test clearing tags
 */
void test_clearRegisteredTags(void) {
    registerRuuviTag("AA:BB:CC:DD:EE:01", "cabin");
    registerRuuviTag("AA:BB:CC:DD:EE:02", "kitchen");
    
    TEST_ASSERT_EQUAL_UINT8(2, getRegisteredTagCount());
    
    clearRegisteredTags();
    
    TEST_ASSERT_EQUAL_UINT8(0, getRegisteredTagCount());
}

/**
 * Test max tags limit
 */
void test_registerRuuviTag_maxLimit(void) {
    // Register max number of tags
    for (int i = 0; i < MAX_RUUVI_TAGS; i++) {
        char mac[18];
        snprintf(mac, sizeof(mac), "AA:BB:CC:DD:EE:%02X", i);
        char location[32];
        snprintf(location, sizeof(location), "loc_%d", i);
        
        bool result = registerRuuviTag(mac, location);
        TEST_ASSERT_TRUE(result);
    }
    
    TEST_ASSERT_EQUAL_UINT8(MAX_RUUVI_TAGS, getRegisteredTagCount());
    
    // Try to register one more
    bool result = registerRuuviTag("FF:FF:FF:FF:FF:FF", "extra");
    TEST_ASSERT_FALSE(result);
    TEST_ASSERT_EQUAL_UINT8(MAX_RUUVI_TAGS, getRegisteredTagCount());
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    
    // Initialization tests
    RUN_TEST(test_initRuuviScanner);
    
    // MAC conversion tests
    RUN_TEST(test_macToString);
    RUN_TEST(test_stringToMac);
    RUN_TEST(test_stringToMac_lowercase);
    RUN_TEST(test_stringToMac_invalid);
    
    // Registration tests
    RUN_TEST(test_registerRuuviTag);
    RUN_TEST(test_registerRuuviTag_multiple);
    RUN_TEST(test_registerRuuviTag_duplicate);
    RUN_TEST(test_registerRuuviTag_maxLimit);
    RUN_TEST(test_clearRegisteredTags);
    
    // Find tests
    RUN_TEST(test_findRegisteredTagByMac);
    RUN_TEST(test_findRegisteredTagByMac_notFound);
    
    // Data update tests
    RUN_TEST(test_updateRuuviTagData);
    RUN_TEST(test_updateRuuviTagData_autoDiscover);
    
    // Freshness tests
    RUN_TEST(test_isTagDataFresh);
    RUN_TEST(test_getFreshTags);
    
    // Utility tests
    RUN_TEST(test_isRuuviManufacturer);
    RUN_TEST(test_downloadTagHistory_placeholder);
    
    return UNITY_END();
}
