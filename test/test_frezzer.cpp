/**
 * @file test_frezzer.cpp
 * @brief Unit tests for Frezzer module
 */
#include <unity.h>
#include "frezzer.h"
#include "frezzer_controller.h"

void setUp(void) {
    resetFrezzerControllerForTesting();
}

void tearDown(void) {
    resetFrezzerControllerForTesting();
}

/**
 * Test initialization
 */
void test_initFrezzerController(void) {
    bool result = initFrezzerController();
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL_UINT8(0, getFrezzerDeviceCount());
}

/**
 * Test MAC string conversion to bytes
 */
void test_frezzerStringToMac(void) {
    uint8_t mac[6];
    
    bool result = frezzerStringToMac("AA:BB:CC:DD:EE:FF", mac);
    
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL_UINT8(0xAA, mac[0]);
    TEST_ASSERT_EQUAL_UINT8(0xBB, mac[1]);
    TEST_ASSERT_EQUAL_UINT8(0xCC, mac[2]);
    TEST_ASSERT_EQUAL_UINT8(0xDD, mac[3]);
    TEST_ASSERT_EQUAL_UINT8(0xEE, mac[4]);
    TEST_ASSERT_EQUAL_UINT8(0xFF, mac[5]);
}

/**
 * Test MAC bytes to string conversion
 */
void test_frezzerMacToString(void) {
    uint8_t mac[6] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
    char buffer[18];
    
    frezzerMacToString(mac, buffer);
    
    TEST_ASSERT_EQUAL_STRING("AA:BB:CC:DD:EE:FF", buffer);
}

/**
 * Test MAC string lowercase
 */
void test_frezzerStringToMac_lowercase(void) {
    uint8_t mac[6];
    
    bool result = frezzerStringToMac("aa:bb:cc:dd:ee:ff", mac);
    
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL_UINT8(0xAA, mac[0]);
    TEST_ASSERT_EQUAL_UINT8(0xFF, mac[5]);
}

/**
 * Test invalid MAC string
 */
void test_frezzerStringToMac_invalid(void) {
    uint8_t mac[6];
    
    TEST_ASSERT_FALSE(frezzerStringToMac("invalid", mac));
    TEST_ASSERT_FALSE(frezzerStringToMac("AA:BB:CC", mac));
    TEST_ASSERT_FALSE(frezzerStringToMac(nullptr, mac));
}

/**
 * Test registering a Frezzer device
 */
void test_registerFrezzerDevice(void) {
    bool result = registerFrezzerDevice("AA:BB:CC:DD:EE:FF", "van_fridge");
    
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL_UINT8(1, getFrezzerDeviceCount());
    
    const FrezzerDevice* device = getFrezzerDevice(0);
    TEST_ASSERT_NOT_NULL(device);
    TEST_ASSERT_EQUAL_STRING("AA:BB:CC:DD:EE:FF", device->macString);
    TEST_ASSERT_EQUAL_STRING("van_fridge", device->location);
    TEST_ASSERT_TRUE(device->registered);
    TEST_ASSERT_FALSE(device->hasData);
    TEST_ASSERT_FALSE(device->connected);
}

/**
 * Test registering multiple devices
 */
void test_registerFrezzerDevice_multiple(void) {
    TEST_ASSERT_TRUE(registerFrezzerDevice("AA:BB:CC:DD:EE:01", "fridge_1"));
    TEST_ASSERT_TRUE(registerFrezzerDevice("AA:BB:CC:DD:EE:02", "fridge_2"));
    
    TEST_ASSERT_EQUAL_UINT8(2, getFrezzerDeviceCount());
    
    TEST_ASSERT_EQUAL_STRING("fridge_1", getFrezzerDevice(0)->location);
    TEST_ASSERT_EQUAL_STRING("fridge_2", getFrezzerDevice(1)->location);
}

/**
 * Test registering duplicate device
 */
void test_registerFrezzerDevice_duplicate(void) {
    TEST_ASSERT_TRUE(registerFrezzerDevice("AA:BB:CC:DD:EE:FF", "fridge_1"));
    TEST_ASSERT_FALSE(registerFrezzerDevice("AA:BB:CC:DD:EE:FF", "fridge_2"));
    
    TEST_ASSERT_EQUAL_UINT8(1, getFrezzerDeviceCount());
}

/**
 * Test finding device by MAC
 */
void test_findFrezzerDeviceByMac(void) {
    registerFrezzerDevice("AA:BB:CC:DD:EE:01", "fridge_1");
    registerFrezzerDevice("AA:BB:CC:DD:EE:02", "fridge_2");
    
    const FrezzerDevice* device = findFrezzerDeviceByMac("AA:BB:CC:DD:EE:02");
    TEST_ASSERT_NOT_NULL(device);
    TEST_ASSERT_EQUAL_STRING("fridge_2", device->location);
    
    // Case insensitive
    device = findFrezzerDeviceByMac("aa:bb:cc:dd:ee:01");
    TEST_ASSERT_NOT_NULL(device);
    TEST_ASSERT_EQUAL_STRING("fridge_1", device->location);
}

/**
 * Test finding non-existent device
 */
void test_findFrezzerDeviceByMac_notFound(void) {
    registerFrezzerDevice("AA:BB:CC:DD:EE:01", "fridge_1");
    
    const FrezzerDevice* device = findFrezzerDeviceByMac("11:22:33:44:55:66");
    TEST_ASSERT_NULL(device);
}

/**
 * Test isFrezzerDevice name matching
 */
void test_isFrezzerDevice(void) {
    TEST_ASSERT_TRUE(isFrezzerDevice("FREZZER"));
    TEST_ASSERT_TRUE(isFrezzerDevice("FREZZER PRO 65L"));
    TEST_ASSERT_TRUE(isFrezzerDevice("frezzer"));
    TEST_ASSERT_TRUE(isFrezzerDevice("Frezzer_123"));
    
    TEST_ASSERT_FALSE(isFrezzerDevice("DOMETIC"));
    TEST_ASSERT_FALSE(isFrezzerDevice("WAECO"));
    TEST_ASSERT_FALSE(isFrezzerDevice("RuuviTag"));
    TEST_ASSERT_FALSE(isFrezzerDevice(""));
    TEST_ASSERT_FALSE(isFrezzerDevice(nullptr));
}

/**
 * Test Frezzer mode to string conversion
 */
void test_frezzerModeToString(void) {
    TEST_ASSERT_EQUAL_STRING("off", frezzerModeToString(FrezzerMode::OFF));
    TEST_ASSERT_EQUAL_STRING("fridge", frezzerModeToString(FrezzerMode::FRIDGE));
    TEST_ASSERT_EQUAL_STRING("freezer", frezzerModeToString(FrezzerMode::FREEZER));
    TEST_ASSERT_EQUAL_STRING("eco", frezzerModeToString(FrezzerMode::ECO));
    TEST_ASSERT_EQUAL_STRING("max_cool", frezzerModeToString(FrezzerMode::MAX_COOL));
    TEST_ASSERT_EQUAL_STRING("unknown", frezzerModeToString(FrezzerMode::UNKNOWN));
}

/**
 * Test Frezzer compressor state to string
 */
void test_frezzerCompressorStateToString(void) {
    TEST_ASSERT_EQUAL_STRING("off", frezzerCompressorStateToString(FrezzerCompressorState::OFF));
    TEST_ASSERT_EQUAL_STRING("running", frezzerCompressorStateToString(FrezzerCompressorState::RUNNING));
    TEST_ASSERT_EQUAL_STRING("standby", frezzerCompressorStateToString(FrezzerCompressorState::STANDBY));
    TEST_ASSERT_EQUAL_STRING("error", frezzerCompressorStateToString(FrezzerCompressorState::ERROR));
    TEST_ASSERT_EQUAL_STRING("unknown", frezzerCompressorStateToString(FrezzerCompressorState::UNKNOWN));
}

/**
 * Test Frezzer error to string
 */
void test_frezzerErrorToString(void) {
    TEST_ASSERT_EQUAL_STRING("none", frezzerErrorToString(FrezzerError::NONE));
    TEST_ASSERT_EQUAL_STRING("temp_sensor_error", frezzerErrorToString(FrezzerError::TEMP_SENSOR));
    TEST_ASSERT_EQUAL_STRING("compressor_error", frezzerErrorToString(FrezzerError::COMPRESSOR));
    TEST_ASSERT_EQUAL_STRING("low_voltage", frezzerErrorToString(FrezzerError::LOW_VOLTAGE));
    TEST_ASSERT_EQUAL_STRING("overtemperature", frezzerErrorToString(FrezzerError::OVERTEMP));
}

/**
 * Test Frezzer result to string
 */
void test_frezzerResultToString(void) {
    TEST_ASSERT_EQUAL_STRING("success", frezzerResultToString(FrezzerResult::SUCCESS));
    TEST_ASSERT_EQUAL_STRING("not connected", frezzerResultToString(FrezzerResult::NOT_CONNECTED));
    TEST_ASSERT_EQUAL_STRING("connection failed", frezzerResultToString(FrezzerResult::CONNECT_FAILED));
    TEST_ASSERT_EQUAL_STRING("timeout", frezzerResultToString(FrezzerResult::TIMEOUT));
    TEST_ASSERT_EQUAL_STRING("invalid parameter", frezzerResultToString(FrezzerResult::INVALID_PARAM));
}

/**
 * Test building Frezzer command
 */
void test_buildFrezzerCommand(void) {
    uint8_t buffer[8];
    
    // Set target temp to -8.0°C (param = -80 in 0.1°C units)
    size_t len = buildFrezzerCommand(FrezzerCommand::SET_TARGET_TEMP, -80, buffer, sizeof(buffer));
    
    TEST_ASSERT_EQUAL_UINT(4, len);
    TEST_ASSERT_EQUAL_UINT8(0xFE, buffer[0]);  // Header
    TEST_ASSERT_EQUAL_UINT8(0x01, buffer[1]);  // SET_TARGET_TEMP
    TEST_ASSERT_EQUAL_UINT8(0xB0, buffer[2]);  // -80 low byte (0xFFB0 & 0xFF)
    TEST_ASSERT_EQUAL_UINT8(0xFF, buffer[3]);  // -80 high byte
}

/**
 * Test building command with null buffer
 */
void test_buildFrezzerCommand_nullBuffer(void) {
    size_t len = buildFrezzerCommand(FrezzerCommand::SET_TARGET_TEMP, -80, nullptr, 8);
    TEST_ASSERT_EQUAL_UINT(0, len);
}

/**
 * Test building command with small buffer
 */
void test_buildFrezzerCommand_smallBuffer(void) {
    uint8_t buffer[2];
    size_t len = buildFrezzerCommand(FrezzerCommand::SET_TARGET_TEMP, -80, buffer, sizeof(buffer));
    TEST_ASSERT_EQUAL_UINT(0, len);
}

/**
 * Test data freshness check
 */
void test_isFrezzerDataFresh(void) {
    registerFrezzerDevice("AA:BB:CC:DD:EE:FF", "fridge");
    
    // Get the device and manually set data
    FrezzerDevice* devices = getInternalFrezzerDevicesForTesting();
    devices[0].hasData = true;
    devices[0].lastSeen = 1000;
    
    // Fresh at 2000 (1 second later)
    TEST_ASSERT_TRUE(isFrezzerDataFresh(&devices[0], 2000));
    
    // Stale at 1000 + FREZZER_STALE_TIMEOUT_MS + 1
    TEST_ASSERT_FALSE(isFrezzerDataFresh(&devices[0], 1000 + FREZZER_STALE_TIMEOUT_MS + 1));
}

/**
 * Test getting fresh devices
 */
void test_getFreshFrezzerDevices(void) {
    registerFrezzerDevice("AA:BB:CC:DD:EE:01", "fridge_1");
    registerFrezzerDevice("AA:BB:CC:DD:EE:02", "fridge_2");
    
    FrezzerDevice* devices = getInternalFrezzerDevicesForTesting();
    
    // Set data for first device
    devices[0].hasData = true;
    devices[0].lastSeen = 1000;
    
    // Second device has no data
    devices[1].hasData = false;
    
    const FrezzerDevice* freshDevices[4];
    uint8_t count = getFreshFrezzerDevices(freshDevices, 4, 2000);
    
    TEST_ASSERT_EQUAL_UINT8(1, count);
    TEST_ASSERT_EQUAL_STRING("fridge_1", freshDevices[0]->location);
}

/**
 * Test clearing devices
 */
void test_clearFrezzerDevices(void) {
    registerFrezzerDevice("AA:BB:CC:DD:EE:01", "fridge_1");
    registerFrezzerDevice("AA:BB:CC:DD:EE:02", "fridge_2");
    
    TEST_ASSERT_EQUAL_UINT8(2, getFrezzerDeviceCount());
    
    clearFrezzerDevices();
    
    TEST_ASSERT_EQUAL_UINT8(0, getFrezzerDeviceCount());
}

/**
 * Test max devices limit
 */
void test_registerFrezzerDevice_maxLimit(void) {
    // Register max number of devices
    for (int i = 0; i < MAX_FREZZER_DEVICES; i++) {
        char mac[18];
        snprintf(mac, sizeof(mac), "AA:BB:CC:DD:EE:%02X", i);
        char location[32];
        snprintf(location, sizeof(location), "fridge_%d", i);
        
        bool result = registerFrezzerDevice(mac, location);
        TEST_ASSERT_TRUE(result);
    }
    
    TEST_ASSERT_EQUAL_UINT8(MAX_FREZZER_DEVICES, getFrezzerDeviceCount());
    
    // Try to register one more
    bool result = registerFrezzerDevice("FF:FF:FF:FF:FF:FF", "extra");
    TEST_ASSERT_FALSE(result);
    TEST_ASSERT_EQUAL_UINT8(MAX_FREZZER_DEVICES, getFrezzerDeviceCount());
}

/**
 * Test connect/disconnect (simulation in unit test mode)
 */
void test_connectDisconnect(void) {
    registerFrezzerDevice("AA:BB:CC:DD:EE:FF", "fridge");
    
    FrezzerDevice* devices = getInternalFrezzerDevicesForTesting();
    
    TEST_ASSERT_FALSE(isFrezzerConnected(&devices[0]));
    
    FrezzerResult result = connectFrezzer(&devices[0]);
    TEST_ASSERT_EQUAL(FrezzerResult::SUCCESS, result);
    TEST_ASSERT_TRUE(isFrezzerConnected(&devices[0]));
    
    // Try connecting again
    result = connectFrezzer(&devices[0]);
    TEST_ASSERT_EQUAL(FrezzerResult::ALREADY_CONNECTED, result);
    
    result = disconnectFrezzer(&devices[0]);
    TEST_ASSERT_EQUAL(FrezzerResult::SUCCESS, result);
    TEST_ASSERT_FALSE(isFrezzerConnected(&devices[0]));
}

/**
 * Test read status (simulation in unit test mode)
 */
void test_readStatus(void) {
    registerFrezzerDevice("AA:BB:CC:DD:EE:FF", "fridge");
    
    FrezzerDevice* devices = getInternalFrezzerDevicesForTesting();
    
    // Should fail when not connected
    FrezzerResult result = readFrezzerStatus(&devices[0]);
    TEST_ASSERT_EQUAL(FrezzerResult::NOT_CONNECTED, result);
    
    // Connect first
    connectFrezzer(&devices[0]);
    
    // Now should succeed
    result = readFrezzerStatus(&devices[0]);
    TEST_ASSERT_EQUAL(FrezzerResult::SUCCESS, result);
    TEST_ASSERT_TRUE(devices[0].hasData);
    TEST_ASSERT_TRUE(devices[0].lastData.valid);
}

/**
 * Test set target temperature (simulation in unit test mode)
 */
void test_setTargetTemp(void) {
    registerFrezzerDevice("AA:BB:CC:DD:EE:FF", "fridge");
    
    FrezzerDevice* devices = getInternalFrezzerDevicesForTesting();
    
    // Connect first
    connectFrezzer(&devices[0]);
    
    // Valid temperature
    FrezzerResult result = setFrezzerTargetTemp(&devices[0], -8.0f);
    TEST_ASSERT_EQUAL(FrezzerResult::SUCCESS, result);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, -8.0f, devices[0].lastData.targetTemp);
    
    // Invalid temperature (too cold)
    result = setFrezzerTargetTemp(&devices[0], -30.0f);
    TEST_ASSERT_EQUAL(FrezzerResult::INVALID_PARAM, result);
    
    // Invalid temperature (too warm)
    result = setFrezzerTargetTemp(&devices[0], 25.0f);
    TEST_ASSERT_EQUAL(FrezzerResult::INVALID_PARAM, result);
}

/**
 * Test set mode (simulation in unit test mode)
 */
void test_setMode(void) {
    registerFrezzerDevice("AA:BB:CC:DD:EE:FF", "fridge");
    
    FrezzerDevice* devices = getInternalFrezzerDevicesForTesting();
    connectFrezzer(&devices[0]);
    
    FrezzerResult result = setFrezzerMode(&devices[0], FrezzerMode::FREEZER);
    TEST_ASSERT_EQUAL(FrezzerResult::SUCCESS, result);
    TEST_ASSERT_EQUAL(FrezzerMode::FREEZER, devices[0].lastData.mode);
    
    // Invalid mode
    result = setFrezzerMode(&devices[0], FrezzerMode::UNKNOWN);
    TEST_ASSERT_EQUAL(FrezzerResult::INVALID_PARAM, result);
}

/**
 * Test power on/off (simulation in unit test mode)
 */
void test_powerOnOff(void) {
    registerFrezzerDevice("AA:BB:CC:DD:EE:FF", "fridge");
    
    FrezzerDevice* devices = getInternalFrezzerDevicesForTesting();
    connectFrezzer(&devices[0]);
    
    FrezzerResult result = turnFrezzerOff(&devices[0]);
    TEST_ASSERT_EQUAL(FrezzerResult::SUCCESS, result);
    TEST_ASSERT_EQUAL(FrezzerMode::OFF, devices[0].lastData.mode);
    
    result = turnFrezzerOn(&devices[0]);
    TEST_ASSERT_EQUAL(FrezzerResult::SUCCESS, result);
    TEST_ASSERT_EQUAL(FrezzerMode::FRIDGE, devices[0].lastData.mode);
}

/**
 * Test auto-discovery during scan
 */
void test_updateFrezzerFromScan_autoDiscover(void) {
    TEST_ASSERT_EQUAL_UINT8(0, getFrezzerDeviceCount());
    
    uint8_t macAddr[6] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66};
    
    bool result = updateFrezzerFromScan(macAddr, "FREZZER PRO 65L", -50, 1000);
    TEST_ASSERT_TRUE(result);
    
    // Should have auto-registered
    TEST_ASSERT_EQUAL_UINT8(1, getFrezzerDeviceCount());
    
    const FrezzerDevice* device = getFrezzerDevice(0);
    TEST_ASSERT_EQUAL_STRING("11:22:33:44:55:66", device->macString);
    TEST_ASSERT_EQUAL_STRING("FREZZER PRO 65L", device->deviceName);
    TEST_ASSERT_FALSE(device->registered);  // Not explicitly registered
}

/**
 * Test parsing valid Frezzer data
 */
void test_parseFrezzerData_valid(void) {
    // Construct test data matching the expected format
    uint8_t testData[] = {
        0x01,              // Header/Protocol version
        0xCE, 0xFF,        // Current temp: -50 (0xFFCE) = -5.0°C
        0xB0, 0xFF,        // Target temp: -80 (0xFFB0) = -8.0°C
        0x68, 0x30,        // Voltage: 12392 mV = 12.392V
        0x01,              // Mode: FRIDGE
        0x01,              // Compressor: RUNNING
        0x30,              // Flags: powerLevel=12 (bits 2-7), no lid open (bit 0), no low voltage (bit 1)
        0x00               // Error: NONE
    };
    
    FrezzerData data = parseFrezzerData(testData, sizeof(testData));
    
    TEST_ASSERT_TRUE(data.valid);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, -5.0f, data.currentTemp);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, -8.0f, data.targetTemp);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 12.392f, data.batteryVoltage);
    TEST_ASSERT_EQUAL(FrezzerMode::FRIDGE, data.mode);
    TEST_ASSERT_EQUAL(FrezzerCompressorState::RUNNING, data.compressor);
    TEST_ASSERT_EQUAL(FrezzerError::NONE, data.error);
    TEST_ASSERT_FALSE(data.lidOpen);
    TEST_ASSERT_FALSE(data.lowVoltageProtection);
}

/**
 * Test parsing invalid Frezzer data
 */
void test_parseFrezzerData_invalid(void) {
    // Too short
    uint8_t shortData[] = {0x01, 0x02, 0x03};
    FrezzerData data = parseFrezzerData(shortData, sizeof(shortData));
    TEST_ASSERT_FALSE(data.valid);
    
    // Null pointer
    data = parseFrezzerData(nullptr, 10);
    TEST_ASSERT_FALSE(data.valid);
    
    // Invalid header
    uint8_t invalidHeader[] = {0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    data = parseFrezzerData(invalidHeader, sizeof(invalidHeader));
    TEST_ASSERT_FALSE(data.valid);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    
    // Initialization tests
    RUN_TEST(test_initFrezzerController);
    
    // MAC conversion tests
    RUN_TEST(test_frezzerStringToMac);
    RUN_TEST(test_frezzerMacToString);
    RUN_TEST(test_frezzerStringToMac_lowercase);
    RUN_TEST(test_frezzerStringToMac_invalid);
    
    // Registration tests
    RUN_TEST(test_registerFrezzerDevice);
    RUN_TEST(test_registerFrezzerDevice_multiple);
    RUN_TEST(test_registerFrezzerDevice_duplicate);
    RUN_TEST(test_registerFrezzerDevice_maxLimit);
    RUN_TEST(test_clearFrezzerDevices);
    
    // Find tests
    RUN_TEST(test_findFrezzerDeviceByMac);
    RUN_TEST(test_findFrezzerDeviceByMac_notFound);
    
    // Device name matching
    RUN_TEST(test_isFrezzerDevice);
    
    // String conversion tests
    RUN_TEST(test_frezzerModeToString);
    RUN_TEST(test_frezzerCompressorStateToString);
    RUN_TEST(test_frezzerErrorToString);
    RUN_TEST(test_frezzerResultToString);
    
    // Command building tests
    RUN_TEST(test_buildFrezzerCommand);
    RUN_TEST(test_buildFrezzerCommand_nullBuffer);
    RUN_TEST(test_buildFrezzerCommand_smallBuffer);
    
    // Freshness tests
    RUN_TEST(test_isFrezzerDataFresh);
    RUN_TEST(test_getFreshFrezzerDevices);
    
    // Connection tests (simulation)
    RUN_TEST(test_connectDisconnect);
    RUN_TEST(test_readStatus);
    RUN_TEST(test_setTargetTemp);
    RUN_TEST(test_setMode);
    RUN_TEST(test_powerOnOff);
    
    // Scan/discovery tests
    RUN_TEST(test_updateFrezzerFromScan_autoDiscover);
    
    // Data parsing tests
    RUN_TEST(test_parseFrezzerData_valid);
    RUN_TEST(test_parseFrezzerData_invalid);
    
    return UNITY_END();
}
