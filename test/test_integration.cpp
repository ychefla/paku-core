/**
 * @file test_integration.cpp
 * @brief Integration tests for paku-core data flow
 * 
 * This tests the end-to-end flow of:
 * 1. Parsing Ruuvi tag BLE data
 * 2. Transforming sensor data to MQTT payloads
 * 3. Validating the complete data pipeline
 */
#include <unity.h>
#include "ruuvi.h"
#include "payload.h"
#include "flow.h"
#include <cstdio>

void setUp(void) {}
void tearDown(void) {}

/**
 * @brief Synthetic Ruuvi RAWv2 test data
 * 
 * Format: 0x05 (format 5)
 * Temperature: 0x0AAA = 2730 * 0.005 = 13.65°C
 * Humidity: 0x9A64 = 39524 * 0.0025 = 98.81%
 * Pressure: 0xC37A = 50042 + 50000 = 100042 Pa
 * Accel X: 0x0004 = 4 mG = 0.004 G
 * Accel Y: 0xFFFC = -4 mG = -0.004 G
 * Accel Z: 0x03E8 = 1000 mG = 1.0 G
 * Power: TX = 4, Battery = 2821 + 1600 = 4421 mV = 4.421 V (encoded: 0xB034)
 * Movement: 0x42 = 66
 * Sequence: 0x00AB = 171
 */
static const uint8_t SYNTHETIC_RUUVI_DATA[] = {
    0x05,              // Format 5 (RAWv2)
    0x0A, 0xAA,        // Temperature: 13.65°C
    0x9A, 0x64,        // Humidity: 98.81%
    0xC3, 0x7A,        // Pressure: 100042 Pa
    0x00, 0x04,        // Accel X: 0.004 G
    0xFF, 0xFC,        // Accel Y: -0.004 G
    0x03, 0xE8,        // Accel Z: 1.0 G
    0xB0, 0x34,        // Power info
    0x42,              // Movement counter: 66
    0x00, 0xAB         // Sequence: 171
};

/**
 * Test that synthetic Ruuvi data is correctly identified as valid
 */
void test_integration_ruuvi_valid(void) {
    bool valid = isValidRuuviV5(SYNTHETIC_RUUVI_DATA, sizeof(SYNTHETIC_RUUVI_DATA));
    TEST_ASSERT_TRUE(valid);
}

/**
 * Test parsing of synthetic Ruuvi data
 */
void test_integration_ruuvi_parse(void) {
    RuuviData data = parseRuuviDataV5(SYNTHETIC_RUUVI_DATA, sizeof(SYNTHETIC_RUUVI_DATA));
    
    TEST_ASSERT_TRUE(data.valid);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 13.65f, data.temperature);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 98.81f, data.humidity);
    TEST_ASSERT_FLOAT_WITHIN(10.0f, 100042.0f, data.pressure);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.004f, data.accelerationX);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -0.004f, data.accelerationY);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.0f, data.accelerationZ);
    TEST_ASSERT_EQUAL_UINT8(66, data.movementCounter);
    TEST_ASSERT_EQUAL_UINT16(171, data.measurementSequence);
}

/**
 * Test full data pipeline: Ruuvi -> Payload
 */
void test_integration_ruuvi_to_payload(void) {
    // Parse Ruuvi data
    RuuviData ruuvi = parseRuuviDataV5(SYNTHETIC_RUUVI_DATA, sizeof(SYNTHETIC_RUUVI_DATA));
    TEST_ASSERT_TRUE(ruuvi.valid);
    
    // Create payloads
    Payload payloads[10];
    int index = 0;
    String timestamp = "2025-01-15T10:30:00Z";
    
    // Store temperature payload
    bool stored = storePayload(payloads, index, 10, 
                               "paku/temperature/ruuvi/test", 
                               ruuvi.temperature, timestamp);
    TEST_ASSERT_TRUE(stored);
    TEST_ASSERT_EQUAL_INT(1, index);
    
    // Store humidity payload
    stored = storePayload(payloads, index, 10,
                          "paku/humidity/ruuvi/test",
                          ruuvi.humidity, timestamp);
    TEST_ASSERT_TRUE(stored);
    TEST_ASSERT_EQUAL_INT(2, index);
    
    // Verify payload content
    TEST_ASSERT_TRUE(payloads[0].topic == "paku/temperature/ruuvi/test");
    TEST_ASSERT_NOT_EQUAL(-1, payloads[0].data.indexOf("13."));
    TEST_ASSERT_TRUE(payloads[1].topic == "paku/humidity/ruuvi/test");
    TEST_ASSERT_NOT_EQUAL(-1, payloads[1].data.indexOf("98."));
}

/**
 * Test combined sensor data processing
 * Simulates receiving Ruuvi data + flow data and creating complete message set
 */
void test_integration_combined_sensors(void) {
    // Parse Ruuvi environmental data
    RuuviData ruuvi = parseRuuviDataV5(SYNTHETIC_RUUVI_DATA, sizeof(SYNTHETIC_RUUVI_DATA));
    
    // Calculate flow data (simulating heater flow sensor)
    FlowData flow = calculateFlowData(330, 5000, 6.6f, 5000.0f);
    
    // Create all payloads
    Payload payloads[MAX_PAYLOADS];
    int index = 0;
    String timestamp = "2025-01-15T10:30:00Z";
    
    // Ruuvi temperature
    storePayload(payloads, index, MAX_PAYLOADS,
                 "paku/temperature/ruuvi/cabin", ruuvi.temperature, timestamp);
    
    // Ruuvi humidity
    storePayload(payloads, index, MAX_PAYLOADS,
                 "paku/humidity/ruuvi/cabin", ruuvi.humidity, timestamp);
    
    // Flow sensor data
    storePayload(payloads, index, MAX_PAYLOADS,
                 "paku/flow/coolant", flow.flowRate, timestamp);
    
    // Required delta T
    storePayload(payloads, index, MAX_PAYLOADS,
                 "paku/temperature/heating/required_dt", flow.requiredDeltaT, timestamp);
    
    // Verify all payloads created
    TEST_ASSERT_EQUAL_INT(4, index);
    
    // Verify data correctness
    TEST_ASSERT_NOT_EQUAL(-1, payloads[2].data.indexOf("600."));  // Flow rate ~600 L/min
    TEST_ASSERT_NOT_EQUAL(-1, payloads[3].data.indexOf("2."));    // DeltaT ~2.38°C
}

/**
 * Test invalid Ruuvi data handling
 */
void test_integration_invalid_ruuvi(void) {
    // Invalid format byte
    uint8_t invalidData[] = {0x03, 0x00, 0x00, 0x00, 0x00};  // Format 3, not 5
    
    bool valid = isValidRuuviV5(invalidData, sizeof(invalidData));
    TEST_ASSERT_FALSE(valid);
    
    RuuviData data = parseRuuviDataV5(invalidData, sizeof(invalidData));
    TEST_ASSERT_FALSE(data.valid);
}

/**
 * Test null data handling
 */
void test_integration_null_data(void) {
    bool valid = isValidRuuviV5(nullptr, 0);
    TEST_ASSERT_FALSE(valid);
    
    RuuviData data = parseRuuviDataV5(nullptr, 0);
    TEST_ASSERT_FALSE(data.valid);
}

/**
 * Test short data handling
 */
void test_integration_short_data(void) {
    uint8_t shortData[] = {0x05, 0x00, 0x00};  // Too short
    
    bool valid = isValidRuuviV5(shortData, sizeof(shortData));
    TEST_ASSERT_FALSE(valid);
}

/**
 * Test end-to-end simulation with multiple Ruuvi tags
 */
void test_integration_multiple_ruuvi_tags(void) {
    Payload payloads[MAX_PAYLOADS];
    int index = 0;
    String timestamp = "2025-01-15T10:30:00Z";
    
    // Simulate 3 Ruuvi tags
    const char* locations[] = {"cabin", "kitchen", "lounge"};
    
    for (int i = 0; i < 3; i++) {
        RuuviData ruuvi = parseRuuviDataV5(SYNTHETIC_RUUVI_DATA, sizeof(SYNTHETIC_RUUVI_DATA));
        TEST_ASSERT_TRUE(ruuvi.valid);
        
        // Create topic for this location
        String tempTopic = String("paku/temperature/ruuvi/") + locations[i];
        String humidTopic = String("paku/humidity/ruuvi/") + locations[i];
        
        storePayload(payloads, index, MAX_PAYLOADS, tempTopic, ruuvi.temperature, timestamp);
        storePayload(payloads, index, MAX_PAYLOADS, humidTopic, ruuvi.humidity, timestamp);
    }
    
    // Should have 6 payloads (2 per tag * 3 tags)
    TEST_ASSERT_EQUAL_INT(6, index);
    
    // Verify topics are correct
    TEST_ASSERT_TRUE(payloads[0].topic == "paku/temperature/ruuvi/cabin");
    TEST_ASSERT_TRUE(payloads[1].topic == "paku/humidity/ruuvi/cabin");
    TEST_ASSERT_TRUE(payloads[2].topic == "paku/temperature/ruuvi/kitchen");
    TEST_ASSERT_TRUE(payloads[3].topic == "paku/humidity/ruuvi/kitchen");
    TEST_ASSERT_TRUE(payloads[4].topic == "paku/temperature/ruuvi/lounge");
    TEST_ASSERT_TRUE(payloads[5].topic == "paku/humidity/ruuvi/lounge");
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    
    // Ruuvi parsing tests
    RUN_TEST(test_integration_ruuvi_valid);
    RUN_TEST(test_integration_ruuvi_parse);
    
    // Pipeline tests
    RUN_TEST(test_integration_ruuvi_to_payload);
    RUN_TEST(test_integration_combined_sensors);
    
    // Error handling tests
    RUN_TEST(test_integration_invalid_ruuvi);
    RUN_TEST(test_integration_null_data);
    RUN_TEST(test_integration_short_data);
    
    // Multi-device test
    RUN_TEST(test_integration_multiple_ruuvi_tags);
    
    return UNITY_END();
}
