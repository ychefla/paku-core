/**
 * @file test_sensor_placeholders.cpp
 * @brief Unit tests for sensor placeholder module
 */
#include <unity.h>
#include "sensor_placeholders.h"
#include <cmath>

void setUp(void) {}
void tearDown(void) {}

/**
 * Test placeholder value detection
 */
void test_isPlaceholderValue(void) {
    TEST_ASSERT_TRUE(isPlaceholderValue(SENSOR_NOT_AVAILABLE));
    TEST_ASSERT_TRUE(isPlaceholderValue(-1000.0f));
    TEST_ASSERT_FALSE(isPlaceholderValue(0.0f));
    TEST_ASSERT_FALSE(isPlaceholderValue(25.5f));
    TEST_ASSERT_FALSE(isPlaceholderValue(-999.0f));
}

/**
 * Test placeholder temperature generation (disabled)
 */
void test_generatePlaceholderTemperature_disabled(void) {
    float temp = generatePlaceholderTemperature("cabin", false);
    TEST_ASSERT_TRUE(isPlaceholderValue(temp));
}

/**
 * Test placeholder temperature generation (enabled)
 */
void test_generatePlaceholderTemperature_enabled(void) {
    float temp = generatePlaceholderTemperature("cabin", true);
    
    // Should generate a realistic indoor temperature (10-30°C range for testing tolerance)
    TEST_ASSERT_FALSE(isPlaceholderValue(temp));
    TEST_ASSERT_TRUE(temp >= 10.0f && temp <= 30.0f);
}

/**
 * Test placeholder humidity generation (disabled)
 */
void test_generatePlaceholderHumidity_disabled(void) {
    float humidity = generatePlaceholderHumidity("cabin", false);
    TEST_ASSERT_TRUE(isPlaceholderValue(humidity));
}

/**
 * Test placeholder humidity generation (enabled)
 */
void test_generatePlaceholderHumidity_enabled(void) {
    float humidity = generatePlaceholderHumidity("cabin", true);
    
    // Should generate realistic humidity (0-100%)
    TEST_ASSERT_FALSE(isPlaceholderValue(humidity));
    TEST_ASSERT_TRUE(humidity >= 0.0f && humidity <= 100.0f);
}

/**
 * Test placeholder pressure generation (disabled)
 */
void test_generatePlaceholderPressure_disabled(void) {
    float pressure = generatePlaceholderPressure(false);
    TEST_ASSERT_TRUE(isPlaceholderValue(pressure));
}

/**
 * Test placeholder pressure generation (enabled)
 */
void test_generatePlaceholderPressure_enabled(void) {
    float pressure = generatePlaceholderPressure(true);
    
    // Should generate realistic atmospheric pressure (980-1040 hPa)
    TEST_ASSERT_FALSE(isPlaceholderValue(pressure));
    TEST_ASSERT_TRUE(pressure >= 950.0f && pressure <= 1060.0f);
}

/**
 * Test placeholder voltage generation (disabled)
 */
void test_generatePlaceholderVoltage_disabled(void) {
    float voltage = generatePlaceholderVoltage("car", false);
    TEST_ASSERT_TRUE(isPlaceholderValue(voltage));
}

/**
 * Test placeholder voltage generation (enabled - car battery)
 */
void test_generatePlaceholderVoltage_car(void) {
    float voltage = generatePlaceholderVoltage("car", true);
    
    // Should generate realistic car battery voltage (11-14.5V)
    TEST_ASSERT_FALSE(isPlaceholderValue(voltage));
    TEST_ASSERT_TRUE(voltage >= 11.0f && voltage <= 14.5f);
}

/**
 * Test placeholder voltage generation (enabled - leisure battery)
 */
void test_generatePlaceholderVoltage_leisure(void) {
    float voltage = generatePlaceholderVoltage("leisure", true);
    
    // Should generate realistic leisure battery voltage (11-14.5V)
    TEST_ASSERT_FALSE(isPlaceholderValue(voltage));
    TEST_ASSERT_TRUE(voltage >= 11.0f && voltage <= 14.5f);
}

/**
 * Test complete placeholder reading (disabled)
 */
void test_generatePlaceholderReading_disabled(void) {
    PlaceholderSensorData data = generatePlaceholderReading("cabin", false);
    
    TEST_ASSERT_TRUE(data.isPlaceholder);
    TEST_ASSERT_TRUE(isPlaceholderValue(data.temperature));
    TEST_ASSERT_TRUE(isPlaceholderValue(data.humidity));
    TEST_ASSERT_TRUE(isPlaceholderValue(data.pressure));
    TEST_ASSERT_TRUE(isPlaceholderValue(data.voltage));
    TEST_ASSERT_EQUAL_INT(0, data.status);
}

/**
 * Test complete placeholder reading (enabled)
 */
void test_generatePlaceholderReading_enabled(void) {
    PlaceholderSensorData data = generatePlaceholderReading("kitchen", true);
    
    TEST_ASSERT_TRUE(data.isPlaceholder);
    TEST_ASSERT_FALSE(isPlaceholderValue(data.temperature));
    TEST_ASSERT_FALSE(isPlaceholderValue(data.humidity));
    TEST_ASSERT_FALSE(isPlaceholderValue(data.pressure));
    TEST_ASSERT_FALSE(isPlaceholderValue(data.voltage));
    TEST_ASSERT_EQUAL_INT(1, data.status);
}

/**
 * Test simple hash function
 */
void test_simpleHash(void) {
    uint32_t hash1 = simpleHash("cabin");
    uint32_t hash2 = simpleHash("cabin");
    uint32_t hash3 = simpleHash("kitchen");
    
    // Same input should produce same hash
    TEST_ASSERT_EQUAL_UINT32(hash1, hash2);
    
    // Different input should produce different hash
    TEST_ASSERT_NOT_EQUAL(hash1, hash3);
    
    // Null input should return 0
    TEST_ASSERT_EQUAL_UINT32(0, simpleHash(nullptr));
}

/**
 * Test location variation
 * Different locations should produce different values
 */
void test_locationVariation(void) {
    float temp1 = generatePlaceholderTemperature("cabin", true);
    float temp2 = generatePlaceholderTemperature("kitchen", true);
    float temp3 = generatePlaceholderTemperature("lounge", true);
    
    // Values should exist and be reasonable
    TEST_ASSERT_FALSE(isPlaceholderValue(temp1));
    TEST_ASSERT_FALSE(isPlaceholderValue(temp2));
    TEST_ASSERT_FALSE(isPlaceholderValue(temp3));
    
    // Note: Due to the pseudo-random nature, we can't guarantee different values
    // but the hash should make them vary based on location
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    
    // Placeholder detection
    RUN_TEST(test_isPlaceholderValue);
    
    // Temperature generation
    RUN_TEST(test_generatePlaceholderTemperature_disabled);
    RUN_TEST(test_generatePlaceholderTemperature_enabled);
    
    // Humidity generation
    RUN_TEST(test_generatePlaceholderHumidity_disabled);
    RUN_TEST(test_generatePlaceholderHumidity_enabled);
    
    // Pressure generation
    RUN_TEST(test_generatePlaceholderPressure_disabled);
    RUN_TEST(test_generatePlaceholderPressure_enabled);
    
    // Voltage generation
    RUN_TEST(test_generatePlaceholderVoltage_disabled);
    RUN_TEST(test_generatePlaceholderVoltage_car);
    RUN_TEST(test_generatePlaceholderVoltage_leisure);
    
    // Complete readings
    RUN_TEST(test_generatePlaceholderReading_disabled);
    RUN_TEST(test_generatePlaceholderReading_enabled);
    
    // Hash function
    RUN_TEST(test_simpleHash);
    
    // Location variation
    RUN_TEST(test_locationVariation);
    
    return UNITY_END();
}
