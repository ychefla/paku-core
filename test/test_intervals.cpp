/**
 * @file test_intervals.cpp
 * @brief Unit tests for interval management module
 */
#include <unity.h>
#include "intervals.h"

void setUp(void) {
    // Called before each test
}

void tearDown(void) {
    // Called after each test
}

/**
 * Test that heater off uses slow intervals
 */
void test_calculateIntervals_heater_off(void) {
    unsigned long heaterOnStartTime = 0;
    
    IntervalConfig config = calculateIntervals(0, 1000, heaterOnStartTime);
    
    TEST_ASSERT_EQUAL_UINT32(MQTT_SLOW_INTERVAL, config.mqttInterval);
    TEST_ASSERT_EQUAL_UINT32(SENSOR_SLOW_INTERVAL, config.sensorInterval);
    TEST_ASSERT_EQUAL_UINT32(0, heaterOnStartTime);
}

/**
 * Test that heater on (just started) uses fast intervals
 */
void test_calculateIntervals_heater_on_just_started(void) {
    unsigned long heaterOnStartTime = 0;
    
    IntervalConfig config = calculateIntervals(1, 5000, heaterOnStartTime);
    
    TEST_ASSERT_EQUAL_UINT32(MQTT_FAST_INTERVAL, config.mqttInterval);
    TEST_ASSERT_EQUAL_UINT32(SENSOR_FAST_INTERVAL, config.sensorInterval);
    TEST_ASSERT_EQUAL_UINT32(5000, heaterOnStartTime);
}

/**
 * Test that heater on for less than 1 hour uses fast intervals
 */
void test_calculateIntervals_heater_on_less_than_hour(void) {
    unsigned long heaterOnStartTime = 1000;  // Started at 1 second
    unsigned long currentTime = 1800000;  // 30 minutes later
    
    IntervalConfig config = calculateIntervals(1, currentTime, heaterOnStartTime);
    
    TEST_ASSERT_EQUAL_UINT32(MQTT_FAST_INTERVAL, config.mqttInterval);
    TEST_ASSERT_EQUAL_UINT32(SENSOR_FAST_INTERVAL, config.sensorInterval);
    TEST_ASSERT_EQUAL_UINT32(1000, heaterOnStartTime);  // Should remain unchanged
}

/**
 * Test that heater on for more than 1 hour uses slow intervals
 */
void test_calculateIntervals_heater_on_more_than_hour(void) {
    unsigned long heaterOnStartTime = 1000;  // Started at 1 second
    unsigned long currentTime = 3601001;  // Just over 1 hour later
    
    IntervalConfig config = calculateIntervals(1, currentTime, heaterOnStartTime);
    
    TEST_ASSERT_EQUAL_UINT32(MQTT_SLOW_INTERVAL, config.mqttInterval);
    TEST_ASSERT_EQUAL_UINT32(SENSOR_SLOW_INTERVAL, config.sensorInterval);
}

/**
 * Test that heater on exactly at threshold uses slow intervals
 */
void test_calculateIntervals_heater_on_at_threshold(void) {
    unsigned long heaterOnStartTime = 1000;
    unsigned long currentTime = 1000 + HEATER_ON_DURATION_THRESHOLD;  // Exactly at threshold
    
    IntervalConfig config = calculateIntervals(1, currentTime, heaterOnStartTime);
    
    TEST_ASSERT_EQUAL_UINT32(MQTT_SLOW_INTERVAL, config.mqttInterval);
    TEST_ASSERT_EQUAL_UINT32(SENSOR_SLOW_INTERVAL, config.sensorInterval);
}

/**
 * Test that turning heater off resets start time
 */
void test_calculateIntervals_heater_off_resets_start_time(void) {
    unsigned long heaterOnStartTime = 5000;  // Was set from previous on state
    
    IntervalConfig config = calculateIntervals(0, 10000, heaterOnStartTime);
    
    TEST_ASSERT_EQUAL_UINT32(0, heaterOnStartTime);  // Should be reset
    TEST_ASSERT_EQUAL_UINT32(MQTT_SLOW_INTERVAL, config.mqttInterval);
}

/**
 * Test heater on-off-on cycle
 */
void test_calculateIntervals_heater_cycle(void) {
    unsigned long heaterOnStartTime = 0;
    
    // Heater turns on
    IntervalConfig config1 = calculateIntervals(1, 1000, heaterOnStartTime);
    TEST_ASSERT_EQUAL_UINT32(1000, heaterOnStartTime);
    TEST_ASSERT_EQUAL_UINT32(MQTT_FAST_INTERVAL, config1.mqttInterval);
    
    // Heater still on after 30 minutes
    IntervalConfig config2 = calculateIntervals(1, 1800001, heaterOnStartTime);
    TEST_ASSERT_EQUAL_UINT32(1000, heaterOnStartTime);  // Unchanged
    TEST_ASSERT_EQUAL_UINT32(MQTT_FAST_INTERVAL, config2.mqttInterval);
    
    // Heater turns off
    IntervalConfig config3 = calculateIntervals(0, 1900000, heaterOnStartTime);
    TEST_ASSERT_EQUAL_UINT32(0, heaterOnStartTime);  // Reset
    TEST_ASSERT_EQUAL_UINT32(MQTT_SLOW_INTERVAL, config3.mqttInterval);
    
    // Heater turns on again
    IntervalConfig config4 = calculateIntervals(1, 2000000, heaterOnStartTime);
    TEST_ASSERT_EQUAL_UINT32(2000000, heaterOnStartTime);  // New start time
    TEST_ASSERT_EQUAL_UINT32(MQTT_FAST_INTERVAL, config4.mqttInterval);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    
    RUN_TEST(test_calculateIntervals_heater_off);
    RUN_TEST(test_calculateIntervals_heater_on_just_started);
    RUN_TEST(test_calculateIntervals_heater_on_less_than_hour);
    RUN_TEST(test_calculateIntervals_heater_on_more_than_hour);
    RUN_TEST(test_calculateIntervals_heater_on_at_threshold);
    RUN_TEST(test_calculateIntervals_heater_off_resets_start_time);
    RUN_TEST(test_calculateIntervals_heater_cycle);
    
    return UNITY_END();
}
