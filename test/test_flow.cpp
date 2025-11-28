/**
 * @file test_flow.cpp
 * @brief Unit tests for flow calculation module
 */
#include <unity.h>
#include "flow.h"
#include <cmath>

void setUp(void) {
    // Called before each test
}

void tearDown(void) {
    // Called after each test
}

/**
 * Test calculateFrequency with normal values
 */
void test_calculateFrequency_normal(void) {
    // 100 pulses in 1000ms = 100 Hz
    float result = calculateFrequency(100, 1000);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 100.0f, result);
}

/**
 * Test calculateFrequency with zero interval
 */
void test_calculateFrequency_zero_interval(void) {
    float result = calculateFrequency(100, 0);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, result);
}

/**
 * Test calculateFrequency with zero pulses
 */
void test_calculateFrequency_zero_pulses(void) {
    float result = calculateFrequency(0, 1000);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, result);
}

/**
 * Test calculateFrequency with short interval
 */
void test_calculateFrequency_short_interval(void) {
    // 50 pulses in 500ms = 100 Hz
    float result = calculateFrequency(50, 500);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 100.0f, result);
}

/**
 * Test calculateFlowRate with normal values
 */
void test_calculateFlowRate_normal(void) {
    // At calibration factor 6.6 and frequency 66 Hz
    // Flow rate = (66 / 6.6) * 60 = 600 L/min
    float result = calculateFlowRate(66.0f, 6.6f);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 600.0f, result);
}

/**
 * Test calculateFlowRate with zero calibration factor
 */
void test_calculateFlowRate_zero_calibration(void) {
    float result = calculateFlowRate(66.0f, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, result);
}

/**
 * Test calculateFlowRate with zero frequency
 */
void test_calculateFlowRate_zero_frequency(void) {
    float result = calculateFlowRate(0.0f, 6.6f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, result);
}

/**
 * Test calculateRequiredDeltaT with normal values
 */
void test_calculateRequiredDeltaT_normal(void) {
    // Power = 5000W, flow = 10 L/min
    // DeltaT = 5000 / (3.5 * 10) = 142.86°C
    float result = calculateRequiredDeltaT(5000.0f, 10.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 142.86f, result);
}

/**
 * Test calculateRequiredDeltaT with zero flow
 */
void test_calculateRequiredDeltaT_zero_flow(void) {
    float result = calculateRequiredDeltaT(5000.0f, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, result);
}

/**
 * Test calculateRequiredDeltaT with high flow rate
 */
void test_calculateRequiredDeltaT_high_flow(void) {
    // Power = 5000W, flow = 100 L/min
    // DeltaT = 5000 / (3.5 * 100) = 14.29°C
    float result = calculateRequiredDeltaT(5000.0f, 100.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 14.29f, result);
}

/**
 * Test calculateFlowData complete calculation
 */
void test_calculateFlowData_complete(void) {
    // 330 pulses in 5000ms with calibration 6.6 and heater power 5000W
    // Frequency = 330 / 5 = 66 Hz
    // Flow rate = (66 / 6.6) * 60 = 600 L/min
    // DeltaT = 5000 / (3.5 * 600) = 2.38°C
    FlowData result = calculateFlowData(330, 5000, 6.6f, 5000.0f);
    
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 66.0f, result.frequency);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 600.0f, result.flowRate);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 2.38f, result.requiredDeltaT);
}

/**
 * Test calculateFlowData with test mode values (from main.cpp)
 * In test mode, count is random between 198 and 462
 * Let's use 330 (middle value) with 5 second interval
 */
void test_calculateFlowData_testmode_values(void) {
    FlowData result = calculateFlowData(330, 5000, DEFAULT_CALIBRATION_FACTOR, DEFAULT_HEATER_POWER);
    
    // Frequency should be 330/5 = 66 Hz
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 66.0f, result.frequency);
    // Flow rate should be (66/6.6)*60 = 600 L/min
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 600.0f, result.flowRate);
    // Required deltaT should be 5000/(3.5*600) ≈ 2.38°C
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 2.38f, result.requiredDeltaT);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    
    RUN_TEST(test_calculateFrequency_normal);
    RUN_TEST(test_calculateFrequency_zero_interval);
    RUN_TEST(test_calculateFrequency_zero_pulses);
    RUN_TEST(test_calculateFrequency_short_interval);
    RUN_TEST(test_calculateFlowRate_normal);
    RUN_TEST(test_calculateFlowRate_zero_calibration);
    RUN_TEST(test_calculateFlowRate_zero_frequency);
    RUN_TEST(test_calculateRequiredDeltaT_normal);
    RUN_TEST(test_calculateRequiredDeltaT_zero_flow);
    RUN_TEST(test_calculateRequiredDeltaT_high_flow);
    RUN_TEST(test_calculateFlowData_complete);
    RUN_TEST(test_calculateFlowData_testmode_values);
    
    return UNITY_END();
}
