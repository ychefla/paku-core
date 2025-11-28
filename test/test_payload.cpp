/**
 * @file test_payload.cpp
 * @brief Unit tests for payload handling module
 */
#include <unity.h>
#include "payload.h"

void setUp(void) {
    // Called before each test
}

void tearDown(void) {
    // Called after each test
}

/**
 * Test that createPayloadJson creates valid JSON format
 */
void test_createPayloadJson_format(void) {
    String result = createPayloadJson(25.5f, "12:34:56");
    
    // Check that result contains expected parts
    TEST_ASSERT_NOT_EQUAL(-1, result.indexOf("\"value\":"));
    TEST_ASSERT_NOT_EQUAL(-1, result.indexOf("25."));
    TEST_ASSERT_NOT_EQUAL(-1, result.indexOf("\"timestamp\":"));
    TEST_ASSERT_NOT_EQUAL(-1, result.indexOf("12:34:56"));
}

/**
 * Test createPayloadJson with zero value
 */
void test_createPayloadJson_zero_value(void) {
    String result = createPayloadJson(0.0f, "00:00:00");
    
    TEST_ASSERT_NOT_EQUAL(-1, result.indexOf("\"value\":"));
    TEST_ASSERT_NOT_EQUAL(-1, result.indexOf("0."));
}

/**
 * Test createPayloadJson with negative value
 */
void test_createPayloadJson_negative_value(void) {
    String result = createPayloadJson(-1000.0f, "12:00:00");
    
    TEST_ASSERT_NOT_EQUAL(-1, result.indexOf("-1000"));
}

/**
 * Test storePayload stores correctly in buffer
 */
void test_storePayload_success(void) {
    Payload payloads[5];
    int index = 0;
    
    bool result = storePayload(payloads, index, 5, "test/topic", 42.0f, "10:00:00");
    
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL_INT(1, index);
    TEST_ASSERT_TRUE(payloads[0].topic == "test/topic");
    TEST_ASSERT_NOT_EQUAL(-1, payloads[0].data.indexOf("42."));
}

/**
 * Test storePayload increments index correctly
 */
void test_storePayload_multiple(void) {
    Payload payloads[5];
    int index = 0;
    
    storePayload(payloads, index, 5, "topic1", 1.0f, "10:00:00");
    storePayload(payloads, index, 5, "topic2", 2.0f, "10:00:01");
    storePayload(payloads, index, 5, "topic3", 3.0f, "10:00:02");
    
    TEST_ASSERT_EQUAL_INT(3, index);
    TEST_ASSERT_TRUE(payloads[0].topic == "topic1");
    TEST_ASSERT_TRUE(payloads[1].topic == "topic2");
    TEST_ASSERT_TRUE(payloads[2].topic == "topic3");
}

/**
 * Test storePayload returns false when buffer is full
 */
void test_storePayload_buffer_full(void) {
    Payload payloads[2];
    int index = 0;
    
    bool result1 = storePayload(payloads, index, 2, "topic1", 1.0f, "10:00:00");
    bool result2 = storePayload(payloads, index, 2, "topic2", 2.0f, "10:00:01");
    bool result3 = storePayload(payloads, index, 2, "topic3", 3.0f, "10:00:02");
    
    TEST_ASSERT_TRUE(result1);
    TEST_ASSERT_TRUE(result2);
    TEST_ASSERT_FALSE(result3);  // Buffer is full
    TEST_ASSERT_EQUAL_INT(2, index);  // Index should not increase
}

/**
 * Test storePayload with empty topic
 */
void test_storePayload_empty_topic(void) {
    Payload payloads[5];
    int index = 0;
    
    bool result = storePayload(payloads, index, 5, "", 42.0f, "10:00:00");
    
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_TRUE(payloads[0].topic.isEmpty());
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    
    RUN_TEST(test_createPayloadJson_format);
    RUN_TEST(test_createPayloadJson_zero_value);
    RUN_TEST(test_createPayloadJson_negative_value);
    RUN_TEST(test_storePayload_success);
    RUN_TEST(test_storePayload_multiple);
    RUN_TEST(test_storePayload_buffer_full);
    RUN_TEST(test_storePayload_empty_topic);
    
    return UNITY_END();
}
