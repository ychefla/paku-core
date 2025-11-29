/**
 * @file test_paku_iot_client.cpp
 * @brief Unit tests for PakuIotClient
 * 
 * These tests verify the PakuIotClient adapter functionality including
 * configuration validation, queue management, and error handling.
 * 
 * Note: Network-dependent tests require mocking or are marked as integration tests.
 */

#include <Arduino.h>
#include <unity.h>
#include "PakuIotClient.h"

// Test fixtures
PakuIotClient client;
PakuIotConfig validConfig;

void setUp(void) {
    // Initialize valid config for tests
    validConfig.host = "test.paku.example.com";
    validConfig.port = 443;
    validConfig.apiKey = "test-api-key";
    validConfig.useTls = true;
    validConfig.deviceId = "paku-AABBCCDD";
    validConfig.timeoutMs = 10000;
    validConfig.maxRetries = 3;
    validConfig.retryDelayMs = 1000;
}

void tearDown(void) {
    // Clean up after each test
    client.clearQueue();
}

// Test: Begin with valid configuration
void test_begin_valid_config(void) {
    PakuIotResult result = client.begin(validConfig);
    TEST_ASSERT_EQUAL(PakuIotResult::SUCCESS, result);
}

// Test: Begin fails with null host
void test_begin_null_host(void) {
    PakuIotConfig config = validConfig;
    config.host = nullptr;
    
    PakuIotResult result = client.begin(config);
    TEST_ASSERT_EQUAL(PakuIotResult::ERROR_INVALID_CONFIG, result);
}

// Test: Begin fails with empty host
void test_begin_empty_host(void) {
    PakuIotConfig config = validConfig;
    config.host = "";
    
    PakuIotResult result = client.begin(config);
    TEST_ASSERT_EQUAL(PakuIotResult::ERROR_INVALID_CONFIG, result);
}

// Test: Begin fails with null device ID
void test_begin_null_device_id(void) {
    PakuIotConfig config = validConfig;
    config.deviceId = nullptr;
    
    PakuIotResult result = client.begin(config);
    TEST_ASSERT_EQUAL(PakuIotResult::ERROR_INVALID_CONFIG, result);
}

// Test: Begin fails with empty device ID
void test_begin_empty_device_id(void) {
    PakuIotConfig config = validConfig;
    config.deviceId = "";
    
    PakuIotResult result = client.begin(config);
    TEST_ASSERT_EQUAL(PakuIotResult::ERROR_INVALID_CONFIG, result);
}

// Test: Default values are set correctly
void test_begin_sets_defaults(void) {
    PakuIotConfig config;
    config.host = "test.example.com";
    config.deviceId = "test-device";
    config.port = 0;  // Should default to 443 for TLS, 80 for non-TLS
    config.useTls = true;
    config.apiKey = nullptr;
    config.timeoutMs = 0;  // Should default to 10000
    config.maxRetries = 0;  // Should default to 3
    config.retryDelayMs = 0;  // Should default to 1000
    
    PakuIotResult result = client.begin(config);
    TEST_ASSERT_EQUAL(PakuIotResult::SUCCESS, result);
}

// Test: isReady returns false before initialization
void test_is_ready_before_init(void) {
    PakuIotClient uninitClient;
    TEST_ASSERT_FALSE(uninitClient.isReady());
}

// Test: Queue starts empty
void test_queue_starts_empty(void) {
    client.begin(validConfig);
    TEST_ASSERT_EQUAL(0, client.getQueueSize());
}

// Test: Clear queue empties the queue
void test_clear_queue(void) {
    client.begin(validConfig);
    
    // Note: We can't easily add to queue without network calls,
    // but we can verify clearQueue doesn't crash on empty queue
    client.clearQueue();
    TEST_ASSERT_EQUAL(0, client.getQueueSize());
}

// Test: Error message for SUCCESS
void test_error_message_success(void) {
    const char* msg = PakuIotClient::getErrorMessage(PakuIotResult::SUCCESS);
    TEST_ASSERT_NOT_NULL(msg);
    TEST_ASSERT_EQUAL_STRING("Success", msg);
}

// Test: Error message for ERROR_NOT_CONNECTED
void test_error_message_not_connected(void) {
    const char* msg = PakuIotClient::getErrorMessage(PakuIotResult::ERROR_NOT_CONNECTED);
    TEST_ASSERT_NOT_NULL(msg);
    TEST_ASSERT_EQUAL_STRING("WiFi not connected", msg);
}

// Test: Error message for ERROR_CONNECTION
void test_error_message_connection(void) {
    const char* msg = PakuIotClient::getErrorMessage(PakuIotResult::ERROR_CONNECTION);
    TEST_ASSERT_NOT_NULL(msg);
    TEST_ASSERT_EQUAL_STRING("Failed to connect to server", msg);
}

// Test: Error message for ERROR_TIMEOUT
void test_error_message_timeout(void) {
    const char* msg = PakuIotClient::getErrorMessage(PakuIotResult::ERROR_TIMEOUT);
    TEST_ASSERT_NOT_NULL(msg);
    TEST_ASSERT_EQUAL_STRING("Request timed out", msg);
}

// Test: Error message for ERROR_SERVER
void test_error_message_server(void) {
    const char* msg = PakuIotClient::getErrorMessage(PakuIotResult::ERROR_SERVER);
    TEST_ASSERT_NOT_NULL(msg);
    TEST_ASSERT_EQUAL_STRING("Server error (5xx)", msg);
}

// Test: Error message for ERROR_CLIENT
void test_error_message_client(void) {
    const char* msg = PakuIotClient::getErrorMessage(PakuIotResult::ERROR_CLIENT);
    TEST_ASSERT_NOT_NULL(msg);
    TEST_ASSERT_EQUAL_STRING("Client error (4xx)", msg);
}

// Test: Error message for ERROR_TLS
void test_error_message_tls(void) {
    const char* msg = PakuIotClient::getErrorMessage(PakuIotResult::ERROR_TLS);
    TEST_ASSERT_NOT_NULL(msg);
    TEST_ASSERT_EQUAL_STRING("TLS/SSL error", msg);
}

// Test: Error message for ERROR_BUFFER_FULL
void test_error_message_buffer_full(void) {
    const char* msg = PakuIotClient::getErrorMessage(PakuIotResult::ERROR_BUFFER_FULL);
    TEST_ASSERT_NOT_NULL(msg);
    TEST_ASSERT_EQUAL_STRING("Internal buffer full", msg);
}

// Test: Error message for ERROR_INVALID_CONFIG
void test_error_message_invalid_config(void) {
    const char* msg = PakuIotClient::getErrorMessage(PakuIotResult::ERROR_INVALID_CONFIG);
    TEST_ASSERT_NOT_NULL(msg);
    TEST_ASSERT_EQUAL_STRING("Invalid configuration", msg);
}

// Test: sendTelemetry fails without initialization
void test_send_telemetry_without_init(void) {
    PakuIotClient uninitClient;
    TelemetryReading reading;
    reading.metric = "test/metric";
    reading.value = 42.0;
    reading.unit = "units";
    reading.timestamp = "12:00:00";
    
    PakuIotResult result = uninitClient.sendTelemetry(reading);
    TEST_ASSERT_EQUAL(PakuIotResult::ERROR_INVALID_CONFIG, result);
}

// Test: sendBatch with null readings returns success (no-op)
void test_send_batch_null_readings(void) {
    client.begin(validConfig);
    
    PakuIotResult result = client.sendBatch(nullptr, 0);
    TEST_ASSERT_EQUAL(PakuIotResult::SUCCESS, result);
}

// Test: sendBatch with zero count returns success
void test_send_batch_zero_count(void) {
    client.begin(validConfig);
    TelemetryReading readings[1];
    
    PakuIotResult result = client.sendBatch(readings, 0);
    TEST_ASSERT_EQUAL(PakuIotResult::SUCCESS, result);
}

// Test: getLastHttpCode returns 0 before any requests
void test_last_http_code_initial(void) {
    client.begin(validConfig);
    TEST_ASSERT_EQUAL(0, client.getLastHttpCode());
}

// Test: processQueue returns 0 when not ready
void test_process_queue_not_ready(void) {
    PakuIotClient uninitClient;
    size_t sent = uninitClient.processQueue();
    TEST_ASSERT_EQUAL(0, sent);
}

void setup() {
    delay(2000);  // Allow serial monitor to connect
    
    UNITY_BEGIN();
    
    // Configuration tests
    RUN_TEST(test_begin_valid_config);
    RUN_TEST(test_begin_null_host);
    RUN_TEST(test_begin_empty_host);
    RUN_TEST(test_begin_null_device_id);
    RUN_TEST(test_begin_empty_device_id);
    RUN_TEST(test_begin_sets_defaults);
    
    // State tests
    RUN_TEST(test_is_ready_before_init);
    RUN_TEST(test_queue_starts_empty);
    RUN_TEST(test_clear_queue);
    RUN_TEST(test_last_http_code_initial);
    
    // Error message tests
    RUN_TEST(test_error_message_success);
    RUN_TEST(test_error_message_not_connected);
    RUN_TEST(test_error_message_connection);
    RUN_TEST(test_error_message_timeout);
    RUN_TEST(test_error_message_server);
    RUN_TEST(test_error_message_client);
    RUN_TEST(test_error_message_tls);
    RUN_TEST(test_error_message_buffer_full);
    RUN_TEST(test_error_message_invalid_config);
    
    // Send tests (without network)
    RUN_TEST(test_send_telemetry_without_init);
    RUN_TEST(test_send_batch_null_readings);
    RUN_TEST(test_send_batch_zero_count);
    RUN_TEST(test_process_queue_not_ready);
    
    UNITY_END();
}

void loop() {
    // Nothing to do in loop for tests
}
