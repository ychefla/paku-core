/**
 * @file test_data_transmitter.cpp
 * @brief Unit tests for data transmitter module
 */
#include <unity.h>
#include "data_transmitter.h"
#include "ruuvi_scanner.h"
#include "ruuvi.h"

void setUp(void) {}
void tearDown(void) {}

/**
 * Test batch initialization
 */
void test_initTransmitBatch(void) {
    TransmitBatch batch;
    initTransmitBatch(&batch, "paku-AABBCC", 12345);
    
    TEST_ASSERT_EQUAL_UINT8(0, batch.count);
    TEST_ASSERT_EQUAL_STRING("paku-AABBCC", batch.deviceId);
    TEST_ASSERT_EQUAL_UINT32(12345, batch.batchTime);
}

/**
 * Test adding reading to batch
 */
void test_addReadingToTransmitBatch(void) {
    TransmitBatch batch;
    initTransmitBatch(&batch, "paku-AABBCC", 1000);
    
    TransmitReading reading;
    reading.topic = "temperature/cabin";
    reading.value = 22.5f;
    reading.unit = "celsius";
    reading.timestamp = "12:00:00";
    reading.location = "cabin";
    reading.source = SensorSourceType::RUUVI_TAG;
    reading.isHistorical = false;
    reading.valid = true;
    
    bool result = addReadingToTransmitBatch(&batch, &reading);
    
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL_UINT8(1, batch.count);
    TEST_ASSERT_EQUAL_STRING("temperature/cabin", batch.readings[0].topic);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 22.5f, batch.readings[0].value);
}

/**
 * Test batch full detection
 */
void test_isTransmitBatchFull(void) {
    TransmitBatch batch;
    initTransmitBatch(&batch, "paku-AABBCC", 1000);
    
    TEST_ASSERT_FALSE(isTransmitBatchFull(&batch));
    
    // Fill the batch
    TransmitReading reading;
    reading.topic = "test";
    reading.value = 1.0f;
    reading.unit = nullptr;
    reading.timestamp = "12:00:00";
    reading.location = nullptr;
    reading.source = SensorSourceType::PLACEHOLDER;
    reading.isHistorical = false;
    reading.valid = true;
    
    for (int i = 0; i < MAX_TRANSMISSION_BATCH; i++) {
        addReadingToTransmitBatch(&batch, &reading);
    }
    
    TEST_ASSERT_TRUE(isTransmitBatchFull(&batch));
    TEST_ASSERT_EQUAL_UINT8(MAX_TRANSMISSION_BATCH, batch.count);
    
    // Cannot add more
    bool result = addReadingToTransmitBatch(&batch, &reading);
    TEST_ASSERT_FALSE(result);
}

/**
 * Test adding flow data to batch
 */
void test_addFlowToTransmitBatch(void) {
    TransmitBatch batch;
    initTransmitBatch(&batch, "paku-AABBCC", 1000);
    
    bool result = addFlowToTransmitBatch(&batch, 100.0f, 66.0f, 2.5f, "12:00:00");
    
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL_UINT8(3, batch.count);  // flow, frequency, deltaT
    
    // Verify flow rate reading
    TEST_ASSERT_EQUAL_STRING("flow/coolant", batch.readings[0].topic);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 100.0f, batch.readings[0].value);
    
    // Verify frequency reading
    TEST_ASSERT_EQUAL_STRING("flow/coolant_frequency", batch.readings[1].topic);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 66.0f, batch.readings[1].value);
    
    // Verify delta T reading
    TEST_ASSERT_EQUAL_STRING("temperature/heating/required_dt", batch.readings[2].topic);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 2.5f, batch.readings[2].value);
}

/**
 * Test adding voltage data
 */
void test_addVoltageToTransmitBatch(void) {
    TransmitBatch batch;
    initTransmitBatch(&batch, "paku-AABBCC", 1000);
    
    bool result = addVoltageToTransmitBatch(&batch, "car", 12.6f, "12:00:00");
    
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL_UINT8(1, batch.count);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 12.6f, batch.readings[0].value);
}

/**
 * Test adding status data
 */
void test_addStatusToTransmitBatch(void) {
    TransmitBatch batch;
    initTransmitBatch(&batch, "paku-AABBCC", 1000);
    
    bool result = addStatusToTransmitBatch(&batch, "heater", 1, "12:00:00");
    
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL_UINT8(1, batch.count);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.0f, batch.readings[0].value);
}

/**
 * Test adding placeholder data
 */
void test_addPlaceholderToTransmitBatch(void) {
    TransmitBatch batch;
    initTransmitBatch(&batch, "paku-AABBCC", 1000);
    
    bool result = addPlaceholderToTransmitBatch(&batch, "power/heat", -1000.0f, "W", "12:00:00");
    
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL_UINT8(1, batch.count);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, -1000.0f, batch.readings[0].value);
    TEST_ASSERT_TRUE(batch.readings[0].source == SensorSourceType::PLACEHOLDER);
}

/**
 * Test clearing batch
 */
void test_clearTransmitBatch(void) {
    TransmitBatch batch;
    initTransmitBatch(&batch, "paku-AABBCC", 1000);
    
    addFlowToTransmitBatch(&batch, 100.0f, 66.0f, 2.5f, "12:00:00");
    TEST_ASSERT_EQUAL_UINT8(3, batch.count);
    
    clearTransmitBatch(&batch);
    
    TEST_ASSERT_EQUAL_UINT8(0, batch.count);
}

/**
 * Test building MQTT topic
 */
void test_buildMqttTopic(void) {
    char buffer[128];
    
    const char* topic = buildMqttTopic("paku-AABBCC", "temperature/cabin", buffer, sizeof(buffer));
    
    TEST_ASSERT_NOT_NULL(topic);
    TEST_ASSERT_EQUAL_STRING("paku/devices/paku-AABBCC/telemetry/temperature/cabin", topic);
}

/**
 * Test building MQTT topic with small buffer
 */
void test_buildMqttTopic_smallBuffer(void) {
    char buffer[10];
    
    const char* topic = buildMqttTopic("paku-AABBCC", "temperature/cabin", buffer, sizeof(buffer));
    
    TEST_ASSERT_NULL(topic);
}

/**
 * Test historical batch initialization
 */
void test_initHistoricalBatch(void) {
    HistoricalBatch batch;
    initHistoricalBatch(&batch, "cabin_tag");
    
    TEST_ASSERT_EQUAL_UINT8(0, batch.count);
    TEST_ASSERT_EQUAL_STRING("cabin_tag", batch.sourceTag);
}

/**
 * Test adding historical reading
 */
void test_addHistoricalReading(void) {
    HistoricalBatch batch;
    initHistoricalBatch(&batch, "cabin_tag");
    
    RuuviHistoryEntry entry;
    entry.data.temperature = 20.5f;
    entry.timestamp = 1000;
    entry.valid = true;
    
    bool result = addHistoricalReading(&batch, &entry, "temperature/cabin", "2025-01-15T10:00:00Z");
    
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL_UINT8(1, batch.count);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 20.5f, batch.readings[0].value);
    TEST_ASSERT_TRUE(batch.readings[0].isHistorical);
}

/**
 * Test get batch count
 */
void test_getTransmitBatchCount(void) {
    TransmitBatch batch;
    initTransmitBatch(&batch, "paku-AABBCC", 1000);
    
    TEST_ASSERT_EQUAL_UINT8(0, getTransmitBatchCount(&batch));
    
    addFlowToTransmitBatch(&batch, 100.0f, 66.0f, 2.5f, "12:00:00");
    
    TEST_ASSERT_EQUAL_UINT8(3, getTransmitBatchCount(&batch));
}

/**
 * Test null handling
 */
void test_nullHandling(void) {
    TEST_ASSERT_FALSE(addReadingToTransmitBatch(nullptr, nullptr));
    TEST_ASSERT_FALSE(addFlowToTransmitBatch(nullptr, 0, 0, 0, nullptr));
    TEST_ASSERT_FALSE(addVoltageToTransmitBatch(nullptr, nullptr, 0, nullptr));
    TEST_ASSERT_TRUE(isTransmitBatchFull(nullptr));
    TEST_ASSERT_EQUAL_UINT8(0, getTransmitBatchCount(nullptr));
    TEST_ASSERT_NULL(buildMqttTopic(nullptr, nullptr, nullptr, 0));
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    
    // Batch initialization
    RUN_TEST(test_initTransmitBatch);
    RUN_TEST(test_clearTransmitBatch);
    
    // Reading addition
    RUN_TEST(test_addReadingToTransmitBatch);
    RUN_TEST(test_addFlowToTransmitBatch);
    RUN_TEST(test_addVoltageToTransmitBatch);
    RUN_TEST(test_addStatusToTransmitBatch);
    RUN_TEST(test_addPlaceholderToTransmitBatch);
    
    // Batch state
    RUN_TEST(test_isTransmitBatchFull);
    RUN_TEST(test_getTransmitBatchCount);
    
    // Topic building
    RUN_TEST(test_buildMqttTopic);
    RUN_TEST(test_buildMqttTopic_smallBuffer);
    
    // Historical data
    RUN_TEST(test_initHistoricalBatch);
    RUN_TEST(test_addHistoricalReading);
    
    // Error handling
    RUN_TEST(test_nullHandling);
    
    return UNITY_END();
}
