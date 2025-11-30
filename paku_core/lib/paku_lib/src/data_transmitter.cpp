/**
 * @file data_transmitter.cpp
 * @brief Data transmission handler implementation
 * 
 * @note Thread Safety: This module uses static buffers for topic strings
 *       to avoid dynamic memory allocation on ESP32. These functions should
 *       only be called from a single thread context (typically the main loop).
 *       The BLE scanner task does NOT call these functions directly.
 */
#include "data_transmitter.h"
#include <cstring>
#include <cstdio>

// Static buffers for topic strings (avoids dynamic allocation)
// Note: Safe for single-threaded use from main loop only
static char topicBuffer[128];
static char tempTopicBuffer[64];
static char humidTopicBuffer[64];

void initTransmitBatch(TransmitBatch* batch, const char* deviceId, unsigned long currentTime) {
    if (batch == nullptr) {
        return;
    }
    
    memset(batch->readings, 0, sizeof(batch->readings));
    batch->count = 0;
    batch->deviceId = deviceId;
    batch->batchTime = currentTime;
}

bool addReadingToTransmitBatch(TransmitBatch* batch, const TransmitReading* reading) {
    if (batch == nullptr || reading == nullptr) {
        return false;
    }
    
    if (batch->count >= MAX_TRANSMISSION_BATCH) {
        return false;
    }
    
    batch->readings[batch->count] = *reading;
    batch->readings[batch->count].valid = true;
    batch->count++;
    
    return true;
}

bool addRuuviToTransmitBatch(TransmitBatch* batch, const RuuviTag* tag, 
                              const char* timestamp) {
    if (batch == nullptr || tag == nullptr || !tag->hasData) {
        return false;
    }
    
    // Add temperature
    TransmitReading tempReading;
    snprintf(tempTopicBuffer, sizeof(tempTopicBuffer), "temperature/ruuvi/%s", tag->location);
    tempReading.topic = tempTopicBuffer;
    tempReading.value = tag->lastData.temperature;
    tempReading.unit = "celsius";
    tempReading.timestamp = timestamp;
    tempReading.location = tag->location;
    tempReading.source = SensorSourceType::RUUVI_TAG;
    tempReading.isHistorical = false;
    tempReading.valid = true;
    
    if (!addReadingToTransmitBatch(batch, &tempReading)) {
        return false;
    }
    
    // Add humidity
    TransmitReading humidReading;
    snprintf(humidTopicBuffer, sizeof(humidTopicBuffer), "humidity/ruuvi/%s", tag->location);
    humidReading.topic = humidTopicBuffer;
    humidReading.value = tag->lastData.humidity;
    humidReading.unit = "percent";
    humidReading.timestamp = timestamp;
    humidReading.location = tag->location;
    humidReading.source = SensorSourceType::RUUVI_TAG;
    humidReading.isHistorical = false;
    humidReading.valid = true;
    
    if (!addReadingToTransmitBatch(batch, &humidReading)) {
        return false;
    }
    
    // Add pressure if valid
    if (tag->lastData.pressure > 0) {
        TransmitReading pressReading;
        static char pressTopicBuffer[64];
        snprintf(pressTopicBuffer, sizeof(pressTopicBuffer), "pressure/ruuvi/%s", tag->location);
        pressReading.topic = pressTopicBuffer;
        pressReading.value = tag->lastData.pressure / 100.0f;  // Convert Pa to hPa
        pressReading.unit = "hPa";
        pressReading.timestamp = timestamp;
        pressReading.location = tag->location;
        pressReading.source = SensorSourceType::RUUVI_TAG;
        pressReading.isHistorical = false;
        pressReading.valid = true;
        
        addReadingToTransmitBatch(batch, &pressReading);
    }
    
    // Add battery voltage if valid
    if (tag->lastData.batteryVoltage > 0) {
        TransmitReading battReading;
        static char battTopicBuffer[64];
        snprintf(battTopicBuffer, sizeof(battTopicBuffer), "voltage/ruuvi/%s", tag->location);
        battReading.topic = battTopicBuffer;
        battReading.value = tag->lastData.batteryVoltage;
        battReading.unit = "V";
        battReading.timestamp = timestamp;
        battReading.location = tag->location;
        battReading.source = SensorSourceType::RUUVI_TAG;
        battReading.isHistorical = false;
        battReading.valid = true;
        
        addReadingToTransmitBatch(batch, &battReading);
    }
    
    return true;
}

bool addFlowToTransmitBatch(TransmitBatch* batch, float flowRate, float frequency,
                             float requiredDeltaT, const char* timestamp) {
    if (batch == nullptr) {
        return false;
    }
    
    // Add flow rate
    TransmitReading flowReading;
    flowReading.topic = "flow/coolant";
    flowReading.value = flowRate;
    flowReading.unit = "l_per_min";
    flowReading.timestamp = timestamp;
    flowReading.location = nullptr;
    flowReading.source = SensorSourceType::FLOW_SENSOR;
    flowReading.isHistorical = false;
    flowReading.valid = true;
    
    if (!addReadingToTransmitBatch(batch, &flowReading)) {
        return false;
    }
    
    // Add frequency
    TransmitReading freqReading;
    freqReading.topic = "flow/coolant_frequency";
    freqReading.value = frequency;
    freqReading.unit = "Hz";
    freqReading.timestamp = timestamp;
    freqReading.location = nullptr;
    freqReading.source = SensorSourceType::FLOW_SENSOR;
    freqReading.isHistorical = false;
    freqReading.valid = true;
    
    if (!addReadingToTransmitBatch(batch, &freqReading)) {
        return false;
    }
    
    // Add required delta T
    TransmitReading deltaReading;
    deltaReading.topic = "temperature/heating/required_dt";
    deltaReading.value = requiredDeltaT;
    deltaReading.unit = "celsius";
    deltaReading.timestamp = timestamp;
    deltaReading.location = nullptr;
    deltaReading.source = SensorSourceType::FLOW_SENSOR;
    deltaReading.isHistorical = false;
    deltaReading.valid = true;
    
    return addReadingToTransmitBatch(batch, &deltaReading);
}

bool addVoltageToTransmitBatch(TransmitBatch* batch, const char* source, 
                                float voltage, const char* timestamp) {
    if (batch == nullptr || source == nullptr) {
        return false;
    }
    
    TransmitReading reading;
    static char voltTopicBuffer[64];
    snprintf(voltTopicBuffer, sizeof(voltTopicBuffer), "voltage/%s", source);
    reading.topic = voltTopicBuffer;
    reading.value = voltage;
    reading.unit = "V";
    reading.timestamp = timestamp;
    reading.location = nullptr;
    reading.source = SensorSourceType::VOLTAGE_SENSOR;
    reading.isHistorical = false;
    reading.valid = true;
    
    return addReadingToTransmitBatch(batch, &reading);
}

bool addStatusToTransmitBatch(TransmitBatch* batch, const char* component,
                               int status, const char* timestamp) {
    if (batch == nullptr || component == nullptr) {
        return false;
    }
    
    TransmitReading reading;
    static char statusTopicBuffer[64];
    snprintf(statusTopicBuffer, sizeof(statusTopicBuffer), "status/%s", component);
    reading.topic = statusTopicBuffer;
    reading.value = static_cast<float>(status);
    reading.unit = nullptr;
    reading.timestamp = timestamp;
    reading.location = nullptr;
    reading.source = SensorSourceType::EXTERNAL;
    reading.isHistorical = false;
    reading.valid = true;
    
    return addReadingToTransmitBatch(batch, &reading);
}

bool addPlaceholderToTransmitBatch(TransmitBatch* batch, const char* topic,
                                    float value, const char* unit, 
                                    const char* timestamp) {
    if (batch == nullptr || topic == nullptr) {
        return false;
    }
    
    TransmitReading reading;
    reading.topic = topic;
    reading.value = value;
    reading.unit = unit;
    reading.timestamp = timestamp;
    reading.location = nullptr;
    reading.source = SensorSourceType::PLACEHOLDER;
    reading.isHistorical = false;
    reading.valid = true;
    
    return addReadingToTransmitBatch(batch, &reading);
}

uint8_t getTransmitBatchCount(const TransmitBatch* batch) {
    if (batch == nullptr) {
        return 0;
    }
    return batch->count;
}

bool isTransmitBatchFull(const TransmitBatch* batch) {
    if (batch == nullptr) {
        return true;
    }
    return batch->count >= MAX_TRANSMISSION_BATCH;
}

void clearTransmitBatch(TransmitBatch* batch) {
    if (batch == nullptr) {
        return;
    }
    
    memset(batch->readings, 0, sizeof(batch->readings));
    batch->count = 0;
}

const char* buildMqttTopic(const char* deviceId, const char* readingTopic,
                           char* buffer, size_t bufferSize) {
    if (deviceId == nullptr || readingTopic == nullptr || 
        buffer == nullptr || bufferSize == 0) {
        return nullptr;
    }
    
    // Format: paku/devices/{device_id}/telemetry/{topic}
    int written = snprintf(buffer, bufferSize, 
                           "paku/devices/%s/telemetry/%s", 
                           deviceId, readingTopic);
    
    if (written < 0 || static_cast<size_t>(written) >= bufferSize) {
        return nullptr;
    }
    
    return buffer;
}

void initHistoricalBatch(HistoricalBatch* batch, const char* sourceTag) {
    if (batch == nullptr) {
        return;
    }
    
    memset(batch->readings, 0, sizeof(batch->readings));
    batch->count = 0;
    batch->sourceTag = sourceTag;
    batch->startTime = 0;
    batch->endTime = 0;
}

bool addHistoricalReading(HistoricalBatch* batch, const RuuviHistoryEntry* entry,
                          const char* topic, const char* timestamp) {
    if (batch == nullptr || entry == nullptr || !entry->valid || topic == nullptr) {
        return false;
    }
    
    if (batch->count >= MAX_TRANSMISSION_BATCH) {
        return false;
    }
    
    TransmitReading reading;
    reading.topic = topic;
    reading.value = entry->data.temperature;  // Could be other fields
    reading.unit = "celsius";
    reading.timestamp = timestamp;
    reading.location = batch->sourceTag;
    reading.source = SensorSourceType::RUUVI_TAG;
    reading.isHistorical = true;
    reading.valid = true;
    
    batch->readings[batch->count] = reading;
    batch->count++;
    
    // Update time range
    if (entry->timestamp < batch->startTime || batch->count == 1) {
        batch->startTime = entry->timestamp;
    }
    if (entry->timestamp > batch->endTime) {
        batch->endTime = entry->timestamp;
    }
    
    return true;
}
