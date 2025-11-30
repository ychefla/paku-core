/**
 * @file data_transmitter.h
 * @brief Data transmission handler for paku-iot
 * 
 * This module provides functions for packaging and transmitting
 * sensor data (from RuuviTags, flow sensors, and placeholders)
 * to the paku-iot backend via MQTT or HTTP.
 */
#pragma once

#include <cstdint>
#include <cstddef>
#include "ruuvi.h"
#include "ruuvi_scanner.h"

/**
 * @brief Maximum number of readings in a transmission batch
 */
#define MAX_TRANSMISSION_BATCH 20

/**
 * @brief Sensor data source type
 */
enum class SensorSourceType {
    RUUVI_TAG,          // RuuviTag BLE sensor
    FLOW_SENSOR,        // Flow rate sensor
    VOLTAGE_SENSOR,     // Voltage measurement
    PLACEHOLDER,        // Placeholder/simulated data
    EXTERNAL            // External sensor (future)
};

/**
 * @brief Structure for a single sensor reading to transmit
 */
struct TransmitReading {
    const char* topic;          // MQTT topic (e.g., "temperature/cabin")
    float value;                // Sensor value
    const char* unit;           // Unit of measurement
    const char* timestamp;      // Timestamp string
    const char* location;       // Location identifier
    SensorSourceType source;    // Data source type
    bool isHistorical;          // Whether this is historical data
    bool valid;                 // Whether reading is valid
};

/**
 * @brief Transmission batch container
 */
struct TransmitBatch {
    TransmitReading readings[MAX_TRANSMISSION_BATCH];
    uint8_t count;              // Number of readings in batch
    const char* deviceId;       // Device identifier
    unsigned long batchTime;    // Batch creation timestamp
};

/**
 * @brief Initializes a new transmission batch
 * 
 * @param batch Pointer to batch to initialize
 * @param deviceId Device identifier
 * @param currentTime Current timestamp
 */
void initTransmitBatch(TransmitBatch* batch, const char* deviceId, unsigned long currentTime);

/**
 * @brief Adds a Ruuvi reading to the transmission batch
 * 
 * @param batch Pointer to batch
 * @param tag Pointer to RuuviTag with data
 * @param timestamp Timestamp string
 * @return true if added successfully
 */
bool addRuuviToTransmitBatch(TransmitBatch* batch, const RuuviTag* tag, 
                              const char* timestamp);

/**
 * @brief Adds a flow reading to the transmission batch
 * 
 * @param batch Pointer to batch
 * @param flowRate Flow rate value (L/min)
 * @param frequency Pulse frequency (Hz)
 * @param requiredDeltaT Required temperature delta
 * @param timestamp Timestamp string
 * @return true if added successfully
 */
bool addFlowToTransmitBatch(TransmitBatch* batch, float flowRate, float frequency,
                             float requiredDeltaT, const char* timestamp);

/**
 * @brief Adds a voltage reading to the transmission batch
 * 
 * @param batch Pointer to batch
 * @param source Voltage source (e.g., "car", "leisure")
 * @param voltage Voltage value
 * @param timestamp Timestamp string
 * @return true if added successfully
 */
bool addVoltageToTransmitBatch(TransmitBatch* batch, const char* source, 
                                float voltage, const char* timestamp);

/**
 * @brief Adds a status reading to the transmission batch
 * 
 * @param batch Pointer to batch
 * @param component Component name (e.g., "heater", "pump")
 * @param status Status value (0/1)
 * @param timestamp Timestamp string
 * @return true if added successfully
 */
bool addStatusToTransmitBatch(TransmitBatch* batch, const char* component,
                               int status, const char* timestamp);

/**
 * @brief Adds a placeholder reading to the transmission batch
 * 
 * @param batch Pointer to batch
 * @param topic Topic path
 * @param value Placeholder value
 * @param unit Unit of measurement
 * @param timestamp Timestamp string
 * @return true if added successfully
 */
bool addPlaceholderToTransmitBatch(TransmitBatch* batch, const char* topic,
                                    float value, const char* unit, 
                                    const char* timestamp);

/**
 * @brief Adds a generic reading to the transmission batch
 * 
 * @param batch Pointer to batch
 * @param reading Reading to add
 * @return true if added successfully
 */
bool addReadingToTransmitBatch(TransmitBatch* batch, const TransmitReading* reading);

/**
 * @brief Gets number of readings in batch
 * 
 * @param batch Pointer to batch
 * @return Number of readings
 */
uint8_t getTransmitBatchCount(const TransmitBatch* batch);

/**
 * @brief Checks if batch is full
 * 
 * @param batch Pointer to batch
 * @return true if batch is full
 */
bool isTransmitBatchFull(const TransmitBatch* batch);

/**
 * @brief Clears all readings from batch
 * 
 * @param batch Pointer to batch
 */
void clearTransmitBatch(TransmitBatch* batch);

/**
 * @brief Builds MQTT topic string for a reading
 * 
 * @param deviceId Device identifier
 * @param readingTopic Reading topic path (e.g., "temperature/cabin")
 * @param buffer Output buffer
 * @param bufferSize Size of output buffer
 * @return Pointer to buffer, or nullptr on error
 */
const char* buildMqttTopic(const char* deviceId, const char* readingTopic,
                           char* buffer, size_t bufferSize);

/**
 * @brief Structure for historical data batch
 */
struct HistoricalBatch {
    TransmitReading readings[MAX_TRANSMISSION_BATCH];
    uint8_t count;
    const char* sourceTag;      // Source tag identifier
    unsigned long startTime;    // Start of historical period
    unsigned long endTime;      // End of historical period
};

/**
 * @brief Initializes a historical data batch
 * 
 * @param batch Pointer to batch
 * @param sourceTag Source tag identifier
 */
void initHistoricalBatch(HistoricalBatch* batch, const char* sourceTag);

/**
 * @brief Adds historical reading to batch
 * 
 * @param batch Pointer to batch
 * @param entry Historical entry
 * @param topic Topic path
 * @param timestamp Timestamp string
 * @return true if added successfully
 */
bool addHistoricalReading(HistoricalBatch* batch, const RuuviHistoryEntry* entry,
                          const char* topic, const char* timestamp);
