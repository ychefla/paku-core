/**
 * @file PakuIotClient.h
 * @brief HTTP client adapter for sending telemetry data to paku-iot
 * 
 * This library provides an HTTP-based transport for sending sensor
 * readings and telemetry data from paku-core (EDGE device) to 
 * paku-iot (host-side services).
 */

#ifndef PAKU_IOT_CLIENT_H
#define PAKU_IOT_CLIENT_H

#include <Arduino.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

/**
 * @brief Result codes for PakuIotClient operations
 */
enum class PakuIotResult {
    SUCCESS = 0,           ///< Operation completed successfully
    ERROR_NOT_CONNECTED,   ///< WiFi not connected
    ERROR_CONNECTION,      ///< Failed to connect to server
    ERROR_TIMEOUT,         ///< Request timed out
    ERROR_SERVER,          ///< Server returned error (5xx)
    ERROR_CLIENT,          ///< Client error (4xx)
    ERROR_TLS,             ///< TLS/SSL error
    ERROR_BUFFER_FULL,     ///< Internal buffer is full
    ERROR_INVALID_CONFIG   ///< Invalid configuration
};

/**
 * @brief Configuration for PakuIotClient
 */
struct PakuIotConfig {
    const char* host;           ///< Server hostname
    uint16_t port;              ///< Server port (default: 443 for HTTPS, 80 for HTTP)
    const char* apiKey;         ///< API key for authentication
    bool useTls;                ///< Whether to use HTTPS (default: true)
    const char* deviceId;       ///< Device identifier
    uint16_t timeoutMs;         ///< Request timeout in milliseconds (default: 10000)
    uint8_t maxRetries;         ///< Maximum retry attempts (default: 3)
    uint16_t retryDelayMs;      ///< Initial retry delay in milliseconds (default: 1000)
};

/**
 * @brief Telemetry reading structure
 */
struct TelemetryReading {
    const char* metric;         ///< Metric path (e.g., "temperature/cabin")
    float value;                ///< Sensor reading value
    const char* unit;           ///< Unit of measurement (optional)
    const char* timestamp;      ///< ISO 8601 timestamp
};

/**
 * @brief Queued message for retry
 */
struct QueuedMessage {
    char metric[64];
    float value;
    char unit[16];
    char timestamp[32];
    bool valid;
};

/**
 * @brief HTTP client adapter for paku-iot integration
 * 
 * This class provides methods to send telemetry data to paku-iot
 * via HTTP/HTTPS with support for retry logic and message queuing.
 */
class PakuIotClient {
public:
    /**
     * @brief Construct a new PakuIotClient
     */
    PakuIotClient();

    /**
     * @brief Destroy the PakuIotClient
     */
    ~PakuIotClient();

    /**
     * @brief Initialize the client with configuration
     * 
     * @param config Client configuration
     * @return PakuIotResult SUCCESS if initialized successfully
     */
    PakuIotResult begin(const PakuIotConfig& config);

    /**
     * @brief Check if client is configured and ready
     * 
     * @return true if client is ready to send data
     */
    bool isReady() const;

    /**
     * @brief Send a single telemetry reading
     * 
     * @param reading Telemetry reading to send
     * @return PakuIotResult Result of the operation
     */
    PakuIotResult sendTelemetry(const TelemetryReading& reading);

    /**
     * @brief Send multiple telemetry readings in a batch
     * 
     * @param readings Array of telemetry readings
     * @param count Number of readings in the array
     * @return PakuIotResult Result of the operation
     */
    PakuIotResult sendBatch(const TelemetryReading* readings, size_t count);

    /**
     * @brief Process queued messages that failed to send
     * 
     * Call this periodically to retry sending failed messages.
     * 
     * @return Number of messages successfully sent from queue
     */
    size_t processQueue();

    /**
     * @brief Get the number of queued messages
     * 
     * @return Number of messages waiting to be sent
     */
    size_t getQueueSize() const;

    /**
     * @brief Clear all queued messages
     */
    void clearQueue();

    /**
     * @brief Get the last HTTP response code
     * 
     * @return HTTP response code from last request
     */
    int getLastHttpCode() const;

    /**
     * @brief Get a human-readable error message
     * 
     * @param result Result code
     * @return Error message string
     */
    static const char* getErrorMessage(PakuIotResult result);

private:
    static const size_t MAX_QUEUE_SIZE = 50;
    static const size_t JSON_BUFFER_SIZE = 512;

    PakuIotConfig _config;
    bool _initialized;
    int _lastHttpCode;
    QueuedMessage _queue[MAX_QUEUE_SIZE];
    size_t _queueHead;
    size_t _queueTail;
    size_t _queueCount;

    /**
     * @brief Build the API endpoint URL
     * 
     * @return Full URL string
     */
    String buildUrl() const;

    /**
     * @brief Send HTTP POST request with JSON payload
     * 
     * @param jsonPayload JSON string to send
     * @return PakuIotResult Result of the operation
     */
    PakuIotResult sendRequest(const String& jsonPayload);

    /**
     * @brief Add a message to the retry queue
     * 
     * @param reading Reading to queue
     * @return true if added successfully
     */
    bool enqueue(const TelemetryReading& reading);

    /**
     * @brief Remove and return the oldest message from queue
     * 
     * @param reading Output reading structure
     * @return true if a message was dequeued
     */
    bool dequeue(TelemetryReading& reading);

    /**
     * @brief Convert result to HTTP-like status
     * 
     * @param httpCode HTTP response code
     * @return PakuIotResult Corresponding result code
     */
    PakuIotResult httpCodeToResult(int httpCode);
};

#endif // PAKU_IOT_CLIENT_H
