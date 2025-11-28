/**
 * @file PakuIotClient.cpp
 * @brief Implementation of HTTP client adapter for paku-iot
 */

#include "PakuIotClient.h"

PakuIotClient::PakuIotClient()
    : _initialized(false)
    , _lastHttpCode(0)
    , _queueHead(0)
    , _queueTail(0)
    , _queueCount(0)
{
    memset(&_config, 0, sizeof(_config));
    for (size_t i = 0; i < MAX_QUEUE_SIZE; i++) {
        _queue[i].valid = false;
    }
}

PakuIotClient::~PakuIotClient() {
    // Nothing to clean up
}

PakuIotResult PakuIotClient::begin(const PakuIotConfig& config) {
    // Validate required configuration
    if (config.host == nullptr || strlen(config.host) == 0) {
        return PakuIotResult::ERROR_INVALID_CONFIG;
    }
    if (config.deviceId == nullptr || strlen(config.deviceId) == 0) {
        return PakuIotResult::ERROR_INVALID_CONFIG;
    }

    _config = config;

    // Set defaults if not specified
    if (_config.port == 0) {
        _config.port = _config.useTls ? 443 : 80;
    }
    if (_config.timeoutMs == 0) {
        _config.timeoutMs = 10000;
    }
    if (_config.maxRetries == 0) {
        _config.maxRetries = 3;
    }
    if (_config.retryDelayMs == 0) {
        _config.retryDelayMs = 1000;
    }

    _initialized = true;
    return PakuIotResult::SUCCESS;
}

bool PakuIotClient::isReady() const {
    return _initialized && WiFi.status() == WL_CONNECTED;
}

PakuIotResult PakuIotClient::sendTelemetry(const TelemetryReading& reading) {
    if (!_initialized) {
        return PakuIotResult::ERROR_INVALID_CONFIG;
    }
    if (WiFi.status() != WL_CONNECTED) {
        // Queue for later if not connected
        enqueue(reading);
        return PakuIotResult::ERROR_NOT_CONNECTED;
    }

    // Build JSON payload
    JsonDocument doc;
    doc["device_id"] = _config.deviceId;
    doc["timestamp"] = reading.timestamp;
    doc["metric"] = reading.metric;
    doc["value"] = reading.value;
    if (reading.unit != nullptr && strlen(reading.unit) > 0) {
        doc["unit"] = reading.unit;
    }

    String jsonPayload;
    serializeJson(doc, jsonPayload);

    // Send with retry logic
    PakuIotResult result = PakuIotResult::ERROR_CONNECTION;
    uint16_t retryDelay = _config.retryDelayMs;

    for (uint8_t attempt = 0; attempt < _config.maxRetries; attempt++) {
        result = sendRequest(jsonPayload);

        if (result == PakuIotResult::SUCCESS) {
            return result;
        }

        // Don't retry client errors (4xx)
        if (result == PakuIotResult::ERROR_CLIENT) {
            return result;
        }

        // Exponential backoff for retryable errors
        if (attempt < _config.maxRetries - 1) {
            delay(retryDelay);
            retryDelay *= 2;
        }
    }

    // Failed after all retries, queue for later
    enqueue(reading);
    return result;
}

PakuIotResult PakuIotClient::sendBatch(const TelemetryReading* readings, size_t count) {
    if (!_initialized) {
        return PakuIotResult::ERROR_INVALID_CONFIG;
    }
    if (readings == nullptr || count == 0) {
        return PakuIotResult::SUCCESS;
    }
    if (WiFi.status() != WL_CONNECTED) {
        // Queue all readings for later
        for (size_t i = 0; i < count; i++) {
            enqueue(readings[i]);
        }
        return PakuIotResult::ERROR_NOT_CONNECTED;
    }

    // Build batch JSON payload
    JsonDocument doc;
    doc["device_id"] = _config.deviceId;
    doc["batch_timestamp"] = readings[0].timestamp;

    JsonArray readingsArray = doc["readings"].to<JsonArray>();
    for (size_t i = 0; i < count; i++) {
        JsonObject reading = readingsArray.add<JsonObject>();
        reading["metric"] = readings[i].metric;
        reading["value"] = readings[i].value;
        reading["timestamp"] = readings[i].timestamp;
        if (readings[i].unit != nullptr && strlen(readings[i].unit) > 0) {
            reading["unit"] = readings[i].unit;
        }
    }

    String jsonPayload;
    serializeJson(doc, jsonPayload);

    // Send with retry logic
    PakuIotResult result = PakuIotResult::ERROR_CONNECTION;
    uint16_t retryDelay = _config.retryDelayMs;

    for (uint8_t attempt = 0; attempt < _config.maxRetries; attempt++) {
        result = sendRequest(jsonPayload);

        if (result == PakuIotResult::SUCCESS) {
            return result;
        }

        if (result == PakuIotResult::ERROR_CLIENT) {
            return result;
        }

        if (attempt < _config.maxRetries - 1) {
            delay(retryDelay);
            retryDelay *= 2;
        }
    }

    // Queue all readings on failure
    for (size_t i = 0; i < count; i++) {
        enqueue(readings[i]);
    }
    return result;
}

size_t PakuIotClient::processQueue() {
    if (!isReady()) {
        return 0;
    }

    size_t sent = 0;
    TelemetryReading reading;
    char metricBuf[64];
    char unitBuf[16];
    char timestampBuf[32];

    while (_queueCount > 0) {
        // Get the oldest message
        QueuedMessage& msg = _queue[_queueTail];
        if (!msg.valid) {
            _queueTail = (_queueTail + 1) % MAX_QUEUE_SIZE;
            _queueCount--;
            continue;
        }

        // Copy to temporary reading structure
        strncpy(metricBuf, msg.metric, sizeof(metricBuf) - 1);
        metricBuf[sizeof(metricBuf) - 1] = '\0';
        strncpy(unitBuf, msg.unit, sizeof(unitBuf) - 1);
        unitBuf[sizeof(unitBuf) - 1] = '\0';
        strncpy(timestampBuf, msg.timestamp, sizeof(timestampBuf) - 1);
        timestampBuf[sizeof(timestampBuf) - 1] = '\0';

        reading.metric = metricBuf;
        reading.value = msg.value;
        reading.unit = unitBuf;
        reading.timestamp = timestampBuf;

        // Try to send (without re-queuing on failure)
        JsonDocument doc;
        doc["device_id"] = _config.deviceId;
        doc["timestamp"] = reading.timestamp;
        doc["metric"] = reading.metric;
        doc["value"] = reading.value;
        if (strlen(reading.unit) > 0) {
            doc["unit"] = reading.unit;
        }

        String jsonPayload;
        serializeJson(doc, jsonPayload);

        PakuIotResult result = sendRequest(jsonPayload);
        if (result == PakuIotResult::SUCCESS || result == PakuIotResult::ERROR_CLIENT) {
            // Successfully sent or client error (don't retry)
            msg.valid = false;
            _queueTail = (_queueTail + 1) % MAX_QUEUE_SIZE;
            _queueCount--;
            if (result == PakuIotResult::SUCCESS) {
                sent++;
            }
        } else {
            // Stop processing on server/connection error
            break;
        }
    }

    return sent;
}

size_t PakuIotClient::getQueueSize() const {
    return _queueCount;
}

void PakuIotClient::clearQueue() {
    for (size_t i = 0; i < MAX_QUEUE_SIZE; i++) {
        _queue[i].valid = false;
    }
    _queueHead = 0;
    _queueTail = 0;
    _queueCount = 0;
}

int PakuIotClient::getLastHttpCode() const {
    return _lastHttpCode;
}

const char* PakuIotClient::getErrorMessage(PakuIotResult result) {
    switch (result) {
        case PakuIotResult::SUCCESS:
            return "Success";
        case PakuIotResult::ERROR_NOT_CONNECTED:
            return "WiFi not connected";
        case PakuIotResult::ERROR_CONNECTION:
            return "Failed to connect to server";
        case PakuIotResult::ERROR_TIMEOUT:
            return "Request timed out";
        case PakuIotResult::ERROR_SERVER:
            return "Server error (5xx)";
        case PakuIotResult::ERROR_CLIENT:
            return "Client error (4xx)";
        case PakuIotResult::ERROR_TLS:
            return "TLS/SSL error";
        case PakuIotResult::ERROR_BUFFER_FULL:
            return "Internal buffer full";
        case PakuIotResult::ERROR_INVALID_CONFIG:
            return "Invalid configuration";
        default:
            return "Unknown error";
    }
}

String PakuIotClient::buildUrl() const {
    String url;
    url.reserve(128);
    url = _config.useTls ? "https://" : "http://";
    url += _config.host;
    if ((_config.useTls && _config.port != 443) || 
        (!_config.useTls && _config.port != 80)) {
        url += ":";
        url += String(_config.port);
    }
    url += "/api/v1/telemetry";
    return url;
}

PakuIotResult PakuIotClient::sendRequest(const String& jsonPayload) {
    HTTPClient http;
    String url = buildUrl();

    // For HTTPS, we need to use WiFiClientSecure
    if (_config.useTls) {
        WiFiClientSecure* secureClient = new WiFiClientSecure();
        secureClient->setInsecure(); // Skip certificate verification for now
        
        if (!http.begin(*secureClient, url)) {
            delete secureClient;
            return PakuIotResult::ERROR_CONNECTION;
        }
    } else {
        WiFiClient* client = new WiFiClient();
        if (!http.begin(*client, url)) {
            delete client;
            return PakuIotResult::ERROR_CONNECTION;
        }
    }

    http.setTimeout(_config.timeoutMs);
    http.addHeader("Content-Type", "application/json");
    
    if (_config.apiKey != nullptr && strlen(_config.apiKey) > 0) {
        String authHeader = "Bearer ";
        authHeader += _config.apiKey;
        http.addHeader("Authorization", authHeader);
    }

    int httpCode = http.POST(jsonPayload);
    _lastHttpCode = httpCode;
    http.end();

    return httpCodeToResult(httpCode);
}

bool PakuIotClient::enqueue(const TelemetryReading& reading) {
    if (_queueCount >= MAX_QUEUE_SIZE) {
        // Queue is full, discard oldest message
        _queueTail = (_queueTail + 1) % MAX_QUEUE_SIZE;
        _queueCount--;
    }

    QueuedMessage& msg = _queue[_queueHead];
    
    // Copy data
    strncpy(msg.metric, reading.metric ? reading.metric : "", sizeof(msg.metric) - 1);
    msg.metric[sizeof(msg.metric) - 1] = '\0';
    msg.value = reading.value;
    strncpy(msg.unit, reading.unit ? reading.unit : "", sizeof(msg.unit) - 1);
    msg.unit[sizeof(msg.unit) - 1] = '\0';
    strncpy(msg.timestamp, reading.timestamp ? reading.timestamp : "", sizeof(msg.timestamp) - 1);
    msg.timestamp[sizeof(msg.timestamp) - 1] = '\0';
    msg.valid = true;

    _queueHead = (_queueHead + 1) % MAX_QUEUE_SIZE;
    _queueCount++;

    return true;
}

bool PakuIotClient::dequeue(TelemetryReading& reading) {
    if (_queueCount == 0) {
        return false;
    }

    // Note: This method is not used in current implementation
    // but provided for future use
    return false;
}

PakuIotResult PakuIotClient::httpCodeToResult(int httpCode) {
    if (httpCode < 0) {
        // HTTPClient error codes
        switch (httpCode) {
            case -1:  // HTTPC_ERROR_CONNECTION_REFUSED
                return PakuIotResult::ERROR_CONNECTION;
            case -2:  // HTTPC_ERROR_SEND_HEADER_FAILED
            case -3:  // HTTPC_ERROR_SEND_PAYLOAD_FAILED
                return PakuIotResult::ERROR_CONNECTION;
            case -4:  // HTTPC_ERROR_NOT_CONNECTED
                return PakuIotResult::ERROR_NOT_CONNECTED;
            case -5:  // HTTPC_ERROR_CONNECTION_LOST
                return PakuIotResult::ERROR_CONNECTION;
            case -6:  // HTTPC_ERROR_NO_STREAM
                return PakuIotResult::ERROR_CONNECTION;
            case -7:  // HTTPC_ERROR_NO_HTTP_SERVER
                return PakuIotResult::ERROR_CONNECTION;
            case -8:  // HTTPC_ERROR_TOO_LESS_RAM
                return PakuIotResult::ERROR_BUFFER_FULL;
            case -9:  // HTTPC_ERROR_ENCODING
                return PakuIotResult::ERROR_CONNECTION;
            case -10: // HTTPC_ERROR_STREAM_WRITE
                return PakuIotResult::ERROR_CONNECTION;
            case -11: // HTTPC_ERROR_READ_TIMEOUT
                return PakuIotResult::ERROR_TIMEOUT;
            default:
                return PakuIotResult::ERROR_CONNECTION;
        }
    }

    if (httpCode >= 200 && httpCode < 300) {
        return PakuIotResult::SUCCESS;
    }
    if (httpCode >= 400 && httpCode < 500) {
        return PakuIotResult::ERROR_CLIENT;
    }
    if (httpCode >= 500) {
        return PakuIotResult::ERROR_SERVER;
    }

    return PakuIotResult::ERROR_CONNECTION;
}
