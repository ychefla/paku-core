/**
 * @file OtaClient.cpp
 * @brief Implementation of secure OTA firmware update client
 */

#include "OtaClient.h"

// Platform-specific WiFi includes
#ifdef ESP32
#include <WiFi.h>
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#endif

// Default configuration values
#define DEFAULT_TIMEOUT_MS 300000      // 5 minutes
#define DEFAULT_BUFFER_SIZE 4096       // 4KB buffer
#define PROGRESS_UPDATE_INTERVAL_MS 1000  // Update progress every second

OtaClient::OtaClient() 
    : _state(OtaState::IDLE)
    , _lastResult(OtaResult::SUCCESS)
    , _progressCallback(nullptr)
    , _initialized(false)
    , _totalBytes(0)
    , _downloadedBytes(0)
    , _startTime(0)
    , _lastProgressUpdate(0)
    , _downloadBuffer(nullptr)
    , _bufferSize(DEFAULT_BUFFER_SIZE)
#ifdef ESP32
    , _updatePartition(nullptr)
    , _runningPartition(nullptr)
#endif
    , _checksumInitialized(false)
{
    memset(&_config, 0, sizeof(_config));
    memset(_lastError, 0, sizeof(_lastError));
}

OtaClient::~OtaClient() {
    if (_downloadBuffer) {
        free(_downloadBuffer);
        _downloadBuffer = nullptr;
    }
    cleanupChecksum();
}

OtaResult OtaClient::begin() {
    if (_initialized) {
        return OtaResult::SUCCESS;
    }

#ifdef ESP32
    // ESP32: Get current running partition
    _runningPartition = esp_ota_get_running_partition();
    if (!_runningPartition) {
        setError(OtaResult::ERROR_NO_PARTITION, "Failed to get running partition");
        return OtaResult::ERROR_NO_PARTITION;
    }

    // Get next OTA partition for updates
    _updatePartition = esp_ota_get_next_update_partition(nullptr);
    if (!_updatePartition) {
        setError(OtaResult::ERROR_NO_PARTITION, "No OTA partition available");
        return OtaResult::ERROR_NO_PARTITION;
    }

    Serial.print("OTA: Running partition: ");
    Serial.println(_runningPartition->label);
    Serial.print("OTA: Update partition: ");
    Serial.println(_updatePartition->label);
#elif defined(ESP8266)
    // ESP8266: Check available space for OTA
    uint32_t freeSketchSpace = ESP.getFreeSketchSpace();
    Serial.print("OTA: Free sketch space: ");
    Serial.println(freeSketchSpace);
    
    if (freeSketchSpace < 100000) {  // Minimum 100KB needed
        setError(OtaResult::ERROR_INSUFFICIENT_SPACE, "Insufficient space for OTA");
        return OtaResult::ERROR_INSUFFICIENT_SPACE;
    }
#endif

    // Allocate download buffer
    _downloadBuffer = (uint8_t*)malloc(_bufferSize);
    if (!_downloadBuffer) {
        setError(OtaResult::ERROR_INSUFFICIENT_SPACE, "Failed to allocate download buffer");
        return OtaResult::ERROR_INSUFFICIENT_SPACE;
    }

    _initialized = true;
    return OtaResult::SUCCESS;
}

bool OtaClient::isUpdateInProgress() const {
    return _state != OtaState::IDLE && 
           _state != OtaState::COMPLETE && 
           _state != OtaState::FAILED &&
           _state != OtaState::ROLLED_BACK;
}

OtaState OtaClient::getState() const {
    return _state;
}

OtaResult OtaClient::startUpdate(const OtaConfig& config, OtaProgressCallback progressCallback) {
    if (!_initialized) {
        OtaResult initResult = begin();
        if (initResult != OtaResult::SUCCESS) {
            return initResult;
        }
    }

    if (isUpdateInProgress()) {
        setError(OtaResult::ERROR_ALREADY_RUNNING, "Update already in progress");
        return OtaResult::ERROR_ALREADY_RUNNING;
    }

    if (WiFi.status() != WL_CONNECTED) {
        setError(OtaResult::ERROR_WIFI_NOT_CONNECTED, "WiFi not connected");
        return OtaResult::ERROR_WIFI_NOT_CONNECTED;
    }

    // Store configuration
    _config = config;
    _progressCallback = progressCallback;
    _totalBytes = 0;
    _downloadedBytes = 0;
    _startTime = millis();
    _lastProgressUpdate = 0;

    // Set defaults if not specified
    if (_config.timeoutMs == 0) {
        _config.timeoutMs = DEFAULT_TIMEOUT_MS;
    }
    if (_config.bufferSize == 0) {
        _config.bufferSize = DEFAULT_BUFFER_SIZE;
    }

    Serial.println("OTA: Starting firmware update");
    Serial.print("OTA: URL: ");
    Serial.println(_config.firmwareUrl);

    setState(OtaState::CHECKING, "Checking for updates");

    // Download firmware
    setState(OtaState::DOWNLOADING, "Downloading firmware");
    OtaResult downloadResult = downloadFirmware();
    if (downloadResult != OtaResult::SUCCESS) {
        setState(OtaState::FAILED, "Download failed");
        return downloadResult;
    }

    // Verify firmware
    if (_config.expectedChecksum && strlen(_config.expectedChecksum) > 0) {
        setState(OtaState::VERIFYING, "Verifying firmware");
        OtaResult verifyResult = verifyFirmware();
        if (verifyResult != OtaResult::SUCCESS) {
            setState(OtaState::FAILED, "Verification failed");
            return verifyResult;
        }
    }

    // Installation happens during download with ESP32 Update library
    setState(OtaState::COMPLETE, "Update completed successfully");
    
    Serial.println("OTA: Update completed. Reboot required.");
    return OtaResult::SUCCESS;
}

// Note: Async update methods removed - OTA updates are blocking operations.
// Future implementations may use FreeRTOS tasks for true async updates.

OtaResult OtaClient::validateCurrentFirmware() {
#ifdef ESP32
    const esp_partition_t* partition = esp_ota_get_running_partition();
    if (!partition) {
        setError(OtaResult::ERROR_NO_PARTITION, "Failed to get running partition");
        return OtaResult::ERROR_NO_PARTITION;
    }

    // Mark current firmware as valid
    esp_err_t err = esp_ota_mark_app_valid_cancel_rollback();
    if (err != ESP_OK) {
        setError(OtaResult::ERROR_INSTALL_FAILED, "Failed to mark firmware as valid");
        return OtaResult::ERROR_INSTALL_FAILED;
    }

    Serial.println("OTA: Current firmware marked as valid");
    return OtaResult::SUCCESS;
#elif defined(ESP8266)
    // ESP8266 doesn't have explicit partition validation
    Serial.println("OTA: Firmware validation not applicable on ESP8266");
    return OtaResult::SUCCESS;
#endif
}

OtaResult OtaClient::rollback() {
#ifdef ESP32
    Serial.println("OTA: Initiating rollback...");

    if (!isRollbackAvailable()) {
        setError(OtaResult::ERROR_ROLLBACK_FAILED, "No rollback partition available");
        return OtaResult::ERROR_ROLLBACK_FAILED;
    }

    // Get the last valid partition
    const esp_partition_t* lastValid = esp_ota_get_last_invalid_partition();
    if (!lastValid) {
        setError(OtaResult::ERROR_ROLLBACK_FAILED, "Failed to get rollback partition");
        return OtaResult::ERROR_ROLLBACK_FAILED;
    }

    // Set boot partition to the previous one
    esp_err_t err = esp_ota_set_boot_partition(lastValid);
    if (err != ESP_OK) {
        setError(OtaResult::ERROR_ROLLBACK_FAILED, "Failed to set boot partition");
        return OtaResult::ERROR_ROLLBACK_FAILED;
    }

    setState(OtaState::ROLLED_BACK, "Rolled back to previous firmware");
    Serial.println("OTA: Rollback successful. Reboot required.");
    
    return OtaResult::SUCCESS;
#elif defined(ESP8266)
    // ESP8266 doesn't support rollback
    setError(OtaResult::ERROR_ROLLBACK_FAILED, "Rollback not supported on ESP8266");
    return OtaResult::ERROR_ROLLBACK_FAILED;
#endif
}

bool OtaClient::getCurrentFirmwareInfo(char* version, size_t maxLen, 
                                       char* partition, size_t partitionMaxLen) {
#ifdef ESP32
    const esp_partition_t* running = esp_ota_get_running_partition();
    if (!running) {
        return false;
    }

    if (partition && partitionMaxLen > 0) {
        strncpy(partition, running->label, partitionMaxLen - 1);
        partition[partitionMaxLen - 1] = '\0';
    }

    // Get firmware version from app description
    const esp_app_desc_t* app_desc = esp_ota_get_app_description();
    if (app_desc && version && maxLen > 0) {
        strncpy(version, app_desc->version, maxLen - 1);
        version[maxLen - 1] = '\0';
    }

    return true;
#elif defined(ESP8266)
    // ESP8266 simplified version info
    if (partition && partitionMaxLen > 0) {
        strncpy(partition, "ESP8266", partitionMaxLen - 1);
        partition[partitionMaxLen - 1] = '\0';
    }

    if (version && maxLen > 0) {
        snprintf(version, maxLen, "ESP8266-SDK:%s", ESP.getSdkVersion());
    }

    return true;
#endif
}

const char* OtaClient::getLastError() const {
    return _lastError;
}

OtaProgress OtaClient::getProgress() const {
    OtaProgress progress;
    progress.state = _state;
    progress.totalBytes = _totalBytes;
    progress.downloadedBytes = _downloadedBytes;
    
    if (_totalBytes > 0) {
        progress.progressPercent = (_downloadedBytes * 100) / _totalBytes;
    } else {
        progress.progressPercent = 0;
    }
    
    progress.elapsedMs = millis() - _startTime;
    progress.statusMessage = _lastError;
    
    return progress;
}

OtaResult OtaClient::cancelUpdate() {
    if (!isUpdateInProgress()) {
        return OtaResult::SUCCESS;
    }

    // Clean up HTTP connection
    _http.end();

    // Abort update if in progress
    if (Update.isRunning()) {
#ifdef ESP32
        Update.abort();
#elif defined(ESP8266)
        Update.end(false); // false = abort
#endif
    }

    setState(OtaState::FAILED, "Update cancelled by user");
    return OtaResult::SUCCESS;
}

bool OtaClient::isRollbackAvailable() const {
#ifdef ESP32
    const esp_partition_t* lastValid = esp_ota_get_last_invalid_partition();
    return (lastValid != nullptr);
#elif defined(ESP8266)
    // ESP8266 doesn't support rollback
    return false;
#endif
}

const char* OtaClient::resultToString(OtaResult result) {
    switch (result) {
        case OtaResult::SUCCESS: return "Success";
        case OtaResult::ERROR_WIFI_NOT_CONNECTED: return "WiFi not connected";
        case OtaResult::ERROR_DOWNLOAD_FAILED: return "Download failed";
        case OtaResult::ERROR_VERIFICATION_FAILED: return "Verification failed";
        case OtaResult::ERROR_INSTALL_FAILED: return "Installation failed";
        case OtaResult::ERROR_NO_PARTITION: return "No partition available";
        case OtaResult::ERROR_INVALID_URL: return "Invalid URL";
        case OtaResult::ERROR_TIMEOUT: return "Timeout";
        case OtaResult::ERROR_INSUFFICIENT_SPACE: return "Insufficient space";
        case OtaResult::ERROR_INVALID_FIRMWARE: return "Invalid firmware";
        case OtaResult::ERROR_ALREADY_RUNNING: return "Update already running";
        case OtaResult::ERROR_ROLLBACK_FAILED: return "Rollback failed";
        default: return "Unknown error";
    }
}

const char* OtaClient::stateToString(OtaState state) {
    switch (state) {
        case OtaState::IDLE: return "Idle";
        case OtaState::CHECKING: return "Checking";
        case OtaState::DOWNLOADING: return "Downloading";
        case OtaState::VERIFYING: return "Verifying";
        case OtaState::INSTALLING: return "Installing";
        case OtaState::COMPLETE: return "Complete";
        case OtaState::FAILED: return "Failed";
        case OtaState::ROLLED_BACK: return "Rolled back";
        default: return "Unknown";
    }
}

// Private methods

void OtaClient::setState(OtaState newState, const char* message) {
    _state = newState;
    
    if (message) {
        strncpy(_lastError, message, sizeof(_lastError) - 1);
        _lastError[sizeof(_lastError) - 1] = '\0';
    }
    
    Serial.print("OTA State: ");
    Serial.println(stateToString(newState));
    
    if (_progressCallback) {
        _progressCallback(getProgress());
    }
}

void OtaClient::updateProgress(size_t downloaded, size_t total) {
    _downloadedBytes = downloaded;
    _totalBytes = total;
    
    unsigned long now = millis();
    if (now - _lastProgressUpdate >= PROGRESS_UPDATE_INTERVAL_MS) {
        _lastProgressUpdate = now;
        
        if (_progressCallback) {
            _progressCallback(getProgress());
        }
        
        // Print progress to serial
        if (total > 0) {
            uint8_t percent = (downloaded * 100) / total;
            Serial.print("OTA Progress: ");
            Serial.print(percent);
            Serial.print("% (");
            Serial.print(downloaded);
            Serial.print("/");
            Serial.print(total);
            Serial.println(" bytes)");
        }
    }
}

void OtaClient::setError(OtaResult result, const char* message) {
    _lastResult = result;
    
    if (message) {
        strncpy(_lastError, message, sizeof(_lastError) - 1);
        _lastError[sizeof(_lastError) - 1] = '\0';
    } else {
        strncpy(_lastError, resultToString(result), sizeof(_lastError) - 1);
        _lastError[sizeof(_lastError) - 1] = '\0';
    }
    
    Serial.print("OTA Error: ");
    Serial.println(_lastError);
}

OtaResult OtaClient::downloadFirmware() {
    if (!_config.firmwareUrl || strlen(_config.firmwareUrl) == 0) {
        setError(OtaResult::ERROR_INVALID_URL, "Firmware URL is empty");
        return OtaResult::ERROR_INVALID_URL;
    }

    // Determine if HTTPS
    bool isHttps = strncmp(_config.firmwareUrl, "https://", 8) == 0;
    
    // Configure HTTP client
    if (isHttps) {
        // SECURITY NOTE: Using setInsecure() accepts any certificate, which is vulnerable
        // to man-in-the-middle attacks. For production, configure with proper CA certificate:
        //   _secureClient.setCACert(ca_cert_pem);
        // This is acceptable for development/testing with self-signed certificates.
        _secureClient.setInsecure();
        _http.begin(_secureClient, _config.firmwareUrl);
    } else {
        _http.begin(_client, _config.firmwareUrl);
    }
    
    _http.setTimeout(_config.timeoutMs);
    _http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);

    Serial.print("OTA: Connecting to ");
    Serial.println(_config.firmwareUrl);

    // Send HTTP GET request
    int httpCode = _http.GET();
    
    if (httpCode != HTTP_CODE_OK) {
        _http.end();
        char errorMsg[128];
        snprintf(errorMsg, sizeof(errorMsg), "HTTP GET failed: %d", httpCode);
        setError(OtaResult::ERROR_DOWNLOAD_FAILED, errorMsg);
        return OtaResult::ERROR_DOWNLOAD_FAILED;
    }

    // Get content length
    int contentLength = _http.getSize();
    if (contentLength <= 0) {
        _http.end();
        setError(OtaResult::ERROR_DOWNLOAD_FAILED, "Invalid content length");
        return OtaResult::ERROR_DOWNLOAD_FAILED;
    }

    Serial.print("OTA: Firmware size: ");
    Serial.print(contentLength);
    Serial.println(" bytes");

#ifdef ESP32
    // Check if we have enough space (ESP32 partitions)
    if ((size_t)contentLength > _updatePartition->size) {
        _http.end();
        setError(OtaResult::ERROR_INSUFFICIENT_SPACE, "Firmware too large for partition");
        return OtaResult::ERROR_INSUFFICIENT_SPACE;
    }
#elif defined(ESP8266)
    // Check if we have enough space (ESP8266)
    uint32_t freeSketchSpace = ESP.getFreeSketchSpace();
    if ((size_t)contentLength > freeSketchSpace) {
        _http.end();
        setError(OtaResult::ERROR_INSUFFICIENT_SPACE, "Firmware too large for available space");
        return OtaResult::ERROR_INSUFFICIENT_SPACE;
    }
#endif

    _totalBytes = contentLength;

    // Begin update
    if (!Update.begin(contentLength, U_FLASH)) {
        _http.end();
        char errorMsg[128];
#ifdef ESP32
        snprintf(errorMsg, sizeof(errorMsg), "Update.begin failed: %s", Update.errorString());
#elif defined(ESP8266)
        snprintf(errorMsg, sizeof(errorMsg), "Update.begin failed: %s", Update.getErrorString().c_str());
#endif
        setError(OtaResult::ERROR_INSTALL_FAILED, errorMsg);
        return OtaResult::ERROR_INSTALL_FAILED;
    }

    // Initialize checksum if needed
    if (_config.expectedChecksum && strlen(_config.expectedChecksum) > 0) {
        if (!initChecksum()) {
            _http.end();
#ifdef ESP32
            Update.abort();
#elif defined(ESP8266)
            Update.end(false);
#endif
            setError(OtaResult::ERROR_VERIFICATION_FAILED, "Failed to initialize checksum");
            return OtaResult::ERROR_VERIFICATION_FAILED;
        }
    }

    // Download and write firmware
    WiFiClient* stream = _http.getStreamPtr();
    size_t bytesWritten = 0;
    unsigned long lastUpdate = millis();

    while (_http.connected() && bytesWritten < (size_t)contentLength) {
        // Check timeout
        if (millis() - lastUpdate > _config.timeoutMs) {
            _http.end();
#ifdef ESP32
            Update.abort();
#elif defined(ESP8266)
            Update.end(false);
#endif
            cleanupChecksum();
            setError(OtaResult::ERROR_TIMEOUT, "Download timeout");
            return OtaResult::ERROR_TIMEOUT;
        }

        // Read available data
        size_t availableSize = stream->available();
        if (availableSize > 0) {
            size_t readSize = availableSize > _bufferSize ? _bufferSize : availableSize;
            int bytesRead = stream->readBytes(_downloadBuffer, readSize);
            
            if (bytesRead > 0) {
                // Update checksum
                if (_checksumInitialized) {
                    updateChecksum(_downloadBuffer, bytesRead);
                }

                // Write to flash
                size_t written = Update.write(_downloadBuffer, bytesRead);
                if (written != (size_t)bytesRead) {
                    _http.end();
#ifdef ESP32
                    Update.abort();
#elif defined(ESP8266)
                    Update.end(false);
#endif
                    cleanupChecksum();
                    setError(OtaResult::ERROR_INSTALL_FAILED, "Failed to write firmware");
                    return OtaResult::ERROR_INSTALL_FAILED;
                }

                bytesWritten += written;
                updateProgress(bytesWritten, contentLength);
                lastUpdate = millis();
            }
        } else {
            yield(); // Allow other tasks to run
        }
    }

    _http.end();

    // Check if download completed
    if (bytesWritten != (size_t)contentLength) {
#ifdef ESP32
        Update.abort();
#elif defined(ESP8266)
        Update.end(false);
#endif
        cleanupChecksum();
        setError(OtaResult::ERROR_DOWNLOAD_FAILED, "Incomplete download");
        return OtaResult::ERROR_DOWNLOAD_FAILED;
    }

    Serial.println("OTA: Download complete");

    // End update
    if (!Update.end(true)) {
        cleanupChecksum();
        char errorMsg[128];
#ifdef ESP32
        snprintf(errorMsg, sizeof(errorMsg), "Update.end failed: %s", Update.errorString());
#elif defined(ESP8266)
        snprintf(errorMsg, sizeof(errorMsg), "Update.end failed: %s", Update.getErrorString().c_str());
#endif
        setError(OtaResult::ERROR_INSTALL_FAILED, errorMsg);
        return OtaResult::ERROR_INSTALL_FAILED;
    }

    Serial.println("OTA: Firmware written successfully");
    return OtaResult::SUCCESS;
}

OtaResult OtaClient::verifyFirmware() {
    if (!_config.expectedChecksum || strlen(_config.expectedChecksum) == 0) {
        Serial.println("OTA: No checksum provided, skipping verification");
        return OtaResult::SUCCESS;
    }

    if (!_checksumInitialized) {
        setError(OtaResult::ERROR_VERIFICATION_FAILED, "Checksum not initialized");
        return OtaResult::ERROR_VERIFICATION_FAILED;
    }

    // Finalize and compare checksum
    if (!finalizeChecksum(_config.expectedChecksum)) {
        setError(OtaResult::ERROR_VERIFICATION_FAILED, "Checksum mismatch");
        return OtaResult::ERROR_VERIFICATION_FAILED;
    }

    Serial.println("OTA: Checksum verified successfully");
    cleanupChecksum();
    return OtaResult::SUCCESS;
}

OtaResult OtaClient::installFirmware() {
    // Installation happens during download with ESP32 Update library
    // This method is a placeholder for future enhancements
    return OtaResult::SUCCESS;
}

bool OtaClient::initChecksum() {
#ifdef ESP32
    mbedtls_md_init(&_sha256Context);
    
    const mbedtls_md_info_t* mdInfo = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
    if (!mdInfo) {
        return false;
    }

    if (mbedtls_md_setup(&_sha256Context, mdInfo, 0) != 0) {
        return false;
    }

    if (mbedtls_md_starts(&_sha256Context) != 0) {
        mbedtls_md_free(&_sha256Context);
        return false;
    }

    _checksumInitialized = true;
    return true;
#elif defined(ESP8266)
    // ESP8266: checksum calculated by Update library automatically
    _checksumInitialized = true;
    return true;
#endif
}

bool OtaClient::updateChecksum(const uint8_t* data, size_t len) {
#ifdef ESP32
    if (!_checksumInitialized) {
        return false;
    }

    return mbedtls_md_update(&_sha256Context, data, len) == 0;
#elif defined(ESP8266)
    // ESP8266: checksum handled by Update library
    return true;
#endif
}

bool OtaClient::finalizeChecksum(const char* expectedChecksum) {
#ifdef ESP32
    if (!_checksumInitialized) {
        return false;
    }

    uint8_t calculatedHash[32]; // SHA256 = 32 bytes
    if (mbedtls_md_finish(&_sha256Context, calculatedHash) != 0) {
        return false;
    }

    // Convert expected checksum from hex to bytes
    uint8_t expectedHash[32];
    size_t expectedLen = hexToBytes(expectedChecksum, expectedHash, sizeof(expectedHash));
    
    if (expectedLen != 32) {
        Serial.println("OTA: Invalid checksum format (expected 64 hex characters)");
        return false;
    }

    // Compare hashes
    bool match = compareBytes(calculatedHash, expectedHash, 32);
    
    if (match) {
        Serial.println("OTA: Checksum match!");
    } else {
        Serial.println("OTA: Checksum mismatch!");
        Serial.print("Expected: ");
        for (int i = 0; i < 32; i++) {
            Serial.printf("%02x", expectedHash[i]);
        }
        Serial.println();
        Serial.print("Calculated: ");
        for (int i = 0; i < 32; i++) {
            Serial.printf("%02x", calculatedHash[i]);
        }
        Serial.println();
    }

    return match;
#elif defined(ESP8266)
    // ESP8266: Use Update library's MD5 verification if available
    // Note: ESP8266 Update uses MD5, not SHA256
    if (expectedChecksum && strlen(expectedChecksum) > 0) {
        Serial.println("OTA: Warning - SHA256 verification not available on ESP8266");
        Serial.println("OTA: Using Update library's MD5 verification instead");
    }
    return true; // Rely on Update library's verification
#endif
}

void OtaClient::cleanupChecksum() {
#ifdef ESP32
    if (_checksumInitialized) {
        mbedtls_md_free(&_sha256Context);
        _checksumInitialized = false;
    }
#elif defined(ESP8266)
    _checksumInitialized = false;
#endif
}

size_t OtaClient::hexToBytes(const char* hex, uint8_t* bytes, size_t maxLen) {
    size_t len = strlen(hex);
    if (len % 2 != 0) {
        return 0; // Invalid hex string
    }

    size_t byteLen = len / 2;
    if (byteLen > maxLen) {
        byteLen = maxLen;
    }

    for (size_t i = 0; i < byteLen; i++) {
        char byteStr[3] = {hex[i * 2], hex[i * 2 + 1], '\0'};
        bytes[i] = (uint8_t)strtol(byteStr, nullptr, 16);
    }

    return byteLen;
}

bool OtaClient::compareBytes(const uint8_t* a, const uint8_t* b, size_t len) {
    for (size_t i = 0; i < len; i++) {
        if (a[i] != b[i]) {
            return false;
        }
    }
    return true;
}
