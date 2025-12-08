/**
 * @file OtaClient.h
 * @brief Secure OTA firmware update client for ESP32
 * 
 * This library provides secure over-the-air (OTA) firmware update
 * functionality for paku-core (EDGE device) with support for:
 * - HTTP/HTTPS firmware downloads
 * - Signature/checksum verification
 * - Atomic updates with rollback support
 * - Resumable downloads
 * - Progress reporting via MQTT
 * - Integration with ESP32 partition scheme
 */

#ifndef OTA_CLIENT_H
#define OTA_CLIENT_H

#include <Arduino.h>
#include <WiFiClientSecure.h>

// Platform-specific HTTP client
#ifdef ESP32
#include <HTTPClient.h>
#include <Update.h>
#include <esp_ota_ops.h>
#include <esp_partition.h>
#include <mbedtls/md.h>
#elif defined(ESP8266)
#include <ESP8266HTTPClient.h>
#include <Updater.h>
#include <Hash.h>
#endif

/**
 * @brief OTA update states
 */
enum class OtaState {
    IDLE,                   ///< No update in progress
    CHECKING,               ///< Checking for updates
    DOWNLOADING,            ///< Downloading firmware
    VERIFYING,              ///< Verifying firmware integrity
    INSTALLING,             ///< Installing firmware
    COMPLETE,               ///< Update completed successfully
    FAILED,                 ///< Update failed
    ROLLED_BACK            ///< Rolled back to previous firmware
};

/**
 * @brief OTA result codes
 */
enum class OtaResult {
    SUCCESS = 0,                ///< Operation successful
    ERROR_WIFI_NOT_CONNECTED,   ///< WiFi not connected
    ERROR_DOWNLOAD_FAILED,      ///< Failed to download firmware
    ERROR_VERIFICATION_FAILED,  ///< Checksum/signature verification failed
    ERROR_INSTALL_FAILED,       ///< Failed to install firmware
    ERROR_NO_PARTITION,         ///< No OTA partition available
    ERROR_INVALID_URL,          ///< Invalid firmware URL
    ERROR_TIMEOUT,              ///< Operation timed out
    ERROR_INSUFFICIENT_SPACE,   ///< Not enough space for firmware
    ERROR_INVALID_FIRMWARE,     ///< Invalid firmware image
    ERROR_ALREADY_RUNNING,      ///< Update already in progress
    ERROR_ROLLBACK_FAILED      ///< Rollback operation failed
};

/**
 * @brief OTA configuration
 */
struct OtaConfig {
    const char* firmwareUrl;        ///< URL to firmware binary
    const char* expectedChecksum;   ///< Expected SHA256 checksum (hex string)
    const char* targetVersion;      ///< Target firmware version
    bool verifySignature;           ///< Whether to verify signature
    bool allowDowngrade;            ///< Allow downgrade to older version
    uint32_t timeoutMs;             ///< Download timeout in milliseconds
    uint16_t bufferSize;            ///< Download buffer size
    bool resumeSupported;           ///< Whether server supports resumable downloads
};

/**
 * @brief OTA progress information
 */
struct OtaProgress {
    OtaState state;                 ///< Current update state
    size_t totalBytes;              ///< Total firmware size in bytes
    size_t downloadedBytes;         ///< Bytes downloaded so far
    uint8_t progressPercent;        ///< Progress percentage (0-100)
    unsigned long elapsedMs;        ///< Elapsed time in milliseconds
    const char* statusMessage;      ///< Human-readable status message
};

/**
 * @brief Callback function type for progress updates
 * 
 * @param progress Current progress information
 */
typedef void (*OtaProgressCallback)(const OtaProgress& progress);

/**
 * @brief Secure OTA update client
 * 
 * Provides methods to download, verify, and install firmware updates
 * with support for rollback and progress reporting.
 */
class OtaClient {
public:
    /**
     * @brief Construct a new OtaClient
     */
    OtaClient();

    /**
     * @brief Destroy the OtaClient
     */
    ~OtaClient();

    /**
     * @brief Initialize the OTA client
     * 
     * Checks partition scheme and prepares for updates.
     * 
     * @return OtaResult SUCCESS if initialized successfully
     */
    OtaResult begin();

    /**
     * @brief Check if an update is currently in progress
     * 
     * @return true if update is in progress
     */
    bool isUpdateInProgress() const;

    /**
     * @brief Get current OTA state
     * 
     * @return Current OtaState
     */
    OtaState getState() const;

    /**
     * @brief Start a firmware update
     * 
     * Downloads, verifies, and installs firmware from the specified URL.
     * This is a blocking operation that can take several minutes.
     * 
     * @param config Update configuration
     * @param progressCallback Optional callback for progress updates
     * @return OtaResult Result of the update operation
     */
    OtaResult startUpdate(const OtaConfig& config, OtaProgressCallback progressCallback = nullptr);

    // Note: Async/non-blocking update methods are not implemented in this version.
    // OTA updates are blocking operations that typically complete in 1-5 minutes.
    // Future versions may implement true async updates using FreeRTOS tasks.

    /**
     * @brief Validate the currently running firmware
     * 
     * Marks the current firmware as valid, preventing rollback.
     * Call this after verifying the device is working correctly
     * after an update.
     * 
     * @return OtaResult SUCCESS if validated
     */
    OtaResult validateCurrentFirmware();

    /**
     * @brief Rollback to previous firmware
     * 
     * Reverts to the previous firmware version stored in the
     * inactive OTA partition.
     * 
     * @return OtaResult SUCCESS if rollback successful
     */
    OtaResult rollback();

    /**
     * @brief Get information about current firmware
     * 
     * @param version Output buffer for version string
     * @param maxLen Maximum length of version buffer
     * @param partition Output for partition name
     * @param partitionMaxLen Maximum length of partition buffer
     * @return true if information retrieved successfully
     */
    bool getCurrentFirmwareInfo(char* version, size_t maxLen, 
                                char* partition, size_t partitionMaxLen);

    /**
     * @brief Get the last error message
     * 
     * @return Human-readable error message
     */
    const char* getLastError() const;

    /**
     * @brief Get current progress information
     * 
     * @return OtaProgress structure with current progress
     */
    OtaProgress getProgress() const;

    /**
     * @brief Cancel an in-progress update
     * 
     * @return OtaResult SUCCESS if cancelled successfully
     */
    OtaResult cancelUpdate();

    /**
     * @brief Check if a rollback is available
     * 
     * @return true if previous firmware is available for rollback
     */
    bool isRollbackAvailable() const;

    /**
     * @brief Get human-readable string for OtaResult
     * 
     * @param result Result code
     * @return Error message string
     */
    static const char* resultToString(OtaResult result);

    /**
     * @brief Get human-readable string for OtaState
     * 
     * @param state State code
     * @return State string
     */
    static const char* stateToString(OtaState state);

private:
    // Configuration and state
    OtaConfig _config;
    OtaState _state;
    OtaResult _lastResult;
    char _lastError[128];
    OtaProgressCallback _progressCallback;
    bool _initialized;

    // Progress tracking
    size_t _totalBytes;
    size_t _downloadedBytes;
    unsigned long _startTime;
    unsigned long _lastProgressUpdate;

    // HTTP client for downloads
    HTTPClient _http;
    WiFiClientSecure _secureClient;
    WiFiClient _client;

    // Update buffers
    uint8_t* _downloadBuffer;
    size_t _bufferSize;

#ifdef ESP32
    // Partition information (ESP32 only)
    const esp_partition_t* _updatePartition;
    const esp_partition_t* _runningPartition;
#endif

#ifdef ESP32
    // Checksum verification (ESP32 with mbedtls)
    mbedtls_md_context_t _sha256Context;
    bool _checksumInitialized;
#elif defined(ESP8266)
    // Checksum verification (ESP8266 with Hash library)
    bool _checksumInitialized;
    uint8_t _sha256Hash[32];
#endif

    /**
     * @brief Set current state and notify callback
     * 
     * @param newState New OTA state
     * @param message Status message
     */
    void setState(OtaState newState, const char* message = nullptr);

    /**
     * @brief Update progress and notify callback
     * 
     * @param downloaded Bytes downloaded
     * @param total Total bytes
     */
    void updateProgress(size_t downloaded, size_t total);

    /**
     * @brief Set error state and message
     * 
     * @param result Error result code
     * @param message Error message
     */
    void setError(OtaResult result, const char* message);

    /**
     * @brief Download firmware from URL
     * 
     * @return OtaResult Result of download
     */
    OtaResult downloadFirmware();

    /**
     * @brief Verify downloaded firmware checksum
     * 
     * @return OtaResult Result of verification
     */
    OtaResult verifyFirmware();

    /**
     * @brief Install firmware to OTA partition
     * 
     * @return OtaResult Result of installation
     */
    OtaResult installFirmware();

    /**
     * @brief Initialize checksum calculation
     * 
     * @return true if initialized successfully
     */
    bool initChecksum();

    /**
     * @brief Update checksum with data chunk
     * 
     * @param data Data buffer
     * @param len Data length
     * @return true if updated successfully
     */
    bool updateChecksum(const uint8_t* data, size_t len);

    /**
     * @brief Finalize checksum and compare
     * 
     * @param expectedChecksum Expected checksum (hex string)
     * @return true if checksum matches
     */
    bool finalizeChecksum(const char* expectedChecksum);

    /**
     * @brief Cleanup checksum context
     */
    void cleanupChecksum();

    /**
     * @brief Convert hex string to bytes
     * 
     * @param hex Hex string
     * @param bytes Output buffer
     * @param maxLen Maximum bytes to write
     * @return Number of bytes written
     */
    size_t hexToBytes(const char* hex, uint8_t* bytes, size_t maxLen);

    /**
     * @brief Compare two byte arrays
     * 
     * @param a First array
     * @param b Second array
     * @param len Length to compare
     * @return true if arrays match
     */
    bool compareBytes(const uint8_t* a, const uint8_t* b, size_t len);
};

#endif // OTA_CLIENT_H
