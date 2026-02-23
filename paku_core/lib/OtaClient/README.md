# OtaClient Library

Secure OTA (Over-The-Air) firmware update client for ESP32 devices.

## Features

- **HTTP/HTTPS Downloads**: Download firmware from any HTTP/HTTPS server
- **Checksum Verification**: SHA256 checksum verification for integrity
- **Atomic Updates**: Write to inactive partition, boot from new partition
- **Automatic Rollback**: Rollback to previous firmware on boot failure
- **Progress Reporting**: Real-time progress callbacks during update
- **Partition Management**: Automatic management of OTA_0 and OTA_1 partitions

## Requirements

- ESP32 board with dual OTA partitions
- PlatformIO or Arduino IDE
- Partition scheme with two OTA app partitions (e.g., `min_spiffs.csv`)

## Quick Start

### 1. Include the Library

```cpp
#include "OtaClient.h"

OtaClient otaClient;
```

### 2. Initialize

```cpp
void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  
  // Initialize OTA client
  OtaResult result = otaClient.begin();
  if (result != OtaResult::SUCCESS) {
    Serial.println("OTA init failed!");
  }
}
```

### 3. Trigger Update

```cpp
void updateFirmware() {
  OtaConfig config;
  config.firmwareUrl = "http://server.com/firmware.bin";
  config.expectedChecksum = "a1b2c3d4..."; // SHA256 hex string
  config.targetVersion = "1.2.0";
  config.timeoutMs = 300000; // 5 minutes
  
  OtaResult result = otaClient.startUpdate(config, otaProgressCallback);
  
  if (result == OtaResult::SUCCESS) {
    Serial.println("Update successful! Rebooting...");
    ESP.restart();
  } else {
    Serial.print("Update failed: ");
    Serial.println(otaClient.getLastError());
  }
}
```

### 4. Progress Callback (Optional)

```cpp
void otaProgressCallback(const OtaProgress& progress) {
  Serial.print("OTA: ");
  Serial.print(OtaClient::stateToString(progress.state));
  Serial.print(" - ");
  Serial.print(progress.progressPercent);
  Serial.println("%");
}
```

## API Reference

### Class: `OtaClient`

#### Methods

##### `begin()`
Initialize the OTA client.
- **Returns**: `OtaResult` - SUCCESS or error code
- **Usage**: Call once during setup

##### `startUpdate(config, callback)`
Start a firmware update (blocking).
- **Parameters**:
  - `config`: OtaConfig structure with update settings
  - `callback`: Optional progress callback function
- **Returns**: `OtaResult` - SUCCESS or error code
- **Note**: This is a blocking call that may take several minutes

##### `validateCurrentFirmware()`
Mark the current firmware as valid to prevent rollback.
- **Returns**: `OtaResult` - SUCCESS or error code
- **Usage**: Call after verifying device works correctly post-update

##### `rollback()`
Rollback to previous firmware.
- **Returns**: `OtaResult` - SUCCESS or error code
- **Usage**: Call if new firmware has issues, then restart

##### `getState()`
Get current OTA state.
- **Returns**: `OtaState` enum

##### `getProgress()`
Get current progress information.
- **Returns**: `OtaProgress` structure

##### `getLastError()`
Get human-readable error message.
- **Returns**: `const char*` error string

### Structures

#### `OtaConfig`
```cpp
struct OtaConfig {
  const char* firmwareUrl;        // Firmware binary URL
  const char* expectedChecksum;   // SHA256 checksum (hex)
  const char* targetVersion;      // Target version string
  bool verifySignature;           // Enable signature verification
  bool allowDowngrade;            // Allow downgrade to older version
  uint32_t timeoutMs;            // Download timeout (default: 300000)
  uint16_t bufferSize;           // Download buffer size (default: 4096)
  bool resumeSupported;          // Server supports resume (future)
};
```

#### `OtaProgress`
```cpp
struct OtaProgress {
  OtaState state;                // Current state
  size_t totalBytes;             // Total firmware size
  size_t downloadedBytes;        // Downloaded bytes
  uint8_t progressPercent;       // Progress 0-100%
  unsigned long elapsedMs;       // Elapsed time
  const char* statusMessage;     // Status message
};
```

### Enumerations

#### `OtaState`
- `IDLE` - No update in progress
- `CHECKING` - Checking for updates
- `DOWNLOADING` - Downloading firmware
- `VERIFYING` - Verifying checksum
- `INSTALLING` - Installing firmware
- `COMPLETE` - Update complete
- `FAILED` - Update failed
- `ROLLED_BACK` - Rolled back to previous version

#### `OtaResult`
- `SUCCESS` - Operation successful
- `ERROR_WIFI_NOT_CONNECTED` - WiFi disconnected
- `ERROR_DOWNLOAD_FAILED` - Download failed
- `ERROR_VERIFICATION_FAILED` - Checksum mismatch
- `ERROR_INSTALL_FAILED` - Installation failed
- `ERROR_NO_PARTITION` - No OTA partition available
- `ERROR_INVALID_URL` - Invalid URL
- `ERROR_TIMEOUT` - Operation timeout
- `ERROR_INSUFFICIENT_SPACE` - Not enough flash space
- `ERROR_INVALID_FIRMWARE` - Invalid firmware format
- `ERROR_ALREADY_RUNNING` - Update already in progress
- `ERROR_ROLLBACK_FAILED` - Rollback failed

## Examples

### Basic OTA Update

```cpp
#include <WiFi.h>
#include "OtaClient.h"

const char* ssid = "your-wifi";
const char* password = "your-password";

OtaClient otaClient;

void setup() {
  Serial.begin(115200);
  
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  otaClient.begin();
  
  // Trigger update
  OtaConfig config;
  config.firmwareUrl = "http://server/firmware.bin";
  config.expectedChecksum = "your-sha256-checksum";
  
  if (otaClient.startUpdate(config) == OtaResult::SUCCESS) {
    ESP.restart();
  }
}

void loop() {
  // Normal operation
}
```

### OTA with Progress Reporting

```cpp
void otaProgressCallback(const OtaProgress& progress) {
  if (progress.progressPercent % 10 == 0) {
    Serial.printf("%d%% complete\n", progress.progressPercent);
  }
}

void triggerOta() {
  OtaConfig config;
  config.firmwareUrl = "https://secure-server/firmware.bin";
  config.expectedChecksum = "checksum-here";
  config.targetVersion = "2.0.0";
  
  OtaResult result = otaClient.startUpdate(config, otaProgressCallback);
  
  Serial.println(OtaClient::resultToString(result));
}
```

### Manual Rollback

```cpp
void performRollback() {
  if (otaClient.isRollbackAvailable()) {
    OtaResult result = otaClient.rollback();
    if (result == OtaResult::SUCCESS) {
      Serial.println("Rollback successful, rebooting...");
      ESP.restart();
    }
  } else {
    Serial.println("No rollback available");
  }
}
```

## Security Considerations

### Checksum Verification
Always provide a SHA256 checksum for production deployments:
```bash
sha256sum firmware.bin
```

### HTTPS with Certificate
For production, configure with proper CA certificate:
```cpp
// In OtaClient.cpp, replace setInsecure() with:
_secureClient.setCACert(your_ca_cert_pem);
```

### Signature Verification (Future)
Digital signature verification will be added in a future release for additional security.

## Troubleshooting

### "Download failed" Error
- Verify URL is accessible from device
- Check firewall rules
- Ensure WiFi connection is stable
- Increase timeout if needed

### "Verification failed" Error
- Verify checksum is correct
- Recalculate checksum: `sha256sum firmware.bin`
- Ensure firmware wasn't corrupted during upload

### "Insufficient space" Error
- Check partition scheme has enough space
- Reduce firmware size
- Use `min_spiffs.csv` or larger OTA partitions

### Device Doesn't Boot After Update
- ESP32 will automatically rollback after 3 failed boots
- Check serial output for crash logs
- Ensure new firmware calls `validateCurrentFirmware()`

## License

Part of the paku-core project. See main repository for license details.

## Further Documentation

- [OTA Integration Guide](../../../docs/edge/ota-integration.md) - Complete integration guide
- [OTA Testing Checklist](../../../docs/edge/ota-testing-checklist.md) - Testing procedures
- [Architecture Documentation](../../../docs/ARCHITECTURE.md) - System architecture

## Contributing

See the main paku-core repository for contribution guidelines.
