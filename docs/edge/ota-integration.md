# OTA Firmware Update Integration Guide

This document describes how to use the OTA (Over-The-Air) firmware update feature in paku-core.

## Overview

The OTA update client enables secure, remote firmware updates for ESP32 devices without physical access. Updates are triggered via MQTT commands and support:

- HTTP/HTTPS firmware downloads
- SHA256 checksum verification
- Atomic updates with automatic rollback on failure
- Progress reporting via MQTT
- Partition management (OTA_0/OTA_1)

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                     OTA Update Flow                             │
└─────────────────────────────────────────────────────────────────┘

 Backend/User                    MQTT Broker           ESP32 Device
      │                               │                      │
      │  1. Publish OTA command       │                      │
      ├──────────────────────────────►│                      │
      │                               │  2. Receive command  │
      │                               ├─────────────────────►│
      │                               │                      │
      │                               │  3. Acknowledge      │
      │                               │◄─────────────────────┤
      │                               │                      │
      │                               │  4. Download FW      │
      │                               │      (HTTP/S)        │
      │◄──────────────────────────────┼──────────────────────┤
      │     Firmware Binary           │                      │
      │                               │                      │
      │                               │  5. Progress updates │
      │                               │◄─────────────────────┤
      │                               │                      │
      │                               │  6. Verify checksum  │
      │                               │      (SHA256)        │
      │                               │                      │
      │                               │  7. Install to       │
      │                               │     OTA partition    │
      │                               │                      │
      │                               │  8. Report result    │
      │                               │◄─────────────────────┤
      │                               │                      │
      │                               │  9. Reboot           │
      │                               │                      │
      │                               │ 10. Validate &       │
      │                               │     prevent rollback │
```

## MQTT Topics

### Command Topic (Subscribe)

**Topic:** `paku/edge/{device_id}/cmd/ota`

**Payload (JSON):**
```json
{
  "url": "https://your-server.com/firmware/paku-core-v1.2.0.bin",
  "checksum": "a1b2c3d4e5f6...",
  "version": "1.2.0"
}
```

**Fields:**
- `url` (required): HTTP/HTTPS URL to firmware binary
- `checksum` (optional): SHA256 checksum in hex format (64 characters)
- `version` (optional): Target firmware version string

### Status Topic (Publish)

**Topic:** `paku/edge/{device_id}/ota/status`

**Payload (JSON):**
```json
{
  "timestamp": "2025-12-08T12:34:56Z",
  "status": "accepted",
  "version": "1.2.0"
}
```

Published when OTA command is received and accepted.

### Progress Topic (Publish)

**Topic:** `paku/edge/{device_id}/ota/progress`

**Payload (JSON):**
```json
{
  "timestamp": "2025-12-08T12:35:10Z",
  "state": "Downloading",
  "percent": 45,
  "downloaded": 458752,
  "total": 1048576,
  "elapsed_ms": 15234
}
```

Published periodically during update (every second).

**States:**
- `Idle` - No update in progress
- `Checking` - Checking for updates
- `Downloading` - Downloading firmware
- `Verifying` - Verifying checksum
- `Installing` - Installing firmware
- `Complete` - Update completed
- `Failed` - Update failed

### Result Topic (Publish)

**Topic:** `paku/edge/{device_id}/ota/result`

**Payload (JSON):**
```json
{
  "timestamp": "2025-12-08T12:36:45Z",
  "version": "1.2.0",
  "success": true,
  "result_code": 0,
  "message": "Success"
}
```

Published when update completes (success or failure).

## Firmware Binary Requirements

### Partition Scheme

The device must use a partition scheme with two OTA app partitions:
- OTA_0 (ota_0)
- OTA_1 (ota_1)

The recommended partition scheme is `min_spiffs.csv` (configured in `platformio.ini`):
```csv
# Name,   Type, SubType, Offset,  Size,     Flags
nvs,      data, nvs,     0x9000,  0x5000,
otadata,  data, ota,     0xe000,  0x2000,
app0,     app,  ota_0,   0x10000, 0x1E0000,
app1,     app,  ota_1,   0x1F0000,0x1E0000,
spiffs,   data, spiffs,  0x3D0000,0x30000,
```

This gives ~1.9MB per OTA partition, which is sufficient for most firmware.

### Building Firmware

Build the firmware using PlatformIO:

```bash
cd paku_core
pio run
```

The binary will be located at:
```
paku_core/.pio/build/esp32-ch340c-30pin/firmware.bin
```

### Hosting Firmware

The firmware binary must be accessible via HTTP or HTTPS. Options:

1. **Static web server** (Apache, Nginx, etc.)
2. **Cloud storage** (AWS S3, Google Cloud Storage, etc.)
3. **GitHub releases**
4. **paku-iot backend** (custom endpoint)

Example using Python HTTP server for testing:
```bash
cd paku_core/.pio/build/esp32-ch340c-30pin/
python3 -m http.server 8080
```

Then use URL: `http://your-ip:8080/firmware.bin`

### Calculating Checksum

Generate SHA256 checksum:
```bash
sha256sum firmware.bin
```

Or on macOS:
```bash
shasum -a 256 firmware.bin
```

The output is a 64-character hex string. Example:
```
a1b2c3d4e5f6789012345678901234567890abcdef1234567890abcdef123456
```

## Triggering an Update

### Using MQTT Explorer

1. Connect to your MQTT broker
2. Navigate to topic: `paku/edge/{device_id}/cmd/ota`
3. Publish JSON payload:
```json
{
  "url": "http://192.168.1.100:8080/firmware.bin",
  "checksum": "a1b2c3d4e5f6789012345678901234567890abcdef1234567890abcdef123456",
  "version": "1.2.0"
}
```

### Using mosquitto_pub

```bash
mosquitto_pub -h mqtt.server.com -p 1883 \
  -t "paku/edge/paku-AABBCCDD/cmd/ota" \
  -m '{"url":"http://192.168.1.100:8080/firmware.bin","checksum":"a1b2...","version":"1.2.0"}'
```

### Using paku-iot Backend (Future)

The paku-iot backend will provide a REST API for triggering updates:

```bash
curl -X POST https://iot.paku.example.com/api/devices/paku-AABBCCDD/ota \
  -H "Authorization: Bearer $API_KEY" \
  -H "Content-Type: application/json" \
  -d '{"version": "1.2.0"}'
```

## Update Process

1. **Receive Command**: Device receives OTA command via MQTT
2. **Validate**: Check if WiFi is connected and update not already in progress
3. **Acknowledge**: Publish status message indicating command accepted
4. **Download**: Download firmware binary from URL
5. **Verify**: Calculate SHA256 checksum and compare with expected value
6. **Install**: Write firmware to inactive OTA partition
7. **Report**: Publish result message
8. **Reboot**: Automatically restart to apply new firmware
9. **Validate**: On first boot, validate firmware is working correctly

## Rollback

If the new firmware fails to boot or doesn't validate itself, the ESP32 will automatically rollback to the previous firmware on the next reboot.

### Manual Rollback

To manually trigger a rollback (if needed for development):

```cpp
OtaResult result = otaClient.rollback();
if (result == OtaResult::SUCCESS) {
  ESP.restart();
}
```

## Security Considerations

### Checksum Verification

Always provide a SHA256 checksum to verify firmware integrity:
- Detects corrupted downloads
- Prevents installation of tampered firmware
- Essential for production deployments

### HTTPS

Use HTTPS for firmware downloads when possible:
- Encrypts firmware during transit
- Prevents man-in-the-middle attacks
- Verifies server identity (with proper CA certificate)

Currently, the client accepts any certificate (`.setInsecure()`). For production, configure with proper CA certificate:

```cpp
_secureClient.setCACert(ca_cert);
```

### Signature Verification (Future)

Digital signature verification will be added in a future release:
- Verifies firmware was built by trusted source
- Uses public/private key cryptography
- Prevents unauthorized firmware updates

### Network Security

- Use WPA2/WPA3 for WiFi
- Use TLS for MQTT if supported by broker
- Restrict MQTT topic access with ACLs
- Use strong passwords/API keys

## Troubleshooting

### Update Fails to Download

**Symptom**: Update fails with "Download failed" error

**Solutions**:
- Check WiFi connection is stable
- Verify firmware URL is accessible from device
- Check firewall rules allow HTTP/HTTPS
- Increase timeout if network is slow
- Verify server is serving correct content-type

### Checksum Mismatch

**Symptom**: Update fails with "Verification failed" error

**Solutions**:
- Verify checksum was calculated correctly
- Check firmware binary wasn't corrupted during upload
- Ensure URL points to correct firmware version
- Try downloading firmware manually to verify

### Insufficient Space

**Symptom**: Update fails with "Firmware too large" error

**Solutions**:
- Use partition scheme with larger OTA partitions
- Optimize firmware size (disable unused features)
- Use compression if supported

### Device Doesn't Reboot

**Symptom**: Update reports success but device doesn't restart

**Solutions**:
- Device may be waiting for other tasks to complete
- Check serial output for errors
- Manually trigger reboot if needed

### Rollback After Update

**Symptom**: Device reboots to old firmware after successful update

**Solutions**:
- New firmware may have failed to validate
- Check for crash/exception during first boot
- Verify new firmware calls `validateCurrentFirmware()`
- Review boot logs for errors

## Testing

### Local Testing

1. Build firmware with different version number
2. Host firmware on local HTTP server
3. Trigger update via MQTT
4. Monitor progress on serial console
5. Verify device reboots with new firmware

### Simulated Failure Testing

Test rollback functionality:

1. Modify firmware to crash on boot
2. Trigger OTA update
3. Device should automatically rollback after failed boot
4. Verify device is running old firmware

### Network Interruption Testing

Test resumable downloads (if implemented):

1. Start OTA update
2. Disconnect WiFi during download
3. Reconnect WiFi
4. Verify update resumes or fails gracefully

## Integration with paku-iot

The paku-iot backend should:

1. **Store firmware binaries**
   - Keep versioned firmware files
   - Generate checksums automatically
   - Serve files via HTTPS

2. **Track device versions**
   - Record current firmware version per device
   - Log update history
   - Monitor update success rate

3. **Provide update API**
   - REST endpoint to trigger updates
   - Batch update support
   - Rollout strategies (canary, phased)

4. **Monitor update status**
   - Subscribe to OTA progress topics
   - Display update status in UI
   - Alert on failures

## Best Practices

1. **Always verify checksums** - Don't skip checksum verification in production
2. **Test updates** - Test new firmware on a development device first
3. **Staged rollout** - Update devices gradually, not all at once
4. **Version control** - Use semantic versioning for firmware
5. **Backup firmware** - Keep previous firmware versions available
6. **Monitor updates** - Watch serial output during first update
7. **Document changes** - Maintain changelog for firmware versions
8. **Handle failures** - Implement proper error handling and reporting

## Appendix: Error Codes

| Code | Name | Description |
|------|------|-------------|
| 0 | SUCCESS | Operation successful |
| 1 | ERROR_WIFI_NOT_CONNECTED | WiFi not connected |
| 2 | ERROR_DOWNLOAD_FAILED | Failed to download firmware |
| 3 | ERROR_VERIFICATION_FAILED | Checksum verification failed |
| 4 | ERROR_INSTALL_FAILED | Failed to install firmware |
| 5 | ERROR_NO_PARTITION | No OTA partition available |
| 6 | ERROR_INVALID_URL | Invalid firmware URL |
| 7 | ERROR_TIMEOUT | Operation timed out |
| 8 | ERROR_INSUFFICIENT_SPACE | Not enough space for firmware |
| 9 | ERROR_INVALID_FIRMWARE | Invalid firmware image |
| 10 | ERROR_ALREADY_RUNNING | Update already in progress |
| 11 | ERROR_ROLLBACK_FAILED | Rollback operation failed |

## Example Workflow

Complete example of OTA update process:

```bash
# 1. Build new firmware
cd paku_core
pio run

# 2. Calculate checksum
cd .pio/build/esp32-ch340c-30pin
sha256sum firmware.bin
# Output: a1b2c3d4...

# 3. Start local web server
python3 -m http.server 8080

# 4. Trigger update (in another terminal)
mosquitto_pub -h localhost -t "paku/edge/paku-AABBCCDD/cmd/ota" \
  -m '{"url":"http://192.168.1.100:8080/firmware.bin","checksum":"a1b2c3d4...","version":"1.2.0"}'

# 5. Monitor progress
mosquitto_sub -h localhost -t "paku/edge/paku-AABBCCDD/ota/#" -v

# 6. Watch serial output
pio device monitor

# 7. Verify new firmware
# Device should reboot and show new version in serial output
```
