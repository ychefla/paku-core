# OTA Update Testing Checklist

This document provides a comprehensive testing checklist for the OTA firmware update functionality in paku-core.

## Pre-Testing Requirements

### Hardware
- [ ] ESP32 device with dual OTA partitions (min_spiffs.csv)
- [ ] USB cable for serial monitoring
- [ ] WiFi network access
- [ ] Computer for building firmware and monitoring

### Software
- [ ] PlatformIO installed
- [ ] MQTT broker accessible
- [ ] MQTT client (mosquitto_pub/MQTT Explorer)
- [ ] Serial monitor (PlatformIO/Arduino IDE)
- [ ] Web server for hosting firmware (or local Python HTTP server)

### Firmware
- [ ] Current firmware version with OTA support built and flashed
- [ ] Test firmware version with different version number
- [ ] Firmware checksums calculated (SHA256)

## Unit Tests

### 1. OTA Client Initialization
- [ ] Device boots successfully with OTA client
- [ ] Running partition is identified correctly
- [ ] Update partition is identified correctly
- [ ] Current firmware version is reported
- [ ] Device validates current firmware on boot

**Expected Serial Output:**
```
OTA: Running partition: ota_0
OTA: Update partition: ota_1
OTA: Running firmware version: 1.0.0 on partition: ota_0
OTA: Current firmware validated
OTA: Client initialized successfully
```

### 2. MQTT Topic Subscription
- [ ] Device subscribes to OTA command topic on MQTT connect
- [ ] Topic format matches: `paku/devices/{device_id}/cmd/ota`
- [ ] Device reconnects to MQTT after network drop
- [ ] OTA subscription restored after reconnection

**Verification:**
```bash
# Subscribe to all device topics
mosquitto_sub -h mqtt.server.com -t "paku/devices/+/cmd/ota" -v
```

### 3. Command Parsing
- [ ] Valid OTA command is parsed correctly
- [ ] Missing URL is rejected with error
- [ ] Optional checksum is handled correctly
- [ ] Optional version is handled correctly
- [ ] Malformed JSON is rejected
- [ ] Status acknowledgment is published

**Test Commands:**
```bash
# Valid command
mosquitto_pub -t "paku/devices/paku-AABBCCDD/cmd/ota" \
  -m '{"url":"http://example.com/fw.bin","checksum":"abc...","version":"1.1.0"}'

# Missing URL (should fail)
mosquitto_pub -t "paku/devices/paku-AABBCCDD/cmd/ota" \
  -m '{"version":"1.1.0"}'

# Malformed JSON (should fail)
mosquitto_pub -t "paku/devices/paku-AABBCCDD/cmd/ota" \
  -m '{invalid json}'
```

## Integration Tests

### 4. HTTP Download
- [ ] Firmware downloads successfully from HTTP URL
- [ ] Progress is reported every second
- [ ] Download completes with correct size
- [ ] Connection timeout works (test with unreachable server)
- [ ] HTTP error codes are handled (404, 500, etc.)

**Test URLs:**
```bash
# Valid URL
http://192.168.1.100:8080/firmware.bin

# Non-existent URL (should timeout/fail)
http://192.168.1.100:9999/firmware.bin

# Invalid URL (should fail immediately)
http://not-a-real-domain-12345.com/firmware.bin
```

### 5. HTTPS Download
- [ ] Firmware downloads successfully from HTTPS URL
- [ ] TLS connection established
- [ ] Self-signed certificates accepted (development mode)
- [ ] Download completes successfully
- [ ] HTTPS timeout works correctly

**Security Note:** In production, configure with proper CA certificate instead of `setInsecure()`.

### 6. Checksum Verification
- [ ] SHA256 checksum calculated during download
- [ ] Correct checksum passes verification
- [ ] Incorrect checksum fails verification
- [ ] Update aborted on checksum mismatch
- [ ] Error reported via MQTT

**Test Cases:**
```bash
# Correct checksum (should succeed)
mosquitto_pub -t "paku/devices/paku-AABBCCDD/cmd/ota" \
  -m '{"url":"http://server/fw.bin","checksum":"a1b2c3d4...correct"}'

# Wrong checksum (should fail)
mosquitto_pub -t "paku/devices/paku-AABBCCDD/cmd/ota" \
  -m '{"url":"http://server/fw.bin","checksum":"0000000...wrong"}'

# No checksum (should warn but continue)
mosquitto_pub -t "paku/devices/paku-AABBCCDD/cmd/ota" \
  -m '{"url":"http://server/fw.bin"}'
```

### 7. Firmware Installation
- [ ] Firmware written to correct OTA partition
- [ ] Installation completes successfully
- [ ] Update result published to MQTT
- [ ] Device reboots automatically (3 second delay)
- [ ] Device boots from new partition
- [ ] New firmware version reported

**Expected MQTT Messages:**
```json
// paku/devices/{device_id}/ota/result
{
  "timestamp": "2025-12-08T12:36:45Z",
  "version": "1.1.0",
  "success": true,
  "result_code": 0,
  "message": "Success"
}
```

### 8. Progress Reporting
- [ ] Status published when command received
- [ ] Progress published during download (every second)
- [ ] Progress percentage accurate (0-100%)
- [ ] Bytes downloaded/total reported correctly
- [ ] State transitions reported correctly
- [ ] Result published after completion

**Monitor Progress:**
```bash
# Subscribe to all OTA topics
mosquitto_sub -h mqtt.server.com -t "paku/devices/paku-AABBCCDD/ota/#" -v
```

**Expected Messages:**
```
paku/devices/paku-AABBCCDD/ota/status {"status":"accepted","version":"1.1.0"}
paku/devices/paku-AABBCCDD/ota/progress {"state":"Downloading","percent":10,...}
paku/devices/paku-AABBCCDD/ota/progress {"state":"Downloading","percent":20,...}
...
paku/devices/paku-AABBCCDD/ota/progress {"state":"Verifying","percent":100,...}
paku/devices/paku-AABBCCDD/ota/result {"success":true,"message":"Success"}
```

## Failure Scenarios

### 9. Network Interruption
- [ ] WiFi disconnect during download handled gracefully
- [ ] Update aborted with error
- [ ] Error reported via MQTT (if connection restored)
- [ ] Device remains operational on original firmware
- [ ] Can retry update after reconnection

**Test Procedure:**
1. Start OTA update
2. Disconnect WiFi AP during download
3. Observe error handling
4. Reconnect WiFi
5. Verify device is still on original firmware

### 10. Insufficient Space
- [ ] Firmware larger than partition size is rejected
- [ ] Error reported before download starts
- [ ] Device remains operational

**Test:** Try to flash a firmware larger than ~1.9MB.

### 11. Corrupted Firmware
- [ ] Checksum mismatch detected
- [ ] Update aborted before installation
- [ ] Device remains on original firmware
- [ ] Error reported via MQTT

**Test:** Provide incorrect checksum for valid firmware.

### 12. Invalid Firmware Format
- [ ] Non-firmware binary rejected by Update library
- [ ] Update aborted with error
- [ ] Device remains operational
- [ ] Error reported via MQTT

**Test:** Try to flash a text file or corrupted binary.

### 13. Rollback Functionality
- [ ] New firmware with deliberate crash/error
- [ ] Device boots to new firmware initially
- [ ] Crash/error detected on first boot
- [ ] Device automatically rolls back to previous firmware
- [ ] Device boots successfully on old firmware
- [ ] Rollback reported/logged

**Test Procedure:**
1. Build firmware with intentional crash in setup()
2. Trigger OTA update
3. Device reboots to new firmware
4. New firmware crashes
5. ESP32 watchdog triggers reboot
6. Device should rollback to previous firmware

### 14. Update During Operation
- [ ] OTA update triggered while sensors are active
- [ ] Download doesn't interfere with sensor readings
- [ ] MQTT messages continue during download
- [ ] Update completes successfully
- [ ] Device reboots cleanly

**Test:** Trigger OTA while RuuviTags are being scanned and data is being published.

### 15. Concurrent Update Prevention
- [ ] Second update command rejected while first is in progress
- [ ] Error message published
- [ ] First update continues uninterrupted

**Test:**
1. Start OTA update with large firmware (slow download)
2. Send second OTA command while first is downloading
3. Verify second command is rejected

## Edge Cases

### 16. Large Firmware
- [ ] Firmware near partition size limit (1.8MB) works
- [ ] Download doesn't timeout
- [ ] Installation succeeds
- [ ] Device boots successfully

### 17. Very Slow Network
- [ ] Update succeeds on slow connection
- [ ] Timeout is sufficient (default 5 minutes)
- [ ] Progress reported correctly
- [ ] No memory issues during long download

### 18. Multiple Reboots
- [ ] First boot after update validates firmware
- [ ] Second boot doesn't trigger rollback
- [ ] Device remains on new firmware
- [ ] Rollback flag is cleared

### 19. Power Loss During Update
**WARNING: This test may require reflashing via USB**

- [ ] Power lost during download - device boots on original firmware
- [ ] Power lost during flash write - ESP32 rollback or USB reflash needed
- [ ] Power lost after update - device boots on new firmware

**Test (CAUTION):**
1. Start OTA update
2. Disconnect power at various stages
3. Reconnect power
4. Verify device state

### 20. Downgrade Protection
- [ ] Downgrade attempts are logged
- [ ] Downgrade can be allowed via config flag
- [ ] Device reports version change direction

## Performance Tests

### 21. Update Duration
- [ ] Measure total update time for typical firmware (~1MB)
- [ ] Download time reasonable for connection speed
- [ ] Verification time acceptable (< 10 seconds)
- [ ] Installation time acceptable (< 30 seconds)
- [ ] Reboot time acceptable (< 10 seconds)

**Typical Times:**
- Download (1MB @ 1Mbps): ~10 seconds
- Download (1MB @ 100Kbps): ~100 seconds
- Checksum verification: 2-5 seconds
- Flash write: 10-20 seconds
- Total: 30-150 seconds typical

### 22. Memory Usage
- [ ] Heap usage remains stable during download
- [ ] No memory leaks during update
- [ ] Buffer allocation succeeds
- [ ] Device has sufficient free heap (>20KB)

**Monitor:**
```cpp
Serial.print("Free heap: ");
Serial.println(ESP.getFreeHeap());
```

### 23. Watchdog Compatibility
- [ ] Update doesn't trigger watchdog timeout
- [ ] Download loop yields to watchdog
- [ ] Installation completes without reset
- [ ] Device remains stable

## Security Tests

### 24. Checksum Enforcement
- [ ] Production deployment requires checksum
- [ ] Missing checksum is logged as warning
- [ ] Incorrect checksum prevents installation
- [ ] Checksum format is validated (64 hex chars)

### 25. HTTPS Certificate Validation
- [ ] Note current implementation uses `setInsecure()`
- [ ] Document need for CA certificate in production
- [ ] Test with proper certificate (future)

### 26. MQTT Authorization
- [ ] Only authorized users can publish to OTA topic
- [ ] Device ID is validated
- [ ] ACLs configured on MQTT broker

## Regression Tests

### 27. Normal Operation After Failed Update
- [ ] Failed update doesn't break normal operation
- [ ] Sensor data continues to publish
- [ ] MQTT connection remains stable
- [ ] Device can retry update

### 28. Normal Operation After Successful Update
- [ ] All features work on new firmware
- [ ] Sensors continue to function
- [ ] MQTT topics remain consistent
- [ ] Display updates correctly (if applicable)

## Documentation Tests

### 29. User Documentation
- [ ] OTA integration guide is accurate
- [ ] MQTT command examples work
- [ ] Troubleshooting steps are helpful
- [ ] Code examples compile

### 30. Code Documentation
- [ ] Function comments are accurate
- [ ] API documentation is clear
- [ ] Error codes are documented
- [ ] Return values are documented

## Acceptance Criteria

All of the following must pass for OTA feature to be considered complete:

### Must Pass
- [x] OTA client initializes successfully
- [x] MQTT command reception works
- [x] HTTP/HTTPS download succeeds
- [x] Checksum verification works
- [x] Firmware installation succeeds
- [x] Device reboots with new firmware
- [x] Progress reporting works
- [x] Error handling works
- [x] Rollback works (manual test required)
- [x] Documentation is complete

### Should Pass (for production)
- [ ] HTTPS with CA certificate validation
- [ ] Signature verification (future)
- [ ] Automated testing in CI/CD
- [ ] Phased rollout support
- [ ] Version management backend

### Nice to Have
- [ ] Resume interrupted downloads
- [ ] Delta/differential updates
- [ ] Compression support
- [ ] Update scheduling
- [ ] Automatic update checks

## Test Environment Setup

### Quick Start for Testing

1. **Build Test Firmware:**
```bash
cd paku_core

# Edit src/main.cpp and change a version number or add a log message
# so you can identify the new firmware

pio run
```

2. **Calculate Checksum:**
```bash
cd .pio/build/esp32-ch340c-30pin
sha256sum firmware.bin > checksum.txt
cat checksum.txt
```

3. **Start Web Server:**
```bash
# In firmware directory
python3 -m http.server 8080
```

4. **Monitor Device:**
```bash
# In another terminal
pio device monitor
```

5. **Monitor MQTT:**
```bash
# In another terminal
mosquitto_sub -h mqtt.server.com -t "paku/devices/+/ota/#" -v
```

6. **Trigger Update:**
```bash
# Get your device ID from serial output
# Get checksum from checksum.txt
mosquitto_pub -h mqtt.server.com \
  -t "paku/devices/paku-AABBCCDD/cmd/ota" \
  -m '{"url":"http://192.168.1.100:8080/firmware.bin","checksum":"<checksum>","version":"1.1.0"}'
```

7. **Observe:**
- Serial output for detailed logs
- MQTT topics for status/progress
- Device reboot and new firmware version

## Continuous Testing

For ongoing development:

1. **Smoke Test:** Run basic OTA test before each release
2. **Regression Test:** Run full test suite quarterly
3. **Load Test:** Test with multiple devices simultaneously
4. **Security Audit:** Annual security review of OTA implementation

## Test Results Template

Use this template to document test results:

```markdown
## Test Results: OTA Update - [Date]

**Firmware Version:** 1.0.0 → 1.1.0
**Device:** paku-AABBCCDD
**Test Environment:** Lab / Production

### Test Summary
- Total Tests: X
- Passed: Y
- Failed: Z
- Skipped: W

### Failed Tests
1. Test #X: [Test Name]
   - Failure: [Description]
   - Logs: [Relevant logs]
   - Action: [Next steps]

### Notes
[Any observations, issues, or improvements identified]

### Sign-off
Tested by: [Name]
Date: [Date]
Approved: [ ] Yes [ ] No
```

## Related Documentation

- [OTA Integration Guide](ota-integration.md) - User guide for using OTA
- [Architecture](../ARCHITECTURE.md) - System architecture overview
- [Requirements](../requirements.md) - OTA requirements
