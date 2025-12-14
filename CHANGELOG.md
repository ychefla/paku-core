# Changelog

All notable changes to the paku-core firmware will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.1.0] - 2025-12-13

### Added
- State machine architecture with 5 states (IDLE, COLLECT_SENSORS, CONNECT_NETWORK, TRANSMIT, DISCONNECT)
- Sensor data buffering system with 50-reading capacity
- Configuration persistence to NVS (ESP32) and EEPROM (ESP8266)
- Remote configuration updates via MQTT topic `paku/edge/{deviceId}/config`
- Network decoupling - WiFi turns OFF between sensor collections
- `addSensorReading()` function for storing sensor data locally
- `saveConfig()` and `loadConfig()` functions for persistent configuration
- `collectSensorData()` - sensor reading with WiFi OFF
- `connectNetwork()` and `disconnectNetwork()` - explicit network control
- `transmitBufferedData()` - batch transmission of buffered readings

### Fixed
- **ESP32 MQTT config publishing** - Increased MQTT buffer size to 512 bytes for both ESP32 environments
  - Added `-DMQTT_MAX_PACKET_SIZE=512` to `lilygo-t-display-s3` build flags
  - Added `-DMQTT_MAX_PACKET_SIZE=512` to `esp32-ch340c-30pin` build flags
  - Config messages (~350 bytes) were silently failing with default 256-byte buffer
  - ESP32 devices now successfully publish config to `paku/edge/{deviceId}/config`

### Changed
- Decoupled sensor collection from network transmission for power efficiency
- WiFi now only active during CONNECT_NETWORK and TRANSMIT states
- Configuration changes now persist across reboots
- Default build environment set to `esp32-ch340c-30pin`

### Performance
- **Power Savings:**
  - Before: WiFi continuously ON (40-80mA baseline)
  - After: WiFi OFF most of the time (~20-30mA in IDLE/COLLECT)
  - Network active only ~10-15 seconds per wake cycle
  - **~50-60% power reduction** in typical operation

## [1.0.0] - 2025-12-13

### Added
- Device status publishing to MQTT topic `paku/edge/{deviceId}/status`
- Device configuration publishing to MQTT topic `paku/edge/{deviceId}/config`
- Status includes: online status, last_seen timestamp, WiFi signal strength, uptime, firmware version, state, heater status, active scenario
- Config includes: timing parameters, sensor configuration (BLE/wired/flow), power settings
- QoS 1 and retained flags for both status and config topics
- Periodic status updates every 60 seconds
- Status/config publishing on MQTT connect and scenario changes

### Changed
- Device ID format changed from `paku-{chip_id}` to `{board_type}-{chip_id}`
  - Examples: `ESP32-20E955A0`, `ESP8266-96036100`
- Enhanced MQTT topic structure following documented schema
- Improved device visibility in MQTT Explorer and backend database

### Fixed
- Unique MQTT client ID using device-specific chip ID
- Proper MQTT connection handling and reconnection logic

## Earlier Versions

### [0.x.x] - Before 2025-12-13
- Basic sensor reading (Ruuvi BLE tags, DS18B20 temperature sensors)
- Flow meter support
- Heater control via MQTT
- WiFi and MQTT connectivity
- OTA update support
- ESP32 and ESP8266 platform support
- Multi-device configurations (LilyGo T-Display S3, ESP32 CH340C, ESP8266)

---

## Version Numbering

- **Major version (X.0.0)**: Incompatible API changes or major architectural changes
- **Minor version (0.X.0)**: New features in a backwards-compatible manner
- **Patch version (0.0.X)**: Backwards-compatible bug fixes

## Upgrading

To upgrade firmware:
1. Build with PlatformIO: `pio run --target upload`
2. Or use OTA update system (see docs/ota_updates.md)
3. Monitor serial output for version confirmation
4. Verify in MQTT status message: `firmware_version` field

## Related Documentation

- `ESP32_CONFIG_PUBLISHING_FIX.md` - Details of v1.1.0 MQTT buffer fix
- `PHASE2_COMPLETE.md` - Phase 2 state machine implementation
- `PHASE2_IMPLEMENTATION.md` - Phase 2 technical details
- `STATUS_CONFIG_PUBLISHING.md` - Status/config publishing documentation
- `TIMING_ARCHITECTURE_PROPOSAL.md` - Timing architecture design
