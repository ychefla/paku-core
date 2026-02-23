# Device Status and Config Publishing - Update

**Date:** 2025-12-13  
**Status:** IMPLEMENTED

## Changes Made

Added publishing of device status and configuration to MQTT topics so they are visible in MQTT Explorer.

### New Functions

#### `publishDeviceStatus()`
Publishes to: `paku/edge/{deviceId}/status`

Content includes:
- `online`: true/false
- `last_seen`: ISO8601 timestamp
- `signal_strength_dbm`: WiFi RSSI
- `uptime_seconds`: Device uptime
- `firmware_version`: Current firmware version
- `state`: Current operational state
- `heater_status`: 0 or 1
- `active_scenario`: "default", "heater_active", or "power_save"

Published with QoS 1 and retained flag.

#### `publishDeviceConfig()`
Publishes to: `paku/edge/{deviceId}/config`

Content includes complete device configuration:
- `timing`: wake_interval_s, connection_duration_max_s, wifi_connect_timeout_s, mqtt_connect_timeout_s
- `sensors.ble`: enabled, scan_duration_s, scan_active
- `sensors.wired`: enabled, sample_count, sample_interval_ms
- `sensors.flow`: enabled, measurement_duration_s
- `power`: deep_sleep_enabled, light_sleep_during_wait, battery_monitor_enabled

Published with QoS 1 and retained flag.

### When Published

1. **On MQTT connection**: Both status and config published after successful MQTT connect
2. **On scenario change**: Both status and config published when switching scenarios (via either `paku/control` or `paku/edge/{deviceId}/control`)
3. **Periodic updates**: Status published every 60 seconds in the main loop

### MQTT Topics Now Visible

In MQTT Explorer you should now see:

```
paku/
├── control                           (legacy control topic - subscribed)
├── edge/
│   └── {deviceId}/
│       ├── control                   (new control topic - subscribed)
│       ├── status                    (published - retained)
│       └── config                    (published - retained)
├── sensors/
│   └── {sensor_id}/
│       └── data                      (sensor telemetry)
└── devices/
    └── {deviceId}/
        ├── cmd/
        │   └── ota                   (OTA commands - subscribed)
        └── telemetry/
            └── ...                   (device telemetry)
```

### Example Status Message

```json
{
  "online": true,
  "last_seen": "2025-12-13T00:05:23Z",
  "signal_strength_dbm": -52,
  "uptime_seconds": 3847,
  "firmware_version": "1.2.3",
  "state": "continuous",
  "heater_status": 1,
  "active_scenario": "heater_active"
}
```

### Example Config Message

```json
{
  "version": "1.0",
  "timing": {
    "wake_interval_s": 10,
    "connection_duration_max_s": 30,
    "wifi_connect_timeout_s": 10,
    "mqtt_connect_timeout_s": 5
  },
  "sensors": {
    "ble": {
      "enabled": true,
      "scan_duration_s": 10,
      "scan_active": true
    },
    "wired": {
      "enabled": true,
      "sample_count": 3,
      "sample_interval_ms": 100
    },
    "flow": {
      "enabled": true,
      "measurement_duration_s": 5
    }
  },
  "power": {
    "deep_sleep_enabled": false,
    "light_sleep_during_wait": true,
    "battery_monitor_enabled": false
  }
}
```

### Testing

After flashing the updated firmware:

1. **Connect to MQTT broker and watch topics**:
   ```bash
   mosquitto_sub -h YOUR_BROKER -t 'paku/edge/#' -v
   ```

2. **Device should publish on connect**:
   - Status message to `paku/edge/{deviceId}/status`
   - Config message to `paku/edge/{deviceId}/config`

3. **Change scenario and verify updates**:
   ```bash
   # Switch to power_save
   mosquitto_pub -h YOUR_BROKER -t "paku/edge/paku-96036100/control" -m '{"scenario":"power_save"}'
   
   # Should see updated status with active_scenario: "power_save"
   # Should see updated config with wake_interval_s: 300
   ```

4. **Verify in MQTT Explorer**:
   - Browse to `paku/edge/{deviceId}/`
   - Should see `status` and `config` topics
   - Both should be retained (persist after disconnect)

5. **Periodic updates**:
   - Status updates automatically every 60 seconds
   - Watch for `last_seen` timestamp changes

### Benefits

- ✅ Device configuration visible in MQTT Explorer
- ✅ Device status monitoring (uptime, WiFi strength, etc.)
- ✅ Active scenario visible at a glance
- ✅ Retained messages survive device/broker restarts
- ✅ Conforms to documented MQTT schema

### Files Modified

- `paku_core/src/main.cpp`: Added ~120 lines
  - `publishDeviceStatus()` function
  - `publishDeviceConfig()` function
  - Calls in `connectMQTT()`, `handleMqttMessage()`, and `loop()`
