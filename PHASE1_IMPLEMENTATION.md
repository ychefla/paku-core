# Phase 1 Implementation - Timing Architecture Refactoring

**Date:** 2025-12-12  
**Status:** IMPLEMENTED - Ready for Testing

## Changes Made

Phase 1 focuses on internal refactoring to separate timing concerns while maintaining existing behavior. No external behavior changes - the device still operates continuously without sleep.

### 1. New Configuration Structure

**File:** `src/timing_config.h` (NEW)

Created a comprehensive configuration structure that separates:
- **Timing**: Wake intervals, connection timeouts
- **Sensors**: BLE, wired, and flow sensor settings  
- **Power**: Sleep and battery management settings

Key features:
- `DeviceConfig` struct with nested configuration sections
- `loadDefaults()` method for initialization
- `applyScenario()` method to switch between timing profiles
- `DeviceState` enum for future state machine implementation

### 2. Main Code Refactoring

**File:** `src/main.cpp` (MODIFIED)

#### Added:
- Include for `timing_config.h`
- Global `deviceConfig` instance
- `currentState` variable (initialized to `STATE_CONTINUOUS`)
- Better organized global variables with section comments

#### Updated setup():
```cpp
// Initialize device configuration with defaults
deviceConfig.loadDefaults();

// Apply scenario based on heater status (maintains existing behavior)
if (heaterStatus == 1) {
    deviceConfig.applyScenario("heater_active");
} else {
    deviceConfig.applyScenario("default");
}

// Set initial state for continuous operation (no sleep in Phase 1)
currentState = STATE_CONTINUOUS;

// Print configuration for debugging
Serial.printf("Wake interval: %d seconds\n", deviceConfig.timing.wake_interval_s);
Serial.printf("BLE scan duration: %d seconds\n", deviceConfig.sensors.ble.scan_duration_s);
Serial.printf("Deep sleep: %s\n", deviceConfig.power.deep_sleep_enabled ? "enabled" : "disabled");
```

#### Updated handleMqttMessage():
Now supports TWO control topics:

1. **Legacy topic** (backward compatible):
   - Topic: `paku/control`
   - Payload: `{"heater": 1}` or `{"heater": 0}`
   - Updates `heaterStatus` and applies corresponding scenario

2. **New schema-compliant topic**:
   - Topic: `paku/edge/{deviceId}/control`
   - Payload: `{"scenario": "heater_active"}` or `{"scenario": "default"}` or `{"scenario": "power_save"}`
   - Directly switches device scenario
   - Also updates `heaterStatus` for backward compatibility

#### Updated connectMQTT():
Added subscription to new edge control topic:
```cpp
// Subscribe to legacy control topic (backward compatibility)
client.subscribe("paku/control");

// Subscribe to new edge device control topic (schema-compliant)
String edgeControlTopic = String("paku/edge/") + deviceId + "/control";
client.subscribe(edgeControlTopic.c_str());
```

### 3. Scenarios Implemented

Three timing scenarios are now available:

#### default
- Wake interval: 60 seconds
- BLE scan: 20 seconds
- Use case: Normal monitoring

#### heater_active
- Wake interval: 10 seconds
- BLE scan: 10 seconds  
- Use case: Fast updates when heater is on

#### power_save
- Wake interval: 300 seconds (5 minutes)
- BLE scan: 30 seconds
- Use case: Battery-powered operation

---

## Backward Compatibility

✅ **Fully backward compatible**

- All existing code paths still work
- Legacy `paku/control` topic still supported
- Existing `heaterStatus` variable still used
- `updateIntervals()` function unchanged
- No changes to sensor reading or MQTT publishing
- No sleep mode enabled (continuous operation as before)

---

## Testing Phase 1

### 1. Compile the Firmware

```bash
cd /Users/jossu/GIT/paku/paku-core/paku_core

# For ESP32
platformio run -e esp32-ch340c-30pin

# For ESP8266
platformio run -e esp8266-wired-sensors
```

### 2. Flash to Device

```bash
# For ESP32
platformio run -e esp32-ch340c-30pin --target upload

# For ESP8266  
platformio run -e esp8266-wired-sensors --target upload
```

### 3. Test Legacy Control (Should Work as Before)

```bash
# Turn heater on (fast mode: 10s intervals)
mosquitto_pub -h YOUR_BROKER -t "paku/control" -m '{"heater":1}'

# Turn heater off (default mode: 60s intervals)
mosquitto_pub -h YOUR_BROKER -t "paku/control" -m '{"heater":0}'
```

### 4. Test New Schema-Compliant Control

```bash
# Switch to heater active scenario
mosquitto_pub -h YOUR_BROKER -t "paku/edge/paku-96036100/control" -m '{"scenario":"heater_active"}'

# Switch to power save scenario
mosquitto_pub -h YOUR_BROKER -t "paku/edge/paku-96036100/control" -m '{"scenario":"power_save"}'

# Switch back to default
mosquitto_pub -h YOUR_BROKER -t "paku/edge/paku-96036100/control" -m '{"scenario":"default"}'
```

### 5. Monitor Serial Output

Connect to serial monitor at 115200 baud to see:
```
Loading device configuration...
Wake interval: 10 seconds
BLE scan duration: 10 seconds
Deep sleep: disabled
Setup complete.
...
MQTT connected and subscribed to control topics
Subscribed to edge control topic: paku/edge/paku-96036100/control
Subscribed to OTA topic: paku/devices/paku-96036100/cmd/ota
```

When changing scenarios:
```
MQTT message received on topic: paku/edge/paku-96036100/control
Message: {"scenario":"power_save"}
Switching to scenario: power_save
```

### 6. Verify in Grafana

After testing control commands, verify in Grafana:
- Data continues to arrive at expected intervals
- Changing scenario adjusts reporting frequency
- No data loss or disruption

---

## What Phase 1 Does NOT Include

❌ Deep sleep implementation (Phase 3)
❌ WiFi on/off management (Phase 2)
❌ Sensor sampling separate from network (Phase 2)
❌ State machine loop refactoring (Phase 2)
❌ MQTT configuration topic support (Phase 2)

These will come in later phases.

---

## Files Modified

```
paku-core/paku_core/src/
├── timing_config.h          (NEW - 116 lines)
└── main.cpp                 (MODIFIED - added ~50 lines)
```

---

## Next Steps (Phase 2)

Once Phase 1 is tested and working:

1. **Separate sensor sampling from network**
   - Create `collectSensorData()` function
   - Create `transmitData()` function
   - WiFi off during sensor sampling

2. **Add MQTT config topic support**
   - Subscribe to `paku/edge/{deviceId}/config`
   - Parse full JSON configuration
   - Persist config to NVS

3. **Refactor main loop to state machine**
   - Replace current loop with state-based flow
   - Prepare for sleep implementation

---

## Summary

Phase 1 successfully refactors the timing architecture with:
- ✅ Clean separation of concerns in data structures
- ✅ Scenario-based configuration
- ✅ Schema-compliant MQTT topics (alongside legacy support)
- ✅ Full backward compatibility
- ✅ No behavioral changes
- ✅ Foundation for power management in Phase 2/3

The device still operates continuously (no sleep), but now has a clean architecture to support advanced power management features.
