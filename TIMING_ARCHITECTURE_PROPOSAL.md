# Edge Device Timing Architecture - Proposal

**Date:** 2025-12-12  
**Status:** PROPOSAL - Not Yet Implemented

## Problem Statement

Current firmware conflates multiple timing concerns into a single "heater status" flag:
- Sensor sampling rate
- MQTT publish rate  
- WiFi connection duration
- Power management (sleep/wake cycles)

This creates issues:
1. **Power inefficiency**: Device stays awake and connected longer than necessary
2. **Inflexible**: Can't independently adjust sensor vs network timing
3. **Not synchronized**: Sensors read at different times, requiring multiple wake cycles
4. **Not configurable**: Timing hardcoded in firmware, requires reflashing to change

## Proposed Architecture

### Core Principles

1. **Separation of Concerns**
   - **Sensor Timing**: How often and how many samples to collect
   - **Network Timing**: How often and how long to connect to WiFi/MQTT
   - **Power Management**: When to sleep, how deep, wake triggers

2. **Synchronization**
   - Read all sensors together in a single wake cycle
   - Only connect WiFi when there's data to transmit
   - Minimize wake time to essentials

3. **Remote Configuration**
   - Each edge device configured via MQTT topic
   - Schema-compliant: `{site_id}/edge/{device_id}/config`
   - Live updates without reflashing

4. **Power Efficiency**
   - WiFi off during sensor sampling
   - Deep sleep between wake cycles
   - Adaptive timing based on scenario (e.g., heater on/off)

---

## MQTT Schema Compliance

### Topic Structure

Following `{site_id}/{system}/{device_id}/{topic_type}`:

| Purpose | Topic | Direction |
|---------|-------|-----------|
| Device Config | `paku/edge/{device_id}/config` | Server → Device |
| Device Status | `paku/edge/{device_id}/status` | Device → Server |
| Sensor Data | `paku/sensors/{sensor_id}/data` | Device → Server |
| Heater Data | `paku/heater/{heater_id}/data` | Device → Server |
| Flow Data | `paku/flow/{flow_id}/data` | Device → Server |

### Example Device IDs

- `paku/edge/paku-96036100/config` - ESP8266 wired sensor device
- `paku/edge/paku-20E955A0/config` - ESP32 with BLE scanner
- `paku/edge/esp32-main/config` - Primary ESP32 controller

---

## Configuration Schema

### Edge Device Configuration

**Topic:** `paku/edge/{device_id}/config`

```json
{
  "version": "1.0",
  "timing": {
    "wake_interval_s": 60,
    "connection_duration_max_s": 30,
    "wifi_connect_timeout_s": 10,
    "mqtt_connect_timeout_s": 5
  },
  "sensors": {
    "ble": {
      "enabled": true,
      "scan_duration_s": 20,
      "scan_active": true,
      "expected_devices": ["D2:C9:0C:3A:2D:E1", "D3:A2:D0:F0:2F:74"]
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
    "deep_sleep_enabled": true,
    "light_sleep_during_wait": true,
    "battery_monitor_enabled": false
  },
  "scenarios": {
    "default": {
      "wake_interval_s": 60
    },
    "heater_active": {
      "wake_interval_s": 10,
      "ble_scan_duration_s": 10
    },
    "power_save": {
      "wake_interval_s": 300,
      "ble_scan_duration_s": 30
    }
  },
  "active_scenario": "default"
}
```

### Scenario Control

**Topic:** `paku/edge/{device_id}/control`

```json
{
  "scenario": "heater_active"
}
```

Changes active scenario on the fly (e.g., switch to fast mode when heater turns on).

---

## Wake Cycle Flow

### Optimal Wake Cycle Design

```
┌─────────────────────────────────────────────────────────────┐
│ DEEP SLEEP (WiFi off, minimal power)                       │
│ Duration: wake_interval_s - cycle_execution_time           │
└─────────────────────────────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│ PHASE 1: SENSOR SAMPLING (WiFi OFF)                        │
│ ┌─────────────────────────────────────────────────────┐   │
│ │ • Wake from sleep                                    │   │
│ │ • Initialize sensors                                 │   │
│ │ • BLE scan (if enabled): scan_duration_s            │   │
│ │ • Read wired sensors: sample_count samples          │   │
│ │ • Read flow sensor: measurement_duration_s          │   │
│ │ • Buffer all readings                                │   │
│ └─────────────────────────────────────────────────────┘   │
│ Time: ~20-30s (configurable)                               │
└─────────────────────────────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│ PHASE 2: DATA TRANSMISSION (WiFi ON)                       │
│ ┌─────────────────────────────────────────────────────┐   │
│ │ • Enable WiFi                                        │   │
│ │ • Connect to WiFi (max wifi_connect_timeout_s)      │   │
│ │ • Connect to MQTT (max mqtt_connect_timeout_s)      │   │
│ │ • Publish all buffered sensor data                   │   │
│ │ • Subscribe to config updates                        │   │
│ │ • Wait for pending messages (brief)                  │   │
│ │ • Disconnect gracefully                              │   │
│ │ • Disable WiFi                                       │   │
│ └─────────────────────────────────────────────────────┘   │
│ Time: ~5-10s (actual network time)                         │
└─────────────────────────────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│ PHASE 3: SLEEP PREPARATION                                 │
│ • Calculate next wake time                                  │
│ • Apply any config updates received                         │
│ • Configure deep sleep timer                                │
│ • Enter deep sleep                                          │
└─────────────────────────────────────────────────────────────┘
```

### Power Savings

**Current approach (fast mode):**
- Awake continuously: 100% (WiFi on most of the time)
- Power: ~170mA continuous (ESP32 with WiFi)

**Proposed approach (scenario: default):**
- Awake: ~30s every 60s = 50% duty cycle
- WiFi on: ~10s every 60s = 16.7% duty cycle
- Power: ~50mA average (30s @ 80mA WiFi off + 10s @ 170mA WiFi on + 20s @ 10uA sleep)

**Proposed approach (scenario: heater_active):**
- Awake: ~20s every 10s = variable
- WiFi on: ~5s every 10s = 50% duty cycle
- Power: ~85mA average

**Proposed approach (scenario: power_save):**
- Awake: ~30s every 300s = 10% duty cycle
- WiFi on: ~10s every 300s = 3.3% duty cycle
- Power: ~15mA average

---

## Implementation Plan

### Phase 1: Refactor Current Code

1. **Extract timing into configuration structure**
   ```cpp
   struct DeviceConfig {
     struct Timing {
       uint32_t wake_interval_s;
       uint32_t connection_duration_max_s;
       uint32_t wifi_connect_timeout_s;
       uint32_t mqtt_connect_timeout_s;
     } timing;
     
     struct SensorConfig {
       bool ble_enabled;
       uint32_t ble_scan_duration_s;
       bool wired_enabled;
       uint8_t wired_sample_count;
       bool flow_enabled;
       uint32_t flow_measurement_duration_s;
     } sensors;
     
     struct PowerConfig {
       bool deep_sleep_enabled;
       bool light_sleep_during_wait;
     } power;
   };
   
   DeviceConfig deviceConfig;
   ```

2. **Separate sensor reading from network operations**
   ```cpp
   void collectSensorData() {
     // WiFi OFF
     WiFi.mode(WIFI_OFF);
     
     // Read BLE sensors
     if (deviceConfig.sensors.ble_enabled) {
       scanBLESensors(deviceConfig.sensors.ble_scan_duration_s);
     }
     
     // Read wired sensors
     if (deviceConfig.sensors.wired_enabled) {
       readWiredSensors(deviceConfig.sensors.wired_sample_count);
     }
     
     // Read flow sensor
     if (deviceConfig.sensors.flow_enabled) {
       readFlowSensor(deviceConfig.sensors.flow_measurement_duration_s);
     }
   }
   
   void transmitData() {
     // WiFi ON
     WiFi.mode(WIFI_STA);
     if (!connectWiFiWithTimeout(deviceConfig.timing.wifi_connect_timeout_s)) {
       return; // Failed to connect
     }
     
     if (!connectMQTTWithTimeout(deviceConfig.timing.mqtt_connect_timeout_s)) {
       return; // Failed to connect
     }
     
     // Publish all buffered data
     publishAllData();
     
     // Brief wait for config updates
     for (int i = 0; i < 20; i++) {
       client.loop();
       delay(100);
     }
     
     // Disconnect
     client.disconnect();
     WiFi.disconnect(true);
     WiFi.mode(WIFI_OFF);
   }
   ```

3. **Replace loop() with state machine**
   ```cpp
   enum DeviceState {
     STATE_INIT,
     STATE_SENSOR_SAMPLING,
     STATE_DATA_TRANSMISSION,
     STATE_SLEEP_PREP,
     STATE_DEEP_SLEEP
   };
   
   void loop() {
     switch (currentState) {
       case STATE_SENSOR_SAMPLING:
         collectSensorData();
         currentState = STATE_DATA_TRANSMISSION;
         break;
         
       case STATE_DATA_TRANSMISSION:
         transmitData();
         currentState = STATE_SLEEP_PREP;
         break;
         
       case STATE_SLEEP_PREP:
         prepareForSleep();
         currentState = STATE_DEEP_SLEEP;
         break;
         
       case STATE_DEEP_SLEEP:
         enterDeepSleep(deviceConfig.timing.wake_interval_s);
         // Execution resumes at setup() after wake
         break;
     }
   }
   ```

### Phase 2: Add MQTT Configuration

1. **Subscribe to device-specific config topic**
   ```cpp
   String configTopic = String("paku/edge/") + deviceId + "/config";
   client.subscribe(configTopic.c_str());
   ```

2. **Parse config JSON and update deviceConfig**
   ```cpp
   void handleConfigMessage(String message) {
     JsonDocument doc;
     if (deserializeJson(doc, message) != DeserializationError::Ok) {
       return;
     }
     
     deviceConfig.timing.wake_interval_s = doc["timing"]["wake_interval_s"] | 60;
     deviceConfig.sensors.ble_scan_duration_s = doc["sensors"]["ble"]["scan_duration_s"] | 20;
     // ... parse all fields
     
     saveConfigToNVS(); // Persist across reboots
     Serial.println("Config updated from MQTT");
   }
   ```

3. **Support scenario switching**
   ```cpp
   void handleControlMessage(String message) {
     JsonDocument doc;
     if (deserializeJson(doc, message) != DeserializationError::Ok) {
       return;
     }
     
     String scenario = doc["scenario"];
     if (scenario == "heater_active") {
       deviceConfig.timing.wake_interval_s = 10;
       deviceConfig.sensors.ble_scan_duration_s = 10;
     } else if (scenario == "power_save") {
       deviceConfig.timing.wake_interval_s = 300;
       deviceConfig.sensors.ble_scan_duration_s = 30;
     } else { // default
       deviceConfig.timing.wake_interval_s = 60;
       deviceConfig.sensors.ble_scan_duration_s = 20;
     }
   }
   ```

### Phase 3: Optimize Power Management

1. **Implement proper deep sleep**
   ```cpp
   void enterDeepSleep(uint32_t seconds) {
     Serial.printf("Entering deep sleep for %d seconds\n", seconds);
     
     #ifdef ESP32
     esp_sleep_enable_timer_wakeup(seconds * 1000000ULL);
     esp_deep_sleep_start();
     #elif defined(ESP8266)
     // Requires D0 -> RST connection for wake
     ESP.deepSleep(seconds * 1000000ULL);
     #endif
   }
   ```

2. **Use light sleep during waits**
   ```cpp
   void lightSleepDelay(uint32_t ms) {
     if (deviceConfig.power.light_sleep_during_wait) {
       #ifdef ESP32
       esp_sleep_enable_timer_wakeup(ms * 1000ULL);
       esp_light_sleep_start();
       #else
       delay(ms);
       #endif
     } else {
       delay(ms);
     }
   }
   ```

---

## Migration Strategy

### Stage 1: Internal Refactoring (No External Changes)
- Refactor code to separate sensor/network phases
- Keep existing timing behavior
- Test thoroughly

### Stage 2: Add Configuration Support (Backward Compatible)
- Add MQTT config topic subscription
- Support both old (`paku/control`) and new (`paku/edge/{id}/config`) topics
- Default to current behavior if no config received

### Stage 3: Enable Power Management (Opt-in)
- Add deep sleep support (requires hardware check - D0->RST for ESP8266)
- Configurable via MQTT
- Default to always-on for backward compatibility

### Stage 4: Full Migration
- Deprecate `paku/control` topic
- Require config via MQTT
- Enable deep sleep by default

---

## Testing Plan

### Unit Tests
- Config parsing from JSON
- Scenario switching logic
- Timing calculations

### Integration Tests
1. **Default scenario**: Verify 60s wake cycle, all sensors working
2. **Heater active scenario**: Verify 10s wake cycle, faster updates
3. **Power save scenario**: Verify 5min wake cycle, longer BLE scans
4. **Config update**: Send new config via MQTT, verify applied
5. **Network failure**: Verify device continues to function if WiFi unavailable
6. **Power measurement**: Measure actual current draw in each scenario

---

## Example Configurations

### Development/Testing (Always On)
```json
{
  "timing": {
    "wake_interval_s": 5,
    "connection_duration_max_s": 60
  },
  "sensors": {
    "ble": {"enabled": true, "scan_duration_s": 10},
    "wired": {"enabled": true, "sample_count": 1},
    "flow": {"enabled": true, "measurement_duration_s": 5}
  },
  "power": {
    "deep_sleep_enabled": false
  }
}
```

### Production - Normal Operation
```json
{
  "timing": {
    "wake_interval_s": 60,
    "connection_duration_max_s": 30
  },
  "sensors": {
    "ble": {"enabled": true, "scan_duration_s": 20},
    "wired": {"enabled": true, "sample_count": 3},
    "flow": {"enabled": true, "measurement_duration_s": 5}
  },
  "power": {
    "deep_sleep_enabled": true
  },
  "scenarios": {
    "heater_active": {
      "wake_interval_s": 10
    }
  }
}
```

### Production - Battery Powered (Ultra Low Power)
```json
{
  "timing": {
    "wake_interval_s": 600,
    "connection_duration_max_s": 30
  },
  "sensors": {
    "ble": {"enabled": true, "scan_duration_s": 30},
    "wired": {"enabled": false},
    "flow": {"enabled": false}
  },
  "power": {
    "deep_sleep_enabled": true,
    "battery_monitor_enabled": true
  }
}
```

---

## Benefits Summary

1. **Power Efficiency**: 80%+ reduction in average power consumption (default scenario)
2. **Flexibility**: Per-device timing configuration via MQTT
3. **Synchronization**: All sensors read together, minimizing wake cycles
4. **Schema Compliance**: Follows documented MQTT hierarchy
5. **Scalability**: Easy to add new scenarios or sensors
6. **Debuggability**: Clear state machine, configurable logging
7. **Maintainability**: Separated concerns, testable components

---

## Open Questions

1. **ESP8266 Deep Sleep**: Requires D0->RST hardware connection. Are all devices wired this way?
2. **Config Storage**: Should config be persisted in NVS/EEPROM or fetched on every boot?
3. **OTA During Sleep**: How to handle OTA updates if device sleeps most of the time? (Could wake on special schedule)
4. **Battery Monitoring**: Should we auto-adjust timing based on battery level?
5. **Network Failures**: Max retries before giving up? Skip sleep cycle on failure?

---

## Next Steps

1. **Review this proposal** - Confirm approach aligns with system goals
2. **Prioritize phases** - Which parts are most critical?
3. **Hardware verification** - Check ESP8266 D0->RST connections
4. **Create test plan** - Define acceptance criteria for each phase
5. **Implementation timeline** - Estimate effort for each phase

