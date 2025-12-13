# Phase 2: Decouple Sensors from Network and Add State Machine

**Date:** 2025-12-13  
**Status:** IN PROGRESS

## Objectives

1. **Separate sensor sampling from network transmission**
   - Sensors read while WiFi is OFF (power savings)
   - Data buffered locally
   - WiFi only ON during transmission

2. **Add MQTT configuration support**
   - Subscribe to `paku/edge/{deviceId}/config`
   - Accept full JSON configuration via MQTT
   - Persist configuration to NVS

3. **State machine architecture**
   - Replace continuous loop with state-based flow
   - Clear state transitions
   - Prepare for sleep implementation (Phase 3)

## Architecture Changes

### State Machine

```
┌─────────────────────────────────────────────────────┐
│                                                       │
│  IDLE                                                │
│  - Wait for sensor interval                          │
│  - WiFi OFF                                          │
│  - Light sleep possible                              │
│                                                       │
└───────────────────┬───────────────────────────────────┘
                    │ sensor_interval elapsed
                    ▼
┌─────────────────────────────────────────────────────┐
│                                                       │
│  COLLECT_SENSORS                                     │
│  - Read BLE sensors                                  │
│  - Read wired sensors                                │
│  - Read flow sensor                                  │
│  - Store in buffer                                   │
│  - WiFi still OFF                                    │
│                                                       │
└───────────────────┬───────────────────────────────────┘
                    │ collection complete
                    ▼
┌─────────────────────────────────────────────────────┐
│                                                       │
│  CONNECT_NETWORK                                     │
│  - Turn WiFi ON                                      │
│  - Connect to AP                                     │
│  - Connect MQTT                                      │
│  - Subscribe to topics                               │
│                                                       │
└───────────────────┬───────────────────────────────────┘
                    │ connected OR timeout
                    ▼
┌─────────────────────────────────────────────────────┐
│                                                       │
│  TRANSMIT                                            │
│  - Send buffered sensor data                         │
│  - Publish status                                    │
│  - Process incoming messages                         │
│  - Handle OTA if pending                             │
│                                                       │
└───────────────────┬───────────────────────────────────┘
                    │ transmission complete OR timeout
                    ▼
┌─────────────────────────────────────────────────────┐
│                                                       │
│  DISCONNECT                                          │
│  - Disconnect MQTT                                   │
│  - Turn WiFi OFF                                     │
│  - Clear transmission flags                          │
│                                                       │
└───────────────────┬───────────────────────────────────┘
                    │
                    ▼
                  IDLE (loop)
```

### Data Structures

#### Sensor Data Buffer
```cpp
struct SensorReading {
  char timestamp[32];
  char sensor_id[32];
  char metric[32];
  float value;
  bool transmitted;
};

#define MAX_BUFFERED_READINGS 50
SensorReading sensorBuffer[MAX_BUFFERED_READINGS];
int bufferCount = 0;
```

#### System State
```cpp
enum SystemState {
  STATE_IDLE,
  STATE_COLLECT_SENSORS,
  STATE_CONNECT_NETWORK,
  STATE_TRANSMIT,
  STATE_DISCONNECT
};

SystemState currentSystemState = STATE_IDLE;
unsigned long stateEnteredAt = 0;
```

## Implementation Steps

### Step 1: Add NVS/Preferences Support

- Add Preferences library for ESP32
- Add EEPROM emulation for ESP8266
- Create `saveConfig()` and `loadConfig()` functions
- Load config on boot

### Step 2: Create Data Buffer

- Define `SensorReading` struct
- Create buffer array
- Add `addSensorReading()` function
- Add `clearTransmittedReadings()` function

### Step 3: Refactor Sensor Collection

- Create `collectSensorData()` function
- Move BLE scanning logic (no network needed)
- Move wired sensor reading (no network needed)
- Move flow sensor reading (no network needed)
- Store all readings in buffer

### Step 4: Refactor Network/Transmission

- Create `connectNetwork()` function
- Create `transmitBufferedData()` function
- Create `disconnectNetwork()` function
- Only connect when needed

### Step 5: Implement State Machine

- Add state enum and variables
- Create `handleState()` function
- Replace main loop with state machine
- Add state transition logic with timeouts

### Step 6: Add Config Subscription

- Subscribe to `paku/edge/{deviceId}/config` in `connectMQTT()`
- Parse full JSON config in `handleMqttMessage()`
- Update `deviceConfig` structure
- Call `saveConfig()` to persist
- Validate config before applying

## Timing Behavior

### Before (Phase 1)
- WiFi always ON
- MQTT always connected
- Sensors read every `sensorInterval`
- Data sent immediately after reading
- No buffering

### After (Phase 2)
- WiFi OFF most of the time
- MQTT only connected during transmission
- Sensors read every `sensorInterval` (WiFi OFF)
- Data buffered locally
- Transmission every `wake_interval_s` or when buffer fills

### Example Timeline (power_save scenario)

```
Time    State              WiFi    Action
----    -----------------  ----    ---------------------------
0s      COLLECT_SENSORS    OFF     Read sensors, store in buffer
2s      IDLE               OFF     Wait (light sleep possible)
60s     COLLECT_SENSORS    OFF     Read sensors again, buffer
62s     IDLE               OFF     Wait
120s    COLLECT_SENSORS    OFF     Read sensors again, buffer
122s    IDLE               OFF     Wait
...
300s    CONNECT_NETWORK    ON      Connect WiFi & MQTT
305s    TRANSMIT           ON      Send all 5 buffered readings
310s    DISCONNECT         OFF     Disconnect, go to IDLE
312s    IDLE               OFF     Wait for next cycle
```

## Configuration via MQTT

### Publish to config topic:
```bash
mosquitto_pub -h broker -t "paku/edge/paku-96036100/config" -m '{
  "timing": {
    "wake_interval_s": 600,
    "connection_duration_max_s": 30
  },
  "sensors": {
    "ble": {
      "enabled": true,
      "scan_duration_s": 5
    }
  }
}'
```

Device will:
1. Receive message
2. Validate JSON
3. Update `deviceConfig`
4. Save to NVS
5. Apply new timing
6. Publish updated config back (confirmation)

## Benefits

- ✅ WiFi OFF most of the time (major power savings)
- ✅ Sensor timing independent of network timing
- ✅ Data buffering (resilient to network failures)
- ✅ Full configuration via MQTT
- ✅ Configuration persists across reboots
- ✅ Clear state-based flow (easier to debug)
- ✅ Foundation for deep sleep (Phase 3)

## Testing Plan

1. **Verify sensor collection works with WiFi OFF**
   - Monitor serial output
   - Confirm BLE scanning works
   - Confirm wired sensors work

2. **Verify data buffering**
   - Collect multiple readings
   - Verify buffer fills correctly
   - Verify transmission sends all buffered data

3. **Verify network cycling**
   - Watch WiFi connect/disconnect
   - Monitor MQTT connection
   - Verify no data loss

4. **Test config persistence**
   - Send config via MQTT
   - Reboot device
   - Verify config persists

5. **Test different scenarios**
   - default (60s intervals)
   - heater_active (10s intervals)
   - power_save (300s intervals)
   - Verify WiFi cycle timing matches wake_interval_s

## Files to Modify

- `src/main.cpp` - Major refactoring (~300 lines changed/added)
- `include/device_config.h` - Add save/load methods
- `platformio.ini` - Add Preferences library dependency

## Backward Compatibility

- Legacy topics still supported
- Scenario switching still works
- Heater control still works
- All existing MQTT commands functional
