# Heater Control and Reporting Interval Fix

**Date:** 2025-12-12  
**Issue:** Devices only reporting for 15 minutes every hour

## Root Cause

The firmware had a bug in the `updateIntervals()` function that caused it to switch from fast reporting mode to slow mode after 1 hour, even when the heater was still on. This created the "15 minutes every hour" pattern because:

1. Device starts with `heaterStatus = 1` (on)
2. Fast mode (MQTT every 10s) runs for first hour
3. After 1 hour, switches to slow mode (MQTT every 1 hour)
4. In slow mode, sensors still read every minute but MQTT only sends once per hour
5. Buffer (`MAX_MQTT_PAYLOADS = 30`) fills up after ~15 minutes
6. Additional sensor readings are silently dropped
7. Every hour, the 15 minutes of buffered data is sent at once

## Changes Made

### 1. Fixed `updateIntervals()` Function

**Before:**
```cpp
void updateIntervals() {
  static unsigned long heaterOnStartTime = 0;

  if (heaterStatus == 1) {
    if (heaterOnStartTime == 0) {
      heaterOnStartTime = millis();
    }

    if (millis() - heaterOnStartTime >= 3600000) { // 1 hour
      mqttInterval = mqttSlowInterval;  // Switch to slow after 1 hour
      sensorInterval = sensorSlowInterval;
    } else {
      mqttInterval = mqttFastInterval;
      sensorInterval = sensorFastInterval;
    }
  } else {
    heaterOnStartTime = 0;
    mqttInterval = mqttSlowInterval;
    sensorInterval = sensorSlowInterval;
  }
}
```

**After:**
```cpp
void updateIntervals() {
  if (heaterStatus == 1) {
    // Heater is on - use fast intervals for the entire duration
    mqttInterval = mqttFastInterval;   // 10 seconds
    sensorInterval = sensorFastInterval; // 5 seconds
  } else {
    // Heater is off - use slow intervals
    mqttInterval = mqttSlowInterval;    // 1 hour
    sensorInterval = sensorSlowInterval; // 1 minute
  }
}
```

Now the device stays in fast mode as long as the heater is on, not just for the first hour.

### 2. Added Heater Control via MQTT

**Updated:** `handleMqttMessage()` function

Added support for controlling the heater via MQTT messages to the `paku/control` topic.

**MQTT Command Format:**
```json
{
  "heater": 1
}
```

- `"heater": 1` - Turn heater on (enables fast reporting: 10s MQTT, 5s sensors)
- `"heater": 0` - Turn heater off (enables slow reporting: 1h MQTT, 1min sensors)

### 3. Changed Default Heater Status

**Before:** `int heaterStatus = 1;` (always on)  
**After:** `int heaterStatus = 0;` (off by default, controlled via MQTT)

## Reporting Intervals

### Fast Mode (Heater ON)
- **MQTT interval:** 10 seconds
- **Sensor interval:** 5 seconds
- **Use case:** When heater is running and you need frequent updates

### Slow Mode (Heater OFF)
- **MQTT interval:** 1 hour
- **Sensor interval:** 1 minute
- **Use case:** When heater is off and you want to conserve bandwidth/power

## How to Use

### 1. Turn Heater On
```bash
mosquitto_pub -h YOUR_MQTT_BROKER -t "paku/control" -m '{"heater":1}'
```

The device will:
- Switch to fast mode (10s MQTT, 5s sensors)
- Stay in fast mode as long as heater is on
- Report data continuously every 10 seconds

### 2. Turn Heater Off
```bash
mosquitto_pub -h YOUR_MQTT_BROKER -t "paku/control" -m '{"heater":0}'
```

The device will:
- Switch to slow mode (1h MQTT, 1min sensors)
- Collect data every minute
- Send accumulated data once per hour

### 3. Check from Server
```bash
ssh -i ~/.ssh/ychefla-GitHub paku@static.107.192.27.37.clients.your-server.de \
  "docker exec paku_mosquitto mosquitto_pub -h localhost -t 'paku/control' -m '{\"heater\":1}'"
```

## Testing

After flashing the updated firmware:

1. **Verify default state** (heater off, slow mode):
   - Device should report every hour
   - Check logs: `Serial.println("Heater status: 0")`

2. **Turn heater on** via MQTT:
   - Device should switch to fast mode immediately
   - Check logs: `Serial.println("Heater status updated to: 1")`
   - Verify data in Grafana arriving every 10 seconds

3. **Turn heater off** via MQTT:
   - Device should switch to slow mode
   - Check logs: `Serial.println("Heater status updated to: 0")`
   - Verify data in Grafana arriving once per hour

## Deployment

### Flash Updated Firmware

**For ESP8266 (paku-96036100_wired):**
```bash
cd paku-core/paku_core
pio run -e esp8266-wired-sensors --target upload
```

**For ESP32 (paku-20E955A0 and ESP32Client):**
```bash
cd paku-core/paku_core
pio run -e esp32-ch340c-30pin --target upload
```

### Via OTA (if configured)

The system supports OTA updates via MQTT. Deploy through the paku-iot workflow.

## Monitoring

Check device behavior with:

```bash
# View MQTT messages
docker exec paku_mosquitto mosquitto_sub -h localhost -t 'paku/#' -v

# Check database for reporting frequency
docker exec paku_postgres psql -U paku -d paku -c \
  "SELECT date_trunc('minute', ts) as minute, COUNT(*) 
   FROM measurements 
   WHERE ts > NOW() - INTERVAL '1 hour' 
   GROUP BY date_trunc('minute', ts) 
   ORDER BY minute DESC;"
```

## Notes

- Heater status is now controlled entirely via MQTT
- Default state is OFF (slow mode)
- No more automatic switch to slow mode after 1 hour
- Fast mode continues as long as heater is on
- The 30-payload buffer is no longer an issue in fast mode (sends every 10s)

## Files Modified

- `paku_core/src/main.cpp`:
  - Line 162: Changed `heaterStatus` default from 1 to 0
  - Lines 1396-1408: Simplified `updateIntervals()` - removed 1-hour timer
  - Lines 1534-1570: Added heater control to `handleMqttMessage()`
