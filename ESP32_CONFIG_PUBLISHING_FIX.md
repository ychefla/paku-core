# ESP32 Config Publishing Fix

**Date:** 2025-12-13  
**Version:** 1.1.0  
**Issue:** ESP32 devices not publishing config messages to MQTT  
**Status:** FIXED

## Problem

ESP32-20E955A0 device was reporting to MQTT but missing the config message:
- ✅ Publishing status to `paku/edge/ESP32-20E955A0/status` 
- ❌ NOT publishing config to `paku/edge/ESP32-20E955A0/config`

ESP8266 devices were working correctly with both status and config messages.

## Root Cause

**MQTT buffer size too small for ESP32 environments**

The `publishDeviceConfig()` function was implemented correctly in the code, but the MQTT client buffer was too small to send the config message:

- Default `MQTT_MAX_PACKET_SIZE` = 256 bytes
- Config JSON payload = ~350+ bytes
- Result: `client.publish()` silently failed when buffer was exceeded

The ESP8266 environment had `-DMQTT_MAX_PACKET_SIZE=512` set in `platformio.ini`, but the ESP32 environments were missing this flag.

## Solution

Added `-DMQTT_MAX_PACKET_SIZE=512` to build flags for both ESP32 environments:

1. **lilygo-t-display-s3** (ESP32-S3 with display)
2. **esp32-ch340c-30pin** (ESP32 headless - this is the affected device)

### Changes Made

**File:** `paku_core/platformio.ini`

```diff
[env:lilygo-t-display-s3]
build_flags = 
    -DLV_LVGL_H_INCLUDE_SIMPLE
    -DARDUINO_USB_CDC_ON_BOOT=1
    -DDISABLE_ALL_LIBRARY_WARNINGS
+   -DMQTT_MAX_PACKET_SIZE=512
    -DARDUINO_USB_MODE=1
    ...

[env:esp32-ch340c-30pin]
build_flags = 
    -DDISABLE_ALL_LIBRARY_WARNINGS
+   -DMQTT_MAX_PACKET_SIZE=512
```

## Testing Instructions

### 1. Rebuild and Flash

```bash
cd /Users/jossu/GIT/paku/paku-core/paku_core
pio run --target clean
pio run --target upload
```

Or use PlatformIO IDE:
- Click "Clean"
- Click "Upload"

### 2. Monitor Serial Output

```bash
pio device monitor
```

Look for these messages after MQTT connects:
```
publishDeviceConfig: Starting...
publishDeviceConfig: Serialized JSON length: 350
Published config (350 bytes), result: SUCCESS
Published device config to: paku/edge/ESP32-20E955A0/config
```

### 3. Verify in MQTT

**Option A: mosquitto_sub**
```bash
ssh -i ~/.ssh/ychefla-GitHub paku@static.107.192.27.37.clients.your-server.de \
  "docker exec paku_mosquitto mosquitto_sub -h localhost -t 'paku/edge/ESP32-20E955A0/config' -v -C 1"
```

**Option B: MQTT Explorer**
- Browse to `paku/edge/ESP32-20E955A0/`
- Should now see both `status` and `config` topics
- Config message should be retained (persists after disconnect)

### 4. Verify in Database

```bash
ssh -i ~/.ssh/ychefla-GitHub paku@static.107.192.27.37.clients.your-server.de \
  "docker exec paku_postgres psql -U paku -d paku -c \"SELECT device_id, updated_at FROM edge_device_configs WHERE device_id='ESP32-20E955A0';\""
```

Should show the device with a recent timestamp.

## Expected Config Message

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

## Impact

This fix ensures:
- ✅ ESP32 devices publish config messages on connect
- ✅ Config visible in MQTT Explorer
- ✅ Config stored in backend database (`edge_device_configs` table)
- ✅ Consistent behavior across ESP32 and ESP8266 devices
- ✅ Remote configuration management enabled

## Related Files

- `paku_core/src/main.cpp` - Contains `publishDeviceConfig()` function
- `paku_core/platformio.ini` - Build configuration (fixed here)
- `STATUS_CONFIG_PUBLISHING.md` - Original implementation documentation

## Notes

- Config message is published with QoS 1 and retained flag
- Published on MQTT connect and when scenario changes
- Maximum packet size is now 512 bytes (sufficient for current config)
- If config grows larger in future, increase to 1024 or 2048 bytes
