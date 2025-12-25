# Logging System Guide

## Overview

The paku-core firmware includes a flexible, multi-level logging system that allows you to control debug output at both global and per-feature levels. This is useful for:
- **Development**: Enable detailed debug output for specific features
- **Production**: Disable all logging to reduce power consumption
- **Troubleshooting**: Enable only the categories relevant to your issue

## Configuration

### Location
Configure logging in `include/secrets.h` by adding defines **before** any other includes.

### Logging Levels

#### 1. Master Control
```cpp
#define LOG_ENABLED 1  // 1 = logging on, 0 = completely silent
```

#### 2. Standard Information
```cpp
#define LOG_INFO_ENABLED 1  // Status, connections, key events
```

#### 3. Debug Categories
Enable specific debug categories (multiple can be active simultaneously):

```cpp
// Network
#define LOG_DEBUG_WIFI 1     // WiFi connection, scanning, credentials
#define LOG_DEBUG_MQTT 1     // MQTT connection, publish, subscribe

// Sensors
#define LOG_DEBUG_BLE 1                // BLE scanning and device discovery
#define LOG_DEBUG_SENSORS_RUUVI 1      // RuuviTag data parsing
#define LOG_DEBUG_SENSORS_MOKO 1       // MoKo sensor data parsing
#define LOG_DEBUG_SENSORS_WIRED 1      // DS18B20 and wired sensors
#define LOG_DEBUG_SENSORS_FLOW 1       // Flow meter sensors

// System
#define LOG_DEBUG_POWER 1     // Battery, sleep modes
#define LOG_DEBUG_OTA 1       // OTA firmware updates
#define LOG_DEBUG_CONFIG 1    // Configuration changes
#define LOG_DEBUG_TIMING 1    // State machine, intervals
#define LOG_DEBUG_BUFFER 1    // Sensor snapshot buffering
```

## Usage Examples

### Example 1: Silent Mode (No Logging)
```cpp
// secrets.h
#define LOG_ENABLED 0  // Everything disabled
```

### Example 2: Standard Info Only
```cpp
// secrets.h
#define LOG_ENABLED 1
#define LOG_INFO_ENABLED 1
// No debug categories enabled
```
**Output:**
```
[INFO] [System] WiFi connected
[INFO] [System] MQTT connected
[INFO] [System] Published sensor data
```

### Example 3: Debug WiFi and MQTT
```cpp
// secrets.h
#define LOG_ENABLED 1
#define LOG_INFO_ENABLED 1
#define LOG_DEBUG_WIFI 1
#define LOG_DEBUG_MQTT 1
```
**Output:**
```
[INFO] [System] Starting WiFi connection...
[DEBUG] [WiFi] Trying SSID: MyNetwork
[DEBUG] [WiFi] Connected, IP: 192.168.1.100
[DEBUG] [WiFi] Signal strength: -45 dBm
[INFO] [System] MQTT connecting...
[DEBUG] [MQTT] Connecting to broker: mqtt.example.com:1883
[DEBUG] [MQTT] Connected successfully
[DEBUG] [MQTT] Subscribed to: paku/devices/ESP32-ABC123/cmd
[DEBUG] [MQTT] Publishing to: paku/sensors/ruuvi_cabin/data
```

### Example 4: Debug RuuviTags Only
```cpp
// secrets.h
#define LOG_ENABLED 1
#define LOG_INFO_ENABLED 1
#define LOG_DEBUG_BLE 1
#define LOG_DEBUG_SENSORS_RUUVI 1
```
**Output:**
```
[INFO] [System] Starting BLE scan...
[DEBUG] [BLE] Found 5 devices
[DEBUG] [BLE] Device: AA:BB:CC:DD:EE:FF, RSSI: -65
[DEBUG] [Ruuvi] Parsing manufacturer data (19 bytes)
[DEBUG] [Ruuvi] Format: RAWv2
[DEBUG] [Ruuvi] Temperature: 22.5°C, Humidity: 45%
[INFO] [System] RuuviTag data updated
```

### Example 5: Multiple Categories
```cpp
// secrets.h
#define LOG_ENABLED 1
#define LOG_INFO_ENABLED 1
#define LOG_DEBUG_WIFI 1
#define LOG_DEBUG_MQTT 1
#define LOG_DEBUG_SENSORS_WIRED 1
#define LOG_DEBUG_BUFFER 1
```
**Output:**
```
[DEBUG] [WiFi] Connected to: JosPar_2.4G
[INFO] [System] Reading DS18B20 sensor...
[DEBUG] [Wired] DS18B20 address: 28:FF:12:34:56:78:90:AB
[DEBUG] [Wired] Raw temperature: 22.06°C
[DEBUG] [Buffer] Adding to buffer: wired_sensor (slot 0/32)
[DEBUG] [MQTT] Publishing: paku/sensors/ESP8266-96036100_wired/data
[INFO] [System] Transmission complete
```

## Code Usage

### In main.cpp or other source files:

```cpp
#include "logging.h"

// Standard informational messages
LOG_INFO("WiFi", "Connected to %s", ssid);
LOG_INFO("System", "Device ID: %s", deviceId.c_str());

// Feature-specific debug
LOG_DEBUG_WIFI("Attempting connection to %s", ssid);
LOG_DEBUG_MQTT("Published to topic: %s, size: %d bytes", topic, length);
LOG_DEBUG_RUUVI("MAC: %s, Temp: %.2f°C, Humidity: %.1f%%", 
                macStr, temp, humidity);

// Generic debug (always enabled if LOG_ENABLED)
LOG_DEBUG("MyFeature", "Custom debug message: %d", value);
```

## Output Format

All log messages follow this format:
```
[LEVEL] [Category] Message
```

Examples:
- `[INFO] [System] WiFi connected`
- `[DEBUG] [WiFi] Signal strength: -45 dBm`
- `[DEBUG] [Ruuvi] Temperature: 22.5°C`

## Performance Impact

- **LOG_ENABLED = 0**: Zero overhead, all logging code compiled out
- **LOG_INFO_ENABLED only**: Minimal overhead (~2-5 ms per message)
- **Multiple debug categories**: Moderate overhead (~5-10 ms per message)

For battery-powered deployments, consider:
- Disable all logging for production
- Enable only during development/troubleshooting
- Use specific categories instead of enabling everything

## Startup Output

On boot, the device displays its logging configuration:
```
=== Logging Configuration ===
Master: ENABLED
Info:   ON

Debug Categories:
  WiFi:        ON
  MQTT:        ON
  BLE:         OFF
  Ruuvi:       OFF
  MoKo:        OFF
  Wired:       ON
  Flow:        OFF
  Power:       OFF
  OTA:         OFF
  Config:      OFF
  Timing:      OFF
  Buffer:      ON
=============================
```

## Troubleshooting

### Problem: No output at all
- Check `LOG_ENABLED 1` is set
- Check serial monitor is running at 115200 baud
- Check USB cable is connected

### Problem: Too much output
- Disable unneeded debug categories
- Keep only `LOG_INFO_ENABLED 1`

### Problem: Missing specific debug messages
- Ensure the specific `LOG_DEBUG_xxx` is set to `1`
- Check that `LOG_ENABLED 1` is set

## Adding New Categories

To add a new debug category:

1. **In `logging.h`**, add the define and macro:
```cpp
#ifndef LOG_DEBUG_MYNEWFEATURE
  #define LOG_DEBUG_MYNEWFEATURE 0
#endif

#if LOG_DEBUG_MYNEWFEATURE
  #define LOG_DEBUG_MYNEWFEATURE(fmt, ...) \
    Serial.printf("[DEBUG] [MyFeature] " fmt "\n", ##__VA_ARGS__)
#else
  #define LOG_DEBUG_MYNEWFEATURE(fmt, ...)
#endif
```

2. **In code**, use the new macro:
```cpp
LOG_DEBUG_MYNEWFEATURE("My custom message: %d", value);
```

3. **In `secrets.h.template`**, document it:
```cpp
// #define LOG_DEBUG_MYNEWFEATURE 1   // Description of feature
```
