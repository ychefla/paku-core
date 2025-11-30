# RuuviTag Integration Guide

This document describes how paku-core collects data from RuuviTag BLE sensors and transmits it to paku-iot.

## Overview

paku-core scans for RuuviTag BLE advertisements, parses the sensor data, and transmits it via MQTT and HTTP to paku-iot. The implementation supports:

- **Automatic discovery** of RuuviTags in range
- **Pre-registration** of known tags with location names
- **Live data** collection from temperature, humidity, pressure, and battery sensors
- **Historical data** download placeholder for future expansion
- **Placeholder data** generation for sensors not yet implemented

## Hardware Requirements

### RuuviTag Sensors

paku-core supports RuuviTag sensors broadcasting in RAWv2 format (format 5):

- **RuuviTag Pro** - Recommended for extended temperature range
- **RuuviTag** - Standard model for indoor use

### Manufacturer ID

RuuviTags use manufacturer ID `0x0499` in their BLE advertisements.

### Data Format

The RAWv2 (format 5) data structure:

| Offset | Field | Resolution | Range |
|--------|-------|------------|-------|
| 0 | Format (0x05) | - | - |
| 1-2 | Temperature | 0.005°C | -163.835°C to +163.835°C |
| 3-4 | Humidity | 0.0025% | 0% to 163.835% |
| 5-6 | Pressure | 1 Pa | 50000 Pa to 115535 Pa |
| 7-8 | Acceleration X | 1 mG | ±32.767 G |
| 9-10 | Acceleration Y | 1 mG | ±32.767 G |
| 11-12 | Acceleration Z | 1 mG | ±32.767 G |
| 13-14 | Power info | - | TX power + Battery voltage |
| 15 | Movement counter | 1 | 0-255 |
| 16-17 | Sequence number | 1 | 0-65535 |

## Configuration

### Pre-registering Known Tags

To pre-register known RuuviTags with specific locations, add the following to `secrets.h`:

```cpp
// Number of known RuuviTags
#define RUUVI_TAG_COUNT 4

// MAC addresses of known tags (uppercase, colon-separated)
static const char* RUUVI_TAG_MACS[] = {
    "AA:BB:CC:DD:EE:01",  // Cabin
    "AA:BB:CC:DD:EE:02",  // Kitchen  
    "AA:BB:CC:DD:EE:03",  // Lounge
    "AA:BB:CC:DD:EE:04"   // Dryer
};

// Location names corresponding to each MAC address
static const char* RUUVI_TAG_LOCATIONS[] = {
    "cabin",
    "kitchen",
    "lounge",
    "dryer"
};
```

### Auto-Discovery

If `RUUVI_TAG_COUNT` is not defined or is 0, paku-core will automatically discover RuuviTags during BLE scans. Discovered tags are assigned location names like `tag_0`, `tag_1`, etc.

### Stale Data Timeout

Data from a RuuviTag is considered stale after 5 minutes (300000 ms) of no updates. This can be modified in `ruuvi_scanner.h`:

```cpp
#define RUUVI_STALE_TIMEOUT_MS 300000  // 5 minutes
```

## Data Flow

### Collection

```
┌─────────────┐      BLE Scan       ┌──────────────┐
│  RuuviTag   │ ─────────────────►  │  paku-core   │
│  (Sensor)   │   Advertisement     │  (ESP32)     │
└─────────────┘                     └──────┬───────┘
                                           │
                                           ▼
                              ┌────────────────────────┐
                              │  Parse RAWv2 Format    │
                              │  Update Tag Registry   │
                              └────────────┬───────────┘
                                           │
                                           ▼
                              ┌────────────────────────┐
                              │  Create MQTT Payloads  │
                              │  Create HTTP Batch     │
                              └────────────┬───────────┘
                                           │
            ┌──────────────────────────────┼──────────────────────────────┐
            ▼                              ▼                              ▼
    ┌───────────────┐            ┌───────────────┐            ┌───────────────┐
    │  MQTT Broker  │            │  paku-iot     │            │  Serial Log   │
    │  (Publish)    │            │  (HTTP POST)  │            │  (Debug)      │
    └───────────────┘            └───────────────┘            └───────────────┘
```

### MQTT Topics

RuuviTag data is published to the following MQTT topics:

| Topic | Data Type | Unit | Description |
|-------|-----------|------|-------------|
| `paku/temperature/ruuvi/{location}` | float | °C | Temperature |
| `paku/humidity/ruuvi/{location}` | float | % | Humidity |
| `paku/pressure/ruuvi/{location}` | float | hPa | Barometric pressure |
| `paku/voltage/ruuvi/{location}` | float | V | Battery voltage |

Where `{location}` is the registered location name (e.g., "cabin", "kitchen").

### Payload Format

All MQTT payloads use the standard paku JSON format:

```json
{
  "value": 22.5,
  "timestamp": "12:34:56"
}
```

### HTTP Transport

When `PAKU_IOT_ENABLED` is set, RuuviTag data is also sent via HTTP to paku-iot with metrics like:

- `temperature/ruuvi/cabin`
- `humidity/ruuvi/cabin`
- etc.

## Placeholder Data

For sensors not yet implemented, placeholder values are generated:

- **Value**: `-1000` (SENSOR_NOT_AVAILABLE)
- **Purpose**: Maintains topic structure for future sensor integration
- **Locations**: cabin, dryer, kitchen, lounge

Placeholder topics:

```
paku/temperature/moko/{location}  
paku/humidity/moko/{location}
paku/temperature/heating/floor
paku/temperature/heating/heater_in
paku/temperature/heating/heater_out
paku/power/heat
paku/power/cool
paku/voltage/car
paku/voltage/leisure
paku/status/heater
paku/status/heater_timer
paku/status/pump
```

## Historical Data Download

> **Note**: Historical data download is a placeholder feature. Standard RuuviTags broadcast data via BLE advertisements and do not support direct memory download. This feature is reserved for future GATT-based implementations or compatible devices.

The `downloadTagHistory()` function exists but returns 0 entries in the current implementation.

## API Reference

### RuuviScanner Module

#### Functions

```cpp
// Initialize the scanner
bool initRuuviScanner(void);

// Register a known tag
bool registerRuuviTag(const char* macAddress, const char* location);

// Get number of registered tags
uint8_t getRegisteredTagCount(void);

// Get a registered tag by index
const RuuviTag* getRegisteredTag(uint8_t index);

// Find tag by MAC address
const RuuviTag* findRegisteredTagByMac(const char* macAddress);

// Update tag data from BLE scan
bool updateRuuviTagData(const uint8_t* macAddress, const uint8_t* data, 
                        size_t length, unsigned long currentTime);

// Get tags with fresh (non-stale) data
uint8_t getFreshTags(const RuuviTag** tags, uint8_t maxTags, unsigned long currentTime);

// Check if manufacturer ID is Ruuvi
bool isRuuviManufacturer(uint16_t manufacturerId);
```

#### Structures

```cpp
struct RuuviTag {
    uint8_t macAddress[6];      // MAC address bytes
    char macString[18];         // MAC as string
    char location[32];          // Location name
    RuuviData lastData;         // Parsed sensor data
    unsigned long lastSeen;     // Last update time (millis)
    bool registered;            // Explicitly registered
    bool hasData;               // Has valid data
};

struct RuuviData {
    float temperature;          // °C
    float humidity;             // %
    float pressure;             // Pa
    float accelerationX;        // G
    float accelerationY;        // G
    float accelerationZ;        // G
    float batteryVoltage;       // V
    int8_t txPower;             // dBm
    uint8_t movementCounter;    
    uint16_t measurementSequence;
    bool valid;                 // Parsing success
};
```

## Troubleshooting

### No RuuviTags Detected

1. Verify RuuviTag has battery installed and is broadcasting
2. Check that ESP32 BLE is initialized (see serial output)
3. Ensure RuuviTag is within range (typically 10-30m)
4. Verify RuuviTag is in RAWv2 mode (default for recent firmware)

### Stale Data

If data appears stale (not updating):

1. Check `RUUVI_STALE_TIMEOUT_MS` setting
2. Verify BLE scan task is running (check serial output)
3. Ensure RuuviTag battery is not depleted

### MAC Address Not Found

If pre-registered tags are not being matched:

1. Verify MAC address format is uppercase with colons: `AA:BB:CC:DD:EE:FF`
2. Check that MAC matches exactly (case-insensitive comparison is used)
3. Verify the tag is broadcasting

## Related Documents

- [Architecture](../ARCHITECTURE.md) - System architecture overview
- [Integration Guide](../INTEGRATION.md) - paku-iot integration
- [Configuration Reference](config.md) - Configuration options
- [E2E Test Guide](e2e-test-ruuvi.md) - End-to-end testing with real hardware
