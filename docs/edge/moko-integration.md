# MoKo Sensor Integration Guide

This guide explains how to integrate MoKo Bluetooth sensors (H2, H3, H4 series) with paku-core ESP32 firmware.

## Overview

MoKo sensors are BLE-based environmental sensors that broadcast:
- **Temperature** (°C)
- **Humidity** (%)
- **Pressure** (hPa) - H4 models only
- **3-axis Accelerometer** (X, Y, Z in g)
- **Battery level** (% and voltage)

### Supported Models

- **H2**: Basic temperature/humidity sensor
- **H3**: Enhanced T/H sensor with accelerometer (most common)
- **H4**: Full-featured sensor with barometric pressure

## Hardware Setup

### 1. Configure Your MoKo Sensor

Using the official MoKo BeaconX Pro app:

1. **Power on** the sensor (insert battery)
2. **Connect** via Bluetooth
3. **Configure settings**:
   - Set broadcast interval (recommended: 1-5 seconds)
   - Enable all sensor features
   - Set transmit power (recommended: 0 dBm)
4. **Note the MAC address** (displayed in app)

### 2. Register Sensors in Firmware

Edit `paku_core/include/secrets.h`:

```cpp
// MoKo Sensor Configuration
#define MOKO_SENSOR_COUNT 2

static const char* MOKO_SENSOR_MACS[] = {
    "AA:BB:CC:DD:EE:11",  // Replace with your sensor's MAC
    "AA:BB:CC:DD:EE:12"
};

static const char* MOKO_SENSOR_LOCATIONS[] = {
    "garage",     // Friendly location name
    "outdoor"
};
```

**Note**: If you don't pre-register sensors, they will be auto-discovered with default names like `auto_AABB`.

### 3. Build and Flash Firmware

```bash
cd paku_core
pio run -t upload
pio device monitor  # View serial output
```

## Data Format

### MQTT Topic Structure

MoKo sensor data is published to:
```
paku/sensors/moko_{location}/data
```

### Payload Example

```json
{
  "timestamp": "2025-12-14T10:30:00Z",
  "device_id": "moko_garage",
  "location": "garage",
  "mac": "AA:BB:CC:DD:EE:11",
  "model": "H3",
  "metrics": {
    "temperature_c": 22.5,
    "humidity_percent": 45.2,
    "pressure_hpa": 1013.25,
    "battery_mv": 2950,
    "battery_percent": 95,
    "accelerometer": {
      "x": 0.02,
      "y": -0.01,
      "z": 0.98
    }
  }
}
```

## Integration with paku-iot

The paku-iot collector service automatically processes MoKo sensor data:

1. **Receives** MQTT messages from `paku/sensors/moko_*/data`
2. **Parses** JSON payload
3. **Stores** metrics in PostgreSQL database
4. **Visualizes** in Grafana dashboards

## Serial Console Output

When a MoKo sensor is detected, you'll see:

```
Starting Bluetooth scan...
Devices found: 5
MoKo sensor found: AA:BB:CC:DD:EE:11 (MK_H3)
  -> MoKo sensor data updated
MoKo sensor [garage] H3: T=22.5°C, H=45.2%
```

## Configuration Reference

### Scanner Settings

Default BLE scan parameters (in `main.cpp`):
```cpp
#define BLE_SCAN_INTERVAL_MS 10000  // Scan every 10 seconds
```

### Data Freshness

Sensors are considered "stale" after 5 minutes without updates:
```cpp
#define MOKO_STALE_TIMEOUT_MS 300000  // 5 minutes
```

Only fresh sensor data is published to MQTT.

### Maximum Sensors

By default, up to 8 MoKo sensors can be tracked simultaneously:
```cpp
#define MAX_MOKO_SENSORS 8
```

To increase this limit, edit `lib/paku_lib/src/moko_scanner.h`.

## Troubleshooting

### Sensor Not Detected

| Symptom | Cause | Solution |
|---------|-------|----------|
| No devices found | Sensor not broadcasting | Check battery; restart sensor |
| Sensor found but no data | Incorrect format | Verify sensor model (H2/H3/H4) |
| MAC address mismatch | Wrong MAC configured | Check MAC in BeaconX Pro app |

### Data Quality Issues

- **Stale data**: Increase sensor broadcast frequency in BeaconX Pro app
- **Missing readings**: Reduce distance between ESP32 and sensor (< 10m)
- **Erratic values**: Replace sensor battery

### Serial Debug Messages

Enable verbose BLE logging by monitoring serial output at 115200 baud:
```bash
pio device monitor -b 115200
```

## API Reference

### MoKoScanner Module

#### Key Functions

```cpp
// Initialize the scanner
bool initMoKoScanner(void);

// Register a known sensor
bool registerMoKoSensor(const char* macAddress, const char* location);

// Get number of registered sensors
uint8_t getRegisteredSensorCount(void);

// Get a registered sensor by index
const MoKoSensor* getRegisteredSensor(uint8_t index);

// Find sensor by MAC address
const MoKoSensor* findRegisteredSensorByMac(const char* macAddress);

// Update sensor data from BLE scan
bool updateMoKoSensorData(const uint8_t* macAddress, const uint8_t* data, 
                          size_t length, unsigned long currentTime);

// Get sensors with fresh (non-stale) data
uint8_t getFreshSensors(const MoKoSensor** sensors, uint8_t maxSensors, 
                        unsigned long currentTime);

// Check if manufacturer ID is MoKo
bool isMoKoManufacturer(uint16_t manufacturerId);

// Check if device name indicates MoKo sensor
bool isMoKoDeviceName(const char* deviceName);
```

#### Data Structures

```cpp
struct MoKoSensor {
    uint8_t macAddress[6];      // MAC address
    char macString[18];         // MAC as string
    char location[32];          // Location name
    MoKoData lastData;          // Most recent sensor data
    unsigned long lastSeen;     // Last update timestamp
    bool registered;            // Pre-registered or auto-discovered
    bool hasData;               // Has valid data
    uint8_t model;              // Model ID (1=H2, 2=H3, 3=H4)
};

struct MoKoData {
    float temperature;          // Temperature in °C
    float humidity;             // Humidity in %
    float pressure;             // Pressure in Pa
    float accelerationX;        // Acceleration X in g
    float accelerationY;        // Acceleration Y in g
    float accelerationZ;        // Acceleration Z in g
    float batteryVoltage;       // Battery voltage in V
    uint8_t batteryPercent;     // Battery percentage 0-100
    uint16_t frameCounter;      // Frame counter
    bool valid;                 // Data validity flag
};
```

## Comparison with RuuviTag

| Feature | MoKo H3/H4 | RuuviTag |
|---------|------------|----------|
| Temperature | ✅ | ✅ |
| Humidity | ✅ | ✅ |
| Pressure | H4 only | ✅ |
| Accelerometer | ✅ | ✅ |
| Battery monitoring | % + voltage | Voltage only |
| Data format | Proprietary | RAWv2 standard |
| Price | $ | $$ |
| Open source | ❌ | ✅ |

## Best Practices

1. **Pre-register sensors** for meaningful location names
2. **Use short location names** (max 31 characters)
3. **Keep sensors within 10m** of ESP32 for reliable connectivity
4. **Monitor battery levels** via MQTT payloads
5. **Label sensors physically** with location name for easy identification

## Future Enhancements

Planned features for MoKo integration:

- [ ] Motion detection alerts based on accelerometer
- [ ] Battery low notifications
- [ ] Historical data logging to sensor memory (if supported)
- [ ] Firmware update capability via BeaconX protocol
- [ ] Extended manufacturer ID support for newer models

## References

- [MoKo Official Website](https://www.mokosmart.com/)
- [BeaconX Pro App](https://www.mokosmart.com/app-download/)
- [paku-core ARCHITECTURE.md](../ARCHITECTURE.md)
- [RuuviTag Integration Guide](ruuvi-integration.md)

## Support

For issues specific to MoKo sensor integration:
1. Check serial console for error messages
2. Verify sensor MAC address in BeaconX Pro app
3. Test with `pio device monitor` for real-time debugging
4. File an issue in the paku-core repository with serial logs
