# MoKo Sensor Quick Setup

Get MoKo H2/H3/H4 sensors working with paku-core in 5 minutes.

## Prerequisites

- MoKo H2, H3, or H4 Bluetooth sensor
- MoKo BeaconX Pro app (iOS/Android)
- ESP32 running paku-core firmware
- MQTT broker accessible

## Step-by-Step Setup

### 1. Configure Your MoKo Sensor

1. Install **BeaconX Pro** app on your phone
2. Insert battery in MoKo sensor
3. Open app and connect to sensor
4. **Important**: Note the MAC address (e.g., `AA:BB:CC:DD:EE:11`)
5. Set broadcast interval to **1-5 seconds**
6. Click "Save" and disconnect

### 2. Update Firmware Configuration

Edit `paku_core/include/secrets.h`:

```cpp
// Add after RuuviTag configuration:

#define MOKO_SENSOR_COUNT 1

static const char* MOKO_SENSOR_MACS[] = {
    "AA:BB:CC:DD:EE:11"  // Your sensor's MAC
};

static const char* MOKO_SENSOR_LOCATIONS[] = {
    "garage"  // Your location name
};
```

### 3. Flash Firmware

```bash
cd paku_core
pio run -t upload
```

### 4. Verify Detection

Open serial monitor:
```bash
pio device monitor
```

Look for:
```
Starting Bluetooth scan...
MoKo sensor found: AA:BB:CC:DD:EE:11 (MK_H3)
  -> MoKo sensor data updated
MoKo sensor [garage] H3: T=22.5°C, H=45.2%
```

### 5. Check MQTT

Subscribe to your MQTT broker:
```bash
mosquitto_sub -h your-broker -t "paku/sensors/moko_#" -v
```

You should see messages like:
```
paku/sensors/moko_garage/data {"timestamp":"2025-12-14T10:30:00Z","device_id":"moko_garage",...}
```

## Common Issues

**No sensor detected?**
- Check battery is inserted correctly
- Verify sensor is broadcasting (LED should blink)
- Move sensor closer to ESP32 (< 5m for testing)

**Wrong MAC address?**
- Double-check MAC in BeaconX Pro app
- Use UPPERCASE format with colons: `AA:BB:CC:DD:EE:FF`

**No MQTT messages?**
- Verify MQTT broker connection in serial console
- Check WiFi connectivity
- Ensure sensor data is "fresh" (< 5 minutes old)

## Auto-Discovery Mode

Don't want to pre-register? Just skip step 2 entirely!

Sensors will be auto-discovered with names like `auto_AABB`.

Check serial console for MAC addresses:
```
MoKo sensor found: AA:BB:CC:DD:EE:11 (MK_H3)
```

Then update `secrets.h` with proper location names.

## Next Steps

- [Full Integration Guide](moko-integration.md)
- [Configure paku-iot backend](../../paku-iot/docs/ecoflow_integration.md)
- [Set up Grafana dashboards](../../paku-iot/docs/ecoflow_dashboard.md)

## Multiple Sensors

To add more sensors, simply expand the arrays:

```cpp
#define MOKO_SENSOR_COUNT 3

static const char* MOKO_SENSOR_MACS[] = {
    "AA:BB:CC:DD:EE:11",
    "AA:BB:CC:DD:EE:12",
    "AA:BB:CC:DD:EE:13"
};

static const char* MOKO_SENSOR_LOCATIONS[] = {
    "garage",
    "basement",
    "outdoor"
};
```

Maximum: 8 sensors by default (configurable in `moko_scanner.h`).
