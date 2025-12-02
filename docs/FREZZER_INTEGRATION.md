# Frezzer PRO Fridge Integration Guide

This guide explains how to integrate your Frezzer PRO 65L 12/24V compressor fridge with paku-core for BLE-based monitoring and control.

## Overview

The paku-core firmware can communicate with Frezzer PRO compressor fridges via Bluetooth Low Energy (BLE). This allows you to:

- **Monitor** current temperature, target temperature, and battery voltage
- **Control** temperature settings and operating mode remotely
- **Receive alerts** for errors, low voltage, or lid open conditions
- **View status** on the paku-core display and via MQTT

## Hardware Requirements

- Frezzer PRO 65L 12/24V (or compatible model with Bluetooth)
- LilyGo T-Display S3 running paku-core firmware
- The fridge must be powered on with Bluetooth enabled

## Finding Your Frezzer's MAC Address

Before configuring paku-core, you need to find your Frezzer's Bluetooth MAC address:

### Option 1: Using nRF Connect App
1. Download "nRF Connect for Mobile" (available for iOS and Android)
2. Open the app and start scanning for BLE devices
3. Look for a device named "FREZZER" or similar
4. Note the MAC address (e.g., `AA:BB:CC:DD:EE:FF`)

### Option 2: Using paku-core Auto-Discovery
1. Flash paku-core with default settings
2. Monitor the serial output when the device boots
3. Look for messages like: `Frezzer discovered: AA:BB:CC:DD:EE:FF`
4. The device will be auto-registered with a name like `fridge_0`

## Configuration

### 1. Edit secrets.h

Add your Frezzer configuration to `paku_core/include/secrets.h`:

```cpp
// Frezzer PRO Configuration
#define FREZZER_COUNT 1
static const char* FREZZER_MACS[] = {
    "AA:BB:CC:DD:EE:FF"   // Replace with your Frezzer's MAC address
};
static const char* FREZZER_LOCATIONS[] = {
    "van_fridge"          // Friendly name for MQTT topics
};
```

### 2. Multiple Fridges

If you have multiple Frezzer units:

```cpp
#define FREZZER_COUNT 2
static const char* FREZZER_MACS[] = {
    "AA:BB:CC:DD:EE:01",
    "AA:BB:CC:DD:EE:02"
};
static const char* FREZZER_LOCATIONS[] = {
    "main_fridge",
    "freezer"
};
```

### 3. Build and Upload

```bash
cd paku_core
pio run -t upload
```

## MQTT Topics

Frezzer data is published to MQTT with the following structure:

### Telemetry Data
Topic: `paku/fridge/{location}/data`

Example payload:
```json
{
  "timestamp": "2024-01-15T10:30:00Z",
  "device_id": "frezzer_van_fridge",
  "location": "van_fridge",
  "mac": "AA:BB:CC:DD:EE:FF",
  "metrics": {
    "current_temp_c": -5.2,
    "target_temp_c": -8.0,
    "battery_voltage": 12.4,
    "mode": "fridge",
    "compressor": "running",
    "power_level": 45,
    "lid_open": false,
    "low_voltage_protection": false,
    "error": "none"
  }
}
```

### Control Commands
Topic: `paku/fridge/{location}/cmd`

#### Set Target Temperature
```json
{
  "command": "set_temp",
  "value": -8.0
}
```

#### Set Operating Mode
```json
{
  "command": "set_mode",
  "value": "freezer"
}
```
Valid modes: `off`, `fridge`, `freezer`, `eco`, `max_cool`

#### Power Control
```json
{
  "command": "power",
  "value": "on"
}
```

## Operating Modes

| Mode | Description | Typical Temperature Range |
|------|-------------|--------------------------|
| `off` | Fridge is powered off | N/A |
| `fridge` | Standard refrigerator mode | 0°C to 8°C |
| `freezer` | Deep freeze mode | -18°C to -22°C |
| `eco` | Power-saving mode | Variable |
| `max_cool` | Maximum cooling | As low as possible |

## Error Codes

| Error | Description | Action |
|-------|-------------|--------|
| `none` | No error | Normal operation |
| `temp_sensor_error` | Temperature sensor fault | Check sensor connection |
| `compressor_error` | Compressor malfunction | Service required |
| `low_voltage` | Input voltage too low | Check battery/power |
| `high_voltage` | Input voltage too high | Check power source |
| `overtemperature` | Overheating protection | Improve ventilation |
| `communication_error` | BLE communication issue | Check range |

## Display Integration

When a Frezzer is connected, the paku-core display shows:
- Current temperature
- Target temperature
- Operating mode
- Compressor status
- Any active errors

## Troubleshooting

### Frezzer Not Found
1. Ensure the fridge is powered on
2. Check that Bluetooth is enabled on the fridge
3. Move the paku-core device closer to the fridge
4. Verify the MAC address is correct (uppercase with colons)

### Cannot Connect
1. Only one BLE device can connect to the Frezzer at a time
2. Disconnect any phone apps before using paku-core
3. Try power-cycling the fridge

### Stale Data
Data is considered stale after 5 minutes without updates. If the fridge
shows stale data:
1. Check BLE signal strength
2. Verify fridge is still powered on
3. Check for interference from other BLE devices

### Command Not Working
1. Ensure paku-core is connected to the fridge
2. Check MQTT connectivity
3. Verify command JSON format is correct
4. Check serial output for error messages

## Technical Notes

### BLE Protocol

The Frezzer PRO uses a proprietary BLE GATT protocol. The paku-core
implementation uses placeholder UUIDs that may need adjustment for your
specific device firmware version:

- Service UUID: Environmental Sensing (0x181A) or proprietary
- Characteristics: Temperature read, control write, notifications

If your Frezzer uses different UUIDs, you can update them in
`frezzer_controller.h`:

```cpp
#define FREZZER_SERVICE_UUID      "your-service-uuid"
#define FREZZER_STATUS_CHAR_UUID  "your-status-char-uuid"
#define FREZZER_CONTROL_CHAR_UUID "your-control-char-uuid"
```

### Discovering Your Frezzer's Protocol

To reverse engineer the actual protocol:

1. Install the official Frezzer app on your phone
2. Use a BLE sniffer (e.g., Ubertooth One) or BLE proxy
3. Capture traffic between the app and fridge
4. Note the service/characteristic UUIDs and data formats
5. Update the paku-core headers accordingly

### Connection Behavior

- paku-core scans for Frezzer devices during BLE scans
- When found, it attempts to connect via GATT
- While connected, it polls status every 30 seconds
- If connection is lost, it automatically attempts to reconnect

## Safety Considerations

- **Battery Protection**: The Frezzer has built-in low voltage protection
  to prevent draining your vehicle battery. paku-core monitors this status.
  
- **Temperature Limits**: Temperature commands are validated to prevent
  setting unsafe values (-25°C to +20°C range).
  
- **Manual Override**: The physical controls on the fridge always work,
  even if BLE communication fails.

## Future Enhancements

- Historical temperature logging
- Grafana dashboard templates
- Home Assistant integration
- Voice control via MQTT
