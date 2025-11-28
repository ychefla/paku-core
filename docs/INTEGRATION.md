# Integration Guide

This document describes how to configure paku-core to communicate with paku-iot and integrate with the broader Paku IoT ecosystem.

## Overview

paku-core (EDGE) sends sensor data to paku-iot (HOST) via MQTT. The host-side services collect, store, and visualize this data.

```
┌────────────────────┐         MQTT          ┌────────────────────┐
│   paku-core        │ ──────────────────►   │   paku-iot         │
│   (ESP32)          │    Broker             │   (Host Services)  │
└────────────────────┘                       └────────────────────┘
```

## MQTT Configuration

### paku-core Configuration

Configure the MQTT connection in `paku_core/include/secrets.h`:

```cpp
// MQTT Broker - Point to your paku-iot instance
#define MQTT_SERVER                   "192.168.1.100"  // or "mqtt.example.com"
#define MQTT_PORT                     1883
```

For secure connections (TLS), additional configuration is required:
- Port 8883 is commonly used for MQTT over TLS
- CA certificate must be embedded in firmware or stored in SPIFFS

### paku-iot Configuration

On the paku-iot side, configure your MQTT broker to accept connections from paku-core devices:

1. **Mosquitto** (common choice):
   ```
   listener 1883
   allow_anonymous true
   ```

2. **For production**, enable authentication:
   ```
   listener 1883
   password_file /etc/mosquitto/passwd
   allow_anonymous false
   ```

## Topic Mapping

### Topics Published by paku-core

| Topic | Data Type | Unit | Description |
|-------|-----------|------|-------------|
| `paku/temperature/moko/cabin` | float | °C | Cabin temperature |
| `paku/temperature/moko/dryer` | float | °C | Dryer temperature |
| `paku/temperature/moko/kitchen` | float | °C | Kitchen temperature |
| `paku/temperature/moko/lounge` | float | °C | Lounge temperature |
| `paku/temperature/heating/floor` | float | °C | Floor heating temperature |
| `paku/temperature/heating/heater_in` | float | °C | Heater inlet temperature |
| `paku/temperature/heating/heater_out` | float | °C | Heater outlet temperature |
| `paku/temperature/heating/required_dt` | float | °C | Required temperature delta |
| `paku/humidity/moko/cabin` | float | % | Cabin humidity |
| `paku/humidity/moko/dryer` | float | % | Dryer humidity |
| `paku/humidity/moko/kitchen` | float | % | Kitchen humidity |
| `paku/humidity/moko/lounge` | float | % | Lounge humidity |
| `paku/flow/coolant` | float | L/min | Coolant flow rate |
| `paku/flow/coolant_frequency` | float | Hz | Flow sensor frequency |
| `paku/power/heat` | float | W | Heating power |
| `paku/power/cool` | float | W | Cooling power |
| `paku/voltage/car` | float | V | Car battery voltage |
| `paku/voltage/leisure` | float | V | Leisure battery voltage |
| `paku/status/heater` | int | - | Heater status (0/1) |
| `paku/status/heater_timer` | int | - | Heater timer status |
| `paku/status/pump` | int | - | Pump status |

### Topics Subscribed by paku-core

| Topic | Payload | Description |
|-------|---------|-------------|
| `paku/control` | JSON | Commands for device control |

### Payload Format

All sensor data follows this JSON format:

```json
{
  "value": 23.5,
  "timestamp": "12:34:56"
}
```

**Note**: Timestamps are in HH:MM:SS format from the NTP client. The paku-iot collector should add full date information server-side.

## Hardware Setup

### Ruuvi Tag Integration

paku-core includes a BLE scanner for reading data from Ruuvi tags (or equivalent BLE sensors).

#### Supported Devices

- **Ruuvi Tag** - Temperature, humidity, pressure, acceleration
- **Other BLE sensors** - Compatible devices advertising sensor data

#### Setup Steps

1. **Power on Ruuvi tags** - Insert batteries and verify LED blinks

2. **Position tags** - Place in locations to monitor:
   - Cabin
   - Kitchen
   - Lounge
   - Dryer/utility area

3. **Note MAC addresses** - Each tag has a unique MAC address printed on or accessible via Ruuvi Station app

4. **Configure paku-core** (future feature):
   ```cpp
   // In future versions, configure known tags
   const char* RUUVI_TAGS[] = {
     "AA:BB:CC:DD:EE:FF",  // Cabin
     "11:22:33:44:55:66",  // Kitchen
     // ...
   };
   ```

#### Current BLE Implementation

The current implementation scans for all BLE devices and logs their information:

```cpp
// Background BLE scanning (runs as FreeRTOS task)
void scanBT(void* parameter) {
  // Scans every second with 5-second scan window
  // Prints device info to serial console
}
```

**Note**: Full Ruuvi data parsing and topic mapping is planned for a future release. Currently, BLE scan results are logged to serial output.

### Flow Sensor

The flow sensor measures coolant flow rate for the heating system.

#### Wiring

| Sensor Pin | ESP32 Pin | Description |
|------------|-----------|-------------|
| VCC | 3.3V | Power supply |
| GND | GND | Ground |
| Signal | GPIO2 | Pulse output |

#### Calibration

The flow sensor uses a calibration factor to convert pulse frequency to flow rate:

```cpp
float calibrationFactor = 6.6;  // Pulses per second per L/min
flowRate = (frequency / calibrationFactor) * 60.0;
```

Adjust `calibrationFactor` in `main.cpp` if your sensor differs.

### Display

The LilyGo T-Display S3 includes a built-in TFT display:

- **Resolution**: 170x320 pixels
- **Controller**: ST7789V
- **Interface**: Parallel 8-bit

The display shows:
- Current time (NTP synchronized)
- Flow rate
- Required temperature delta
- Heater status
- Connection status

## Network Requirements

### WiFi

Configure multiple WiFi networks for fallback:

```cpp
// In secrets.h
#define WIFI_SSID_HOME                "primary-network"
#define WIFI_PASSWORD_HOME            "password1"
#define WIFI_SSID_IPHONE              "backup-hotspot"
#define WIFI_PASSWORD_IPHONE          "password2"
#define WIFI_SSID_PAKU                "on-site-network"
#define WIFI_PASSWORD_PAKU            "password3"
```

The device tries networks in order until connected.

### Firewall Rules

Ensure the following connectivity:

| Source | Destination | Port | Protocol | Purpose |
|--------|-------------|------|----------|---------|
| paku-core | MQTT Broker | 1883 | TCP | MQTT messages |
| paku-core | NTP Server | 123 | UDP | Time sync |

## Troubleshooting

### Connection Issues

**WiFi won't connect:**
- Verify SSID and password in `secrets.h`
- Check that network is 2.4 GHz (ESP32 doesn't support 5 GHz)
- Monitor serial output for connection attempts

**MQTT disconnects:**
- Check broker is running and accessible
- Verify firewall allows port 1883
- Check for broker authentication requirements

**No data appearing in paku-iot:**
- Confirm MQTT broker address is correct
- Subscribe to `paku/#` on broker to verify messages
- Check serial output for MQTT publish confirmation

### Serial Monitoring

Connect to serial output for debugging:

```bash
cd paku_core
pio device monitor -b 115200
```

Expected output on successful startup:
```
Starting setup...
Setup Wifi Connection...
Connecting to your-ssid
WiFi connected
IP address: 192.168.1.50
Setup MQTT Connection...
MQTT connected and subscribed to 'paku/control'
Setup Sensor...
Setup complete.
```

## Out-of-Scope Features

The following features are defined in requirements but not yet implemented. They should not be removed from the codebase:

| Feature | Status | Notes |
|---------|--------|-------|
| OTA Updates | Planned v1.1 | ArduinoOTA or HTTP-triggered |
| Runtime Configuration | Planned v1.2 | MQTT or NVS-based config |
| TLS/SSL for MQTT | Not implemented | Requires certificate handling |
| Ruuvi Tag Parsing | Partial | BLE scan works, parsing pending |
| Command Handler | Subscribed only | Topic subscribed, no command handling |
| Boot Counter | Not implemented | Planned for observability |

These features are documented in [requirements.md](requirements.md) and should remain as future implementation targets.

## Next Steps

1. **paku-iot setup**: Follow the paku-iot documentation to set up collectors and storage
2. **Dashboard configuration**: Configure Grafana or similar for visualization
3. **Home Assistant integration**: Add MQTT sensors to Home Assistant configuration

## Related Documents

- [Architecture](ARCHITECTURE.md) - System architecture overview
- [Configuration Reference](edge/config.md) - Detailed configuration options
- [Requirements](requirements.md) - Full requirements specification
