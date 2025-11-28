# Architecture

This document describes the architecture of paku-core (EDGE firmware) and how it integrates with the broader Paku IoT system.

## System Overview

The Paku system consists of two main components:

1. **paku-core (EDGE)** - ESP32 firmware that runs on edge devices, collecting sensor data and communicating via MQTT
2. **paku-iot (HOST)** - Host-side services for data collection, storage, and visualization

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              Paku IoT System                                │
└─────────────────────────────────────────────────────────────────────────────┘

    ┌──────────────────────┐           MQTT            ┌──────────────────────┐
    │   paku-core (EDGE)   │◄─────────────────────────►│   paku-iot (HOST)    │
    │   ESP32 Firmware     │         Broker            │   Backend Services   │
    └──────────────────────┘                           └──────────────────────┘
              │                                                   │
              │                                                   │
    ┌─────────┴─────────┐                             ┌──────────┴──────────┐
    │     Sensors       │                             │   Data Storage &    │
    │   - Temperature   │                             │   Visualization     │
    │   - Humidity      │                             │   - InfluxDB        │
    │   - Flow          │                             │   - Grafana         │
    │   - Ruuvi Tags    │                             │   - Home Assistant  │
    └───────────────────┘                             └─────────────────────┘
```

## paku-core Component Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           paku-core (ESP32)                                 │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│   ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐   │
│   │    WiFi      │  │    MQTT      │  │     BLE      │  │   Display    │   │
│   │   Manager    │  │   Client     │  │   Scanner    │  │   Driver     │   │
│   └──────┬───────┘  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘   │
│          │                 │                 │                 │           │
│          └─────────────────┴─────────────────┴─────────────────┘           │
│                                    │                                       │
│                          ┌─────────┴─────────┐                             │
│                          │    Main Loop      │                             │
│                          │   (FreeRTOS)      │                             │
│                          └─────────┬─────────┘                             │
│                                    │                                       │
│   ┌────────────────────────────────┴────────────────────────────────────┐  │
│   │                        Sensor Abstraction                           │  │
│   ├────────────┬────────────┬────────────┬────────────┬────────────────┤  │
│   │   Flow     │   Temp     │  Humidity  │   Ruuvi    │   Voltage      │  │
│   │  Sensor    │  Sensors   │  Sensors   │   Tags     │   Sensors      │  │
│   └────────────┴────────────┴────────────┴────────────┴────────────────┘  │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
                │                                                   │
                ▼                                                   ▼
         ┌──────────────┐                                   ┌──────────────┐
         │  Hardware    │                                   │   Network    │
         │  Interfaces  │                                   │   Stack      │
         └──────────────┘                                   └──────────────┘
```

## Data Flow

### Sensor Data Collection

```
┌───────────┐    ┌─────────────┐    ┌────────────┐    ┌──────────────┐
│  Sensor   │───►│  Process    │───►│  Create    │───►│   Payload    │
│  Reading  │    │  Data       │    │  Payload   │    │   Queue      │
└───────────┘    └─────────────┘    └────────────┘    └──────┬───────┘
                                                              │
                                                              ▼
┌───────────┐    ┌─────────────┐    ┌────────────┐    ┌──────────────┐
│  paku-iot │◄───│   MQTT      │◄───│   Batch    │◄───│   Timer      │
│  Backend  │    │   Publish   │    │   Send     │    │   Interval   │
└───────────┘    └─────────────┘    └────────────┘    └──────────────┘
```

### MQTT Topic Structure

All topics are prefixed with `paku/` and follow this hierarchy:

| Topic Pattern | Direction | Description |
|---------------|-----------|-------------|
| `paku/temperature/{sensor}/{location}` | EDGE → IOT | Temperature readings |
| `paku/humidity/{sensor}/{location}` | EDGE → IOT | Humidity readings |
| `paku/flow/{type}` | EDGE → IOT | Coolant flow measurements |
| `paku/power/{type}` | EDGE → IOT | Power metrics |
| `paku/voltage/{source}` | EDGE → IOT | Battery voltage readings |
| `paku/status/{device}` | EDGE → IOT | Device status updates |
| `paku/control` | IOT → EDGE | Commands for device control |

### Payload Format

All sensor data is published as JSON with a consistent format:

```json
{
  "value": 23.5,
  "timestamp": "12:34:56"
}
```

## Key Components

### WiFi Manager

Handles network connectivity with multi-SSID support:

- **Priority-based connection**: Tries networks in configured order (HOME, IPHONE, PAKU)
- **Auto-reconnect**: Monitors connection state and reconnects on failure
- **Visual feedback**: Displays connection status on TFT screen

### MQTT Client

Manages communication with the MQTT broker:

- **Connection handling**: Auto-connect and reconnect with retry logic
- **Topic subscription**: Listens on `paku/control` for commands
- **Batched publishing**: Collects payloads and sends at configured intervals

### BLE Scanner

Discovers and reads data from Bluetooth Low Energy sensors:

- **Background scanning**: Runs as a FreeRTOS task
- **Device discovery**: Scans for Ruuvi tags and compatible BLE sensors
- **Data extraction**: Parses manufacturer data for temperature/humidity

### Display Driver

Manages the onboard TFT display:

- **Status display**: Shows current readings and connection state
- **Boot logo**: Displays Paku logo on startup
- **Low-power mode**: Supports display dimming during sleep

### Interval Manager

Controls data collection and transmission timing:

- **Fast mode**: 5s sensor / 10s MQTT when heater is active
- **Slow mode**: 60s sensor / 1h MQTT when idle
- **Power management**: Coordinates with deep sleep

## Timing and Power Management

### Operational Modes

| Mode | Sensor Interval | MQTT Interval | Duration |
|------|-----------------|---------------|----------|
| Active (heater on, first hour) | 5 seconds | 10 seconds | 1 hour |
| Active (heater on, after 1 hour) | 60 seconds | 1 hour | Until heater off |
| Idle | 60 seconds | 1 hour | Until heater on |

### Sleep Cycle

The device follows a duty cycle to conserve power:

1. **Awake period**: 30 seconds of active data collection
2. **Deep sleep**: 15 seconds of low-power state
3. **Wake**: Timer-triggered wake from deep sleep
4. **Resume**: Reconnect WiFi/MQTT and continue

## Hardware Abstraction

### Pin Configuration

Hardware-specific pin mappings are defined in `src/pin_config.h`:

- **Display pins**: SPI interface for ST7789V TFT
- **Sensor pins**: GPIO for flow sensor interrupts
- **Power control**: Backlight and power management

### Board Support

Currently supported boards:

- **LilyGo T-Display S3** (ESP32-S3) - Primary development target

Custom board definitions are stored in `boards/` directory.

## Future Architecture Considerations

### Planned Improvements

1. **Module extraction**: Refactor components into separate libraries
2. **Configuration system**: Runtime configuration via MQTT or NVS
3. **OTA updates**: Over-the-air firmware updates
4. **Enhanced BLE**: Ruuvi tag parsing and additional sensor support

### Extension Points

- **New sensors**: Add to sensor abstraction layer
- **Additional MQTT topics**: Extend topic structure
- **Custom displays**: Implement alternative display drivers
- **Command handlers**: Add new control commands

## Related Documents

- [Integration Guide](INTEGRATION.md) - Connecting to paku-iot
- [Configuration Reference](edge/config.md) - Configuration options
- [Requirements](requirements.md) - System requirements
