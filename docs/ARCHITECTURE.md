# Architecture

System design of paku-core (EDGE firmware) and its integration with the Paku IoT system.

## System Overview

```
    ┌──────────────────────┐           MQTT            ┌──────────────────────┐
    │   paku-core (EDGE)   │◄─────────────────────────►│   paku-iot (HOST)    │
    │   ESP32 Firmware     │        Mosquitto           │   Backend Services   │
    └──────────────────────┘                           └──────────────────────┘
              │                                                   │
    ┌─────────┴─────────┐                             ┌──────────┴──────────┐
    │     Sensors       │                             │   Data Storage &    │
    │  - RuuviTag (BLE) │                             │   Visualization     │
    │  - DS18B20 (1-W)  │                             │   - PostgreSQL      │
    │  - Flow (pulse)   │                             │   - Grafana         │
    │  - Voltage (ADC)  │                             └─────────────────────┘
    └───────────────────┘
```

- **paku-core (EDGE)** — ESP32 firmware: sensor collection, MQTT publishing
- **paku-iot (HOST)** — Collector, PostgreSQL, Grafana, OTA service (Docker Compose)

## EDGE Component Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                       paku-core (ESP32)                      │
├──────────┬──────────┬──────────┬──────────┬────────────────┤
│   WiFi   │   MQTT   │   BLE    │ Display  │  OTA Client    │
│  Manager │  Client  │ Scanner  │  Driver  │  (HTTP pull)   │
├──────────┴──────────┴──────────┴──────────┴────────────────┤
│                       Main Loop                             │
│                  (Interval Manager)                          │
├────────────┬────────────┬────────────┬─────────────────────┤
│   Flow     │  DS18B20   │   Ruuvi    │   Voltage / Status  │
│  Sensor    │  (1-Wire)  │   Tags     │   Sensors           │
└────────────┴────────────┴────────────┴─────────────────────┘
```

## MQTT Topic Structure

The firmware uses multiple topic namespaces. The canonical schema is documented
in paku-iot's [mqtt_schema.md](https://github.com/ychefla/paku-iot/blob/main/docs/mqtt_schema.md).

### Sensor Data (collector consumes `+/+/+/data`)

| Topic | Direction | Description |
|-------|-----------|-------------|
| `paku/sensors/{sensor_id}/data` | EDGE → HOST | RuuviTag / wired sensor JSON |
| `paku/devices/{device_id}/telemetry/{type}/{location}` | EDGE → HOST | Per-metric legacy topics |

### Edge Status & Config (`+/edge/+/status`, `+/edge/+/config`)

| Topic | Direction | Description |
|-------|-----------|-------------|
| `paku/edge/{device_id}/status` | EDGE → HOST | Device status (retained) |
| `paku/edge/{device_id}/config/report` | EDGE → HOST | Current config (retained) |
| `paku/edge/{device_id}/config/set` | HOST → EDGE | Push config changes |
| `paku/edge/{device_id}/control` | HOST → EDGE | Control commands |

### OTA (`+/edge/+/ota/+`)

| Topic | Direction | Description |
|-------|-----------|-------------|
| `paku/edge/{device_id}/cmd/ota` | HOST → EDGE | OTA command (firmware URL) |
| `paku/edge/{device_id}/ota/status` | EDGE → HOST | OTA started / acknowledged |
| `paku/edge/{device_id}/ota/progress` | EDGE → HOST | Download progress % |
| `paku/edge/{device_id}/ota/result` | EDGE → HOST | Success / failure |

### Other Commands

| Topic | Direction | Description |
|-------|-----------|-------------|
| `paku/devices/{device_id}/cmd/wifi` | HOST → EDGE | WiFi scan command |
| `paku/devices/{device_id}/cmd/ruuvi` | HOST → EDGE | Ruuvi scan command |
| `paku/heater/{device_id}/cmd` | HOST → EDGE | Heater control |
| `paku/control` | HOST → EDGE | Legacy broadcast control |

### Payload Format

```json
{
  "value": 23.5,
  "timestamp": "12:34:56",
  "device_id": "paku-AABBCC"
}
```

## Timing & Power Management

| Mode | Sensor Interval | MQTT Interval |
|------|-----------------|---------------|
| Active (heater on, first hour) | 5 s | 10 s |
| Active (after 1 hour) | 60 s | 1 h |
| Idle | 60 s | 1 h |

Sleep cycle: 30 s awake → 15 s deep sleep → timer wake → reconnect.

## Hardware

- **Board**: LilyGo T-Display S3 (ESP32-S3)
- **Display**: ST7789V TFT via SPI
- **Pin config**: `src/pin_config.h`
- **Board definitions**: `boards/`

## Key Components

| Component | Responsibility |
|-----------|---------------|
| WiFi Manager | Multi-SSID with fallback, auto-reconnect |
| MQTT Client | Connect, subscribe, batched publish |
| BLE Scanner | FreeRTOS task, Ruuvi & MoKo parsing |
| Display Driver | Status screen, boot logo |
| OTA Client | HTTP firmware download, progress reporting |
| Interval Manager | Fast/slow mode switching |

## Related Documents

- [Quickstart](edge/quickstart.md) — Build and flash
- [Integration Guide](INTEGRATION.md) — Connecting to paku-iot
- [Logging](edge/logging.md) — Debug output configuration
- [OTA](edge/ota-integration.md) — OTA firmware update guide
