# MQTT Broker Failover Architecture

## Overview

The Paku system uses a **local-preferred, cloud-synced** MQTT architecture.
The ESP32 edge devices prefer the local RPi broker for low latency and offline
operation, but automatically fail over to a cloud broker when the RPi is
unreachable.

## Data Flow

```
┌─────────────────────────────────────────────────────────┐
│                    Cloud MQTT Broker                     │
│              (always has all paku/ data)                 │
└──────────┬──────────────────────────────┬───────────────┘
           │ Mosquitto bridge             │ direct (fallback)
           │ (bidirectional)              │
           ▼                              ▲
    ┌──────────────┐               ┌──────────────┐
    │  RPi / HA    │◄──local──────►│  ESP32 Edge  │
    │  (anywhere)  │  if same LAN  │  (van)       │
    └──────────────┘               └──────────────┘
```

## Scenarios

| Situation                  | ESP32 connects to | HA gets data via        |
|----------------------------|-------------------|-------------------------|
| Van, RPi onboard           | Local (fast)      | Direct local sub        |
| Van, RPi at home           | Cloud             | Cloud bridge on RPi     |
| Camping, no internet       | Local             | Direct local sub        |
| RPi down, internet up      | Cloud             | N/A (RPi is down)       |
| Everything down            | Retry loop        | N/A                     |

## ESP32 Boot Flow

```
1. Connect WiFi (multi-SSID scan)
2. TCP probe homeassistant.local:1883 (2 s timeout)
   ├─ reachable  → connect local MQTT
   └─ unreachable → connect cloud MQTT immediately
3. If on cloud, re-probe local every 60 s
   └─ local comes back → disconnect cloud, reconnect local
```

### mDNS Resolution

The ESP32 resolves `homeassistant.local` via mDNS to find the RPi on any
network. If mDNS fails, it falls back to a hardcoded IP (if configured in
`secrets.h`). This means:

- No hardcoded IPs required for normal operation
- Works when RPi gets a new DHCP lease
- Works on any local network (home, campsite, mobile hotspot)

### Failover Timing

| Parameter                   | Default | Notes                              |
|-----------------------------|---------|------------------------------------|
| Local probe timeout         | 2 s     | Short to avoid blocking            |
| Max connect failures        | 3       | Before switching to other broker   |
| Primary retry interval      | 60 s    | How often to re-probe local        |

## RPi Side: Mosquitto Bridge

The RPi runs a Mosquitto bridge (`/share/mosquitto/bridge.conf`) that
syncs all `paku/#` topics bidirectionally with the cloud broker:

- **QoS 1** — at-least-once delivery
- **cleansession false** — cloud queues messages while RPi is offline
- **Auto-reconnect** — 10-30 second backoff

This means:
- When ESP32 publishes to cloud (RPi unreachable), the data syncs back to
  HA when the RPi comes online
- When ESP32 publishes locally, the data syncs to cloud for remote monitoring
- Commands sent via cloud reach the ESP32 regardless of which broker it's on

## Configuration

### ESP32 (`secrets.h`)

```cpp
// Primary: local RPi (mDNS or static IP)
#define MQTT_SERVER           "homeassistant.local"
#define MQTT_PORT             1883

// Fallback: cloud broker
#define MQTT_FALLBACK_SERVER  "your-cloud-mqtt.example.com"
#define MQTT_FALLBACK_PORT    8883
#define MQTT_FALLBACK_USER    "paku-edge"
#define MQTT_FALLBACK_PASSWORD "changeme"
```

### RPi (`/share/mosquitto/bridge.conf`)

See `paku-ha/mosquitto/bridge.conf`.

## Telemetry

The device status message (`paku/edge/{deviceId}/status`) includes:

```json
{
  "mqtt_broker": "LOCAL",
  "mqtt_host": "homeassistant.local",
  ...
}
```

This lets dashboards show which broker the ESP32 is currently using.

## LWT (Last Will and Testament)

The MqttManager configures an LWT on every connection:

- **Topic:** `paku/edge/{deviceId}/status`
- **Payload:** `{"online": false}`
- **QoS:** 1
- **Retained:** true

When the ESP32 disconnects unexpectedly, the broker publishes the LWT
automatically, marking the device offline.
