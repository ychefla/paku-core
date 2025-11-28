# paku-iot Integration

This document describes how paku-core (EDGE firmware) sends data to paku-iot (host-side services).

## Overview

paku-core supports two transport mechanisms for sending telemetry data to paku-iot:

1. **MQTT** (Primary) — Real-time messaging via an MQTT broker
2. **HTTP** (Secondary) — REST API calls directly to paku-iot endpoints

The firmware uses MQTT as the primary transport when a broker is configured. HTTP transport is available as a fallback or alternative when direct REST API integration is preferred.

---

## Data Format

All payloads use JSON format with consistent structure:

### Single Telemetry Message

```json
{
  "device_id": "paku-AABBCCDD",
  "timestamp": "2024-01-15T10:30:00Z",
  "metric": "temperature/cabin",
  "value": 21.5,
  "unit": "celsius"
}
```

### Batch Telemetry Message

```json
{
  "device_id": "paku-AABBCCDD",
  "batch_timestamp": "2024-01-15T10:30:00Z",
  "readings": [
    {
      "metric": "temperature/cabin",
      "value": 21.5,
      "unit": "celsius",
      "timestamp": "2024-01-15T10:30:00Z"
    },
    {
      "metric": "humidity/cabin",
      "value": 45.2,
      "unit": "percent",
      "timestamp": "2024-01-15T10:30:00Z"
    },
    {
      "metric": "flow/coolant",
      "value": 3.5,
      "unit": "l_per_min",
      "timestamp": "2024-01-15T10:30:00Z"
    }
  ]
}
```

### Field Definitions

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `device_id` | string | Yes | Unique device identifier (MAC-based) |
| `timestamp` | string | Yes | ISO 8601 timestamp |
| `metric` | string | Yes | Metric path (e.g., `temperature/cabin`) |
| `value` | number | Yes | Sensor reading value |
| `unit` | string | No | Unit of measurement |

---

## Transport: MQTT

### Topics

The device publishes to topics under the `paku/` prefix:

| Topic Pattern | Description | Example |
|---------------|-------------|---------|
| `paku/temperature/{location}` | Temperature readings | `paku/temperature/moko/cabin` |
| `paku/humidity/{location}` | Humidity readings | `paku/humidity/moko/kitchen` |
| `paku/flow/{type}` | Flow rate measurements | `paku/flow/coolant` |
| `paku/voltage/{source}` | Voltage readings | `paku/voltage/car` |
| `paku/status/{component}` | Component status | `paku/status/heater` |
| `paku/power/{type}` | Power readings | `paku/power/heat` |

### Example MQTT Payload

```json
{"value": 21.5, "timestamp": "10:30:00"}
```

### Configuration

MQTT settings are configured in `include/secrets.h`:

```cpp
#define MQTT_SERVER   "your-mqtt-server-hostname"
#define MQTT_PORT     1883
```

### QoS and Retry

- Default QoS: 0 (at most once)
- Reconnection with exponential backoff on disconnect
- Last Will and Testament (LWT) set to indicate offline status

---

## Transport: HTTP

### Endpoint

The paku-iot HTTP endpoint accepts POST requests:

```
POST https://{PAKU_IOT_HOST}/api/v1/telemetry
Content-Type: application/json
Authorization: Bearer {API_KEY}
```

### Example HTTP Request

```bash
curl -X POST https://iot.paku.example.com/api/v1/telemetry \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer your-api-key" \
  -d '{
    "device_id": "paku-AABBCCDD",
    "timestamp": "2024-01-15T10:30:00Z",
    "metric": "temperature/cabin",
    "value": 21.5
  }'
```

### Response

Success (HTTP 200/201):
```json
{
  "status": "ok",
  "message_id": "abc123"
}
```

Error (HTTP 4xx/5xx):
```json
{
  "status": "error",
  "code": "INVALID_PAYLOAD",
  "message": "Missing required field: device_id"
}
```

### Configuration

HTTP settings are configured in `include/secrets.h`:

```cpp
#define PAKU_IOT_HOST     "iot.paku.example.com"
#define PAKU_IOT_PORT     443
#define PAKU_IOT_API_KEY  "your-api-key"
#define PAKU_IOT_USE_TLS  true
```

---

## Retry and Queue Semantics

### MQTT Transport

1. **Connection Retry**: On MQTT disconnect, the device attempts reconnection with exponential backoff (initial: 5s, max: 5min)
2. **Message Queue**: Messages are buffered in memory (up to 30 payloads) during temporary disconnects
3. **Delivery**: Messages are published when connection is restored; no persistent queue across reboots

### HTTP Transport

1. **Request Retry**: Failed HTTP requests are retried up to 3 times with exponential backoff
2. **Retry Delays**: 1s → 2s → 4s
3. **Timeout**: 10 second connection timeout, 30 second total request timeout
4. **Queue**: Failed messages are stored in a circular buffer (max 50 entries) and retried on next cycle
5. **Fallback**: If HTTP fails consistently, device logs error and continues sensor collection

### Error Handling

| Error Type | Action | Retry |
|------------|--------|-------|
| Network unreachable | Buffer message, retry later | Yes |
| DNS resolution failed | Buffer message, retry later | Yes |
| Connection timeout | Retry with backoff | Yes |
| HTTP 4xx (client error) | Log error, discard message | No |
| HTTP 5xx (server error) | Retry with backoff | Yes |
| TLS certificate error | Log error, check config | No |

---

## Local Development

### Testing MQTT Locally

1. Run a local MQTT broker (e.g., Mosquitto):
   ```bash
   docker run -d -p 1883:1883 eclipse-mosquitto
   ```

2. Configure `secrets.h`:
   ```cpp
   #define MQTT_SERVER   "192.168.1.100"  // Your local IP
   #define MQTT_PORT     1883
   ```

3. Subscribe to topics:
   ```bash
   mosquitto_sub -h localhost -t "paku/#" -v
   ```

### Testing HTTP Locally

1. Run a simple test server:
   ```bash
   # Python example
   python -m http.server 8080
   
   # Or use a mock server like Mockoon
   ```

2. Configure `secrets.h`:
   ```cpp
   #define PAKU_IOT_HOST     "192.168.1.100"
   #define PAKU_IOT_PORT     8080
   #define PAKU_IOT_USE_TLS  false
   ```

### Example Integration Test

```python
# test_integration.py
import paho.mqtt.client as mqtt
import json

received_messages = []

def on_message(client, userdata, msg):
    payload = json.loads(msg.payload.decode())
    received_messages.append({
        "topic": msg.topic,
        "payload": payload
    })
    print(f"Received: {msg.topic} -> {payload}")

client = mqtt.Client()
client.on_message = on_message
client.connect("localhost", 1883)
client.subscribe("paku/#")
client.loop_start()

# Keep running to receive messages
import time
time.sleep(60)
client.loop_stop()

print(f"Total messages received: {len(received_messages)}")
```

---

## Metrics Reference

### Temperature Metrics

| Metric Path | Location | Unit |
|-------------|----------|------|
| `temperature/moko/cabin` | Main cabin | celsius |
| `temperature/moko/dryer` | Dryer area | celsius |
| `temperature/moko/kitchen` | Kitchen | celsius |
| `temperature/moko/lounge` | Lounge | celsius |
| `temperature/heating/floor` | Floor heating | celsius |
| `temperature/heating/heater_in` | Heater inlet | celsius |
| `temperature/heating/heater_out` | Heater outlet | celsius |

### Flow Metrics

| Metric Path | Description | Unit |
|-------------|-------------|------|
| `flow/coolant` | Coolant flow rate | l_per_min |
| `flow/coolant_frequency` | Sensor frequency | hz |

### Status Metrics

| Metric Path | Description | Values |
|-------------|-------------|--------|
| `status/heater` | Heater on/off | 0 or 1 |
| `status/heater_timer` | Timer active | 0 or 1 |
| `status/pump` | Pump running | 0 or 1 |

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2024-01 | Initial integration documentation |
