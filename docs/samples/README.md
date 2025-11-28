# Sample MQTT Messages

This directory contains sample MQTT messages for reference and testing purposes.

## Message Format

All paku-core telemetry messages follow this JSON structure:

```json
{
  "value": <number>,
  "timestamp": "<HH:MM:SS>"
}
```

## Sample Files

| File | Description |
|------|-------------|
| `temperature.json` | Temperature sensor payloads |
| `humidity.json` | Humidity sensor payloads |
| `flow.json` | Flow sensor payloads |
| `status.json` | Device status payloads |
| `e2e-capture.json` | Complete E2E test capture |

## Usage

### Publishing Test Messages

Use `mosquitto_pub` to publish sample messages:

```bash
# Publish a temperature reading
mosquitto_pub -h localhost -t "paku/temperature/moko/cabin" \
  -m '{"value": 22.5, "timestamp": "14:32:15"}'
```

### Subscribing to All Topics

```bash
# Subscribe to all paku topics
mosquitto_sub -h localhost -t "paku/#" -v
```

## Notes

- Values of `-1000` indicate sensor not available or error condition
- Timestamps are in 24-hour HH:MM:SS format from NTP
- All numeric values are floating-point
