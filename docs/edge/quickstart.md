# EDGE Quickstart

## Supported Devices

| Environment | Board | Features |
|------------|-------|----------|
| `lilygo-t-display-s3` | LilyGo T-Display S3 (ESP32-S3) | Display, touch, BLE (Ruuvi) |
| `esp32-ch340c-30pin` | Generic ESP32 DevKit 30-pin | BLE (Ruuvi), headless |
| `esp8266-wired-sensors` | ESP8266 NodeMCU v2 | DS18B20 temp sensor, no BLE |

See [wiring.md](wiring.md) for pin assignments and wiring diagrams.

## Requirements
- VS Code + PlatformIO extension, or PlatformIO CLI
- USB cable
- Your target board (see table above)
- For ESP8266: DS18B20 sensor + 4.7kΩ pull-up resistor ([wiring](wiring.md#esp8266--ds18b20-temperature-sensor))

## Setup

1. **Select your device** — set `default_envs` in `platformio.ini`:
   ```ini
   [platformio]
   default_envs = lilygo-t-display-s3  ; or esp32-ch340c-30pin / esp8266-wired-sensors
   ```

2. **Configure secrets**:
   ```bash
   cp paku_core/include/secrets.h.template paku_core/include/secrets.h
   ```
   Edit `secrets.h` with your WiFi and MQTT credentials. See [config.md](config.md) for all options.

3. **Build & flash**:
   ```bash
   cd paku_core
   pio run            # Build
   pio run -t upload  # Upload to device
   pio device monitor # Serial monitor (115200 baud)
   ```
   Or use the PlatformIO panel in VS Code (Build → Upload → Monitor).

   To target a specific environment without changing `platformio.ini`:
   ```bash
   pio run -e esp8266-wired-sensors -t upload
   ```

## Next Steps

After flashing the firmware:

- See [Architecture](../ARCHITECTURE.md) for system overview and data flow
- See [Integration Guide](../INTEGRATION.md) for connecting to paku-iot
- See [E2E Testing with Ruuvi](e2e-test-ruuvi.md) for hardware testing with Ruuvi tags

## paku-iot Integration

The EDGE device can send telemetry data to paku-iot using two transport methods:

1. **MQTT** (default): Real-time messaging via an MQTT broker
2. **HTTP**: Direct REST API calls to paku-iot endpoints

### Enabling HTTP Transport

To enable direct HTTP communication with paku-iot:

1. Edit `paku_core/include/secrets.h`:
   ```cpp
   #define PAKU_IOT_ENABLED              1
   #define PAKU_IOT_HOST                 "iot.paku.example.com"
   #define PAKU_IOT_PORT                 443
   #define PAKU_IOT_API_KEY              "your-api-key"
   #define PAKU_IOT_USE_TLS              true
   ```

2. Rebuild and flash the firmware.

### Local Testing

For local development and testing:

1. **Test MQTT locally** with Mosquitto:
   ```bash
   # Start local MQTT broker
   docker run -d -p 1883:1883 eclipse-mosquitto
   
   # Subscribe to all paku topics
   mosquitto_sub -h localhost -t "paku/#" -v
   ```

2. **Test HTTP locally** with a mock server:
   ```bash
   # Simple Python mock server
   python -c "
   from http.server import HTTPServer, BaseHTTPRequestHandler
   import json

   class Handler(BaseHTTPRequestHandler):
       def do_POST(self):
           content_length = int(self.headers.get('Content-Length', 0))
           body = self.rfile.read(content_length)
           print('Received:', json.loads(body))
           self.send_response(200)
           self.send_header('Content-Type', 'application/json')
           self.end_headers()
           self.wfile.write(b'{\"status\": \"ok\"}')

   HTTPServer(('0.0.0.0', 8080), Handler).serve_forever()
   "
   ```

3. Configure `secrets.h` for local testing:
   ```cpp
   #define PAKU_IOT_ENABLED              1
   #define PAKU_IOT_HOST                 "192.168.1.100"  // Your local IP
   #define PAKU_IOT_PORT                 8080
   #define PAKU_IOT_API_KEY              "test-key"
   #define PAKU_IOT_USE_TLS              false
   ```

See [Integration Guide](../INTEGRATION.md) for complete protocol documentation.
