# EDGE quickstart (ESP32 + PlatformIO)

## Development Modes

This project supports both **container** and **local** development:

- **Container**: Isolated, reproducible environment (use "Reopen in Container" in VS Code)
- **Local**: Direct USB access for flashing physical devices

For embedded development with USB flashing, **local development is recommended** for reliable device access. See [development-modes.md](../development-modes.md) for details.

## Requirements
- VS Code + PlatformIO extension, or PlatformIO CLI
- LilyGo T-Display S3 (ESP32-S3) board
- USB cable

## Configuration
Before building, you must set up your secrets file:

1. Copy the template:
   ```bash
   cp paku_core/include/secrets.h.template paku_core/include/secrets.h
   ```

2. Edit `paku_core/include/secrets.h` with your WiFi and MQTT credentials.

See [config.md](config.md) for detailed configuration options.

## Build & flash (VS Code)
1. Open the `paku_core` folder in VS Code.
2. Install the PlatformIO extension (if not installed).
3. Connect the ESP32 via USB.
4. In the PlatformIO panel:
   - **Build** (checkmark icon)
   - **Upload** (arrow icon)
   - **Monitor** (plug icon, optional)

## Build & flash (CLI)
```bash
cd paku_core
pio run            # Build
pio run -t upload  # Upload to device
pio device monitor # Serial monitor (optional)
```

## Next Steps

After flashing the firmware:

- See [Architecture](../ARCHITECTURE.md) for system overview and data flow
- See [Integration Guide](../INTEGRATION.md) for connecting to paku-iot

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

See [paku-iot-integration.md](paku-iot-integration.md) for complete protocol documentation.
