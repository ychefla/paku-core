# Perform OTA Update Now

## Current Status ✅

- ✅ Firmware built with analog sensor support
- ✅ Firmware size: 425 KB (431,069 bytes)
- ✅ SHA256: `6c0227d21b5566462e62c0a0951509f4adbfb839cb727ebeb515fde1cc0821de`
- ✅ OTA update script ready
- ✅ mosquitto tools installed
- ✅ Your local IP: `192.168.1.240`

## Quick OTA Update (Automatic)

### Run this single command:

```bash
cd /Users/jossu/GIT/paku/paku-core/paku_core
./ota_update.sh esp8266_wired
```

**What this does:**
1. Starts HTTP server on port 8888 hosting firmware
2. Calculates and verifies checksum
3. Sends MQTT command to your ESP8266
4. Monitors OTA progress in real-time
5. Reports when complete

**Expected output:**
```
================================================
ESP8266 OTA Update Script
================================================

✓ Firmware file found
  Size: 431069 bytes (420.97 KB)
  SHA256: 6c0227d21b5566462e62c0a0951509f4adbfb839cb727ebeb515fde1cc0821de

Starting HTTP server...
  Host: 192.168.1.240:8888

✓ OTA command sent successfully

Monitoring OTA progress...
[STATUS] accepted
[PROGRESS] downloading - 25%
[PROGRESS] downloading - 50%
[PROGRESS] downloading - 75%
[PROGRESS] downloading - 100%
[PROGRESS] installing - 100%
[RESULT] success

========================================
OTA Update Successful!
========================================
```

## Manual OTA Update (Step by Step)

If the automatic script doesn't work, follow these steps:

### Step 1: Start HTTP Server

```bash
cd /Users/jossu/GIT/paku/paku-core/paku_core/.pio/build/esp8266-wired-sensors
python3 -m http.server 8888
```

Leave this terminal open.

### Step 2: Send OTA Command (New Terminal)

**Update these values for your setup:**
```bash
# Your device ID (check serial monitor or use default)
DEVICE_ID="esp8266_wired"

# Your MQTT broker address (update this!)
MQTT_BROKER="localhost"  # or "192.168.1.x" or your cloud MQTT broker

# Firmware URL (use your actual local IP)
FIRMWARE_URL="http://192.168.1.240:8888/firmware.bin"

# Checksum
CHECKSUM="6c0227d21b5566462e62c0a0951509f4adbfb839cb727ebeb515fde1cc0821de"

# Version
VERSION="v$(date +%Y%m%d)_analog_sensor"
```

**Send the command:**
```bash
mosquitto_pub -h $MQTT_BROKER -p 1883 \
  -t "paku/devices/${DEVICE_ID}/cmd/ota" \
  -m "{\"url\":\"${FIRMWARE_URL}\",\"checksum\":\"${CHECKSUM}\",\"version\":\"${VERSION}\"}"
```

### Step 3: Monitor Progress (New Terminal)

```bash
mosquitto_sub -h $MQTT_BROKER -p 1883 -v \
  -t "paku/devices/${DEVICE_ID}/ota/#"
```

Watch for:
- `ota/status` - Device acknowledges command
- `ota/progress` - Download/install progress
- `ota/result` - Final success/failure

## Important Notes

### Device ID
Your device ID is probably `esp8266_wired` but check your device's serial output on boot:
```
Device ID: esp8266_wired
```

### MQTT Broker Address
Update `MQTT_BROKER` to match your setup:
- Local: `localhost` or `127.0.0.1` (if on same machine)
- Local network: `192.168.1.x` (your server IP)
- Cloud: Your cloud MQTT broker address

You can find this in your `include/secrets.h`:
```cpp
#define MQTT_SERVER "your-mqtt-server-hostname"
```

### Firewall
The HTTP server (port 8888) must be reachable from your ESP8266:
```bash
# Test from another terminal
curl http://192.168.1.240:8888/firmware.bin -o /tmp/test.bin
ls -lh /tmp/test.bin  # Should show 425K
```

## Verification After OTA

### 1. Check Serial Monitor
You should see:
```
Initializing wired sensors...
Analog temperature sensor enabled on A0
Wired Sensor: Analog T=23.5°C
```

### 2. Check MQTT Messages
```bash
mosquitto_sub -h $MQTT_BROKER -t "paku/sensors/+/data" -v
```

Look for `analog_temp_c` in the payload:
```json
{
  "metrics": {
    "analog_temp_c": 23.5,
    "temperature_c": 23.2,
    "humidity_percent": 45.0
  }
}
```

### 3. Check Dashboard
In Grafana, you should now see a new metric: `analog_temp_c`

## Troubleshooting

### "Connection refused" when sending MQTT
- Check MQTT broker is running: `docker ps | grep mosquitto`
- Verify MQTT broker address is correct
- Test MQTT: `mosquitto_pub -h $MQTT_BROKER -t test -m hello`

### Device doesn't respond
- Check device is connected to WiFi (serial monitor)
- Check device is connected to MQTT (serial monitor shows "Connected to MQTT")
- Verify device ID matches

### "Cannot download firmware"
- Verify HTTP server is running on port 8888
- Test from device's network: `curl http://192.168.1.240:8888/firmware.bin`
- Check firewall isn't blocking port 8888
- Use device network IP, not `localhost`

### OTA fails after download starts
- Check serial monitor for error details
- Verify firmware file isn't corrupted (rebuild if needed)
- Ensure ESP8266 has enough free space (~400KB needed)

## Quick Commands Reference

```bash
# Build firmware
platformio run -e esp8266-wired-sensors

# Run automatic OTA
./ota_update.sh esp8266_wired

# Monitor serial output
platformio device monitor -e esp8266-wired-sensors

# Monitor MQTT
mosquitto_sub -h localhost -t "paku/#" -v

# Test MQTT connection
mosquitto_pub -h localhost -t test -m "hello"

# Check firmware size
ls -lh .pio/build/esp8266-wired-sensors/firmware.bin

# Calculate checksum
shasum -a 256 .pio/build/esp8266-wired-sensors/firmware.bin
```

---

## Ready? Let's Do It! 🚀

Run the automatic script:
```bash
./ota_update.sh esp8266_wired
```

Or follow the manual steps above if you need more control.

The update takes about 30-60 seconds. Watch for the success message!

**After success:** Your ESP8266 will restart and start reporting analog temperature data! 🌡️
