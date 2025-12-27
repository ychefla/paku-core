# Quick OTA Update Guide for ESP8266

## Overview

Your ESP8266 device supports OTA (Over-The-Air) updates via MQTT. The device listens for OTA commands and automatically downloads and installs new firmware.

## Prerequisites

- Firmware built: `.pio/build/esp8266-wired-sensors/firmware.bin`
- Device connected to WiFi and MQTT broker
- `mosquitto` client tools installed
- HTTP server to host firmware (or use the automated script)

## Method 1: Automated Script (Easiest)

### Quick Start

```bash
# Build latest firmware
platformio run -e esp8266-wired-sensors

# Run OTA update
./ota_update.sh esp8266_wired

# Or specify custom device ID and version
./ota_update.sh my_device_id v2.0.1
```

The script will:
1. ✓ Verify firmware file exists
2. ✓ Start HTTP server to host firmware
3. ✓ Calculate SHA256 checksum
4. ✓ Send MQTT command to device
5. ✓ Monitor OTA progress in real-time
6. ✓ Report success/failure

### What You'll See

```
================================================
ESP8266 OTA Update Script
================================================

✓ Firmware file found
  File: .pio/build/esp8266-wired-sensors/firmware.bin
  Size: 431069 bytes (420.97 KB)
  SHA256: a1b2c3d4e5f6...

Starting HTTP server...
  Host: 192.168.1.100:8888

✓ Firmware accessible at: http://192.168.1.100:8888/firmware.bin

Sending OTA command to device: esp8266_wired

✓ OTA command sent successfully

Monitoring OTA progress...
[STATUS] 2024-01-01T12:00:00Z - accepted - 0%
[PROGRESS] 2024-01-01T12:00:05Z - downloading - 10%
[PROGRESS] 2024-01-01T12:00:10Z - downloading - 25%
[PROGRESS] 2024-01-01T12:00:15Z - downloading - 50%
[PROGRESS] 2024-01-01T12:00:20Z - downloading - 75%
[PROGRESS] 2024-01-01T12:00:25Z - downloading - 100%
[PROGRESS] 2024-01-01T12:00:28Z - installing - 100%
[RESULT] 2024-01-01T12:00:30Z - success - 0%

========================================
OTA Update Successful!
========================================
Device should restart with new firmware now.
```

## Method 2: Manual Steps

### Step 1: Build Firmware

```bash
cd /path/to/paku-core/paku_core
platformio run -e esp8266-wired-sensors
```

### Step 2: Host Firmware on HTTP Server

**Option A: Python HTTP Server**
```bash
cd .pio/build/esp8266-wired-sensors
python3 -m http.server 8888
```

**Option B: nginx or Apache**
Copy firmware.bin to your web server's document root.

### Step 3: Get Your Local IP

```bash
# macOS/Linux
ifconfig | grep "inet " | grep -v 127.0.0.1

# Example output: 192.168.1.100
```

### Step 4: Calculate Firmware Checksum (Optional but Recommended)

```bash
shasum -a 256 .pio/build/esp8266-wired-sensors/firmware.bin
```

### Step 5: Send OTA Command via MQTT

```bash
# Set your values
DEVICE_ID="esp8266_wired"
FIRMWARE_URL="http://192.168.1.100:8888/firmware.bin"
FIRMWARE_SHA256="your_sha256_checksum_here"
VERSION="v2.0.1"

# Send MQTT command
mosquitto_pub -h localhost -p 1883 \
  -t "paku/edge/${DEVICE_ID}/cmd/ota" \
  -m "{\"url\":\"${FIRMWARE_URL}\",\"checksum\":\"${FIRMWARE_SHA256}\",\"version\":\"${VERSION}\"}"
```

### Step 6: Monitor Progress

```bash
# In another terminal, subscribe to OTA topics
mosquitto_sub -h localhost -p 1883 -v \
  -t "paku/edge/${DEVICE_ID}/ota/#"
```

You'll see messages like:
```
paku/edge/esp8266_wired/ota/status {"status":"accepted","version":"v2.0.1"}
paku/edge/esp8266_wired/ota/progress {"state":"downloading","percent":25}
paku/edge/esp8266_wired/ota/progress {"state":"downloading","percent":50}
paku/edge/esp8266_wired/ota/progress {"state":"installing","percent":100}
paku/edge/esp8266_wired/ota/result {"status":"success"}
```

## MQTT Topics

### Command Topic (Publish to this)
```
paku/edge/{device_id}/cmd/ota
```

**Payload format:**
```json
{
  "url": "http://192.168.1.100:8888/firmware.bin",
  "checksum": "sha256_hash_optional",
  "version": "v2.0.1"
}
```

### Status Topics (Subscribe to these)
```
paku/edge/{device_id}/ota/status     # Acknowledgment
paku/edge/{device_id}/ota/progress   # Download/install progress
paku/edge/{device_id}/ota/result     # Final result (success/failed)
```

## OTA Process Flow

```
1. Device receives MQTT command
   ↓
2. Device validates URL
   ↓
3. Device sends "accepted" status
   ↓
4. Device downloads firmware
   - Reports progress every 10%
   ↓
5. Device verifies checksum (if provided)
   ↓
6. Device installs firmware
   - Reports "installing" status
   ↓
7. Device sends result (success/failed)
   ↓
8. Device restarts with new firmware
```

## Troubleshooting

### Device Not Responding to OTA Command

**Check:**
1. Is device connected to MQTT?
   ```bash
   mosquitto_sub -h localhost -t "paku/edge/${DEVICE_ID}/#" -v
   ```

2. Is device ID correct?
   - Check serial monitor for device ID on boot
   - Look for: "Device ID: esp8266_wired"

3. Is MQTT broker reachable?
   ```bash
   mosquitto_pub -h localhost -t "test" -m "hello"
   ```

### Firmware Download Fails

**Common Issues:**

1. **HTTP server not accessible from device**
   - Use device's network IP, not `localhost`
   - Check firewall settings
   - Test: `curl http://YOUR_IP:8888/firmware.bin`

2. **URL incorrect**
   - Full path must be: `http://IP:PORT/firmware.bin`
   - Don't use `https://` (ESP8266 needs special setup for HTTPS)

3. **Checksum mismatch**
   - Recalculate: `shasum -a 256 firmware.bin`
   - Or omit checksum from command (not recommended)

### Firmware Install Fails

1. **Not enough space**
   - ESP8266 needs ~400KB free for OTA
   - Check partition table in `platformio.ini`

2. **Corrupted firmware**
   - Rebuild firmware
   - Verify download completed (check file size)

3. **Network interruption**
   - Ensure stable WiFi during update
   - Update takes ~30-60 seconds

### Device Doesn't Restart

1. **Check serial monitor** for errors
2. **Power cycle device** if update succeeded but didn't restart
3. **Check OTA result** message for error details

## Best Practices

### 1. Test First
Always test OTA on a development device before deploying to production devices.

### 2. Use Checksums
Include SHA256 checksum to prevent corrupted firmware installation:
```bash
CHECKSUM=$(shasum -a 256 firmware.bin | awk '{print $1}')
```

### 3. Version Tracking
Use meaningful version numbers:
```bash
VERSION="v$(date +%Y%m%d)_analog_sensor"
```

### 4. Backup Current Firmware
Before OTA, save a copy of working firmware:
```bash
cp firmware.bin firmware_backup_$(date +%Y%m%d).bin
```

### 5. Monitor Serial Output
Keep serial monitor open during OTA to see detailed progress:
```
OTA: Update scheduled - URL: http://192.168.1.100:8888/firmware.bin
OTA: Target version: v2.0.1
OTA: Connecting to http://192.168.1.100:8888/firmware.bin
OTA: Firmware size: 431069 bytes
OTA Progress: 10%
OTA Progress: 25%
...
OTA: Firmware written successfully
OTA: Update successful, rebooting...
```

### 6. Have a Fallback Plan
If OTA fails, be prepared to:
- Physically access device for USB upload
- Or use ESP8266's built-in bootloader recovery

## Network Requirements

### Firewall
Ensure HTTP port (default 8888) is open:
```bash
# macOS
sudo pfctl -d  # Temporarily disable firewall for testing

# Linux
sudo ufw allow 8888
```

### Same Network
Device and firmware server should be on same network, or:
- Use port forwarding if needed
- Use public HTTP server (for production)

### Bandwidth
OTA requires:
- Download: ~400-500 KB
- Time: 30-60 seconds on good WiFi
- Network should be stable during update

## Production Considerations

For production deployments:

1. **Use HTTPS** with proper certificates
2. **Host firmware** on reliable server (not laptop)
3. **Implement rollout strategy** (test → canary → full)
4. **Monitor update success rate**
5. **Keep old firmware** available for rollback
6. **Use OTA service** (paku-iot has full OTA service)

## Next Steps

- ✅ Test OTA update with analog sensor firmware
- ✅ Verify analog temperature sensor works after update
- ✅ Check dashboard for new `analog_temp_c` metric
- 📝 Document OTA process for your team
- 🔧 Set up automated OTA pipeline (optional)

## Related Documentation

- `ota_update.sh` - Automated OTA script
- `OtaClient.h` - ESP8266/ESP32 OTA client library
- `docs/ota_updates.md` - Full OTA service documentation
- `ANALOG_SENSOR_QUICK_START.md` - Analog sensor setup

---

**Ready to update?** Run `./ota_update.sh` and watch the magic happen! ✨
