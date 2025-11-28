# End-to-End Testing with Ruuvi Tag

This guide documents how to perform end-to-end testing of the paku-core (EDGE) firmware with a real Ruuvi tag and paku-iot backend.

## Hardware Checklist

Before starting, ensure you have the following components:

### Required Hardware

| Component | Description | Notes |
|-----------|-------------|-------|
| **LilyGo T-Display S3** | ESP32-S3 development board | Primary edge device |
| **Ruuvi Tag** | Bluetooth environmental sensor | Any Ruuvi tag model (RuuviTag, RuuviTag Pro) |
| **USB-C Cable** | Data-capable cable | For programming and serial monitoring |
| **WiFi Network** | 2.4 GHz network with internet access | Required for MQTT connectivity |
| **MQTT Broker** | Running instance (Mosquitto, HiveMQ, etc.) | Can be local or cloud-hosted |

### Optional Hardware

| Component | Description | Notes |
|-----------|-------------|-------|
| **Power Supply** | 5V USB power adapter | For standalone operation |
| **Additional Ruuvi Tags** | For multi-sensor testing | Supports multiple simultaneous devices |
| **Serial Monitor** | Terminal or screen utility | For debugging (built into PlatformIO) |

### Ruuvi Tag Specifications

- **Communication**: Bluetooth Low Energy (BLE)
- **Data Format**: RAWv2 (format 5) recommended
- **Broadcast Interval**: 1–10 seconds (configurable via Ruuvi app)
- **Sensors**: Temperature, humidity, pressure, acceleration, battery voltage

## Connection Steps

### 1. Prepare the Ruuvi Tag

1. **Insert battery** into the Ruuvi tag (if not already installed)
2. **Wait for LED** to blink red briefly, indicating power-on
3. **Configure broadcast mode** using the [Ruuvi Station app](https://ruuvi.com/ruuvi-station/)
   - Open app → Select tag → Settings → Data format: RAWv2
   - Set broadcast interval to 1–5 seconds for testing
4. **Note the MAC address** displayed in the app (e.g., `E3:F5:6A:1B:2C:3D`)

### 2. Prepare the ESP32

1. **Connect the LilyGo T-Display S3** to your computer via USB-C
2. **Verify connection**:
   ```bash
   # Linux
   ls /dev/ttyUSB* /dev/ttyACM*
   
   # macOS
   ls /dev/cu.usbserial* /dev/cu.usbmodem*
   
   # Windows (Device Manager → Ports)
   ```

### 3. Configure WiFi and MQTT

Follow the [configuration guide](config.md) to set up your credentials:

```bash
# Copy the secrets template
cp paku_core/include/secrets.h.template paku_core/include/secrets.h
```

Edit `paku_core/include/secrets.h`:

```cpp
#pragma once

// Wi-Fi Networks (in order of priority)
#define WIFI_SSID_HOME                "MyHomeNetwork"
#define WIFI_PASSWORD_HOME            "wifi-password-here"
#define WIFI_SSID_IPHONE              ""  // Optional fallback
#define WIFI_PASSWORD_IPHONE          ""
#define WIFI_SSID_PAKU                ""  // Optional fallback
#define WIFI_PASSWORD_PAKU            ""

// MQTT Broker
#define MQTT_SERVER                   "192.168.1.100"  // Your broker IP
#define MQTT_PORT                     1883
```

### 4. Flash the Firmware

```bash
cd paku_core
pio run -t upload
```

### 5. Verify Initial Connection

Open serial monitor:

```bash
pio device monitor --baud 115200
```

Expected output:
```
Starting setup...
Setup Wifi Connection...
Connecting to MyHomeNetwork
..........
WiFi connected
IP address: 192.168.1.50
Setup MQTT Connection...
Attempting MQTT connection...
MQTT connected and subscribed to 'paku/control'
Setup Sensor...
Setup complete.
Starting Bluetooth scan...
Devices found: 3
Device 1: Name: , Address: e3:f5:6a:1b:2c:3d, ...
```

## Configuring paku-iot

> **Note**: paku-iot is a separate repository for host-side tools and services. Refer to the [paku-iot documentation](https://github.com/ychefla/paku-iot) for detailed setup.

### MQTT Broker Setup

If you don't have an MQTT broker running, install Mosquitto:

```bash
# Debian/Ubuntu
sudo apt-get install mosquitto mosquitto-clients

# macOS (Homebrew)
brew install mosquitto

# Start the broker
mosquitto -c /etc/mosquitto/mosquitto.conf
```

Verify the broker is running:

```bash
# Subscribe to all paku topics
mosquitto_sub -h localhost -t "paku/#" -v
```

### Expected paku-iot Integration

The paku-core firmware publishes telemetry to the following MQTT topics:

| Topic Pattern | Description |
|--------------|-------------|
| `paku/temperature/...` | Temperature readings |
| `paku/humidity/...` | Humidity readings |
| `paku/flow/...` | Flow sensor data |
| `paku/status/...` | Device status |
| `paku/voltage/...` | Battery voltage |

paku-iot should be configured to:
1. Subscribe to `paku/#` topics
2. Parse the JSON payloads
3. Store data in InfluxDB or another time-series database
4. Provide visualization via Grafana or similar

## End-to-End Test Procedure

### Test 1: Verify WiFi Connection

**Expected**: Device connects to WiFi within 10 seconds.

**Verify**:
```bash
# Check serial output for:
# - "WiFi connected"
# - Valid IP address
```

### Test 2: Verify MQTT Connection

**Expected**: Device connects to MQTT broker and subscribes to `paku/control`.

**Verify**:
```bash
# On the broker host, check connections:
mosquitto_sub -h localhost -t '$SYS/broker/clients/connected' -v
```

### Test 3: Verify BLE Scanning

**Expected**: Device discovers Ruuvi tag via BLE.

**Verify**:
```bash
# Check serial output for:
# - "Starting Bluetooth scan..."
# - "Devices found: X" (where X > 0)
# - Device listing including your Ruuvi tag MAC address
```

### Test 4: Verify Telemetry Publishing

**Expected**: Device publishes telemetry data to MQTT.

**Verify**:
```bash
# Subscribe to paku topics
mosquitto_sub -h localhost -t "paku/#" -v

# Expected output (every sensor interval):
# paku/temperature/moko/cabin {"value": 22.5, "timestamp": "14:32:15"}
# paku/humidity/moko/cabin {"value": 55.2, "timestamp": "14:32:15"}
# paku/flow/coolant {"value": 4.8, "timestamp": "14:32:15"}
```

### Test 5: Verify Round-Trip Command

**Expected**: Device receives and acknowledges commands.

**Verify**:
```bash
# Publish a test command
mosquitto_pub -h localhost -t "paku/control" -m '{"cmd": "status"}'

# Check serial output for command receipt
```

## Sample Payloads

### Temperature Payload

```json
{
  "value": 22.5,
  "timestamp": "14:32:15"
}
```

**Topic**: `paku/temperature/moko/cabin`

### Humidity Payload

```json
{
  "value": 55.2,
  "timestamp": "14:32:15"
}
```

**Topic**: `paku/humidity/moko/cabin`

### Flow Sensor Payload

```json
{
  "value": 4.8,
  "timestamp": "14:32:15"
}
```

**Topic**: `paku/flow/coolant`

### Heater Status Payload

```json
{
  "value": 1,
  "timestamp": "14:32:15"
}
```

**Topic**: `paku/status/heater`

See [Sample Messages](../samples/) for additional examples.

## Sample Log Output

### Successful Boot Sequence

```
Starting setup...
Setup Wifi Connection...
Connecting to MyHomeNetwork
..........
WiFi connected
IP address: 192.168.1.50
Setup MQTT Connection...
Attempting MQTT connection...
MQTT connected and subscribed to 'paku/control'
Setup Sensor...
Setup complete.
```

### BLE Scan Results

```
Starting Bluetooth scan...
Devices found: 3
Device 1: Name: , Address: e3:f5:6a:1b:2c:3d, serviceUUID: , rssi: -65 
Device 2: Name: Ruuvi E3F5, Address: e3:f5:6a:1b:2c:3d, manufacturerData: 0x050F...
Device 3: Name: , Address: aa:bb:cc:dd:ee:ff, rssi: -78
```

### MQTT Publishing

```
.......send to MQTT
.......send to MQTT
```

## Troubleshooting

### WiFi Connection Issues

| Symptom | Cause | Solution |
|---------|-------|----------|
| Repeated "Connecting to..." | Wrong credentials | Verify SSID and password in secrets.h |
| "Failed to connect" | Signal too weak | Move closer to router |
| No networks found | Wrong WiFi mode | Ensure 2.4 GHz network (not 5 GHz only) |

### MQTT Connection Issues

| Symptom | Cause | Solution |
|---------|-------|----------|
| "failed, rc=-2" | Cannot reach broker | Check broker IP and port |
| "failed, rc=5" | Auth required | Add MQTT_USER/MQTT_PASS if broker requires auth |
| No messages received | Topic mismatch | Verify topic subscription on paku-iot side |

### BLE Scanning Issues

| Symptom | Cause | Solution |
|---------|-------|----------|
| "Devices found: 0" | Ruuvi not broadcasting | Check Ruuvi battery; ensure tag is active |
| Tag not in list | Too far away | Move Ruuvi closer (< 10m line of sight) |
| Incorrect data | Wrong data format | Set Ruuvi to RAWv2 format via app |

## Hardware Test Script (Simulation)

For testing without physical Ruuvi hardware, you can simulate Ruuvi BLE advertisements using another ESP32 or a BLE-capable computer. See `tools/ruuvi-simulator.py` for a Python-based simulation script.

> **Note**: The firmware currently has `testMode = true` which generates simulated flow data. This can be used to verify MQTT publishing without actual sensors.

## Known Limitations

1. **BLE Data Parsing**: Current firmware scans for BLE devices but does not parse Ruuvi-specific data formats. Future versions will decode RAWv2 payloads.

2. **Sensor Placeholders**: Many sensor values are set to `-1000` indicating no data. These placeholders will be replaced when hardware sensors are connected.

3. **Sleep Mode**: Device enters deep sleep after 30 seconds and wakes every 15 seconds. This may affect continuous monitoring during testing.

## Follow-Up Tasks

If you encounter issues during testing, please file them as separate issues:

- [ ] BLE data parsing for Ruuvi RAWv2 format
- [ ] MQTT TLS support for secure connections
- [ ] Configurable sleep/wake intervals
- [ ] Dashboard integration with paku-iot

## References

- [Ruuvi Data Formats](https://docs.ruuvi.com/communication/bluetooth-advertisements)
- [ESP32 BLE Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/bluetooth/bt_le.html)
- [Mosquitto MQTT Broker](https://mosquitto.org/documentation/)
- [PlatformIO ESP32 Guide](https://docs.platformio.org/en/latest/platforms/espressif32.html)
