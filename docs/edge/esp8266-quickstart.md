# ESP8266 Wired Sensor Quickstart Guide

This guide will help you set up an ESP8266 device with wired I2C sensors (BME280) for the Paku IoT system.

## Why ESP8266?

The ESP8266 is ideal for scenarios where:
- **Bluetooth is unreliable** or has interference issues
- **Wired sensors** are preferred for stability and reliability
- **Lower cost** is important (ESP8266 is cheaper than ESP32)
- **Lower power consumption** is needed
- **Continuous data collection** from fixed sensor locations

## Hardware Requirements

### Required Components

1. **ESP8266 Development Board**
   - NodeMCU v2 or v3 (recommended)
   - Or any ESP8266 board with at least 4MB flash

2. **BME280 Sensor Module**
   - I2C interface (not SPI)
   - Available from most electronics suppliers
   - Alternative: BMP280 (temperature and pressure only, no humidity)

3. **Additional Components**
   - USB cable (Micro-USB for NodeMCU)
   - Breadboard or PCB for mounting (optional)
   - Jumper wires (female-to-female recommended)

### Hardware Connections

Connect the BME280 sensor to the ESP8266 NodeMCU:

```
BME280 Module    →    NodeMCU Pin
────────────────────────────────
VCC (3.3V)       →    3V3
GND              →    GND
SDA              →    D2 (GPIO4)
SCL              →    D1 (GPIO5)
```

**Important Notes:**
- Use 3.3V power, NOT 5V (BME280 is 3.3V only)
- Keep I2C wires short (< 20cm) for best reliability
- Some BME280 modules have pull-up resistors, some don't
- If you have issues, try adding 4.7kΩ pull-ups on SDA and SCL

## Software Setup

### Step 1: Clone and Configure

```bash
# Clone the repository
git clone https://github.com/ychefla/paku-core.git
cd paku-core
```

### Step 2: Select ESP8266 Device

Copy and edit the device configuration:

```bash
cd paku_core/src
cp device_config.h.template device_config.h
```

Edit `device_config.h` and uncomment the ESP8266 device:

```cpp
// Uncomment this line:
#define DEVICE_ESP8266_WIRED_SENSORS

// Comment out other devices:
// #define DEVICE_LILYGO_T_DISPLAY_S3
// #define DEVICE_ESP32_CH340C_30PIN
```

### Step 3: Configure PlatformIO Environment

Edit `paku_core/platformio.ini`:

```ini
[platformio]
default_envs = esp8266-wired-sensors    # Set this
# default_envs = lilygo-t-display-s3   # Comment out others
# default_envs = esp32-ch340c-30pin
```

### Step 4: Configure WiFi and MQTT

Copy and edit the secrets file:

```bash
cd paku_core/include
cp secrets.h.template secrets.h
```

Edit `secrets.h` with your credentials:

```cpp
// Wi-Fi Networks (in order of priority)
static const char* WIFI_SSIDS[] = { "your-wifi-name" };
static const char* WIFI_PASSWORDS[] = { "your-wifi-password" };
static const size_t WIFI_COUNT = 1;

// MQTT Broker
#define MQTT_SERVER    "mqtt.example.com"
#define MQTT_PORT      1883
```

### Step 5: Build and Upload

Using VS Code with PlatformIO:
1. Open the `paku_core` folder in VS Code
2. Connect the ESP8266 via USB
3. Click the Upload button (→) in the PlatformIO toolbar

Using command line:
```bash
cd paku_core
pio run -e esp8266-wired-sensors -t upload
```

### Step 6: Monitor Serial Output

```bash
pio device monitor -b 115200
```

You should see output like:
```
Starting setup...
Device: ESP8266 Wired Sensors
Setup Wifi Connection...
Connecting to your-wifi-name
WiFi connected
IP address: 192.168.1.100
Setup MQTT Connection...
Setup Wired Sensors (I2C)...
Wired sensor type: BME280
Setup complete.
```

## Data Format

The ESP8266 publishes sensor data to MQTT in the following format:

**Topic:** `paku/sensors/{device_id}_wired/data`

**Payload Example:**
```json
{
  "timestamp": "2024-01-15T10:30:00Z",
  "device_id": "paku-AABBCC_wired",
  "location": "wired_sensor",
  "sensor_type": "BME280",
  "metrics": {
    "temperature_c": 22.5,
    "humidity_percent": 45.2,
    "pressure_hpa": 1013.25
  }
}
```

## Sensor Reading Interval

By default, wired sensors are read every 60 seconds. To change this, edit `main.cpp`:

```cpp
#define WIRED_SENSOR_INTERVAL_MS 60000  // 60 seconds
```

Change to your desired interval in milliseconds (e.g., 30000 for 30 seconds).

## Troubleshooting

### Sensor Not Detected

If you see "Warning: No wired sensors detected" in the serial output:

1. **Check Wiring**
   - Verify all connections are secure
   - Ensure SDA is on D2 (GPIO4) and SCL is on D1 (GPIO5)
   - Check power connections (3.3V and GND)

2. **Check I2C Address**
   - Most BME280 sensors use address 0x76
   - Some use 0x77
   - The code tries both addresses automatically

3. **Test I2C Bus**
   - Use an I2C scanner sketch to verify the sensor is responding
   - Check if the sensor appears at 0x76 or 0x77

4. **Power Issues**
   - Ensure you're using 3.3V (not 5V)
   - Some USB ports don't provide stable power
   - Try a powered USB hub or external 3.3V power supply

5. **Module Issues**
   - BME280 modules can be damaged by 5V
   - Try a different sensor module if available

### WiFi Connection Issues

If WiFi doesn't connect:
- Verify SSID and password in `secrets.h`
- Check that WiFi is 2.4GHz (ESP8266 doesn't support 5GHz)
- Move closer to the WiFi router
- Check serial output for specific error messages

### MQTT Connection Issues

If MQTT doesn't connect:
- Verify MQTT server hostname/IP and port
- Check that the MQTT broker is running
- Verify network connectivity (can you ping the MQTT server?)
- Check MQTT broker logs for connection attempts

### Invalid Sensor Readings

If readings seem wrong:
- Check that sensor is not physically damaged
- Ensure sensor is not exposed to extreme conditions
- Compare with a known good thermometer/sensor
- Valid ranges: Temp -40 to 85°C, Humidity 0-100%, Pressure 300-1100 hPa

## Advanced Configuration

### Custom I2C Pins

To use different I2C pins, edit `device_config.h.template`:

```cpp
#define PIN_I2C_SDA 4       // Change GPIO number
#define PIN_I2C_SCL 5       // Change GPIO number
```

### Multiple Sensors

To support multiple BME280 sensors on the same I2C bus:
- Use sensors with different I2C addresses (0x76 and 0x77)
- Modify the wired_sensors code to support multiple instances
- Or use an I2C multiplexer (TCA9548A)

## Performance Characteristics

- **Power Consumption**: ~70mA active, <1mA deep sleep
- **WiFi Range**: Typical 50-100m indoors
- **Sensor Accuracy**:
  - Temperature: ±1.0°C
  - Humidity: ±3% RH
  - Pressure: ±1 hPa
- **Update Rate**: Configurable, default 60 seconds
- **Flash Usage**: ~400KB (plenty of room for OTA updates)

## Next Steps

- Configure [paku-iot](https://github.com/ychefla/paku-iot) to receive and process data
- Set up Grafana dashboards for visualization
- Configure alerts based on sensor thresholds
- Add more ESP8266 nodes for distributed sensing

## Related Documentation

- [Main README](../../README.md) - Overview and general setup
- [Architecture](../ARCHITECTURE.md) - System design
- [Integration Guide](../INTEGRATION.md) - Connecting to backend
- [Configuration Reference](config.md) - All configuration options
