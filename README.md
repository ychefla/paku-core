# paku-core (EDGE)

ESP32/ESP8266-based edge firmware for the Paku IoT system.

## Hardware

### Supported Platforms

#### ESP32-S3 (LilyGo T-Display S3)
- **Board**: LilyGo T-Display S3 (ESP32-S3)
- **Display**: ST7789V 170x320 TFT
- **Connectivity**: WiFi, Bluetooth Low Energy (BLE)
- **Sensors**: Flow sensors, Ruuvi tags (via BLE), optional wired I2C sensors

#### ESP32 (Generic/CH340C)
- **Board**: Generic ESP32 development board
- **Connectivity**: WiFi, Bluetooth Low Energy (BLE)
- **Sensors**: Flow sensors, Ruuvi tags (via BLE), optional wired I2C sensors

#### ESP8266 (NodeMCU/Generic) **NEW**
- **Board**: NodeMCU v2/v3 or generic ESP8266 development board
- **Connectivity**: WiFi only (no BLE)
- **Sensors**: Wired I2C sensors (BME280 for temperature, humidity, pressure)

## Features

- Heater control and monitoring
  - Measuring heater temperatures
  - Measuring coolant flows
  - Controlling heater
  - Controlling extra pump for floor heating
- Environmental monitoring
  - Room temperatures and humidities (Cabin, Kitchen, Lounge, Dryer)
- **BLE sensor integration** (ESP32 only)
  - Ruuvi tags or equivalent BLE sensors
- **Wired I2C sensor integration** (ESP8266 and ESP32)
  - BME280: Temperature, humidity, and atmospheric pressure
  - Expandable to other I2C sensors
- Car battery monitoring
- Data processing and forwarding to cloud via MQTT

## Development Setup

This workspace supports both **container** and **local** development modes:

- **Container**: Use "Reopen in Container" in VS Code for an isolated, pre-configured environment
- **Local**: Open directly on host machine for USB device access (recommended for flashing)

> **Note**: For embedded development requiring USB flashing, local development is recommended for reliable device access. While USB passthrough in containers is supported, it may not work on all platforms. See [Development Modes](docs/development-modes.md) for details.

## Quick Start

### Prerequisites

- [VS Code](https://code.visualstudio.com/) with [PlatformIO extension](https://platformio.org/install/ide?install=vscode)
- Or [PlatformIO CLI](https://docs.platformio.org/en/latest/core/installation.html)
- USB cable for flashing the ESP32

### Step 1: Clone the Repository

```bash
git clone https://github.com/ychefla/paku-core.git
cd paku-core
```

### Step 2: Configure Device and Secrets

**Important**: Create your device configuration file from the template:

```bash
cp paku_core/src/device_config.h.template paku_core/src/device_config.h
```

The template defaults to ESP8266 with wired sensors. For other devices, edit the file and uncomment the appropriate device definition.

Create your secrets file from the template:

```bash
cp paku_core/include/secrets.h.template paku_core/include/secrets.h
```

Edit `paku_core/include/secrets.h` with your WiFi and MQTT credentials:

```cpp
#pragma once

// Wi-Fi Networks (in order of priority)
static const char* WIFI_SSIDS[] = { "your-home-wifi-ssid", "your-iphone-hotspot-ssid", "your-paku-wifi-ssid" };
static const char* WIFI_PASSWORDS[] = { "your-home-wifi-password", "your-iphone-hotspot-password", "your-paku-wifi-password" };
static const size_t WIFI_COUNT = sizeof(WIFI_SSIDS) / sizeof(WIFI_SSIDS[0]);

// MQTT Broker
#define MQTT_SERVER                   "your-mqtt-server-hostname"
#define MQTT_PORT                     1883
```

### Step 3: Select Your Hardware Platform

Edit `paku_core/src/device_config.h` (copy from `device_config.h.template` if needed):

```cpp
// Uncomment ONE of these based on your hardware:
#define DEVICE_LILYGO_T_DISPLAY_S3    // ESP32-S3 with display and BLE
// #define DEVICE_ESP32_CH340C_30PIN   // ESP32 headless with BLE
// #define DEVICE_ESP8266_WIRED_SENSORS // ESP8266 with wired I2C sensors
```

And set the matching environment in `paku_core/platformio.ini`:

```ini
[platformio]
default_envs = lilygo-t-display-s3         # For ESP32-S3 with display
# default_envs = esp32-ch340c-30pin        # For ESP32 headless
# default_envs = esp8266-wired-sensors     # For ESP8266 with wired sensors
```

### Step 4: Build & Flash

#### Using VS Code + PlatformIO

1. Open the `paku_core` folder in VS Code
2. Install the PlatformIO extension (if not installed)
3. Connect your device via USB
4. Use the PlatformIO panel to:
   - **Build** (checkmark icon)
   - **Upload** (arrow icon)
   - **Monitor** (plug icon, optional)

#### Using CLI

```bash
cd paku_core
pio run            # Build
pio run -t upload  # Upload to device
pio device monitor # Serial monitor (optional)
```

### Step 5: Verify

Once flashed, the device will:
1. (ESP32-S3 only) Display the Paku logo on the TFT screen
2. Connect to WiFi (cycling through configured networks)
3. Connect to the MQTT broker
4. Start publishing telemetry data

Monitor the serial output to verify connections:
```bash
pio device monitor -b 115200
```

## ESP8266 Wired Sensor Setup

The ESP8266 platform is ideal for locations where Bluetooth is unreliable or unnecessary, and a wired, stable connection is preferred.

### Hardware Requirements

- ESP8266 development board (NodeMCU v2/v3 recommended)
- BME280 sensor module (I2C interface)
- Jumper wires for I2C connections

### Wiring (for NodeMCU)

Connect BME280 to NodeMCU:
- **VCC** → 3.3V
- **GND** → GND
- **SDA** → D2 (GPIO4)
- **SCL** → D1 (GPIO5)

### Configuration

1. Set device type in `paku_core/src/device_config.h`:
   ```cpp
   #define DEVICE_ESP8266_WIRED_SENSORS
   ```

2. Set PlatformIO environment in `paku_core/platformio.ini`:
   ```ini
   [platformio]
   default_envs = esp8266-wired-sensors
   ```

3. Configure WiFi and MQTT in `paku_core/include/secrets.h` (same as ESP32)

### Features

- **Temperature**: Accurate temperature readings (-40°C to +85°C)
- **Humidity**: Relative humidity (0-100%)
- **Pressure**: Atmospheric pressure (300-1100 hPa)
- **MQTT Publishing**: Data sent to `paku/sensors/{device_id}_wired/data`
- **Low Power**: ESP8266 consumes less power than ESP32
- **Stable**: Wired I2C connection is more reliable than BLE in noisy environments

### Troubleshooting

If the BME280 sensor is not detected:
1. Check I2C wiring (SDA and SCL connections)
2. Verify sensor I2C address (default 0x76, some use 0x77)
3. Monitor serial output for initialization messages
4. Test I2C bus with an I2C scanner sketch

### Smoke Test

To verify the build environment is set up correctly:
```bash
cd paku_core
pio run            # Should compile without errors
```

## Dependencies

The firmware uses the following libraries (managed via PlatformIO):

| Library | Version | Purpose | Platforms |
|---------|---------|---------|-----------|
| PubSubClient | ^2.8 | MQTT client | All |
| ArduinoJson | ^7.2.0 | JSON serialization | All |
| NTPClient | ^3.2.1 | Time synchronization | All |
| Adafruit BME280 Library | ^2.2.4 | Wired sensor driver | All |
| Adafruit Unified Sensor | ^1.1.14 | Sensor abstraction | All |
| TFT_eSPI | (bundled) | Display driver | ESP32-S3 only |

## Documentation

- [Architecture](docs/ARCHITECTURE.md) - System architecture and data flow
- [Integration Guide](docs/INTEGRATION.md) - Connecting paku-core to paku-iot
- [Development Modes](docs/development-modes.md) - Container vs. local development
- [Quickstart Guide](docs/edge/quickstart.md) - Detailed setup instructions
- [ESP8266 Quickstart](docs/edge/esp8266-quickstart.md) - **NEW** Setup guide for ESP8266 with wired sensors
- [Configuration Reference](docs/edge/config.md) - Configuration options
- [Naming Conventions](docs/naming.md) - Terminology and naming
- [Requirements](docs/requirements.md) - Functional and non-functional requirements

## Repository Structure


```
paku-core/
├── .devcontainer/      # Development container configuration
├── paku_core/          # PlatformIO project
│   ├── src/            # Source code
│   │   ├── main.cpp    # Main firmware entry point
│   │   ├── pin_config.h # Hardware pin definitions
│   │   └── img_logo.h  # Display logo data
│   ├── include/        # Header files (including secrets.h)
│   ├── lib/            # Bundled libraries
│   ├── boards/         # Custom board definitions
│   └── platformio.ini  # PlatformIO configuration
├── docs/               # Documentation
└── README.md           # This file
```

## Reusable Modules

The following modules are designed for reuse in other projects:

| Module | Location | Description |
|--------|----------|-------------|
| WiFi Manager | `src/main.cpp` (connect_wifi) | Multi-SSID WiFi connection with fallback |
| MQTT Client | `src/main.cpp` (connectMQTT, sendToMQTT) | MQTT connection and message publishing |
| BLE Scanner | `src/main.cpp` (scanBT) | Bluetooth Low Energy device discovery |
| Display Driver | `src/main.cpp` (updateDisplay) | TFT display output for LilyGo T-Display S3 |

> **Note**: These modules are currently embedded in main.cpp. Future refactoring may extract them into separate libraries for easier reuse.

## Related Repositories

- **[paku-iot](https://github.com/ychefla/paku-iot)** - Host-side tools/services (collectors, monitors, integrations)

## License

See [LICENSE](LICENSE) for details.
