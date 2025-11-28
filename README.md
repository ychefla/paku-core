# paku-core (EDGE)

ESP32-based edge firmware for the Paku IoT system.

## Hardware

- **Board**: LilyGo T-Display S3 (ESP32-S3)
- **Display**: ST7789V 170x320 TFT
- **Connectivity**: WiFi, Bluetooth Low Energy (BLE)
- **Sensors**: Flow sensors, temperature sensors, Ruuvi tags (via BLE)

## Features

- Heater control and monitoring
  - Measuring heater temperatures
  - Measuring coolant flows
  - Controlling heater
  - Controlling extra pump for floor heating
- Environmental monitoring
  - Room temperatures and humidities (Cabin, Kitchen, Lounge, Dryer)
- BLE sensor integration (Ruuvi tags or equivalent)
- Car battery monitoring
- Data processing and forwarding to cloud via MQTT

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

### Step 2: Configure Secrets

Create your secrets file from the template:

```bash
cp paku_core/include/secrets.h.template paku_core/include/secrets.h
```

Edit `paku_core/include/secrets.h` with your WiFi and MQTT credentials:

```cpp
#pragma once

// Wi-Fi Networks (in order of priority)
#define WIFI_SSID_HOME                "your-home-wifi-ssid"
#define WIFI_PASSWORD_HOME            "your-home-wifi-password"
#define WIFI_SSID_IPHONE              "your-iphone-hotspot-ssid"
#define WIFI_PASSWORD_IPHONE          "your-iphone-hotspot-password"
#define WIFI_SSID_PAKU                "your-paku-wifi-ssid"
#define WIFI_PASSWORD_PAKU            "your-paku-wifi-password"

// MQTT Broker
#define MQTT_SERVER                   "your-mqtt-server-hostname"
#define MQTT_PORT                     1883
```

### Step 3: Build & Flash

#### Using VS Code + PlatformIO

1. Open the `paku_core` folder in VS Code
2. Install the PlatformIO extension (if not installed)
3. Connect the ESP32 via USB
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

### Step 4: Verify

Once flashed, the device will:
1. Display the Paku logo on the TFT screen
2. Connect to WiFi (cycling through configured networks)
3. Connect to the MQTT broker
4. Start publishing telemetry data

Monitor the serial output to verify connections:
```bash
pio device monitor -b 115200
```

## Development Setup

This workspace supports both **container** and **local** development modes:

- **Container**: Use "Reopen in Container" in VS Code for an isolated, pre-configured environment
- **Local**: Open directly on host machine for USB device access (recommended for flashing)

> **Note**: For embedded development requiring USB flashing, local development is recommended for reliable device access. While USB passthrough in containers is supported, it may not work on all platforms. See [Development Modes](docs/development-modes.md) for details.

## Dependencies

The firmware uses the following libraries (managed via PlatformIO):

| Library | Version | Purpose |
|---------|---------|---------|
| PubSubClient | ^2.8 | MQTT client |
| ArduinoJson | ^7.2.0 | JSON serialization |
| NTPClient | ^3.2.1 | Time synchronization |
| TFT_eSPI | (bundled) | Display driver |

## Documentation

- [Architecture](docs/ARCHITECTURE.md) - System architecture and data flow
- [Integration Guide](docs/INTEGRATION.md) - Connecting paku-core to paku-iot
- [Development Modes](docs/development-modes.md) - Container vs. local development
- [Quickstart Guide](docs/edge/quickstart.md) - Detailed setup instructions
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
