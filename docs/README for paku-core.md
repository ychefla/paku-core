# paku-core (EDGE)

ESP32-based edge firmware for the Paku IoT system.

## Hardware
- **Board**: LilyGo T-Display S3 (ESP32-S3)
- **Display**: ST7789V 170x320 TFT
- **Connectivity**: WiFi, Bluetooth

## Features
- Heater control and monitoring
  - Measuring heater temperatures
  - Measuring coolant flows
  - Controlling heater
  - Controlling extra pump for floor heating
- Environmental monitoring
  - Room temperatures and humidities (Cabin, Kitchen, Lounge, Dryer)
- Car battery monitoring
- Data processing and forwarding to cloud via MQTT

## Development Setup

This workspace supports both **container** and **local** development modes:

- **Container**: Use "Reopen in Container" in VS Code for an isolated, pre-configured environment
- **Local**: Open directly on host machine for USB device access (recommended for flashing)

> **Note**: For embedded development requiring USB flashing, local development is recommended for reliable device access. While USB passthrough in containers is supported, it may not work on all platforms. See [Development Modes](docs/development-modes.md) for details.

### Prerequisites
- [VS Code](https://code.visualstudio.com/) with [PlatformIO extension](https://platformio.org/install/ide?install=vscode)
- Or [PlatformIO CLI](https://docs.platformio.org/en/latest/core/installation.html)
- USB cable for flashing the ESP32

### Configuration
1. Copy the secrets template:
   ```bash
   cp paku_core/include/secrets.h.template paku_core/include/secrets.h
   ```
2. Edit `paku_core/include/secrets.h` with your WiFi and MQTT credentials

### Build & Flash

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

## Documentation
- [Architecture](ARCHITECTURE.md) - System architecture and data flow
- [Integration Guide](INTEGRATION.md) - Connecting paku-core to paku-iot
- [Development Modes](development-modes.md) - Container vs. local development
- [Quickstart Guide](edge/quickstart.md) - Detailed setup instructions
- [Configuration Reference](edge/config.md) - Configuration options
- [Naming Conventions](naming.md) - Terminology and naming
- [Requirements](requirements.md) - Functional and non-functional requirements

## Repository Structure
```
paku-core/
├── .devcontainer/      # Development container configuration
├── paku_core/          # PlatformIO project
│   ├── src/            # Source code
│   ├── include/        # Header files (including secrets.h)
│   ├── lib/            # Libraries
│   ├── boards/         # Custom board definitions
│   └── platformio.ini  # PlatformIO configuration
├── docs/               # Documentation
└── README.md           # This file
```

## Related Repositories
- **paku-iot** - Host-side tools/services (collectors, monitors, integrations)
