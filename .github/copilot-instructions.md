# Copilot Instructions for paku-core

This repository contains ESP32-based edge firmware for the Paku IoT system. Below are guidelines to help Copilot understand the project and provide effective assistance.

## Communication Style

- **Never present assumptions, guesses, or generated logic as verified facts.**
- If something is uncertain, say so explicitly using phrases like "I assume", "I'm not sure", "this is my best guess", or "I don't have a verified source for this".
- When generating protocol implementations, hardware interfaces, or integration code, always state whether the implementation is based on verified documentation or inference.
- Prefer "I don't know" over a confident-sounding but unverified answer.

## Project Overview

**paku-core (EDGE)** is ESP32 firmware that:
- Runs on LilyGo T-Display S3 (ESP32-S3) hardware
- Collects sensor data (temperature, humidity, flow, voltage)
- Communicates via MQTT to the paku-iot backend
- Integrates with BLE sensors (Ruuvi tags)
- Displays status on an onboard ST7789V TFT display

### Terminology
- **EDGE** = the ESP32 device firmware (canonical term)
- **CORE** = legacy term (synonym for EDGE)
- Repository `paku-core` → EDGE firmware
- Repository `paku-iot` → Host-side services

## Development Environment

### Prerequisites
- PlatformIO CLI or VS Code with PlatformIO extension
- USB cable for flashing

### Building and Flashing
```bash
cd paku_core
pio run            # Build
pio run -t upload  # Upload to device
pio device monitor # Serial monitor (115200 baud)
```

### Configuration
- **Secrets**: Copy `paku_core/include/secrets.h.template` to `paku_core/include/secrets.h` (git-ignored)
- **Build flags**: Defined in `paku_core/platformio.ini`
- **Pin mappings**: Defined in `paku_core/src/pin_config.h`

## Code Style and Conventions

### C++ Style
- Use descriptive function and variable names
- Add Doxygen-style comments (`@brief`, `@param`, `@note`) for public functions
- Prefer `const` where applicable
- Use `static` for file-local variables
- Avoid dynamic allocation in hot paths; prefer static buffers

### MQTT Topics
Topics use the prefix `paku/devices/{device_id}/`:
- `state` - Device online/offline status (retained)
- `telemetry` - Periodic sensor data
- `cmd` - Inbound commands (JSON)
- `lwt` - Last Will and Testament

### JSON Payload Format
```json
{
  "value": 23.5,
  "timestamp": "12:34:56",
  "device_id": "paku-AABBCC"
}
```

## Architecture

### Key Components
- **WiFi Manager**: Multi-SSID connection with fallback
- **MQTT Client**: Connection handling and message publishing
- **BLE Scanner**: Bluetooth sensor discovery (FreeRTOS task)
- **Display Driver**: TFT display output
- **Interval Manager**: Controls timing for data collection

### Hardware Abstraction
- Pin configurations in `src/pin_config.h`
- Board definitions in `boards/` directory
- Target board: LilyGo T-Display S3

## Security Guidelines

- **Never commit credentials** - `secrets.h` must be git-ignored
- All sensitive configs belong in `include/secrets.h`
- Use MQTT TLS when the broker supports it
- Set MQTT LWT to `offline` for connection monitoring

## Testing

### Local Testing
```bash
cd paku_core
pio run  # Verify build succeeds
```

### Smoke Test Checklist
1. WiFi connects successfully
2. MQTT connects and subscribes to control topic
3. Telemetry publishes at expected intervals
4. Device recovers from WiFi/MQTT disconnection

## Documentation

Key documentation files:
- [Architecture](docs/ARCHITECTURE.md) - System design and data flow
- [Integration Guide](docs/INTEGRATION.md) - Connecting to paku-iot
- [Configuration Reference](docs/edge/config.md) - Configuration options
- [Requirements](docs/requirements.md) - Functional and non-functional requirements
- [Naming Conventions](docs/naming.md) - Terminology

## Repository Structure

```
paku-core/
├── .github/            # GitHub configuration and Copilot instructions
├── .devcontainer/      # Development container configuration
├── paku_core/          # PlatformIO project
│   ├── src/            # Source code (main.cpp, pin_config.h)
│   ├── include/        # Header files (secrets.h - git-ignored)
│   ├── lib/            # Bundled libraries
│   ├── boards/         # Custom board definitions
│   └── platformio.ini  # PlatformIO configuration
├── docs/               # Documentation
└── README.md           # Project overview
```

## Dependencies

Managed via PlatformIO (see `paku_core/platformio.ini`):
- PubSubClient@^2.8 - MQTT client
- ArduinoJson@^7.2.0 - JSON serialization
- NTPClient@^3.2.1 - Time synchronization
- TFT_eSPI (bundled) - Display driver

## Best Practices for Tasks

When working on this codebase:
1. Keep changes focused and minimal
2. Maintain non-blocking behavior in the main loop
3. Preserve watchdog compatibility
4. Test builds locally with `pio run` before committing
5. Document new functions with Doxygen comments
6. Update relevant documentation if adding features
