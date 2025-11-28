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
