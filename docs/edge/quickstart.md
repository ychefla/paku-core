# EDGE quickstart (ESP32 + PlatformIO)

## Requirements
- VS Code + PlatformIO extension, or PlatformIO CLI
- An ESP32 board and a USB cable

## Build & flash (VS Code)
1. Open this repo folder in VS Code.
2. Install the PlatformIO extension (if not installed).
3. Connect the ESP32 via USB.
4. In the PlatformIO panel:
   - **Build**
   - **Upload**
   - **Monitor** (optional)

## Build & flash (CLI)
```bash
# from repo root
pio run
pio run -t upload
pio device monitor
