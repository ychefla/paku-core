# Display UI Quick Reference

## Button Layout

```
┌─────────────────────────────────┐
│                                 │
│    LilyGo T-Display S3         │
│    ┌─────────────────────┐     │
│    │                     │     │
│    │   320x170 TFT LCD   │     │
│    │                     │     │
│    └─────────────────────┘     │
│                                 │
│  [BTN1]              [BTN2]    │
│  GPIO 0              GPIO 14    │
│  Display             Screen     │
│  On/Off             Switch      │
└─────────────────────────────────┘
```

## Button Functions

| Button | GPIO | Function | Action |
|--------|------|----------|--------|
| BTN1 | 0 | Display Power | Toggle display on/off (saves power) |
| BTN2 | 14 | Screen Navigation | Switch to next screen (cycles) |

## Screen Navigation

```
┌──────────────┐
│  1. Status   │ ─┐
└──────────────┘  │
                  │
┌──────────────┐  │
│ 5. Network   │ ◄┘
└──────────────┘  
       ▲          
       │          
┌──────────────┐  
│ 4. System    │  
└──────────────┘  
       ▲          
       │          
┌──────────────┐  
│ 3. Wired     │  
└──────────────┘  
       ▲          
       │          
┌──────────────┐  
│ 2. BLE       │  
└──────────────┘  

Press BTN2 to cycle through screens
```

## Screen Contents

### 1️⃣ Status Screen
```
┌────────────────────────────────┐
│ Status                         │
├────────────────────────────────┤
│ WiFi: Connected        [GREEN] │
│ MQTT: Connected        [GREEN] │
│ Uptime: 2d 14:32:45            │
│ Device: paku-AABBCC            │
│ Free Heap: 234 KB              │
│ CPU: 240 MHz                   │
│                                │
│ BTN1:On/Off BTN2:Next [1/5]   │
└────────────────────────────────┘
```

### 2️⃣ BLE Sensors Screen
```
┌────────────────────────────────┐
│ BLE Sensors                    │
├────────────────────────────────┤
│ cabin:                         │
│   21.5C  45%  1013hPa         │
│                                │
│ kitchen:                       │
│   23.2C  52%  1012hPa         │
│                                │
│ lounge:                        │
│   22.8C  48%  1013hPa         │
│                                │
│ BTN1:On/Off BTN2:Next [2/5]   │
└────────────────────────────────┘
```

### 3️⃣ Wired Sensors Screen
```
┌────────────────────────────────┐
│ Wired Sensors                  │
├────────────────────────────────┤
│ Type: DS18B20                  │
│ Sensors: 2                     │
│                                │
│ Sensor 1:                      │
│   24.50 C          [LARGE]    │
│                                │
│ Sensor 2:                      │
│   23.75 C          [LARGE]    │
│                                │
│ BTN1:On/Off BTN2:Next [3/5]   │
└────────────────────────────────┘
```

### 4️⃣ System Info Screen
```
┌────────────────────────────────┐
│ System Info                    │
├────────────────────────────────┤
│ Device ID:                     │
│   paku-AABBCCDDEE              │
│ Firmware:                      │
│   v1.1.1                       │
│ Chip: ESP32-S3                 │
│ Flash: 16 MB                   │
│ PSRAM: 8 MB                    │
│                                │
│                                │
│ BTN1:On/Off BTN2:Next [4/5]   │
└────────────────────────────────┘
```

### 5️⃣ Network Screen
```
┌────────────────────────────────┐
│ Network                        │
├────────────────────────────────┤
│ SSID:                          │
│   MyWiFiNetwork                │
│ IP Address:                    │
│   192.168.1.100                │
│ Signal: -45 dBm     [GREEN]   │
│                                │
│ MAC:                           │
│   AA:BB:CC:DD:EE:FF            │
│                                │
│ BTN1:On/Off BTN2:Next [5/5]   │
└────────────────────────────────┘
```

## Status Colors

| Color | Meaning | Usage |
|-------|---------|-------|
| 🟢 Green | Good/Connected | WiFi connected, MQTT connected, strong signal |
| 🟡 Yellow | Warning | Moderate signal, missing data |
| 🔴 Red | Error/Disconnected | No connection, weak signal, errors |
| 🔵 Cyan | Highlight | Headers, labels, important values |
| ⚪ White | Normal | Regular text and data |
| ⚫ Grey | Secondary | MAC addresses, hints, less important info |

## Power Saving

### Display Off Mode
- **Activation**: Press BTN1
- **Behavior**: 
  - Screen goes black
  - Backlight turns off
  - System continues running (WiFi, MQTT, sensors)
- **Power Saved**: ~30-50mA (depends on backlight current)
- **Reactivation**: Press BTN1 again

### Tips
- Turn off display when not actively monitoring to save battery
- Display automatically updates when turned back on
- Button 2 only works when display is on
- All system functions continue regardless of display state

## Troubleshooting

| Issue | Solution |
|-------|----------|
| Display blank | Press BTN1 to turn on |
| Buttons not working | Check GPIO connections (0 and 14) |
| No sensor data | Verify sensors are configured and in range |
| Screen frozen | Check serial logs for errors, power cycle device |
| Flickering | Increase update interval in code |

## Code Integration

### Minimal Example
```cpp
#include "display_ui.h"

TFT_eSPI tft;

void setup() {
    // Initialize display hardware
    tft.begin();
    tft.setRotation(3);
    
    // Initialize UI system
    displayUI.begin(&tft);
}

void loop() {
    // Update UI (handles buttons and refresh)
    displayUI.update();
    
    // Your other code...
}
```

### Setting Status
```cpp
// Update WiFi status
displayUI.setWiFiStatus("Connected: MyNetwork");

// Update MQTT status
displayUI.setMQTTConnected(true);

// Set device ID
displayUI.setDeviceId("paku-123456");
```

## File Locations

```
paku_core/
├── src/
│   ├── display_ui.h         # UI header
│   ├── display_ui.cpp       # UI implementation
│   └── main.cpp            # Integration
└── lib/
    └── OneButton/          # Button library
```

## Build Requirements

### PlatformIO Dependencies
```ini
lib_deps =
    TFT_eSPI
    OneButton
```

### Compiler Flags
```ini
build_flags =
    -DHAS_DISPLAY=1
```

## Performance

- **Screen Refresh**: 1 second (configurable)
- **Button Response**: <50ms (debounced)
- **Memory Usage**: ~2KB RAM
- **CPU Usage**: <5% (mostly display updates)

## Compatibility

- ✅ LilyGo T-Display S3 (ESP32-S3)
- ✅ ESP32 devices with TFT_eSPI support
- ❌ ESP8266 (different display support)
- ⚠️ Requires HAS_DISPLAY flag enabled
