# Display UI Visual Reference

## Hardware Button Layout

```
                  LilyGo T-Display S3
    ┌─────────────────────────────────────────┐
    │                                         │
    │  ┌─────────────────────────────────┐   │
    │  │░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░│   │
    │  │░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░│   │
    │  │░░░░░░  ST7789V TFT Display ░░░░░│   │
    │  │░░░░░░░   320 x 170 pixels  ░░░░░│   │
    │  │░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░│   │
    │  │░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░│   │
    │  └─────────────────────────────────┘   │
    │                                         │
    │                                         │
    │   ┌──────┐                  ┌──────┐   │
    │   │BTN 1 │                  │BTN 2 │   │
    │   └──────┘                  └──────┘   │
    │   GPIO 0                    GPIO 14    │
    │  ON / OFF                    NEXT      │
    └─────────────────────────────────────────┘
```

## Screen Examples

### 1. Status Screen
```
┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓
┃ Status                         ┃ ← Cyan header
┣━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┫
┃                                ┃
┃ WiFi: Connected        ✓       ┃ ← Green = good
┃ MQTT: Connected        ✓       ┃
┃ Uptime: 2d 14:32:45            ┃
┃ Device: paku-AABBCC            ┃
┃ Free Heap: 234 KB              ┃
┃ CPU: 240 MHz                   ┃
┃                                ┃
┃________________________________┃
┃ BTN1:On/Off BTN2:Next [1/5]   ┃ ← Grey footer
┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛
```

### 2. BLE Sensors Screen
```
┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓
┃ BLE Sensors                    ┃
┣━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┫
┃                                ┃
┃ cabin:                         ┃ ← Cyan label
┃   21.5°C  45%  1013hPa        ┃
┃                                ┃
┃ kitchen:                       ┃
┃   23.2°C  52%  1012hPa        ┃
┃                                ┃
┃ lounge:                        ┃
┃   22.8°C  48%  1013hPa        ┃
┃________________________________┃
┃ BTN1:On/Off BTN2:Next [2/5]   ┃
┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛
```

### 3. Wired Sensors Screen
```
┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓
┃ Wired Sensors                  ┃
┣━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┫
┃                                ┃
┃ Type: DS18B20                  ┃
┃ Sensors: 2                     ┃
┃                                ┃
┃ Sensor 1:                      ┃
┃   24.50 °C                     ┃ ← Large font
┃                                ┃
┃ Sensor 2:                      ┃
┃   23.75 °C                     ┃
┃________________________________┃
┃ BTN1:On/Off BTN2:Next [3/5]   ┃
┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛
```

### 4. System Info Screen
```
┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓
┃ System Info                    ┃
┣━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┫
┃                                ┃
┃ Device ID:                     ┃
┃   paku-AABBCCDDEE              ┃ ← Cyan value
┃ Firmware:                      ┃
┃   v1.1.1                       ┃
┃ Chip: ESP32-S3                 ┃
┃ Flash: 16 MB                   ┃
┃ PSRAM: 8 MB                    ┃
┃                                ┃
┃________________________________┃
┃ BTN1:On/Off BTN2:Next [4/5]   ┃
┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛
```

### 5. Network Screen
```
┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓
┃ Network                        ┃
┣━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┫
┃                                ┃
┃ SSID:                          ┃
┃   MyWiFiNetwork                ┃ ← Cyan value
┃ IP Address:                    ┃
┃   192.168.1.100                ┃
┃ Signal: -45 dBm        ✓       ┃ ← Green = strong
┃                                ┃
┃ MAC:                           ┃
┃   AA:BB:CC:DD:EE:FF            ┃ ← Grey value
┃________________________________┃
┃ BTN1:On/Off BTN2:Next [5/5]   ┃
┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛
```

## Navigation Flow

```
        Press BTN2 →

   ┌──────────────┐
   │  1. Status   │
   └──────────────┘
          ↓
   ┌──────────────┐
   │ 2. BLE       │
   │    Sensors   │
   └──────────────┘
          ↓
   ┌──────────────┐
   │ 3. Wired     │
   │    Sensors   │
   └──────────────┘
          ↓
   ┌──────────────┐
   │ 4. System    │
   │    Info      │
   └──────────────┘
          ↓
   ┌──────────────┐
   │ 5. Network   │
   └──────────────┘
          ↓
   ┌──────────────┐
   │ 1. Status    │ ← Cycles back
   └──────────────┘
```

## Display States

### Display ON
```
┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓
┃                                ┃
┃  ░░░░░░░ SCREEN VISIBLE ░░░░░  ┃  ← Backlight ON
┃  ░░░░░░ DATA DISPLAYED  ░░░░░  ┃  ← Content rendered
┃  ░░░░░░ BUTTONS ACTIVE  ░░░░░  ┃  ← BTN2 works
┃                                ┃
┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛

Power: ~120-150mA
```

### Display OFF
```
┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓
┃                                ┃
┃                                ┃  ← Backlight OFF
┃       ████████████████         ┃  ← Screen black
┃       ████████████████         ┃  ← BTN2 inactive
┃                                ┃  ← BTN1 still works
┃                                ┃
┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛

Power: ~70-100mA (saves 30-50mA)
```

## Color Legend

```
┌────────────────────────────────────┐
│ Color Usage                        │
├────────────────────────────────────┤
│                                    │
│ ███ CYAN    Headers & Labels       │
│ ███ WHITE   Normal Text            │
│ ███ GREEN   Good Status            │
│ ███ YELLOW  Warnings               │
│ ███ RED     Errors                 │
│ ███ GREY    Secondary Info         │
│ ███ BLACK   Background             │
│                                    │
└────────────────────────────────────┘
```

## Button Operation

### Single Click Detection
```
Button Press Timeline:

    ↓ Press           ↑ Release
    |                 |
    |<---- 50ms ----->|  Debounce period
    |                 |<- 400ms ->| Click detected
    ├─────────────────┼───────────┤
    │   Button LOW    │  Trigger  │
    └─────────────────┴───────────┘
```

### BTN1 Behavior
```
State Machine:

   Display OFF              Display ON
   ┌─────────┐             ┌─────────┐
   │ ░░░░░░░ │  BTN1 →    │ ░░░░░░░ │
   │ ░░░░░░░ │  Click     │ ░░░░░░░ │
   │ ████░░░ │  ←──────── │ ░░Data░ │
   │ ████░░░ │             │ ░Shows░ │
   └─────────┘             └─────────┘
    Backlight 0%          Backlight 100%
```

### BTN2 Behavior
```
Screen Cycling:

Screen 1  →  Screen 2  →  Screen 3
   ↑                           ↓
   └──────  Screen 5  ←  Screen 4

Each BTN2 click advances by one screen
```

## Real-World Display Examples

### Morning Check (Status Screen)
```
┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓
┃ Status                         ┃
┣━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┫
┃                                ┃
┃ WiFi: Connected        ✓       ┃ ← All systems OK
┃ MQTT: Connected        ✓       ┃
┃ Uptime: 0d 08:23:11            ┃ ← Running ~8 hours
┃ Device: paku-cabin01           ┃
┃ Free Heap: 198 KB              ┃ ← Plenty of memory
┃ CPU: 240 MHz                   ┃
┃                                ┃
┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛
```

### Cold Day (BLE Sensors)
```
┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓
┃ BLE Sensors                    ┃
┣━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┫
┃                                ┃
┃ cabin:                         ┃
┃   18.2°C  38%  1019hPa        ┃ ← Cold & dry
┃                                ┃
┃ kitchen:                       ┃
┃   21.5°C  45%  1018hPa        ┃ ← Warmer
┃                                ┃
┃ outdoor:                       ┃
┃   -5.3°C  82%  1020hPa        ┃ ← Freezing!
┃                                ┃
┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛
```

### WiFi Issues (Status Screen)
```
┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓
┃ Status                         ┃
┣━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┫
┃                                ┃
┃ WiFi: Disconnected     ✗       ┃ ← RED = problem
┃ MQTT: Disconnected     ✗       ┃ ← RED = problem
┃ Uptime: 1d 04:15:33            ┃
┃ Device: paku-cabin01           ┃
┃ Free Heap: 187 KB              ┃
┃ CPU: 240 MHz                   ┃
┃                                ┃
┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛
```

### Network Details (Network Screen)
```
┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓
┃ Network                        ┃
┣━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┫
┃                                ┃
┃ SSID:                          ┃
┃   HomeNetwork5G                ┃
┃ IP Address:                    ┃
┃   192.168.1.42                 ┃
┃ Signal: -38 dBm        ✓       ┃ ← Excellent!
┃                                ┃
┃ MAC:                           ┃
┃   48:E7:DA:AA:BB:CC            ┃
┃                                ┃
┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛
```

## Pin Connections

### LilyGo T-Display S3 GPIO Map
```
┌─────────────────────────────────┐
│ Function        GPIO   Usage    │
├─────────────────────────────────┤
│ Button 1        0      ON/OFF   │
│ Button 2        14     NEXT     │
│ LCD Backlight   38     PWM      │
│ LCD Reset       5      Control  │
│ LCD CS          6      Control  │
│ LCD DC          7      Control  │
│ LCD WR          8      Control  │
│ LCD RD          9      Control  │
│ LCD D0-D7       39-48  Data     │
│ Power Enable    15     Control  │
└─────────────────────────────────┘
```

## Size Reference

### Display Dimensions
```
┌───────────── 320px ─────────────┐
│                                 │ ↑
│ ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░ │ │
│ ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░ │ │
│ ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░ │ 170px
│ ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░ │ │
│ ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░ │ ↓
└─────────────────────────────────┘

Landscape orientation (rotation = 3)
```

### Text Sizes
```
┌────────────────────────────────┐
│ Size 1: Small text (8px)      │
│                                │
│ Size 2: Headers                │
│                                │
│ Size 3: Not used               │
└────────────────────────────────┘
```

## Usage Tips

### Quick Access
```
1. Want device info?      → BTN2 x3 to System Info
2. Check WiFi signal?     → BTN2 x4 to Network
3. Monitor temperatures?  → BTN2 x1 to BLE Sensors
4. Check system status?   → Stay on Status (default)
5. Save power?            → BTN1 to turn off display
```

### Best Practices
```
✓ Turn off display when not monitoring
✓ Check Status screen first for quick overview
✓ Use Network screen for WiFi troubleshooting
✓ Monitor BLE Sensors for temperature trends
✓ Check System Info for memory/uptime issues

✗ Don't press buttons rapidly (debounce delay)
✗ Don't expect BTN2 to work when display is off
✗ Don't rely solely on display (check MQTT too)
```

## Mounting Suggestion
```
    Wall/Desk Mount

    ┌─────────────────┐
    │   [Display]     │ ← Eye level
    │   [  Unit  ]    │ ← Easy button access
    │   [Buttons ]    │ ← USB cable below
    └────────┬────────┘
             │ USB-C
             └─────────→ Power
```
