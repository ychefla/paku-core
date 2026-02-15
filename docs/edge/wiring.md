# Wiring Reference

Pin assignments and wiring diagrams for each supported device variant.

## ESP8266 — DS18B20 Temperature Sensor

**Board**: NodeMCU v2 (or generic ESP8266)
**PlatformIO env**: `esp8266-wired-sensors`

### Wiring

```
ESP8266 NodeMCU          DS18B20 (TO-92 package)
───────────────          ──────────────────────
3.3V (or 5V) ──────┬──── VDD  (pin 3, red)
                    │
                   [4.7kΩ]   ← pull-up resistor (required)
                    │
GPIO5 (D1)  ───────┴──── DQ   (pin 2, yellow/white)
GND         ──────────── GND  (pin 1, black)
```

**DS18B20 pinout** (flat side facing you, legs down): GND | DQ | VDD

### Notes
- The **4.7kΩ pull-up** between DQ and VDD is mandatory for reliable 1-Wire communication
- Multiple DS18B20 sensors can share the same bus (parallel DQ wires)
- For long cable runs (>2m), add a **100nF capacitor** between VDD and GND near the sensor
- Pin defined in `device_config.h`: `PIN_DS18B20 5` (GPIO5 = D1 on NodeMCU)

### Troubleshooting

| Symptom | Cause | Fix |
|---------|-------|-----|
| Reads **-127°C** | Sensor not detected | Check wiring, verify pull-up resistor |
| Reads **85°C** | Sensor not yet initialized | Power cycle, check code timing |
| Intermittent readings | Noise on long wires | Add 100nF cap, shorten cable, check connections |

## ESP32 CH340C 30PIN

**Board**: Generic ESP32 DevKit (30-pin)
**PlatformIO env**: `esp32-ch340c-30pin`

Headless (no display). Uses BLE for Ruuvi tag scanning — no wiring required beyond USB power.

| Pin | Function |
|-----|----------|
| GPIO2 | Onboard LED (active HIGH) |

## LilyGo T-Display S3

**Board**: LilyGo T-Display S3 (ESP32-S3)
**PlatformIO env**: `lilygo-t-display-s3`

Built-in ST7789V TFT display and capacitive touch. Uses BLE for Ruuvi tags — no external wiring required.

| Pin | Function |
|-----|----------|
| GPIO43 | RGB LED (WS2812, board-version dependent) |
| GPIO38 | LCD backlight (do NOT repurpose) |

## Hydronic Heater Add-on

See [hydronic-heater wiring](../../../../hydronic-heater/docs/wiring.md) for Autoterm UART, DS18B20 coolant sensor, and flow sensor connections.
