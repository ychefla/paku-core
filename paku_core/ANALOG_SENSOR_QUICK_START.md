# Quick Start: Analog Temperature Sensor

## 🔌 Hardware Connection (3-wire sensor)

```
ESP8266 NodeMCU     →     Temperature Sensor
================          ==================
3.3V (3V3)          →     VCC (Red wire)
GND                 →     GND (Black wire)  
A0                  →     Signal (Yellow/White wire)
```

⚠️ **CRITICAL**: ESP8266 A0 accepts 0-1V only! NodeMCU boards have built-in voltage divider allowing 0-3.3V.

## 📊 What You Get

Your analog temperature sensor data will appear in the Ruuvi dashboard as:
- **Metric name**: `analog_temp_c`
- **Location**: `wired_sensor`
- **Updates**: Every 60 seconds
- **Alongside**: BME280 temperature, humidity, pressure (if connected)

## 🔧 Default Configuration

The code is pre-configured for the most common sensor type:

**NTC Thermistor (10kΩ at 25°C)**
- Uses voltage divider with 10kΩ series resistor
- Beta coefficient: 3950K
- Temperature range: -40°C to 125°C
- This is typical for waterproof probe sensors

## ✅ Verify It Works

1. **Upload firmware** to ESP8266
2. **Open serial monitor** (115200 baud):
   ```
   Initializing wired sensors...
   Analog temperature sensor enabled on A0
   Wired Sensor: Analog T=23.5°C
   ```
3. **Check MQTT** messages (optional):
   ```bash
   mosquitto_sub -h YOUR_BROKER -t "paku/sensors/+/data"
   ```
4. **View in Grafana** dashboard - look for `analog_temp_c`

## 🎛️ Adjust Calibration (if needed)

If your readings are off, edit `src/wired_sensors.cpp`:

### For NTC Thermistor:
```cpp
#define ANALOG_TEMP_SERIES_R     10000  // Your series resistor value
#define ANALOG_TEMP_NTC_NOMINAL  10000  // Your NTC resistance at 25°C
#define ANALOG_TEMP_B_COEFFICIENT 3950  // Your NTC beta value
```

### For LM35 Linear Sensor:
```cpp
#define ANALOG_TEMP_SERIES_R 0  // No voltage divider

// In readAnalogTemperature() function, replace return statement:
return voltage * 100.0;  // LM35: 10mV/°C = 100°C/V
```

### For TMP36 Linear Sensor:
```cpp
#define ANALOG_TEMP_SERIES_R 0  // No voltage divider

// In readAnalogTemperature() function:
return (voltage - 0.5) * 100.0;  // TMP36: 500mV offset
```

## 🔍 Sensor Type Identification

**If unsure what sensor you have:**

1. **Measure resistance** (sensor disconnected):
   - ~10kΩ at room temp? → NTC thermistor (default config ✓)
   - Infinite/very high? → Probably not thermistor

2. **Measure voltage** (connected to 3.3V/GND only):
   - ~0.25V at 25°C → LM35
   - ~0.75V at 25°C → TMP36

3. **Look for markings** on sensor body

## 🚨 Troubleshooting

| Problem | Solution |
|---------|----------|
| No reading / NaN | Check wiring, verify A0 voltage <1V |
| Wrong temperature | Adjust calibration constants |
| Unstable readings | Increase `ANALOG_TEMP_SAMPLES` to 20 |
| No data in dashboard | Check MQTT connection in serial monitor |

## 📖 Full Documentation

See `ANALOG_TEMP_SENSOR_GUIDE.md` for:
- Detailed wiring diagrams
- Multiple sensor types
- Advanced calibration
- Voltage divider design
- Complete troubleshooting

## ⚡ Quick Test

Touch the sensor probe → temperature should increase within 10-20 seconds!

---

**Status**: ✅ Committed and ready to use!
