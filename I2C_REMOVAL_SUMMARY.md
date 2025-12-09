# I2C Sensor Removal Summary

## Overview
Removed I2C sensor support (BME280, BMP280) from the firmware while keeping analog temperature sensor functionality intact.

## Changes Made

### 1. Library Dependencies (`platformio.ini`)
**Removed:**
- `adafruit/Adafruit BME280 Library@^2.2.4`
- `adafruit/Adafruit Unified Sensor@^1.1.14`

These libraries are no longer needed since we're not using I2C sensors.

### 2. Wired Sensors Header (`src/wired_sensors.h`)
**Changes:**
- Removed `#include <Wire.h>`
- Removed `#include <Adafruit_Sensor.h>`
- Removed `#include <Adafruit_BME280.h>`
- Updated file description to remove I2C sensor references
- Simplified `WiredSensorData` structure:
  - **Removed:** `temperature`, `humidity`, `pressure`, `hasAnalogTemp`
  - **Kept:** `analogTemp`, `valid`, `timestamp`
- Simplified `WiredSensors` class:
  - **Removed:** `_bme` (Adafruit_BME280), `_initialized`, `initBME280()`
  - **Kept:** `_hasAnalogTemp`, `_sensorType`, analog sensor methods
- Updated `begin()` signature documentation (sda/scl parameters now unused but kept for API compatibility)
- Updated `isAvailable()` to check analog sensor instead of I2C sensors

### 3. Wired Sensors Implementation (`src/wired_sensors.cpp`)
**Changes:**
- Updated file description
- Removed `_initialized` from constructor
- Completely rewrote `begin()` function to only initialize analog sensors (no I2C bus initialization)
- Removed `initBME280()` function entirely
- Simplified `readSensors()` function:
  - Removed all BME280 I2C sensor reading code
  - Now only reads analog temperature sensor
  - Removed `hasAnalogTemp` flag from data structure
- Kept `readAnalogTemperature()` function unchanged (still supports NTC thermistors and voltage sensors)

### 4. Device Configuration (`src/device_config.h`)
**Changes for ESP8266 Wired Sensors:**
- Changed device name from "ESP8266 Wired Sensors" to "ESP8266 Analog Sensors"
- Removed I2C pin definitions: `PIN_I2C_SDA` and `PIN_I2C_SCL`
- Kept analog sensor configuration: `HAS_ANALOG_TEMP` and `PIN_ANALOG_TEMP`
- Updated comments to reflect analog sensors instead of I2C sensors

**Changes for ESP32 variants:**
- Removed unused I2C pin definitions from ESP32-S3 and ESP32 configurations
- These boards don't use wired sensors anyway, so removed unnecessary definitions

### 5. Main Application (`src/main.cpp`)
**Changes:**
- Updated comment from "Wired sensor support" to "Analog sensor support"
- Updated initialization message from "Setup Wired Sensors (I2C)..." to "Setup Analog Sensors..."
- Changed `begin()` call to pass dummy values (0, 0) since I2C pins are no longer used
- Simplified MQTT payload creation:
  - Removed code for `temperature_c`, `humidity_percent`, `pressure_hpa`
  - Only sends `analog_temp_c` metric
- Updated serial output to only show analog temperature
- Updated function documentation for `createWiredSensorPayloads()`:
  - Changed description from "BME280, etc." to "analog sensor data"
  - Updated comment to mention temperature only (no humidity/pressure)
- Updated loop comments to refer to "Analog sensor data" instead of "Wired sensor data (BME280, etc.)"

## What Still Works

### Analog Temperature Sensors
The analog temperature sensor functionality is fully intact and continues to work:
- **NTC Thermistor support** (10K @ 25°C with beta coefficient)
- **Direct voltage sensors** (LM35, TMP36, etc.)
- Configurable calibration constants in `wired_sensors.cpp`:
  - `ANALOG_TEMP_ENABLED` - Enable/disable analog sensor
  - `ANALOG_TEMP_SAMPLES` - Number of samples to average
  - `ANALOG_TEMP_REF_VOLTAGE` - ADC reference voltage
  - `ANALOG_TEMP_SERIES_R` - Series resistor for voltage divider
  - `ANALOG_TEMP_NTC_NOMINAL` - NTC resistance at 25°C
  - `ANALOG_TEMP_B_COEFFICIENT` - Beta coefficient for NTC

### MQTT Data Format
The sensor now publishes:
```json
{
  "timestamp": "2025-12-09T07:00:00Z",
  "device_id": "paku_wired",
  "location": "wired_sensor",
  "sensor_type": "Analog",
  "metrics": {
    "analog_temp_c": 21.5
  }
}
```

Topic: `paku/sensors/{device_id}_wired/data`

## Testing Recommendations

1. **Verify compilation:**
   ```bash
   cd paku_core
   platformio run -e esp8266-wired-sensors
   ```

2. **Test analog sensor:**
   - Connect your analog temperature sensor to A0
   - Verify the calibration constants in `wired_sensors.cpp` match your sensor
   - Monitor serial output to see temperature readings
   - Check MQTT messages are published correctly

3. **Verify no I2C errors:**
   - Ensure no "BME280" or "I2C" error messages appear in serial output
   - Confirm device boots successfully without I2C sensors connected

## Migration Notes

If you need to add I2C sensors back in the future:
1. Add library dependencies back to `platformio.ini`
2. Restore I2C includes and pin definitions
3. Expand `WiredSensorData` structure with additional fields
4. Add back I2C initialization in `begin()` and sensor reading in `readSensors()`
5. Update MQTT payload creation to include additional metrics

## Configuration

To adjust analog sensor calibration for your specific sensor, edit the constants at the top of `src/wired_sensors.cpp`:

### For NTC Thermistor (default):
```cpp
#define ANALOG_TEMP_ENABLED      1
#define ANALOG_TEMP_SERIES_R     10000  // Series resistor (ohms)
#define ANALOG_TEMP_NTC_NOMINAL  10000  // NTC at 25°C (ohms)
#define ANALOG_TEMP_B_COEFFICIENT 3950  // Beta coefficient
```

### For Direct Voltage Sensor (LM35):
```cpp
#define ANALOG_TEMP_ENABLED      1
#define ANALOG_TEMP_SERIES_R     0      // No voltage divider
// Then uncomment and modify in readAnalogTemperature():
// return voltage * 100.0;  // 10mV/°C = 100°C/V
```
