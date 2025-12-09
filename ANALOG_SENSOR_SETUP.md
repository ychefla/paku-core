# Analog Temperature Sensor Setup Guide

## Quick Start

The firmware now supports **only analog temperature sensors** on the ESP8266 platform. I2C sensors (BME280, BMP280) have been removed.

## Supported Sensor Types

### 1. NTC Thermistor (Default Configuration)
The most common analog temperature sensor. Uses a voltage divider circuit.

**Wiring:**
```
VCC (3.3V) ─── [10kΩ resistor] ─── A0 ─── [NTC Thermistor] ─── GND
```

**Default calibration:**
- Series resistor: 10kΩ
- NTC nominal resistance: 10kΩ @ 25°C
- Beta coefficient: 3950
- Valid for most common 10K NTC thermistors

### 2. Linear Voltage Output Sensors
Sensors that output voltage directly (no voltage divider needed).

**Examples:**
- **LM35**: 10mV/°C (0°C = 0V, 100°C = 1V)
- **TMP36**: 10mV/°C with 500mV offset (-40°C = 0.1V, 25°C = 0.75V)

**Wiring:**
```
Sensor VCC → 3.3V
Sensor GND → GND
Sensor OUT → A0
```

## Configuration

Edit `/Users/jossu/GIT/paku/paku-core/paku_core/src/wired_sensors.cpp` to adjust for your sensor:

### NTC Thermistor Configuration
```cpp
#define ANALOG_TEMP_ENABLED      1      // Enable analog sensor
#define ANALOG_TEMP_SAMPLES      10     // Samples to average (reduce noise)
#define ANALOG_TEMP_REF_VOLTAGE  1.0    // ESP8266 ADC max voltage (1.0V)
#define ANALOG_TEMP_SERIES_R     10000  // Series resistor value (ohms)
#define ANALOG_TEMP_NTC_NOMINAL  10000  // NTC resistance at 25°C (ohms)
#define ANALOG_TEMP_TEMP_NOMINAL 25.0   // Reference temperature (°C)
#define ANALOG_TEMP_B_COEFFICIENT 3950  // NTC beta coefficient
```

### LM35 Configuration
```cpp
#define ANALOG_TEMP_ENABLED      1
#define ANALOG_TEMP_SAMPLES      10
#define ANALOG_TEMP_REF_VOLTAGE  1.0
#define ANALOG_TEMP_SERIES_R     0      // No voltage divider!

// Then in readAnalogTemperature(), uncomment:
// return voltage * 100.0;  // 10mV/°C = 100°C/V
```

### TMP36 Configuration
```cpp
#define ANALOG_TEMP_ENABLED      1
#define ANALOG_TEMP_SAMPLES      10
#define ANALOG_TEMP_REF_VOLTAGE  1.0
#define ANALOG_TEMP_SERIES_R     0

// Then in readAnalogTemperature(), uncomment:
// return (voltage - 0.5) * 100.0;  // TMP36 offset
```

## ESP8266 ADC Limitations

**Important:** ESP8266 ADC can only measure 0-1V!

If your sensor outputs higher voltages:
1. Use a voltage divider to scale down to 0-1V
2. Adjust `ANALOG_TEMP_REF_VOLTAGE` to match your divider
3. OR use an external ADC module (ADS1115) connected via I2C (requires code changes)

**Example voltage divider for 0-3.3V input:**
```
Sensor OUT ─── [22kΩ] ─── A0 ─── [10kΩ] ─── GND

Output voltage = Input × (10kΩ / (22kΩ + 10kΩ)) = Input × 0.3125
3.3V input → 1.03V at A0 (slightly over, use 20kΩ instead of 22kΩ)
```

## Hardware Setup

### ESP8266 NodeMCU Pin Mapping
- **A0**: Analog input (0-1V max)
- Located between D0 and RSV pins on NodeMCU board

### Testing Your Sensor

1. **Upload the firmware:**
   ```bash
   cd paku_core
   platformio run -e esp8266-wired-sensors --target upload
   ```

2. **Monitor serial output:**
   ```bash
   platformio device monitor
   ```

3. **Look for these messages:**
   ```
   Setup Analog Sensors...
   Sensor type: Analog
   Analog Sensor: T=21.5°C
   ```

4. **Verify MQTT messages:**
   - Topic: `paku/sensors/paku_wired/data`
   - Payload should include: `"analog_temp_c": 21.5`

## Troubleshooting

### Temperature reads as NaN or invalid
- Check wiring - ensure A0 is connected
- Verify voltage is within 0-1V range
- Check `ANALOG_TEMP_ENABLED` is set to 1

### Temperature is way off
- **Too high/low:** Check calibration constants match your sensor
- **Unstable:** Increase `ANALOG_TEMP_SAMPLES` (try 20-50)
- **Always same value:** Sensor might be open or shorted

### No sensor readings at all
- Check `HAS_ANALOG_TEMP` is defined in `device_config.h` (should be 1 for ESP8266)
- Verify `HAS_WIRED_SENSORS` is enabled (should be 1)
- Check serial output for error messages

### Voltage divider calculations
For NTC thermistor, measure actual resistor value with multimeter:
- Series resistor value affects readings
- Use actual measured value in `ANALOG_TEMP_SERIES_R`
- For best accuracy, use 1% tolerance resistors

## MQTT Data Format

Published to: `paku/sensors/{device_id}_wired/data`

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

## References

- **NTC Thermistor Calculations:** Steinhart-Hart equation (simplified beta formula)
- **ESP8266 ADC:** 10-bit (0-1023) for 0-1V input
- **Sampling:** Multiple samples averaged to reduce noise

## Next Steps

After confirming your analog sensor works:
1. Adjust calibration if needed for accuracy
2. Change `WIRED_SENSOR_INTERVAL_MS` in `main.cpp` if you need different reading frequency (default: 60 seconds)
3. Update `location` field in `main.cpp` if you want more descriptive location names
