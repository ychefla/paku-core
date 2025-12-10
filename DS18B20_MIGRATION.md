# DS18B20 Digital Temperature Sensor Migration

## Overview
Replaced analog temperature sensor code with DS18B20 1-Wire digital temperature sensor support.

## Hardware Changes Required

### Wiring for DS18B20
**Connect your DS18B20 to:**
- **Red wire (VCC)** → 3.3V or 5V (DS18B20 works with both)
- **Black wire (GND)** → GND
- **Yellow wire (Data)** → GPIO5 (D1 on NodeMCU)
- **4.7kΩ pull-up resistor** between Data and VCC (required for 1-Wire protocol)

```
         3.3V/5V
            │
            ├─────[4.7kΩ]─────┐
            │                  │
       DS18B20              GPIO5 (D1)
         Red                Yellow
            │
           GND
          Black
```

**Important:** The 4.7kΩ pull-up resistor is **mandatory** for reliable 1-Wire communication!

## Software Changes Made

### 1. Library Dependencies (`platformio.ini`)
**Added:**
- `paulstoffregen/OneWire@^2.3.7` - 1-Wire protocol support
- `milesburton/DallasTemperature@^3.11.0` - DS18B20 driver

### 2. Device Configuration
**Updated ESP8266 config:**
- Changed device name to "ESP8266 DS18B20 Sensor"
- Removed `HAS_ANALOG_TEMP` flag
- Added `HAS_DS18B20` flag
- Changed pin from `PIN_ANALOG_TEMP (A0)` to `PIN_DS18B20 (GPIO5)`

### 3. Wired Sensors Implementation
**Completely rewrote `wired_sensors.h` and `wired_sensors.cpp`:**
- Removed all analog/NTC thermistor code
- Added OneWire and DallasTemperature library support
- Changed `WiredSensorData` structure:
  - Removed: `analogTemp`
  - Added: `temperature` (standard field name)
- Sensor initialization now detects DS18B20 devices on 1-Wire bus
- Reading uses `DallasTemperature::requestTemperatures()` API

### 4. Main Application Updates
- Changed setup message to "Setup DS18B20 Temperature Sensor..."
- Updated MQTT payload to use `temperature_c` instead of `analog_temp_c`
- Serial output shows "DS18B20 Sensor: T=XX.XX°C"

## DS18B20 Features

### Specifications
- **Temperature range:** -55°C to +125°C
- **Accuracy:** ±0.5°C (-10°C to +85°C)
- **Resolution:** 9-12 bit configurable (default: 12-bit = 0.0625°C)
- **Protocol:** 1-Wire (only needs one data pin)
- **Multiple sensors:** Can connect multiple DS18B20s on same pin

### Advantages over Analog
1. **Pre-calibrated** - no calibration needed
2. **Digital** - immune to ADC noise and voltage issues
3. **No ADC limitations** - ESP8266 ADC 0-1V limit doesn't apply
4. **Unique 64-bit ID** - can identify individual sensors
5. **Long cables** - can work with cables up to 100m with proper setup

## MQTT Data Format

Published to: `paku/sensors/{device_id}_wired/data`

```json
{
  "timestamp": "2025-12-09T23:00:00Z",
  "device_id": "paku_wired",
  "location": "wired_sensor",
  "sensor_type": "DS18B20",
  "metrics": {
    "temperature_c": 22.5
  }
}
```

## Testing

1. **Build and upload:**
   ```bash
   cd paku_core
   platformio run -e esp8266-wired-sensors --target upload
   ```

2. **Expected serial output:**
   ```
   Setup DS18B20 Temperature Sensor...
   Initializing DS18B20 temperature sensor...
   Found 1 DS18B20 sensor(s)
   DS18B20 sensor initialized successfully
   Resolution: 12 bits
   Sensor type: DS18B20
   
   ...
   
   Reading DS18B20 sensor...
     Temperature: 22.50°C
   DS18B20 Sensor: T=22.50°C
   ```

3. **Verify MQTT:**
   - Topic: `paku/sensors/paku-XXXXXXXX_wired/data`
   - Should receive temperature updates every 10 seconds
   - Temperature should be accurate (±0.5°C)

## Troubleshooting

### No sensors detected
- **Check wiring** - ensure Data pin is on GPIO5 (D1)
- **Add pull-up resistor** - 4.7kΩ between Data and VCC is required
- **Check power** - DS18B20 needs 3.3V-5V
- **Test sensor** - try with a known-good DS18B20

### Reading shows -127°C or 85°C
- **-127°C** = Sensor disconnected or not responding
- **85°C** = Power-on reset value, sensor not initialized properly
- Check pull-up resistor and connections

### Intermittent readings
- **Add capacitor** - 100nF between VCC and GND close to sensor
- **Shorten wires** - long wires may need lower pull-up (2.2kΩ)
- **Check power supply** - ensure stable 3.3V/5V

### Multiple sensors
If you have multiple DS18B20 sensors on the same wire:
```cpp
// In wired_sensors.cpp, modify readSensors():
for (uint8_t i = 0; i < _sensorCount; i++) {
    float temp = _sensors->getTempCByIndex(i);
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(temp);
}
```

## Reverting to Analog
If you need to go back to analog sensors:
1. Restore old `wired_sensors.h` and `wired_sensors.cpp` from git
2. Remove OneWire and DallasTemperature libraries from `platformio.ini`
3. Change `HAS_DS18B20` back to `HAS_ANALOG_TEMP` in device_config
4. Change `PIN_DS18B20` back to `PIN_ANALOG_TEMP`

## GPIO5 Pin Conflicts
GPIO5 was previously suggested for I2C SCL but is now used for DS18B20. 
If you need I2C sensors later, use:
- ESP32: GPIO21 (SDA), GPIO22 (SCL)  
- ESP8266: Use software I2C on other pins

## Next Steps
- Sensor reads every 10 seconds (adjust `WIRED_SENSOR_INTERVAL_MS` in main.cpp)
- Change back to 60 seconds for production: `#define WIRED_SENSOR_INTERVAL_MS 60000`
- Add multiple DS18B20 sensors if needed (same pin, different addresses)
- Adjust resolution if faster readings needed (9-bit is faster but less precise)
