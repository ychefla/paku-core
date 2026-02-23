# Analog Temperature Sensor Connection Guide for ESP8266

## Hardware Connection

### ESP8266 NodeMCU Pin Connections

Your 3-wire analog temperature sensor connects to the ESP8266 as follows:

```
ESP8266 NodeMCU          Temperature Sensor
===============          ==================
3.3V (3V3)       <--->   VCC (Red wire)
GND              <--->   GND (Black wire)
A0               <--->   Signal (Yellow/White wire)
```

**IMPORTANT NOTES:**
- The ESP8266 A0 pin can only read 0-1V maximum (NOT 3.3V!)
- If your sensor outputs 0-3.3V, you MUST use a voltage divider
- NodeMCU boards often have a built-in voltage divider allowing 0-3.3V input

### Pin Locations on NodeMCU

```
       NodeMCU v1.0
       
  RST  [ ]  [ ] A0    <- Analog input (only one!)
  ADC  [ ]  [ ] RSV
  EN   [ ]  [ ] RSV
  16   [ ]  [ ] GND
  14   [ ]  [ ] 27
  12   [ ]  [ ] 25
  13   [ ]  [ ] 32
  15   [ ]  [ ] TXD
  3V3  [ ]  [ ] RXD   <- 3.3V power
  GND  [ ]  [ ] 8     <- Ground
       [ ]  [ ] 7
```

## Sensor Types Supported

### 1. NTC Thermistor (Default Configuration)
**Most common for probe sensors with 3 wires**

Wiring with voltage divider:
```
3.3V ----[10kΩ]---- A0 ----[NTC 10kΩ]---- GND
```

Current configuration in code:
- Series resistor: 10kΩ
- NTC nominal resistance: 10kΩ at 25°C
- Beta coefficient: 3950K
- Temperature range: -40°C to 125°C

### 2. LM35 Linear Temperature Sensor
Outputs 10mV per degree Celsius (0°C = 0V, 100°C = 1V)

To use LM35, modify `wired_sensors.cpp`:
```cpp
#define ANALOG_TEMP_SERIES_R 0  // No voltage divider
// In readAnalogTemperature():
return voltage * 100.0;  // 10mV/°C = 100°C/V
```

### 3. TMP36 Temperature Sensor
Outputs voltage with offset: 500mV at 0°C, 10mV/°C scale

To use TMP36, modify `wired_sensors.cpp`:
```cpp
#define ANALOG_TEMP_SERIES_R 0  // No voltage divider
// In readAnalogTemperature():
return (voltage - 0.5) * 100.0;
```

## Calibration

### How to Identify Your Sensor Type

1. **Measure the resistance** (if thermistor):
   - Disconnect sensor from ESP8266
   - Use multimeter to measure resistance between Signal and GND
   - At room temperature (~25°C), typical NTC is 10kΩ
   - If infinite resistance → likely not a thermistor

2. **Measure the voltage** (if linear sensor):
   - Connect sensor to 3.3V and GND only
   - Measure voltage on Signal wire
   - LM35: ~0.25V at 25°C (250mV)
   - TMP36: ~0.75V at 25°C (750mV)

3. **Check sensor markings**:
   - Look for part number printed on sensor
   - Common markings: "LM35", "TMP36", "NTC", "103" (10kΩ)

### Calibration Constants

Edit `src/wired_sensors.cpp` to adjust these values:

```cpp
// For NTC Thermistor:
#define ANALOG_TEMP_SERIES_R     10000  // Series resistor (Ω)
#define ANALOG_TEMP_NTC_NOMINAL  10000  // NTC at 25°C (Ω)
#define ANALOG_TEMP_B_COEFFICIENT 3950  // Beta coefficient

// For direct voltage sensors:
#define ANALOG_TEMP_SERIES_R     0      // Set to 0
// Then modify readAnalogTemperature() function
```

### Voltage Divider for High-Voltage Sensors

If your sensor outputs 0-3.3V and you have a basic ESP8266 (not NodeMCU):

```
3.3V ----[Sensor]---- Signal ----[10kΩ]---- A0 ----[10kΩ]---- GND
```

This divides the voltage by 2, allowing 0-2V input on A0.

## Software Configuration

### Enable/Disable Analog Sensor

In `src/wired_sensors.cpp`:
```cpp
#define ANALOG_TEMP_ENABLED 1  // Set to 1 to enable, 0 to disable
```

### Adjust Sampling

```cpp
#define ANALOG_TEMP_SAMPLES 10  // Number of readings to average (reduces noise)
```

## Data Flow to Dashboard

Once connected and configured:

1. ESP8266 reads analog sensor every 60 seconds
2. Temperature is converted using Steinhart-Hart equation (NTC) or linear formula
3. Data is sent to MQTT broker with topic:
   ```
   paku/sensors/{device_id}_wired/data
   ```
4. Payload includes:
   ```json
   {
     "timestamp": "2024-01-01T12:00:00Z",
     "device_id": "esp8266_wired",
     "location": "wired_sensor",
     "metrics": {
       "analog_temp_c": 23.5,
       "temperature_c": 23.2,  // BME280 if connected
       "humidity_percent": 45.0,
       "pressure_hpa": 1013.25
     }
   }
   ```

5. Backend collector processes the data
6. Grafana dashboard displays `analog_temp_c` as "Analog Temperature"

## Testing

### 1. Monitor Serial Output

Connect to ESP8266 via USB and open serial monitor (115200 baud):

```
Initializing wired sensors...
Analog temperature sensor enabled on A0
Wired Sensor: Analog T=23.5°C
```

### 2. Test with Known Temperature

- Place sensor in ice water (should read ~0°C)
- Place sensor at room temperature (should read ~20-25°C)
- Compare with BME280 reading if available

### 3. Check MQTT Messages

Subscribe to topic:
```bash
mosquitto_sub -h YOUR_MQTT_BROKER -t "paku/sensors/+/data"
```

## Troubleshooting

### Reading shows NaN or invalid
- Check wiring connections
- Verify sensor type configuration
- Ensure A0 voltage is 0-1V (measure with multimeter)

### Temperature too high/low
- Adjust calibration constants
- For NTC: check beta coefficient and nominal resistance
- For linear sensors: verify voltage-to-temp formula

### Erratic readings
- Increase `ANALOG_TEMP_SAMPLES` (try 20)
- Add 100nF capacitor between A0 and GND
- Keep sensor wires short (<1 meter)
- Use shielded cable if possible

### No data in dashboard
- Check MQTT connection in serial monitor
- Verify topic name matches backend expectations
- Check Grafana query for `analog_temp_c` metric

## Example Sensor Connections

### Waterproof DS18B20-style probe (if NTC-based)
```
Red    -> 3.3V
Black  -> GND  
Yellow -> through 10kΩ resistor to A0
       -> NTC thermistor inside probe to GND
```

### Generic 3-wire temperature sensor module
```
VCC/+  -> 3.3V
GND/-  -> GND
OUT/S  -> A0 (check if needs voltage divider)
```

## Safety Notes

- **Never exceed 1V on ESP8266 A0 pin** (NodeMCU with voltage divider: 3.3V max)
- Double-check voltage with multimeter before connecting
- Use proper current-limiting resistors
- Avoid static discharge when handling ESP8266
- Don't connect sensors while ESP8266 is powered on

## Next Steps

1. Connect sensor as described above
2. Upload firmware to ESP8266
3. Monitor serial output to verify readings
4. Adjust calibration if needed
5. Check dashboard for new `analog_temp_c` metric
6. Create Grafana panel to visualize analog temperature

---

For questions or issues, check the serial output first. The ESP8266 will print detailed
information about sensor initialization and readings.
