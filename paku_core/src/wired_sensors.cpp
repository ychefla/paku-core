/**
 * @file wired_sensors.cpp
 * @brief Implementation of wired sensor support
 */

#include "wired_sensors.h"

#if HAS_WIRED_SENSORS

// Analog temperature sensor calibration constants
// These are for a typical NTC thermistor with 10K resistance at 25°C
// Adjust these values based on your specific sensor and voltage divider circuit
#define ANALOG_TEMP_ENABLED      1      // Set to 1 to enable analog temp sensor
#define ANALOG_TEMP_SAMPLES      10     // Number of samples to average
#define ANALOG_TEMP_REF_VOLTAGE  1.0    // ESP8266 ADC reference voltage (1.0V max)
#define ANALOG_TEMP_SERIES_R     10000  // Series resistor value in voltage divider (ohms)
#define ANALOG_TEMP_NTC_NOMINAL  10000  // NTC resistance at 25°C (ohms)
#define ANALOG_TEMP_TEMP_NOMINAL 25.0   // Temperature for nominal resistance (°C)
#define ANALOG_TEMP_B_COEFFICIENT 3950  // Beta coefficient of the NTC thermistor
// Voltage divider: VCC --- [SERIES_R] --- A0 --- [NTC] --- GND
// For a sensor that outputs 0-1V directly, set SERIES_R to 0

WiredSensors::WiredSensors() 
    : _initialized(false), _hasAnalogTemp(false), _sensorType("None") {
}

bool WiredSensors::begin(int sda, int scl) {
    Serial.println("Initializing wired sensors...");
    
    // Initialize I2C bus
    Wire.begin(sda, scl);
    
    // Try to initialize BME280
    if (initBME280()) {
        _initialized = true;
        _sensorType = "BME280";
        Serial.println("BME280 sensor initialized successfully");
    } else {
        Serial.println("Warning: No I2C sensors detected");
    }
    
    // Check for analog temperature sensor
#if defined(HAS_ANALOG_TEMP) && HAS_ANALOG_TEMP && ANALOG_TEMP_ENABLED
    _hasAnalogTemp = true;
    Serial.println("Analog temperature sensor enabled on A0");
#endif
    
    return _initialized || _hasAnalogTemp;
}

bool WiredSensors::initBME280() {
    // Try default I2C address 0x76
    if (_bme.begin(0x76)) {
        // Configure BME280 settings for weather monitoring
        _bme.setSampling(
            Adafruit_BME280::MODE_FORCED,     // Take reading on demand
            Adafruit_BME280::SAMPLING_X1,     // Temperature oversampling
            Adafruit_BME280::SAMPLING_X1,     // Pressure oversampling
            Adafruit_BME280::SAMPLING_X1,     // Humidity oversampling
            Adafruit_BME280::FILTER_OFF       // Filter off for responsive readings
        );
        return true;
    }
    
    // Try alternate I2C address 0x77
    if (_bme.begin(0x77)) {
        _bme.setSampling(
            Adafruit_BME280::MODE_FORCED,
            Adafruit_BME280::SAMPLING_X1,
            Adafruit_BME280::SAMPLING_X1,
            Adafruit_BME280::SAMPLING_X1,
            Adafruit_BME280::FILTER_OFF
        );
        return true;
    }
    
    return false;
}

WiredSensorData WiredSensors::readSensors() {
    WiredSensorData data;
    data.valid = false;
    data.hasAnalogTemp = false;
    data.timestamp = millis();
    
    // Read I2C sensor (BME280)
    if (_initialized) {
        // Force BME280 to take a reading
        _bme.takeForcedMeasurement();
        
        // Wait for measurement to complete (BME280 typically takes ~8ms in forced mode)
        delay(10);
        
        // Read sensor values
        data.temperature = _bme.readTemperature();
        data.pressure = _bme.readPressure() / 100.0F; // Convert Pa to hPa
        data.humidity = _bme.readHumidity();
        
        // Validate readings (BME280 returns reasonable values)
        // Temperature: -40 to 85°C, Humidity: 0-100%, Pressure: 300-1100 hPa
        if (data.temperature >= -40.0 && data.temperature <= 85.0 &&
            data.humidity >= 0.0 && data.humidity <= 100.0 &&
            data.pressure >= 300.0 && data.pressure <= 1100.0) {
            data.valid = true;
        } else {
            Serial.println("Warning: Invalid BME280 readings detected");
        }
    }
    
    // Read analog temperature sensor
    if (_hasAnalogTemp) {
        data.analogTemp = readAnalogTemperature();
        if (!isnan(data.analogTemp) && data.analogTemp >= -40.0 && data.analogTemp <= 125.0) {
            data.hasAnalogTemp = true;
            data.valid = true; // Mark as valid if analog sensor provides good data
        } else {
            Serial.println("Warning: Invalid analog temperature reading");
        }
    }
    
    return data;
}

float WiredSensors::readAnalogTemperature() {
#if defined(HAS_ANALOG_TEMP) && HAS_ANALOG_TEMP && ANALOG_TEMP_ENABLED
    // Read multiple samples and average to reduce noise
    uint32_t analogSum = 0;
    for (int i = 0; i < ANALOG_TEMP_SAMPLES; i++) {
        analogSum += analogRead(PIN_ANALOG_TEMP);
        delay(10);
    }
    float analogValue = (float)analogSum / ANALOG_TEMP_SAMPLES;
    
    // ESP8266 ADC is 10-bit (0-1023) for 0-1V input
    float voltage = (analogValue / 1023.0) * ANALOG_TEMP_REF_VOLTAGE;
    
    // If sensor outputs voltage directly (no voltage divider), use linear conversion
    // Common sensors: LM35 (10mV/°C), TMP36 (varies), etc.
    // Uncomment and adjust for your specific sensor:
    // For LM35: temperature = voltage * 100.0;  // 10mV/°C = 100°C/V
    // For TMP36: temperature = (voltage - 0.5) * 100.0;
    
    // For NTC thermistor with voltage divider:
    if (ANALOG_TEMP_SERIES_R > 0) {
        // Calculate NTC resistance from voltage divider
        // V_out = V_in * (R_ntc / (R_series + R_ntc))
        // R_ntc = (V_out * R_series) / (V_in - V_out)
        float ntcResistance = (voltage * ANALOG_TEMP_SERIES_R) / (ANALOG_TEMP_REF_VOLTAGE - voltage);
        
        // Steinhart-Hart equation (simplified beta formula)
        // 1/T = 1/T0 + (1/B) * ln(R/R0)
        float steinhart;
        steinhart = ntcResistance / ANALOG_TEMP_NTC_NOMINAL;     // (R/R0)
        steinhart = log(steinhart);                               // ln(R/R0)
        steinhart /= ANALOG_TEMP_B_COEFFICIENT;                   // 1/B * ln(R/R0)
        steinhart += 1.0 / (ANALOG_TEMP_TEMP_NOMINAL + 273.15);  // + (1/T0)
        steinhart = 1.0 / steinhart;                              // Invert
        steinhart -= 273.15;                                      // Convert to Celsius
        
        return steinhart;
    } else {
        // Direct voltage-to-temperature conversion (adjust for your sensor)
        // Default: LM35-style (10mV/°C)
        return voltage * 100.0;
    }
#else
    return NAN;
#endif
}

#endif // HAS_WIRED_SENSORS
