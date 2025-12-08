/**
 * @file wired_sensors.cpp
 * @brief Implementation of wired sensor support
 */

#include "wired_sensors.h"

#if HAS_WIRED_SENSORS

WiredSensors::WiredSensors() 
    : _initialized(false), _sensorType("None") {
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
        return true;
    }
    
    Serial.println("Warning: No wired sensors detected");
    return false;
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
    data.timestamp = millis();
    
    if (!_initialized) {
        Serial.println("Warning: Sensors not initialized");
        return data;
    }
    
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
        Serial.println("Warning: Invalid sensor readings detected");
    }
    
    return data;
}

#endif // HAS_WIRED_SENSORS
