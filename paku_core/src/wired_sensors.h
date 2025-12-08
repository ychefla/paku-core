/**
 * @file wired_sensors.h
 * @brief Wired sensor support for ESP8266 and ESP32 devices
 * 
 * Provides abstraction layer for wired I2C sensors such as:
 * - BME280: Temperature, humidity, and pressure sensor
 * - BMP280: Temperature and pressure sensor
 * 
 * This module is designed to work alongside BLE sensors on ESP32
 * or as the primary sensor interface on ESP8266 (which lacks BLE).
 */
#pragma once

#include <Arduino.h>
#include "device_config.h"

#if HAS_WIRED_SENSORS

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

/**
 * @brief Wired sensor data structure
 * 
 * Contains readings from a BME280 or similar sensor
 */
struct WiredSensorData {
    float temperature;  // Temperature in Celsius (from BME280)
    float humidity;     // Relative humidity in %
    float pressure;     // Atmospheric pressure in hPa
    float analogTemp;   // Temperature from analog sensor (if available)
    bool valid;         // True if readings are valid
    bool hasAnalogTemp; // True if analog temperature is available
    unsigned long timestamp; // millis() when reading was taken
};

/**
 * @brief Wired sensor manager class
 * 
 * Manages initialization and reading of wired I2C sensors
 */
class WiredSensors {
public:
    /**
     * @brief Constructor
     */
    WiredSensors();
    
    /**
     * @brief Initialize wired sensors
     * 
     * Initializes I2C bus and detects connected sensors
     * 
     * @param sda I2C SDA pin (use PIN_I2C_SDA from device_config.h)
     * @param scl I2C SCL pin (use PIN_I2C_SCL from device_config.h)
     * @return true if at least one sensor was initialized successfully
     */
    bool begin(int sda, int scl);
    
    /**
     * @brief Read data from sensors
     * 
     * Reads temperature, humidity, and pressure from BME280 sensor
     * 
     * @return WiredSensorData structure with sensor readings
     */
    WiredSensorData readSensors();
    
    /**
     * @brief Check if sensors are available
     * 
     * @return true if sensors are initialized and working
     */
    bool isAvailable() const { return _initialized; }
    
    /**
     * @brief Get sensor type string
     * 
     * @return Sensor type (e.g., "BME280", "BMP280", or "None")
     */
    const char* getSensorType() const { return _sensorType; }
    
    /**
     * @brief Check if analog temperature sensor is available
     * 
     * @return true if analog sensor is enabled
     */
    bool hasAnalogSensor() const { return _hasAnalogTemp; }

private:
    Adafruit_BME280 _bme;
    bool _initialized;
    bool _hasAnalogTemp;
    const char* _sensorType;
    
    /**
     * @brief Detect and initialize BME280 sensor
     * 
     * @return true if BME280 was found and initialized
     */
    bool initBME280();
    
    /**
     * @brief Read analog temperature sensor
     * 
     * Reads voltage from A0 and converts to temperature.
     * Default calibration assumes NTC thermistor with voltage divider.
     * Adjust ANALOG_TEMP_* constants for your specific sensor.
     * 
     * @return Temperature in Celsius, or NaN if invalid
     */
    float readAnalogTemperature();
};

#endif // HAS_WIRED_SENSORS
