/**
 * @file wired_sensors.h
 * @brief Wired sensor support for ESP8266 and ESP32 devices
 * 
 * Provides abstraction layer for analog temperature sensors.
 * 
 * This module is designed to work alongside BLE sensors on ESP32
 * or as the primary sensor interface on ESP8266 (which lacks BLE).
 */
#pragma once

#include <Arduino.h>
#include "device_config.h"

#if HAS_WIRED_SENSORS

/**
 * @brief Wired sensor data structure
 * 
 * Contains readings from analog temperature sensors
 */
struct WiredSensorData {
    float analogTemp;   // Temperature from analog sensor
    bool valid;         // True if readings are valid
    unsigned long timestamp; // millis() when reading was taken
};

/**
 * @brief Wired sensor manager class
 * 
 * Manages initialization and reading of analog temperature sensors
 */
class WiredSensors {
public:
    /**
     * @brief Constructor
     */
    WiredSensors();
    
    /**
     * @brief Initialize analog sensor
     * 
     * @param sda Unused (kept for API compatibility)
     * @param scl Unused (kept for API compatibility)
     * @return true if analog sensor is enabled
     */
    bool begin(int sda, int scl);
    
    /**
     * @brief Read data from sensors
     * 
     * Reads temperature from analog sensor
     * 
     * @return WiredSensorData structure with sensor readings
     */
    WiredSensorData readSensors();
    
    /**
     * @brief Check if sensors are available
     * 
     * @return true if analog sensor is enabled
     */
    bool isAvailable() const { return _hasAnalogTemp; }
    
    /**
     * @brief Get sensor type string
     * 
     * @return Sensor type (e.g., "Analog")
     */
    const char* getSensorType() const { return _sensorType; }
    
    /**
     * @brief Check if analog temperature sensor is available
     * 
     * @return true if analog sensor is enabled
     */
    bool hasAnalogSensor() const { return _hasAnalogTemp; }

private:
    bool _hasAnalogTemp;
    const char* _sensorType;
    
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
