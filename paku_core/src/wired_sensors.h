/**
 * @file wired_sensors.h
 * @brief Wired sensor support for ESP8266 and ESP32 devices
 * 
 * Provides abstraction layer for DS18B20 1-Wire digital temperature sensors.
 * 
 * This module is designed to work alongside BLE sensors on ESP32
 * or as the primary sensor interface on ESP8266 (which lacks BLE).
 */
#pragma once

#include <Arduino.h>
#include "device_config.h"

#if HAS_WIRED_SENSORS

#include <OneWire.h>
#include <DallasTemperature.h>

/**
 * @brief Wired sensor data structure
 * 
 * Contains readings from DS18B20 temperature sensor
 */
struct WiredSensorData {
    float temperature;  // Temperature in Celsius from DS18B20
    bool valid;         // True if readings are valid
    unsigned long timestamp; // millis() when reading was taken
};

/**
 * @brief Wired sensor manager class
 * 
 * Manages initialization and reading of DS18B20 1-Wire temperature sensors
 */
class WiredSensors {
public:
    /**
     * @brief Constructor
     */
    WiredSensors();
    
    /**
     * @brief Initialize DS18B20 sensor
     * 
     * @param sda Unused (kept for API compatibility)
     * @param scl Unused (kept for API compatibility)
     * @return true if DS18B20 sensor was found
     */
    bool begin(int sda, int scl);
    
    /**
     * @brief Read data from sensors (blocking, ~750ms)
     * 
     * Reads temperature from DS18B20 sensor. Blocks while waiting
     * for conversion to complete.
     * 
     * @return WiredSensorData structure with sensor readings
     */
    WiredSensorData readSensors();

    /**
     * @brief Request a temperature conversion (non-blocking)
     * 
     * Starts an async temperature conversion. Call isConversionReady()
     * to poll for completion, then readConversion() to get the result.
     * 
     * @return true if request was issued successfully
     */
    bool requestConversion();

    /**
     * @brief Check if the async conversion has completed
     * 
     * @return true if conversion is done and readConversion() can be called
     */
    bool isConversionReady();

    /**
     * @brief Read the result of a previously requested async conversion
     * 
     * Must only be called after isConversionReady() returns true.
     * 
     * @return WiredSensorData with the reading
     */
    WiredSensorData readConversion();
    
    /**
     * @brief Check if sensors are available
     * 
     * @return true if DS18B20 sensor is detected
     */
    bool isAvailable() const { return _initialized; }
    
    /**
     * @brief Get sensor type string
     * 
     * @return Sensor type ("DS18B20")
     */
    const char* getSensorType() const { return _sensorType; }

private:
    OneWire* _oneWire;
    DallasTemperature* _sensors;
    bool _initialized;
    const char* _sensorType;
    uint8_t _sensorCount;
    bool _conversionPending;    ///< True while an async conversion is in progress
};

#endif // HAS_WIRED_SENSORS
