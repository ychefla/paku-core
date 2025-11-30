/**
 * @file sensor_placeholders.h
 * @brief Placeholder data generation for future sensor types
 * 
 * This module provides placeholder data generation for sensors
 * that are planned but not yet implemented. This allows the data
 * pipeline to be tested and validated before hardware is available.
 */
#pragma once

#include <cstdint>

/**
 * @brief Placeholder value indicating sensor not available
 */
#define SENSOR_NOT_AVAILABLE -1000.0f

/**
 * @brief Structure for placeholder sensor readings
 */
struct PlaceholderSensorData {
    float temperature;          // Temperature in Celsius
    float humidity;             // Humidity in %
    float pressure;             // Pressure in hPa
    float voltage;              // Voltage in V
    int status;                 // Status (0=off, 1=on)
    bool isPlaceholder;         // True if this is generated placeholder data
};

/**
 * @brief Generates placeholder temperature data
 * 
 * @param location Location identifier (affects base value for variation)
 * @param simulateReal If true, generates realistic random values
 * @return Temperature value or SENSOR_NOT_AVAILABLE
 */
float generatePlaceholderTemperature(const char* location, bool simulateReal);

/**
 * @brief Generates placeholder humidity data
 * 
 * @param location Location identifier
 * @param simulateReal If true, generates realistic random values
 * @return Humidity value or SENSOR_NOT_AVAILABLE
 */
float generatePlaceholderHumidity(const char* location, bool simulateReal);

/**
 * @brief Generates placeholder pressure data
 * 
 * @param simulateReal If true, generates realistic random values
 * @return Pressure value in hPa or SENSOR_NOT_AVAILABLE
 */
float generatePlaceholderPressure(bool simulateReal);

/**
 * @brief Generates placeholder voltage data
 * 
 * @param source Source identifier (e.g., "car", "leisure")
 * @param simulateReal If true, generates realistic random values
 * @return Voltage value or SENSOR_NOT_AVAILABLE
 */
float generatePlaceholderVoltage(const char* source, bool simulateReal);

/**
 * @brief Generates a complete placeholder sensor reading
 * 
 * @param location Location identifier
 * @param simulateReal If true, generates realistic random values
 * @return PlaceholderSensorData with generated values
 */
PlaceholderSensorData generatePlaceholderReading(const char* location, bool simulateReal);

/**
 * @brief Checks if a value is a placeholder (not available)
 * 
 * @param value Sensor value to check
 * @return true if value indicates sensor not available
 */
bool isPlaceholderValue(float value);

/**
 * @brief Simple pseudo-random number generator for placeholder values
 * 
 * Uses a linear congruential generator seeded with location hash.
 * This is deterministic for testing but varies by location.
 * 
 * @param seed Seed value (use location hash)
 * @param min Minimum value
 * @param max Maximum value
 * @return Random value between min and max
 */
float pseudoRandom(uint32_t seed, float min, float max);

/**
 * @brief Simple hash function for location strings
 * 
 * @param str String to hash
 * @return Hash value
 */
uint32_t simpleHash(const char* str);
