/**
 * @file sensor_placeholders.cpp
 * @brief Placeholder data generation implementation
 */
#include "sensor_placeholders.h"
#include <cstring>
#include <cmath>

// Static seed for pseudo-random generation
static uint32_t prngState = 12345;

uint32_t simpleHash(const char* str) {
    if (str == nullptr) {
        return 0;
    }
    
    uint32_t hash = 5381;
    int c;
    while ((c = *str++) != 0) {
        hash = ((hash << 5) + hash) + static_cast<uint32_t>(c);
    }
    return hash;
}

float pseudoRandom(uint32_t seed, float min, float max) {
    // Linear congruential generator
    prngState = (prngState * 1103515245 + seed + 12345) & 0x7FFFFFFF;
    float normalized = static_cast<float>(prngState) / static_cast<float>(0x7FFFFFFF);
    return min + normalized * (max - min);
}

float generatePlaceholderTemperature(const char* location, bool simulateReal) {
    if (!simulateReal) {
        return SENSOR_NOT_AVAILABLE;
    }
    
    uint32_t hash = simpleHash(location);
    
    // Generate temperature based on location hash
    // Indoor temperatures typically 15-25°C
    float baseTemp = 18.0f + (hash % 7);  // Base varies by location
    float variation = pseudoRandom(hash, -2.0f, 2.0f);
    
    return baseTemp + variation;
}

float generatePlaceholderHumidity(const char* location, bool simulateReal) {
    if (!simulateReal) {
        return SENSOR_NOT_AVAILABLE;
    }
    
    uint32_t hash = simpleHash(location);
    
    // Indoor humidity typically 30-70%
    float baseHumidity = 40.0f + (hash % 20);
    float variation = pseudoRandom(hash + 1, -5.0f, 5.0f);
    
    float result = baseHumidity + variation;
    // Clamp to valid range
    if (result < 0.0f) result = 0.0f;
    if (result > 100.0f) result = 100.0f;
    
    return result;
}

float generatePlaceholderPressure(bool simulateReal) {
    if (!simulateReal) {
        return SENSOR_NOT_AVAILABLE;
    }
    
    // Atmospheric pressure typically 980-1040 hPa
    float variation = pseudoRandom(0, -20.0f, 20.0f);
    return 1013.25f + variation;
}

float generatePlaceholderVoltage(const char* source, bool simulateReal) {
    if (!simulateReal) {
        return SENSOR_NOT_AVAILABLE;
    }
    
    uint32_t hash = simpleHash(source);
    
    // Battery voltages
    float baseVoltage;
    if (source != nullptr && strstr(source, "car") != nullptr) {
        // Car battery: 11.5-14.4V
        baseVoltage = 12.6f;
    } else if (source != nullptr && strstr(source, "leisure") != nullptr) {
        // Leisure battery: 11.0-14.4V
        baseVoltage = 12.4f;
    } else {
        // Generic battery
        baseVoltage = 12.0f;
    }
    
    float variation = pseudoRandom(hash, -0.5f, 0.5f);
    return baseVoltage + variation;
}

PlaceholderSensorData generatePlaceholderReading(const char* location, bool simulateReal) {
    PlaceholderSensorData data;
    
    data.temperature = generatePlaceholderTemperature(location, simulateReal);
    data.humidity = generatePlaceholderHumidity(location, simulateReal);
    data.pressure = generatePlaceholderPressure(simulateReal);
    data.voltage = generatePlaceholderVoltage(location, simulateReal);
    data.status = simulateReal ? 1 : 0;
    data.isPlaceholder = true;
    
    return data;
}

bool isPlaceholderValue(float value) {
    // Check if value is the placeholder sentinel
    // Use epsilon comparison for floating point
    return fabs(value - SENSOR_NOT_AVAILABLE) < 0.01f;
}
