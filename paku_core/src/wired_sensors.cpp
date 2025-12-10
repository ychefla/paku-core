/**
 * @file wired_sensors.cpp
 * @brief Implementation of wired sensor support (DS18B20 digital)
 */

#include "wired_sensors.h"

#if HAS_WIRED_SENSORS

WiredSensors::WiredSensors() 
    : _oneWire(nullptr), _sensors(nullptr), _initialized(false), 
      _sensorType("None"), _sensorCount(0) {
}

bool WiredSensors::begin(int sda, int scl) {
    Serial.println("Initializing DS18B20 temperature sensor...");
    
#if defined(HAS_DS18B20) && HAS_DS18B20
    // Initialize 1-Wire bus on PIN_DS18B20
    _oneWire = new OneWire(PIN_DS18B20);
    _sensors = new DallasTemperature(_oneWire);
    
    // Start up the library
    _sensors->begin();
    
    // Count devices
    _sensorCount = _sensors->getDeviceCount();
    
    Serial.print("Found ");
    Serial.print(_sensorCount);
    Serial.println(" DS18B20 sensor(s)");
    
    if (_sensorCount > 0) {
        _initialized = true;
        _sensorType = "DS18B20";
        
        // Set resolution to 12-bit (0.0625°C precision)
        _sensors->setResolution(12);
        
        Serial.println("DS18B20 sensor initialized successfully");
        Serial.print("Resolution: ");
        Serial.print(_sensors->getResolution());
        Serial.println(" bits");
        
        return true;
    } else {
        Serial.println("Warning: No DS18B20 sensors detected");
        delete _sensors;
        delete _oneWire;
        _sensors = nullptr;
        _oneWire = nullptr;
        return false;
    }
#else
    Serial.println("Warning: DS18B20 support not enabled");
    return false;
#endif
}

WiredSensorData WiredSensors::readSensors() {
    WiredSensorData data;
    data.valid = false;
    data.timestamp = millis();
    
    if (!_initialized || !_sensors) {
        Serial.println("DS18B20 sensor not available");
        return data;
    }
    
    Serial.println("Reading DS18B20 sensor...");
    
    // Request temperature reading
    _sensors->requestTemperatures();
    
    // Read temperature (sensor index 0 for first sensor)
    float temp = _sensors->getTempCByIndex(0);
    
    Serial.print("  Temperature: ");
    Serial.print(temp, 2);
    Serial.println("°C");
    
    // Validate reading
    // DS18B20 returns -127°C or 85°C on error
    if (temp != DEVICE_DISCONNECTED_C && temp != 85.0 && 
        temp >= -55.0 && temp <= 125.0) {
        data.temperature = temp;
        data.valid = true;
    } else {
        Serial.println("Warning: Invalid DS18B20 reading (sensor disconnected or error)");
    }
    
    return data;
}

#endif // HAS_WIRED_SENSORS
