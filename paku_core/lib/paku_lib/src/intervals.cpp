/**
 * @file intervals.cpp
 * @brief Interval management implementation
 */
#include "intervals.h"

IntervalConfig calculateIntervals(int heaterStatus, unsigned long currentTime, 
                                   unsigned long& heaterOnStartTime) {
    IntervalConfig config;
    
    if (heaterStatus == 1) {
        // Heater is on
        if (heaterOnStartTime == 0) {
            heaterOnStartTime = currentTime;
        }
        
        if (currentTime - heaterOnStartTime >= HEATER_ON_DURATION_THRESHOLD) {
            // Heater has been on for more than threshold, use slow intervals
            config.mqttInterval = MQTT_SLOW_INTERVAL;
            config.sensorInterval = SENSOR_SLOW_INTERVAL;
        } else {
            // Heater recently turned on, use fast intervals
            config.mqttInterval = MQTT_FAST_INTERVAL;
            config.sensorInterval = SENSOR_FAST_INTERVAL;
        }
    } else {
        // Heater is off, reset start time and use slow intervals
        heaterOnStartTime = 0;
        config.mqttInterval = MQTT_SLOW_INTERVAL;
        config.sensorInterval = SENSOR_SLOW_INTERVAL;
    }
    
    return config;
}
