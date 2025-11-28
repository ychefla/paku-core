/**
 * @file intervals.h
 * @brief Interval management for sensor and MQTT operations
 * 
 * This module provides functions for managing timing intervals
 * based on heater status and other conditions.
 */
#pragma once

/**
 * @brief Fast interval for MQTT messages (10 seconds in ms)
 */
#define MQTT_FAST_INTERVAL 10000

/**
 * @brief Slow interval for MQTT messages (1 hour in ms)
 */
#define MQTT_SLOW_INTERVAL 3600000

/**
 * @brief Fast interval for sensor readings (5 seconds in ms)
 */
#define SENSOR_FAST_INTERVAL 5000

/**
 * @brief Slow interval for sensor readings (1 minute in ms)
 */
#define SENSOR_SLOW_INTERVAL 60000

/**
 * @brief Duration after which heater intervals switch from fast to slow (1 hour in ms)
 */
#define HEATER_ON_DURATION_THRESHOLD 3600000

/**
 * @brief Structure to hold interval configuration
 */
struct IntervalConfig {
    unsigned long mqttInterval;
    unsigned long sensorInterval;
};

/**
 * @brief Calculates intervals based on heater status and time
 * 
 * @param heaterStatus Current heater status (1 = on, 0 = off)
 * @param currentTime Current time in milliseconds
 * @param heaterOnStartTime Time when heater was turned on (updated if needed)
 * @return IntervalConfig The calculated interval configuration
 */
IntervalConfig calculateIntervals(int heaterStatus, unsigned long currentTime, 
                                   unsigned long& heaterOnStartTime);
