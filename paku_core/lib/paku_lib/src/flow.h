/**
 * @file flow.h
 * @brief Flow calculation and data processing
 * 
 * This module provides functions for calculating flow rates
 * and related heating parameters.
 */
#pragma once

/**
 * @brief Default heater power in watts
 */
#define DEFAULT_HEATER_POWER 5000.0f

/**
 * @brief Default calibration factor for flow sensor
 */
#define DEFAULT_CALIBRATION_FACTOR 6.6f

/**
 * @brief Structure to hold flow calculation results
 */
struct FlowData {
    float frequency;      // Pulses per second (Hz)
    float flowRate;       // Flow rate in L/min
    float requiredDeltaT; // Required temperature delta in Celsius
};

/**
 * @brief Calculates flow data from pulse count and time interval
 * 
 * @param pulseCount Number of pulses counted
 * @param intervalMs Time interval in milliseconds
 * @param calibrationFactor Flow sensor calibration factor
 * @param heaterPower Heater power in watts
 * @return FlowData Calculated flow data
 */
FlowData calculateFlowData(unsigned int pulseCount, unsigned long intervalMs,
                            float calibrationFactor, float heaterPower);

/**
 * @brief Calculates frequency from pulse count and time interval
 * 
 * @param pulseCount Number of pulses counted
 * @param intervalMs Time interval in milliseconds
 * @return float Frequency in Hz
 */
float calculateFrequency(unsigned int pulseCount, unsigned long intervalMs);

/**
 * @brief Calculates flow rate from frequency
 * 
 * @param frequency Frequency in Hz
 * @param calibrationFactor Flow sensor calibration factor
 * @return float Flow rate in L/min
 */
float calculateFlowRate(float frequency, float calibrationFactor);

/**
 * @brief Calculates required temperature delta for heating
 * 
 * @param heaterPower Heater power in watts
 * @param flowRate Flow rate in L/min
 * @return float Required temperature delta in Celsius
 */
float calculateRequiredDeltaT(float heaterPower, float flowRate);
