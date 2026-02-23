/**
 * @file flow_sensor.h
 * @brief Flow sensor measurement module (FUTURE DEVELOPMENT)
 * 
 * This module handles coolant flow measurement for the Webasto heater system.
 * Currently disabled pending hardware integration and calibration.
 * 
 * Hardware: Hall effect flow sensor on GPIO4
 * Measurement: Pulse counting with interrupt-driven edge detection
 * Output: Flow rate (L/min), frequency (Hz), required delta-T (°C)
 * 
 * @note This is a placeholder for future development
 * @note Hardware calibration required before production use
 */

#ifndef FLOW_SENSOR_H
#define FLOW_SENSOR_H

#include <Arduino.h>

// ============================================================================
// Flow Sensor Configuration
// ============================================================================

#define PIN_FLOW_SENSOR 4  // GPIO4 (moved from GPIO2 to avoid LED conflict)

// ============================================================================
// Flow Sensor Variables
// ============================================================================

// Pulse counter (updated by ISR)
extern volatile unsigned int flowPulseCount;

// Calculated values
extern float flowRate;           // Flow rate in L/min
extern float calibrationFactor;  // Pulses per liter
extern float requiredDeltaT;     // Required temperature delta for heater
extern const float heaterPower;  // Heater power in watts

// Mode flags
extern bool flowTestMode;        // Enable test mode with simulated data
extern bool flowSensorEnabled;   // Enable/disable flow sensor

// ============================================================================
// Function Declarations
// ============================================================================

/**
 * @brief Initialize flow sensor hardware
 * 
 * Sets up GPIO pin and attaches interrupt for pulse counting.
 * Must be called during system setup.
 */
void initFlowSensor();

/**
 * @brief Interrupt Service Routine for pulse counting
 * 
 * Increments pulse counter on each rising edge.
 * Called by hardware interrupt.
 */
void IRAM_ATTR countFlowPulses();

/**
 * @brief Process flow sensor data
 * 
 * Calculates flow rate and required delta-T from pulse count.
 * Should be called periodically (e.g., every 5 seconds).
 * 
 * @param intervalMs Time interval since last measurement (milliseconds)
 * @return Calculated flow rate in L/min
 */
float processFlowData(unsigned long intervalMs);

/**
 * @brief Get required heater delta-T for current flow rate
 * 
 * Calculates the temperature delta required to maintain heater efficiency.
 * Formula: ΔT = Power / (3.5 × Flow)
 * 
 * @return Required delta-T in degrees Celsius
 */
float getRequiredDeltaT();

/**
 * @brief Reset flow sensor counters
 * 
 * Resets pulse counter and clears measurements.
 * Useful for starting fresh measurement cycles.
 */
void resetFlowSensor();

/**
 * @brief Deinitialize flow sensor
 * 
 * Detaches interrupt and releases GPIO pin.
 * Call when disabling flow measurement.
 */
void deinitFlowSensor();

#endif // FLOW_SENSOR_H
