/**
 * @file flow_sensor.cpp
 * @brief Flow sensor measurement implementation (FUTURE DEVELOPMENT)
 * 
 * This module provides coolant flow measurement functionality for the
 * Webasto diesel heater system. It uses a Hall effect flow sensor with
 * pulse counting to calculate flow rate and required temperature delta.
 * 
 * @note Currently disabled - awaiting hardware integration and calibration
 * @note Requires production calibration factor for accurate measurements
 */

#include "flow_sensor.h"

// ============================================================================
// Global Variables
// ============================================================================

volatile unsigned int flowPulseCount = 0;
float flowRate = 0.0;
float calibrationFactor = 6.6;  // Pulses per liter (requires calibration)
float requiredDeltaT = 0.0;
const float heaterPower = 5000.0;  // 5kW heater

bool flowTestMode = true;       // Enable simulated data by default
bool flowSensorEnabled = false; // Disabled by default

// ============================================================================
// Private Variables
// ============================================================================

static unsigned long lastMeasurementTime = 0;

// ============================================================================
// Function Implementations
// ============================================================================

void initFlowSensor() {
    pinMode(PIN_FLOW_SENSOR, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_FLOW_SENSOR), countFlowPulses, RISING);
    flowPulseCount = 0;
    lastMeasurementTime = millis();
    
    Serial.println("[FLOW] Flow sensor initialized on GPIO4");
    Serial.printf("[FLOW] Calibration factor: %.2f pulses/L\n", calibrationFactor);
    Serial.printf("[FLOW] Test mode: %s\n", flowTestMode ? "enabled" : "disabled");
}

void IRAM_ATTR countFlowPulses() {
    flowPulseCount++;
}

float processFlowData(unsigned long intervalMs) {
    // Detach interrupt during processing
    detachInterrupt(digitalPinToInterrupt(PIN_FLOW_SENSOR));
    
    // Simulate data in test mode
    if (flowTestMode) {
        flowPulseCount = random(198, 462);  // Simulates ~0.5-1.5 L/min
    }
    
    // Calculate frequency in Hz (pulses per second)
    float intervalSeconds = intervalMs / 1000.0;
    float frequency = flowPulseCount / intervalSeconds;
    
    // Calculate flow rate in L/min
    // Formula: Flow (L/min) = (Frequency / CalibrationFactor) × 60
    flowRate = (frequency / calibrationFactor) * 60.0;
    
    // Calculate required delta-T for heater efficiency
    // Formula: ΔT (°C) = Power (W) / (3.5 × Flow (L/min))
    if (flowRate > 0.001) {  // Avoid division by zero
        requiredDeltaT = heaterPower / (3.5 * flowRate);
    } else {
        requiredDeltaT = 0.0;
    }
    
    // Reset counter for next measurement
    flowPulseCount = 0;
    lastMeasurementTime = millis();
    
    // Reattach interrupt
    attachInterrupt(digitalPinToInterrupt(PIN_FLOW_SENSOR), countFlowPulses, RISING);
    
    Serial.printf("[FLOW] Frequency: %.2f Hz, Flow: %.2f L/min, ΔT: %.2f°C\n", 
                  frequency, flowRate, requiredDeltaT);
    
    return flowRate;
}

float getRequiredDeltaT() {
    return requiredDeltaT;
}

void resetFlowSensor() {
    detachInterrupt(digitalPinToInterrupt(PIN_FLOW_SENSOR));
    flowPulseCount = 0;
    flowRate = 0.0;
    requiredDeltaT = 0.0;
    attachInterrupt(digitalPinToInterrupt(PIN_FLOW_SENSOR), countFlowPulses, RISING);
    
    Serial.println("[FLOW] Flow sensor counters reset");
}

void deinitFlowSensor() {
    detachInterrupt(digitalPinToInterrupt(PIN_FLOW_SENSOR));
    pinMode(PIN_FLOW_SENSOR, INPUT);  // Release pin to high-impedance
    flowPulseCount = 0;
    flowRate = 0.0;
    requiredDeltaT = 0.0;
    
    Serial.println("[FLOW] Flow sensor deinitialized");
}
