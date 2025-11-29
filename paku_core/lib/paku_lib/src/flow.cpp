/**
 * @file flow.cpp
 * @brief Flow calculation implementation
 */
#include "flow.h"

float calculateFrequency(unsigned int pulseCount, unsigned long intervalMs) {
    if (intervalMs == 0) {
        return 0.0f;
    }
    return pulseCount / (intervalMs / 1000.0f);
}

float calculateFlowRate(float frequency, float calibrationFactor) {
    if (calibrationFactor == 0.0f) {
        return 0.0f;
    }
    return (frequency / calibrationFactor) * 60.0f;
}

float calculateRequiredDeltaT(float heaterPower, float flowRate) {
    if (flowRate == 0.0f) {
        return 0.0f;
    }
    // Formula: deltaT = Power / (3.5 * flowRate)
    // where 3.5 is the specific heat capacity factor for water in kW/(L/min*°C)
    return heaterPower / (3.5f * flowRate);
}

FlowData calculateFlowData(unsigned int pulseCount, unsigned long intervalMs,
                            float calibrationFactor, float heaterPower) {
    FlowData data;
    
    data.frequency = calculateFrequency(pulseCount, intervalMs);
    data.flowRate = calculateFlowRate(data.frequency, calibrationFactor);
    data.requiredDeltaT = calculateRequiredDeltaT(heaterPower, data.flowRate);
    
    return data;
}
