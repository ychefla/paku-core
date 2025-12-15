/**
 * @file moko.cpp
 * @brief MoKo sensor message parsing implementation
 * 
 * BeaconX Pro "Temperature and Humidity" Frame Format:
 * 
 * After manufacturer ID (4C 00) is removed by caller:
 * 
 * Type 0x10 (Temperature and Humidity):
 *   Byte 0: Frame type (0x10)
 *   Byte 1: Data length (0x06 = 6 bytes following)
 *   Byte 2-3: Temperature (signed 16-bit big-endian, 0.01°C resolution)
 *   Byte 4-5: Humidity (unsigned 16-bit big-endian, 0.01% resolution)
 *   Byte 6-7: Optional battery/additional data
 * 
 * Example frame: 4C 00 10 06 0D 1E 1E 7C = 22.22°C, 45.0%
 *   - 0x0D1E = 3358 * 0.01 = 33.58°C
 *   - 0x1E7C = 7804 * 0.01 = 78.04%
 * 
 * Type 0x16 (Service Data - alternative format):
 *   Similar structure but with service UUID prefix
 * 
 * Device Info frames (0x10 0x02) and beacon frames (0x12) are ignored.
 */

#include "moko.h"

bool isValidMoKoData(const uint8_t* data, size_t length) {
    if (data == nullptr || length < 4) {
        return false;
    }
    
    // Check for valid BeaconX Pro frame types
    uint8_t frameType = data[0];
    uint8_t frameLength = data[1];
    
    // Type 0x10 with length 0x06 = Temperature and Humidity frame
    // Format: 10 06 [temp LSB] [temp MSB] [humid LSB] [humid MSB]
    if (frameType == 0x10 && frameLength == 0x06 && length >= 6) {
        return true;
    }
    
    // Type 0x10 with length 0x02 = Device info frame (battery, etc.)
    // Format: 10 02 [data1] [data2]
    if (frameType == 0x10 && frameLength == 0x02 && length >= 4) {
        return true;
    }
    
    // Type 0x09 = TLM or other extended data (may contain sensor data)
    // Accept if we have enough data
    if (frameType == 0x09 && length >= 6) {
        return true;
    }
    
    // Type 0x16 with service data format (less common but valid)
    // Service data UUID + temp/humid data
    if (frameType == 0x16 && frameLength >= 0x06 && length >= 9) {
        return true;
    }
    
    // Reject other frame types:
    // - 0x12 = Other beacon types (not sensor data)
    return false;
}

uint8_t detectMoKoModel(const uint8_t* data, size_t length) {
    if (!isValidMoKoData(data, length)) {
        return 0; // Unknown
    }
    
    uint8_t frameType = data[0];
    
    // Heuristic model detection based on frame type and length
    if (frameType == 0x08 && length >= 14) {
        return 2; // H3 (most common)
    } else if (frameType == 0x0A && length >= 16) {
        return 3; // H4 (with pressure)
    } else if (frameType == 0x0C && length >= 12) {
        return 1; // H2 (basic)
    }
    
    return 0; // Unknown model
}

MoKoData parseMoKoData(const uint8_t* data, size_t length) {
    MoKoData result = {};
    result.valid = false;
    
    if (!isValidMoKoData(data, length)) {
        return result;
    }
    
    // BeaconX Pro format after manufacturer ID (4C 00) is removed:
    // Byte 0: Frame type (0x10, 0x16, etc.)
    // Byte 1: Length or subtype
    // Byte 2+: Sensor data
    
    // For BeaconX Pro H3/H4 sensors, the common format is:
    // 4C 00 10 06 [4 bytes of sensor data]
    // where:
    //   Byte 0-1: Temperature (signed 16-bit, 0.01°C resolution)
    //   Byte 2-3: Humidity (unsigned 16-bit, 0.01% resolution)
    
    if (length < 2) {
        return result;
    }
    
    uint8_t frameType = data[0];
    uint8_t frameLength = data[1];
    
    // BeaconX Pro "Temperature and Humidity" frame format
    if (frameType == 0x10 && frameLength == 0x06 && length >= 6) {
        // Type 0x10, Length 0x06 = Temperature and Humidity frame
        // Format: 10 06 [temp bytes] [humid bytes]
        // Data starts at offset 2
        uint8_t offset = 2;
        
        // Try little-endian first (most common)
        int16_t tempRaw = data[offset] | (data[offset + 1] << 8);
        uint16_t humidRaw = data[offset + 2] | (data[offset + 3] << 8);
        result.temperature = tempRaw * 0.01f;
        result.humidity = humidRaw * 0.01f;
        
        // If values are out of range, try big-endian (some BeaconX Pro variants)
        if (result.humidity > 100.0f || result.humidity < 0.0f || 
            result.temperature < -40.0f || result.temperature > 85.0f) {
            tempRaw = (data[offset] << 8) | data[offset + 1];
            humidRaw = (data[offset + 2] << 8) | data[offset + 3];
            result.temperature = tempRaw * 0.01f;
            result.humidity = humidRaw * 0.01f;
        }
        
        // Validate ranges
        if (result.temperature < -40.0f || result.temperature > 85.0f) {
            return result; // Invalid temperature
        }
        if (result.humidity < 0.0f || result.humidity > 100.0f) {
            return result; // Invalid humidity
        }
        
        offset += 4;
        
        // Battery data may be present (optional)
        if (length >= offset + 1) {
            result.batteryPercent = data[offset];
            // Estimate voltage from percentage (CR2032: 3.0V max, 2.0V min)
            result.batteryVoltage = 2.0f + (result.batteryPercent / 100.0f) * 1.0f;
        }
        
        result.valid = true;
        return result;
    }
    else if (frameType == 0x10 && frameLength == 0x02 && length >= 4) {
        // Type 0x10, Length 0x02 = Device Info frame
        // Format: 10 02 [data1] [data2]
        // This typically contains battery level or firmware info
        // Common format: battery percentage in first byte
        uint8_t offset = 2;
        
        result.batteryPercent = data[offset];
        // Estimate voltage from percentage (CR2032: 3.0V max, 2.0V min)
        result.batteryVoltage = 2.0f + (result.batteryPercent / 100.0f) * 1.0f;
        
        // Mark as valid but note: no temp/humid data in this frame
        // Caller should merge this with temp/humid data from other frames
        result.valid = true;
        return result;
    }
    else if (frameType == 0x12 && frameLength == 0x02 && length >= 4) {
        // Type 0x12, Length 0x02 = iBeacon format (NOT a sensor!)
        // These are generic Apple beacon frames (iPhones, AirPods, etc.)
        // Return invalid to skip these non-sensor devices
        return result;
    }
    else if (frameType == 0x09 && length >= 10) {
        // Type 0x09 format (TLM or extended data) - BeaconX Pro extended format
        // This can contain additional sensor data beyond basic T/H
        // Byte 2-3: might be length/subtype
        // Data typically starts at offset 4 or later
        // Try parsing if we have enough data
        uint8_t offset = 4;
        
        if (length >= offset + 4) {
            // Temperature: 2 bytes, signed, 0.01°C resolution (little-endian)
            int16_t tempRaw = data[offset] | (data[offset + 1] << 8);
            result.temperature = tempRaw * 0.01f;
            offset += 2;
            
            // Humidity: 2 bytes, unsigned, 0.01% resolution (little-endian)
            uint16_t humidRaw = data[offset] | (data[offset + 1] << 8);
            result.humidity = humidRaw * 0.01f;
            offset += 2;
            
            // Validate ranges
            if (result.temperature >= -40.0f && result.temperature <= 85.0f &&
                result.humidity >= 0.0f && result.humidity <= 100.0f) {
                result.valid = true;
                return result;
            }
        }
        
        // If validation failed or not enough data, return invalid
        return result;
    }
    else if (frameType == 0x16 && frameLength >= 6 && length >= 9) {
        // Type 0x16 format (service data) - BeaconX Pro extended format
        // Byte 2: Service UUID LSB (usually 0x00 for MoKo)
        // Byte 3-4: Temperature
        // Byte 5-6: Humidity
        uint8_t offset = 3;
        
        // Try little-endian first
        int16_t tempRaw = data[offset] | (data[offset + 1] << 8);
        uint16_t humidRaw = data[offset + 2] | (data[offset + 3] << 8);
        result.temperature = tempRaw * 0.01f;
        result.humidity = humidRaw * 0.01f;
        
        // If values are out of range, try big-endian
        if (result.humidity > 100.0f || result.humidity < 0.0f || 
            result.temperature < -40.0f || result.temperature > 85.0f) {
            tempRaw = (data[offset] << 8) | data[offset + 1];
            humidRaw = (data[offset + 2] << 8) | data[offset + 3];
            result.temperature = tempRaw * 0.01f;
            result.humidity = humidRaw * 0.01f;
        }
        
        // Validate ranges
        if (result.temperature < -40.0f || result.temperature > 85.0f) {
            return result; // Invalid temperature
        }
        if (result.humidity < 0.0f || result.humidity > 100.0f) {
            return result; // Invalid humidity
        }
        
        result.valid = true;
        return result;
    }
    
    // Not a recognized BeaconX Pro format
    return result;
}
