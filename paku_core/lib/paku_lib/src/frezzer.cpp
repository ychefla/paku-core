/**
 * @file frezzer.cpp
 * @brief Frezzer PRO compressor fridge BLE data parsing implementation
 */
#include "frezzer.h"
#include <cstring>
#include <cctype>

FrezzerData parseFrezzerData(const uint8_t* data, size_t length) {
    FrezzerData result;
    memset(&result, 0, sizeof(result));
    result.valid = false;
    result.mode = FrezzerMode::UNKNOWN;
    result.compressor = FrezzerCompressorState::UNKNOWN;
    result.error = FrezzerError::NONE;
    
    if (!isValidFrezzerData(data, length)) {
        return result;
    }
    
    // Parse based on expected data format
    // Note: This is a hypothetical format based on common BLE fridge protocols
    // The actual format will need to be determined by reverse engineering
    // the specific Frezzer PRO device
    
    // Expected format (hypothetical - 10+ bytes):
    // Byte 0: Header/Protocol version
    // Byte 1-2: Current temperature (signed int16, 0.1°C resolution)
    // Byte 3-4: Target temperature (signed int16, 0.1°C resolution)
    // Byte 5-6: Battery voltage (uint16, mV)
    // Byte 7: Mode (enum)
    // Byte 8: Compressor state (enum)
    // Byte 9: Status flags (bit field)
    // Byte 10: Error code
    
    if (length >= 10) {
        // Current temperature (0.1°C resolution, signed)
        int16_t tempRaw = (int16_t)((data[2] << 8) | data[1]);
        result.currentTemp = tempRaw / 10.0f;
        
        // Target temperature
        int16_t targetRaw = (int16_t)((data[4] << 8) | data[3]);
        result.targetTemp = targetRaw / 10.0f;
        
        // Battery voltage in mV
        uint16_t voltageRaw = (data[6] << 8) | data[5];
        result.batteryVoltage = voltageRaw / 1000.0f;
        
        // Operating mode
        uint8_t modeVal = data[7];
        if (modeVal <= 4) {
            result.mode = static_cast<FrezzerMode>(modeVal);
        } else {
            result.mode = FrezzerMode::UNKNOWN;
        }
        
        // Compressor state
        uint8_t compressorVal = data[8];
        if (compressorVal <= 3) {
            result.compressor = static_cast<FrezzerCompressorState>(compressorVal);
        } else {
            result.compressor = FrezzerCompressorState::UNKNOWN;
        }
        
        // Status flags
        uint8_t flags = data[9];
        result.lidOpen = (flags & 0x01) != 0;
        result.lowVoltageProtection = (flags & 0x02) != 0;
        result.powerLevel = (flags >> 2) & 0x3F;  // bits 2-7 for power level
        
        // Error code (if present)
        if (length >= 11) {
            uint8_t errorVal = data[10];
            if (errorVal <= 6) {
                result.error = static_cast<FrezzerError>(errorVal);
            } else {
                result.error = FrezzerError::UNKNOWN;
            }
        }
        
        // Validate temperature ranges
        if (result.currentTemp >= -40.0f && result.currentTemp <= 50.0f &&
            result.targetTemp >= -25.0f && result.targetTemp <= 20.0f) {
            result.valid = true;
        }
    }
    
    return result;
}

bool isValidFrezzerData(const uint8_t* data, size_t length) {
    if (data == nullptr || length < 10) {
        return false;
    }
    
    // Check for valid header byte (hypothetical protocol version)
    // Adjust this based on actual device protocol
    uint8_t header = data[0];
    if (header < 0x01 || header > 0x10) {
        return false;  // Invalid protocol version
    }
    
    return true;
}

const char* frezzerModeToString(FrezzerMode mode) {
    switch (mode) {
        case FrezzerMode::OFF: return "off";
        case FrezzerMode::FRIDGE: return "fridge";
        case FrezzerMode::FREEZER: return "freezer";
        case FrezzerMode::ECO: return "eco";
        case FrezzerMode::MAX_COOL: return "max_cool";
        case FrezzerMode::UNKNOWN: 
        default: return "unknown";
    }
}

const char* frezzerCompressorStateToString(FrezzerCompressorState state) {
    switch (state) {
        case FrezzerCompressorState::OFF: return "off";
        case FrezzerCompressorState::RUNNING: return "running";
        case FrezzerCompressorState::STANDBY: return "standby";
        case FrezzerCompressorState::ERROR: return "error";
        case FrezzerCompressorState::UNKNOWN:
        default: return "unknown";
    }
}

const char* frezzerErrorToString(FrezzerError error) {
    switch (error) {
        case FrezzerError::NONE: return "none";
        case FrezzerError::TEMP_SENSOR: return "temp_sensor_error";
        case FrezzerError::COMPRESSOR: return "compressor_error";
        case FrezzerError::LOW_VOLTAGE: return "low_voltage";
        case FrezzerError::HIGH_VOLTAGE: return "high_voltage";
        case FrezzerError::OVERTEMP: return "overtemperature";
        case FrezzerError::COMMUNICATION: return "communication_error";
        case FrezzerError::UNKNOWN:
        default: return "unknown_error";
    }
}

const char* frezzerResultToString(FrezzerResult result) {
    switch (result) {
        case FrezzerResult::SUCCESS: return "success";
        case FrezzerResult::NOT_CONNECTED: return "not connected";
        case FrezzerResult::CONNECT_FAILED: return "connection failed";
        case FrezzerResult::WRITE_FAILED: return "write failed";
        case FrezzerResult::READ_FAILED: return "read failed";
        case FrezzerResult::TIMEOUT: return "timeout";
        case FrezzerResult::INVALID_PARAM: return "invalid parameter";
        case FrezzerResult::DEVICE_NOT_FOUND: return "device not found";
        case FrezzerResult::SERVICE_NOT_FOUND: return "service not found";
        case FrezzerResult::CHARACTERISTIC_NOT_FOUND: return "characteristic not found";
        case FrezzerResult::ALREADY_CONNECTED: return "already connected";
        case FrezzerResult::BUSY: return "busy";
        case FrezzerResult::UNKNOWN_ERROR:
        default: return "unknown error";
    }
}

bool isFrezzerDevice(const char* deviceName) {
    if (deviceName == nullptr || strlen(deviceName) == 0) {
        return false;
    }
    
    // Check if device name starts with FREZZER (case-insensitive)
    const char* prefix = FREZZER_DEVICE_NAME_PREFIX;
    size_t prefixLen = strlen(prefix);
    
    if (strlen(deviceName) < prefixLen) {
        return false;
    }
    
    for (size_t i = 0; i < prefixLen; i++) {
        char c1 = deviceName[i];
        char c2 = prefix[i];
        // Case-insensitive comparison
        if (toupper((unsigned char)c1) != toupper((unsigned char)c2)) {
            return false;
        }
    }
    
    return true;
}

size_t buildFrezzerCommand(FrezzerCommand cmd, int16_t param, 
                            uint8_t* buffer, size_t bufferSize) {
    if (buffer == nullptr || bufferSize < 4) {
        return 0;
    }
    
    // Command format (hypothetical):
    // Byte 0: Command header (0xFE)
    // Byte 1: Command type
    // Byte 2-3: Parameter (little-endian)
    
    buffer[0] = 0xFE;  // Command header
    buffer[1] = static_cast<uint8_t>(cmd);
    buffer[2] = param & 0xFF;          // Low byte
    buffer[3] = (param >> 8) & 0xFF;   // High byte
    
    return 4;
}

#ifdef UNIT_TEST
void resetFrezzerForTesting(void) {
    // Nothing to reset in the stateless parsing functions
}
#endif
