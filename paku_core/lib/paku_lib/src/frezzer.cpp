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
    result.batPercent = 0x7F;  // unknown

    if (!isValidFrezzerData(data, length)) {
        return result;
    }

    // Alpicool protocol query response (cmd=0x01).
    // Full frame: FE FE len cmd body[18] checksum_hi checksum_lo
    // Body starts at frame[4]. Layout (single-zone, 18 bytes):
    //   [0]  locked         (bool)
    //   [1]  poweredOn      (bool)
    //   [2]  runMode        (0=Max, 1=Eco)
    //   [3]  batSaver       (0=Low, 1=Mid, 2=High)
    //   [4]  leftTarget     (signed int8, °C)
    //   [5]  tempMax        (signed int8, °C)
    //   [6]  tempMin        (signed int8, °C)
    //   [7]  leftRetDiff    (hysteresis °C)
    //   [8]  startDelay     (minutes)
    //   [9]  unit           (0=Celsius, 1=Fahrenheit)
    //  [10]  leftTCHot      (temp correction >−6 °C)
    //  [11]  leftTCMid      (temp correction −12 to −6 °C)
    //  [12]  leftTCCold     (temp correction <−12 °C)
    //  [13]  leftTCHalt     (temp correction at shutdown)
    //  [14]  leftCurrent    (signed int8, °C) ← ACTUAL current temperature
    //  [15]  batPercent     (0–100, 0x7F = unknown)
    //  [16]  batVolInt      (integer part of battery voltage)
    //  [17]  batVolDec      (decimal part, divide by 10)
    const uint8_t* body = &data[4];

    result.locked       = (body[0] != 0);
    bool poweredOn      = (body[1] != 0);
    uint8_t runMode     = body[2];
    result.batSaverMode = body[3];
    result.targetTemp   = static_cast<float>(static_cast<int8_t>(body[4]));
    result.currentTemp  = static_cast<float>(static_cast<int8_t>(body[14]));
    result.batPercent   = body[15];
    result.batteryVoltage = body[16] + (body[17] / 10.0f);

    // Map Alpicool poweredOn + runMode to FrezzerMode
    if (!poweredOn) {
        result.mode       = FrezzerMode::OFF;
        result.compressor = FrezzerCompressorState::OFF;
    } else if (runMode == 1) {
        result.mode       = FrezzerMode::ECO;
        result.compressor = FrezzerCompressorState::RUNNING;
    } else {
        result.mode       = FrezzerMode::MAX_COOL;
        result.compressor = FrezzerCompressorState::RUNNING;
    }

    result.error = FrezzerError::NONE;
    result.valid = true;
    return result;
}

bool isValidFrezzerData(const uint8_t* data, size_t length) {
    if (data == nullptr || length < 24) {
        return false;
    }
    // Alpicool frame header must be FE FE
    if (data[0] != 0xFE || data[1] != 0xFE) {
        return false;
    }
    // Length byte: must equal total_bytes - 3
    uint8_t pktLen = data[2];
    if (pktLen != (uint8_t)(length - 3)) {
        return false;
    }
    // Only handle query responses (cmd = 0x01)
    if (data[3] != 0x01) {
        return false;
    }
    return true;
}

const char* frezzerModeToString(FrezzerMode mode) {
    switch (mode) {
        case FrezzerMode::OFF:      return "off";
        case FrezzerMode::MAX_COOL: return "max_cool";
        case FrezzerMode::ECO:      return "eco";
        case FrezzerMode::UNKNOWN:
        default:                    return "unknown";
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
    if (deviceName == nullptr || deviceName[0] == '\0') {
        return false;
    }
    // Frezzer PRO advertises as "WT-0001" (confirmed Alpicool OEM platform).
    // Other compatible brands use A1-/AK1-/AK2-/AK3- prefixes.
    if (strcmp(deviceName, FREZZER_DEVICE_NAME_WT) == 0) return true;
    if (strncmp(deviceName, FREZZER_DEVICE_NAME_A1,  3) == 0) return true;
    if (strncmp(deviceName, FREZZER_DEVICE_NAME_AK1, 4) == 0) return true;
    if (strncmp(deviceName, FREZZER_DEVICE_NAME_AK2, 4) == 0) return true;
    if (strncmp(deviceName, FREZZER_DEVICE_NAME_AK3, 4) == 0) return true;
    return false;
}

size_t buildFrezzerCommand(FrezzerCommand cmd, int16_t param,
                            uint8_t* buffer, size_t bufferSize) {
    if (buffer == nullptr) {
        return 0;
    }

    // Alpicool frame: FE FE | lenByte | cmd [int8_param] | checksum_hi checksum_lo
    // create_packet equivalent (klightspeed/BrassMonkeyFridgeMonitor):
    //   data = cmd_byte [+ param_byte]
    //   pkt  = FE FE | (len(data)+2) | data | big_endian_16(sum(all_prefix_bytes))
    //
    // Commands WITHOUT parameter (BIND, QUERY, RESET): frame = 6 bytes
    // Commands WITH int8 parameter (SET_LEFT_TARGET, SET_RIGHT_TARGET): frame = 7 bytes

    bool hasParam = false;
    switch (cmd) {
        case FrezzerCommand::BIND:
        case FrezzerCommand::QUERY:
        case FrezzerCommand::RESET:
            hasParam = false;
            break;
        case FrezzerCommand::SET_LEFT_TARGET:
        case FrezzerCommand::SET_RIGHT_TARGET:
            hasParam = true;
            break;
        default:
            return 0;  // SET (full payload) handled separately via buildSetCommand
    }

    const size_t frameLen = hasParam ? 7u : 6u;
    if (bufferSize < frameLen) {
        return 0;
    }

    buffer[0] = 0xFE;
    buffer[1] = 0xFE;
    buffer[2] = hasParam ? 4 : 3;           // length byte
    buffer[3] = static_cast<uint8_t>(cmd);
    if (hasParam) {
        buffer[4] = static_cast<uint8_t>(static_cast<int8_t>(param));  // signed int8
    }

    // Checksum = big-endian 16-bit sum of all bytes before the 2 checksum bytes
    uint16_t checksum = 0;
    for (size_t i = 0; i < frameLen - 2u; i++) {
        checksum += buffer[i];
    }
    buffer[frameLen - 2] = static_cast<uint8_t>(checksum >> 8);
    buffer[frameLen - 1] = static_cast<uint8_t>(checksum & 0xFF);

    return frameLen;
}

#ifdef UNIT_TEST
void resetFrezzerForTesting(void) {
    // Nothing to reset in the stateless parsing functions
}
#endif
