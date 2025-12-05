/**
 * @file device_config.h
 * @brief Device-specific configuration and feature detection
 * 
 * This file reads the device selection from secrets.h and defines
 * hardware capability macros used for conditional compilation.
 * 
 * To select a device, edit secrets.h and uncomment ONE device define:
 *   #define DEVICE_LILYGO_T_DISPLAY_S3   // LilyGo with display (default)
 *   #define DEVICE_ESP32_CH340C_30PIN    // Generic ESP32 (no display)
 */
#pragma once

#include "secrets.h"

// =============================================================================
// Device Configuration: LilyGo T-Display S3
// =============================================================================
#if defined(DEVICE_LILYGO_T_DISPLAY_S3)
    #define DEVICE_NAME "LilyGo T-Display S3"
    #define HAS_DISPLAY 1
    #define HAS_TOUCH 1
    #define HAS_PSRAM 1

// =============================================================================
// Device Configuration: ESP32 CH340C 30PIN (Generic)
// =============================================================================
#elif defined(DEVICE_ESP32_CH340C_30PIN)
    #define DEVICE_NAME "ESP32 CH340C 30PIN"
    #define HAS_DISPLAY 0
    #define HAS_TOUCH 0
    #define HAS_PSRAM 0

// =============================================================================
// Default / Error
// =============================================================================
#else
    // Default to LilyGo T-Display S3 for backward compatibility
    #warning "No device defined in secrets.h, defaulting to DEVICE_LILYGO_T_DISPLAY_S3"
    #define DEVICE_NAME "LilyGo T-Display S3"
    #define HAS_DISPLAY 1
    #define HAS_TOUCH 1
    #define HAS_PSRAM 1
#endif
