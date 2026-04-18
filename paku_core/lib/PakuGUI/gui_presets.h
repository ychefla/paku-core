/**
 * @file gui_presets.h
 * @brief Preset data structures and NVS persistence for lighting and climate presets.
 *
 * Provides up to 4 user-saveable slots each for lighting and climate presets.
 * Presets are stored in ESP32 NVS (Preferences) and survive reboots.
 *
 * Slot 0 is always "All Off" and cannot be overwritten.
 */
#pragma once

#include <cstdint>

// ---------------------------------------------------------------------------
//  Constants
// ---------------------------------------------------------------------------

#define PRESET_COUNT        4   ///< Number of preset slots per category
#define PRESET_NAME_LEN     16  ///< Max chars for a preset name (including \0)
#define LIGHT_ZONE_COUNT    4   ///< Number of light zones

// ---------------------------------------------------------------------------
//  Data structures
// ---------------------------------------------------------------------------

/** @brief State of a single light zone within a preset. */
struct LightZonePreset {
    bool     on;
    uint8_t  brightness;   ///< 0-100 %
    uint16_t colorTempK;   ///< 2700-6500 K
};

/** @brief A complete lighting preset (all 4 zones). */
struct LightPreset {
    char             name[PRESET_NAME_LEN];
    LightZonePreset  zones[LIGHT_ZONE_COUNT];
    bool             saved;   ///< true if slot has user data
};

/** @brief A complete climate preset (heater + fan). */
struct ClimatePreset {
    char    name[PRESET_NAME_LEN];
    bool    heaterOn;
    uint8_t heaterMode;      ///< 0=power, 1=thermostat (HeaterMode)
    uint8_t powerLevel;      ///< 0-9 (power mode)
    uint8_t targetTempC;     ///< 10-30 (thermostat mode)
    bool    fanPower;
    uint8_t fanSpeed;        ///< 0-100
    bool    fanDirIn;
    bool    lidOpen;
    bool    saved;           ///< true if slot has user data
};

// ---------------------------------------------------------------------------
//  Public API
// ---------------------------------------------------------------------------

/**
 * @brief Load all presets from NVS into RAM.
 * Call once during gui_init().  Slot 0 is initialised to hard-coded defaults.
 */
void gui_presets_init();

/** @brief Get a read-only pointer to the light presets array [PRESET_COUNT]. */
const LightPreset   *gui_presets_get_lights();

/** @brief Get a read-only pointer to the climate presets array [PRESET_COUNT]. */
const ClimatePreset *gui_presets_get_climate();

/**
 * @brief Save the current light zone states into a preset slot.
 * @param idx   Slot index 1-3 (slot 0 is reserved).
 * @param name  Display name for the preset.
 * @param zones Array of LIGHT_ZONE_COUNT zone states.
 * @return true on success.
 */
bool gui_presets_save_light(uint8_t idx, const char *name,
                            const LightZonePreset zones[LIGHT_ZONE_COUNT]);

/**
 * @brief Save the current climate state into a preset slot.
 * @param idx  Slot index 1-3 (slot 0 is reserved).
 * @param name Display name for the preset.
 * @param p    Climate preset data (saved flag is set automatically).
 * @return true on success.
 */
bool gui_presets_save_climate(uint8_t idx, const char *name,
                              const ClimatePreset *p);
