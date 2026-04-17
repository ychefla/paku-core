/**
 * @file gui_presets.cpp
 * @brief NVS persistence and management for lighting and climate presets.
 *
 * All 4 slots are user-saveable and stored as NVS blobs.
 * Unsaved slots appear as empty placeholders on the UI.
 */
#include "gui_presets.h"
#include <Preferences.h>
#include <cstring>

// ---------------------------------------------------------------------------
//  Local storage
// ---------------------------------------------------------------------------

static LightPreset   _lightPresets[PRESET_COUNT];
static ClimatePreset _climatePresets[PRESET_COUNT];

static Preferences _prefs;

// ---------------------------------------------------------------------------
//  Default initialisation (all slots empty until loaded from NVS)
// ---------------------------------------------------------------------------

static void init_defaults() {
    for (int i = 0; i < PRESET_COUNT; i++) {
        snprintf(_lightPresets[i].name, PRESET_NAME_LEN, "Slot %d", i + 1);
        _lightPresets[i].saved = false;
        for (int z = 0; z < LIGHT_ZONE_COUNT; z++) {
            _lightPresets[i].zones[z] = {false, 0, 4000};
        }

        snprintf(_climatePresets[i].name, PRESET_NAME_LEN, "Slot %d", i + 1);
        _climatePresets[i].saved     = false;
        _climatePresets[i].heaterOn  = false;
        _climatePresets[i].targetTempC = 21;
        _climatePresets[i].fanPower  = false;
        _climatePresets[i].fanSpeed  = 0;
        _climatePresets[i].fanDirIn  = true;
        _climatePresets[i].lidOpen   = false;
    }
}

// ---------------------------------------------------------------------------
//  NVS helpers
// ---------------------------------------------------------------------------

static void load_from_nvs() {
    _prefs.begin("presets", true);  // read-only

    for (int i = 0; i < PRESET_COUNT; i++) {
        char keyL[8], keyC[8];
        snprintf(keyL, sizeof(keyL), "lp%d", i);
        snprintf(keyC, sizeof(keyC), "cp%d", i);

        size_t lenL = _prefs.getBytesLength(keyL);
        if (lenL == sizeof(LightPreset)) {
            _prefs.getBytes(keyL, &_lightPresets[i], sizeof(LightPreset));
        }

        size_t lenC = _prefs.getBytesLength(keyC);
        if (lenC == sizeof(ClimatePreset)) {
            _prefs.getBytes(keyC, &_climatePresets[i], sizeof(ClimatePreset));
        }
    }

    _prefs.end();
}

static bool save_light_to_nvs(uint8_t idx) {
    _prefs.begin("presets", false);
    char key[8];
    snprintf(key, sizeof(key), "lp%d", idx);
    size_t written = _prefs.putBytes(key, &_lightPresets[idx], sizeof(LightPreset));
    _prefs.end();
    return written == sizeof(LightPreset);
}

static bool save_climate_to_nvs(uint8_t idx) {
    _prefs.begin("presets", false);
    char key[8];
    snprintf(key, sizeof(key), "cp%d", idx);
    size_t written = _prefs.putBytes(key, &_climatePresets[idx], sizeof(ClimatePreset));
    _prefs.end();
    return written == sizeof(ClimatePreset);
}

// ---------------------------------------------------------------------------
//  Public API
// ---------------------------------------------------------------------------

void gui_presets_init() {
    init_defaults();
    load_from_nvs();
}

const LightPreset *gui_presets_get_lights() {
    return _lightPresets;
}

const ClimatePreset *gui_presets_get_climate() {
    return _climatePresets;
}

bool gui_presets_save_light(uint8_t idx, const char *name,
                            const LightZonePreset zones[LIGHT_ZONE_COUNT]) {
    if (idx >= PRESET_COUNT) return false;

    strncpy(_lightPresets[idx].name, name, PRESET_NAME_LEN - 1);
    _lightPresets[idx].name[PRESET_NAME_LEN - 1] = '\0';
    for (int z = 0; z < LIGHT_ZONE_COUNT; z++) {
        _lightPresets[idx].zones[z] = zones[z];
    }
    _lightPresets[idx].saved = true;

    return save_light_to_nvs(idx);
}

bool gui_presets_save_climate(uint8_t idx, const char *name,
                              const ClimatePreset *p) {
    if (idx >= PRESET_COUNT) return false;

    strncpy(_climatePresets[idx].name, name, PRESET_NAME_LEN - 1);
    _climatePresets[idx].name[PRESET_NAME_LEN - 1] = '\0';
    _climatePresets[idx].heaterOn    = p->heaterOn;
    _climatePresets[idx].targetTempC = p->targetTempC;
    _climatePresets[idx].fanPower    = p->fanPower;
    _climatePresets[idx].fanSpeed    = p->fanSpeed;
    _climatePresets[idx].fanDirIn    = p->fanDirIn;
    _climatePresets[idx].lidOpen     = p->lidOpen;
    _climatePresets[idx].saved       = true;

    return save_climate_to_nvs(idx);
}
