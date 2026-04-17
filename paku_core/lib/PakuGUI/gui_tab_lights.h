/**
 * @file gui_tab_lights.h
 * @brief Lights tab — master controls and per-zone lighting.
 */
#pragma once
#include <lvgl.h>
#include "gui_presets.h"

/** Create the Lights tab content. */
lv_obj_t *gui_tab_lights_create(lv_obj_t *parent);

/** Update a single zone's displayed state. */
void gui_tab_lights_set_zone(uint8_t zone, bool on, uint8_t brightness, uint16_t colorTemp);

/** Capture the current light zone states into an array of LightZonePreset. */
void gui_tab_lights_get_state(LightZonePreset out[LIGHT_ZONE_COUNT]);

/** Open the save-preset modal for the Lights tab (called on long-press). */
void gui_tab_lights_open_save_modal();
