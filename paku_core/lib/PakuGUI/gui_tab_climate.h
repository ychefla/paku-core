/**
 * @file gui_tab_climate.h
 * @brief Climate tab — fan and heater controls.
 */
#pragma once
#include <lvgl.h>
#include "gui_presets.h"

/** Create the Climate tab content. */
lv_obj_t *gui_tab_climate_create(lv_obj_t *parent);

/** Update heater data shown on the Climate tab. */
void gui_tab_climate_set_heater(int state, float target, float actual);

/** Update fan data shown on the Climate tab. */
void gui_tab_climate_set_fan(int speed, bool dirIn, bool lidOpen);

/** Capture the current climate tab state into a ClimatePreset struct. */
void gui_tab_climate_get_state(ClimatePreset *out);

/** Open the save-preset modal for the Climate tab (called on long-press). */
void gui_tab_climate_open_save_modal();
