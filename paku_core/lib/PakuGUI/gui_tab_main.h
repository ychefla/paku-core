/**
 * @file gui_tab_main.h
 * @brief Main/Presets tab — quick preset access for lighting and climate.
 */
#pragma once
#include <lvgl.h>

/** Create the Main presets tab content inside the given parent container. */
lv_obj_t *gui_tab_main_create(lv_obj_t *parent);

/**
 * @brief Refresh preset button labels after a new preset has been saved.
 * Call this from the save-preset modals in the lights/climate tabs.
 */
void gui_tab_main_refresh_presets();
