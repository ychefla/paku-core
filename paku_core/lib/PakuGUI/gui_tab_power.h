/**
 * @file gui_tab_power.h
 * @brief Power tab — input/output bar graphs and live wattage values.
 */
#pragma once
#include <lvgl.h>

/** Create the Power tab content. */
lv_obj_t *gui_tab_power_create(lv_obj_t *parent);

/** Update power readings. */
void gui_tab_power_set_data(float solarW, float acChargerW,
                            float inverterW, float dcLoadW);
