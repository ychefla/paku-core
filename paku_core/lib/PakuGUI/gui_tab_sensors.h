/**
 * @file gui_tab_sensors.h
 * @brief Sensors tab — temperature and humidity trend charts.
 */
#pragma once
#include <lvgl.h>

/** Create the Sensors tab content. */
lv_obj_t *gui_tab_sensors_create(lv_obj_t *parent);

/** Push a temperature data point (series 0-3). */
void gui_tab_sensors_push_temp(uint8_t series, float value);

/** Push a humidity data point (series 0-2). */
void gui_tab_sensors_push_hum(uint8_t series, float value);
