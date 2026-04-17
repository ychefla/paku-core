/**
 * @file gui_header.h
 * @brief Status header bar for the Paku GUI.
 */
#pragma once

#include <lvgl.h>

/**
 * @brief Create the header bar at the top of the screen.
 * @param parent  The parent object (typically lv_scr_act()).
 * @return The header container object.
 */
lv_obj_t *gui_header_create(lv_obj_t *parent);

/** Update WiFi indicator */
void gui_header_set_wifi(bool connected, int rssi);

/** Update MQTT indicator */
void gui_header_set_mqtt(bool connected);

/** Update BLE indicator */
void gui_header_set_ble(bool active);

/** Update clock text */
void gui_header_set_time(const char *timeStr);

/** Update heater status badge in header */
void gui_header_set_heater(int state);
