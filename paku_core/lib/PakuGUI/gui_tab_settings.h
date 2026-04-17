/**
 * @file gui_tab_settings.h
 * @brief Settings tab — display brightness, sleep timer, network, firmware info.
 */
#pragma once
#include <lvgl.h>

/** Create the Settings tab content. */
lv_obj_t *gui_tab_settings_create(lv_obj_t *parent);

/** Update firmware version string. */
void gui_tab_settings_set_version(const char *ver);
