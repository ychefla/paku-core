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

/** Call from the main loop to check sleep timer inactivity. */
void gui_tab_settings_tick();

/**
 * @brief Wake the screen immediately (restore backlight, swallow the wake touch).
 *
 * Safe to call from any LVGL context. No-op if not currently sleeping.
 * Used by waveshare_hal's dim overlay PRESSED handler so a touch on the
 * blacked-out screen always wakes, regardless of LVGL inactivity timing.
 */
void gui_tab_settings_wake();
