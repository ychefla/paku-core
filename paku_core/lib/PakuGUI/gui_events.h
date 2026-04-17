/**
 * @file gui_events.h
 * @brief Internal GUI action-event callback storage.
 *
 * Individual tab files include this header to fire user-action callbacks.
 * External code (main.cpp) registers handlers via the public API declared
 * in paku_gui.h.
 *
 * All callbacks are invoked from the LVGL/loop() task — safe to call
 * client.publish() directly from a registered handler.
 */
#pragma once

#include "paku_gui.h"   // brings in the callback type definitions

// ---------------------------------------------------------------------------
//  Extern declarations — storage is in gui_events.cpp
// ---------------------------------------------------------------------------

extern GuiLightCb         _cb_light;
extern GuiFanCb           _cb_fan;
extern GuiHeaterCb        _cb_heater;
extern GuiBacklightCb     _cb_backlight;
extern GuiRestartCb       _cb_restart;
extern GuiLightPresetCb   _cb_light_preset;
extern GuiClimatePresetCb _cb_climate_preset;
