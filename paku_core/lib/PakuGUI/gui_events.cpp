/**
 * @file gui_events.cpp
 * @brief GUI action-event callback storage and registration implementations.
 */
#include "gui_events.h"

// ---------------------------------------------------------------------------
//  Callback storage (nullptr = not registered)
// ---------------------------------------------------------------------------

GuiLightCb         _cb_light          = nullptr;
GuiFanCb           _cb_fan            = nullptr;
GuiHeaterCb        _cb_heater         = nullptr;
GuiBacklightCb     _cb_backlight      = nullptr;
GuiRestartCb       _cb_restart        = nullptr;
GuiLightPresetCb   _cb_light_preset   = nullptr;
GuiClimatePresetCb _cb_climate_preset = nullptr;

// ---------------------------------------------------------------------------
//  Registration functions (public API — declared in paku_gui.h)
// ---------------------------------------------------------------------------

void gui_on_light_changed(GuiLightCb cb)          { _cb_light          = cb; }
void gui_on_fan_changed(GuiFanCb cb)              { _cb_fan            = cb; }
void gui_on_heater_changed(GuiHeaterCb cb)        { _cb_heater         = cb; }
void gui_on_backlight_changed(GuiBacklightCb cb)  { _cb_backlight      = cb; }
void gui_on_restart(GuiRestartCb cb)              { _cb_restart        = cb; }
void gui_on_light_preset(GuiLightPresetCb cb)     { _cb_light_preset   = cb; }
void gui_on_climate_preset(GuiClimatePresetCb cb) { _cb_climate_preset = cb; }
