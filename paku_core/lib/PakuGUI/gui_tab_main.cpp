/**
 * @file gui_tab_main.cpp
 * @brief Main/Presets tab — 2-column grid with Lighting and Climate presets.
 *
 * Layout: Two side-by-side panels, each with a coloured header and a 2×2
 * grid of large preset buttons (icon + label).  Only one preset can be
 * active at a time per section (radio behaviour).
 *
 * Presets are loaded from NVS via gui_presets.  Slot 0 is always "All Off".
 * Slots 1-3 are user-saveable from the Lights/Climate tabs.
 * When a preset is activated, the individual _cb_light / _cb_heater / _cb_fan
 * callbacks are invoked directly to apply the stored settings.
 */
#include "gui_tab_main.h"
#include "gui_theme.h"
#include "gui_events.h"
#include "gui_presets.h"
#include "gui_tab_lights.h"
#include "gui_tab_climate.h"

// ---------------------------------------------------------------------------
//  LVGL symbol icons for each preset slot
// ---------------------------------------------------------------------------

static const char *_lightIcons[PRESET_COUNT]   = {LV_SYMBOL_POWER, LV_SYMBOL_IMAGE, LV_SYMBOL_LIST, LV_SYMBOL_EYE_CLOSE};
static const char *_climateIcons[PRESET_COUNT]  = {LV_SYMBOL_POWER, LV_SYMBOL_CHARGE, LV_SYMBOL_REFRESH, LV_SYMBOL_EYE_CLOSE};

/// Handles for the 4 buttons in each section (for active state toggling)
static lv_obj_t *_lightBtns[PRESET_COUNT]   = {};
static lv_obj_t *_climateBtns[PRESET_COUNT] = {};

/// Label handles inside the buttons (for refresh)
static lv_obj_t *_lightBtnLabels[PRESET_COUNT]   = {};
static lv_obj_t *_climateBtnLabels[PRESET_COUNT] = {};

// ---------------------------------------------------------------------------
//  Helpers
// ---------------------------------------------------------------------------

/**
 * @brief Apply card styling to a container (dark card with rounded corners).
 */
static void style_card(lv_obj_t *obj) {
    lv_obj_set_style_bg_color(obj, GUI_COLOR_CARD, 0);
    lv_obj_set_style_bg_opa(obj, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(obj, GUI_RADIUS, 0);
    lv_obj_set_style_border_width(obj, 0, 0);
    lv_obj_set_style_pad_all(obj, GUI_PAD_MD, 0);
    lv_obj_clear_flag(obj, LV_OBJ_FLAG_SCROLLABLE);
}

// ---------------------------------------------------------------------------
//  Apply preset actions
// ---------------------------------------------------------------------------

/**
 * @brief Apply a lighting preset by calling _cb_light for each zone.
 */
static void apply_light_preset(uint8_t idx) {
    const LightPreset *presets = gui_presets_get_lights();
    const LightPreset *p = &presets[idx];
    if (!p->saved) return;

    for (int z = 0; z < LIGHT_ZONE_COUNT; z++) {
        // Update the Lights tab widgets
        gui_tab_lights_set_zone((uint8_t)z, p->zones[z].on,
                                p->zones[z].brightness, p->zones[z].colorTempK);
        // Notify external callback (MQTT etc.)
        if (_cb_light) {
            _cb_light((uint8_t)z, p->zones[z].on,
                      p->zones[z].brightness, p->zones[z].colorTempK);
        }
    }
}

/**
 * @brief Apply a climate preset by calling _cb_heater and _cb_fan.
 */
static void apply_climate_preset(uint8_t idx) {
    const ClimatePreset *presets = gui_presets_get_climate();
    const ClimatePreset *p = &presets[idx];
    if (!p->saved) return;

    // Update the Climate tab widgets
    gui_tab_climate_set_heater(p->heaterOn ? 1 : 0,
                               (float)p->targetTempC, 0.0f);
    gui_tab_climate_set_fan(p->fanSpeed, p->fanDirIn, p->lidOpen);

    // Notify external callbacks (MQTT etc.)
    if (_cb_heater) {
        HeaterMode mode = (HeaterMode)p->heaterMode;
        if (p->heaterOn) {
            _cb_heater(true, mode, p->powerLevel, p->targetTempC);
        } else {
            _cb_heater(false, mode, 0, 0);
        }
    }
    if (_cb_fan) {
        _cb_fan(p->fanPower, p->fanSpeed, p->fanDirIn, p->lidOpen);
    }
}

// ---------------------------------------------------------------------------
//  Callbacks
// ---------------------------------------------------------------------------

/**
 * @brief Radio-toggle callback: deactivate siblings, activate pressed button.
 */
static void preset_btn_cb(lv_event_t *e) {
    lv_obj_t *btn = lv_event_get_target(e);

    // Determine which array this belongs to and the index
    lv_obj_t **btns = nullptr;
    bool isLight = false;
    int pressedIdx = -1;
    for (int i = 0; i < PRESET_COUNT; i++) {
        if (_lightBtns[i] == btn)   { btns = _lightBtns;   isLight = true;  pressedIdx = i; break; }
        if (_climateBtns[i] == btn) { btns = _climateBtns;  isLight = false; pressedIdx = i; break; }
    }
    if (!btns || pressedIdx < 0) return;

    // Check if preset is saved (unsaved slots are inert)
    if (isLight) {
        const LightPreset *lp = gui_presets_get_lights();
        if (!lp[pressedIdx].saved) return;
    } else {
        const ClimatePreset *cp = gui_presets_get_climate();
        if (!cp[pressedIdx].saved) return;
    }

    // Deactivate all, activate pressed
    for (int i = 0; i < PRESET_COUNT; i++) {
        if (btns[i]) {
            lv_obj_set_style_border_color(btns[i], GUI_COLOR_CARD, 0);
            lv_obj_set_style_border_width(btns[i], 0, 0);
            lv_obj_set_style_bg_color(btns[i], GUI_COLOR_CARD, 0);
        }
    }
    lv_obj_set_style_border_color(btn, GUI_COLOR_ACCENT, 0);
    lv_obj_set_style_border_width(btn, GUI_BORDER_W, 0);
    lv_obj_set_style_bg_color(btn, lv_color_hex(0x3d2a00), 0);  // accent tint

    // Apply the preset
    if (isLight) {
        apply_light_preset((uint8_t)pressedIdx);
    } else {
        apply_climate_preset((uint8_t)pressedIdx);
    }
}

// ---------------------------------------------------------------------------
//  Section builder
// ---------------------------------------------------------------------------

/**
 * @brief Create a single preset section (header + 2x2 grid).
 */
static lv_obj_t *create_preset_section(lv_obj_t *parent, const char *title,
                                        lv_color_t headerColor,
                                        const char *icons[],
                                        const char *names[],
                                        const bool saved[],
                                        lv_obj_t *btnArr[],
                                        lv_obj_t *lblArr[]) {
    lv_obj_t *section = lv_obj_create(parent);
    lv_obj_set_flex_grow(section, 1);
    lv_obj_set_height(section, LV_PCT(100));
    style_card(section);
    lv_obj_set_flex_flow(section, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_gap(section, GUI_PAD_MD, 0);

    // Section header
    lv_obj_t *hdr = lv_label_create(section);
    lv_label_set_text(hdr, title);
    lv_obj_set_style_text_font(hdr, GUI_FONT_LG, 0);
    lv_obj_set_style_text_color(hdr, headerColor, 0);

    // 2x2 grid
    static lv_coord_t col_dsc[] = {LV_GRID_FR(1), LV_GRID_FR(1), LV_GRID_TEMPLATE_LAST};
    static lv_coord_t row_dsc[] = {LV_GRID_FR(1), LV_GRID_FR(1), LV_GRID_TEMPLATE_LAST};

    lv_obj_t *grid = lv_obj_create(section);
    lv_obj_set_size(grid, LV_PCT(100), LV_PCT(100));
    lv_obj_set_flex_grow(grid, 1);
    lv_obj_set_style_bg_opa(grid, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(grid, 0, 0);
    lv_obj_set_style_pad_all(grid, 0, 0);
    lv_obj_clear_flag(grid, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_grid_dsc_array(grid, col_dsc, row_dsc);
    lv_obj_set_style_pad_gap(grid, GUI_PAD_SM, 0);

    for (int i = 0; i < PRESET_COUNT; i++) {
        lv_obj_t *btn = lv_obj_create(grid);
        lv_obj_set_grid_cell(btn,
            LV_GRID_ALIGN_STRETCH, i % 2, 1,
            LV_GRID_ALIGN_STRETCH, i / 2, 1);
        lv_obj_set_style_bg_color(btn, GUI_COLOR_CARD, 0);
        lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, 0);
        lv_obj_set_style_radius(btn, GUI_RADIUS, 0);
        lv_obj_set_style_border_width(btn, 0, 0);
        lv_obj_set_flex_flow(btn, LV_FLEX_FLOW_COLUMN);
        lv_obj_set_flex_align(btn, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        lv_obj_clear_flag(btn, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_flag(btn, LV_OBJ_FLAG_CLICKABLE);

        // Icon
        lv_obj_t *icon = lv_label_create(btn);
        lv_label_set_text(icon, icons[i]);
        lv_obj_set_style_text_font(icon, GUI_FONT_XL, 0);
        lv_obj_set_style_text_color(icon, saved[i] ? GUI_COLOR_TEXT_PRI : GUI_COLOR_TEXT_MUTED, 0);

        // Label
        lv_obj_t *lbl = lv_label_create(btn);
        lv_label_set_text(lbl, names[i]);
        lv_obj_set_style_text_font(lbl, GUI_FONT_MD, 0);
        lv_obj_set_style_text_color(lbl, saved[i] ? GUI_COLOR_TEXT_SEC : GUI_COLOR_TEXT_MUTED, 0);

        lv_obj_add_event_cb(btn, preset_btn_cb, LV_EVENT_CLICKED, nullptr);
        btnArr[i] = btn;
        lblArr[i] = lbl;
    }

    return section;
}

// ---------------------------------------------------------------------------
//  Public API
// ---------------------------------------------------------------------------

lv_obj_t *gui_tab_main_create(lv_obj_t *parent) {
    lv_obj_t *cont = lv_obj_create(parent);
    lv_obj_set_size(cont, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_opa(cont, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(cont, 0, 0);
    lv_obj_set_style_pad_all(cont, GUI_PAD_MD, 0);
    lv_obj_clear_flag(cont, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_gap(cont, GUI_PAD_MD, 0);

    const LightPreset   *lp = gui_presets_get_lights();
    const ClimatePreset *cp = gui_presets_get_climate();

    // Build name/saved arrays for the section builder
    const char *lightNames[PRESET_COUNT];
    bool        lightSaved[PRESET_COUNT];
    const char *climateNames[PRESET_COUNT];
    bool        climateSaved[PRESET_COUNT];

    for (int i = 0; i < PRESET_COUNT; i++) {
        lightNames[i]   = lp[i].name;
        lightSaved[i]   = lp[i].saved;
        climateNames[i] = cp[i].name;
        climateSaved[i] = cp[i].saved;
    }

    create_preset_section(cont, "Lighting Presets", GUI_COLOR_BLUE,
                          _lightIcons, lightNames, lightSaved,
                          _lightBtns, _lightBtnLabels);
    create_preset_section(cont, "Climate Presets", GUI_COLOR_ACCENT,
                          _climateIcons, climateNames, climateSaved,
                          _climateBtns, _climateBtnLabels);

    return cont;
}

void gui_tab_main_refresh_presets() {
    const LightPreset   *lp = gui_presets_get_lights();
    const ClimatePreset *cp = gui_presets_get_climate();

    for (int i = 0; i < PRESET_COUNT; i++) {
        // Update light preset button labels
        if (_lightBtnLabels[i]) {
            lv_label_set_text(_lightBtnLabels[i], lp[i].name);
            lv_obj_set_style_text_color(_lightBtnLabels[i],
                lp[i].saved ? GUI_COLOR_TEXT_SEC : GUI_COLOR_TEXT_MUTED, 0);
        }
        // Update icon color based on saved state
        if (_lightBtns[i]) {
            lv_obj_t *icon = lv_obj_get_child(_lightBtns[i], 0);
            if (icon) {
                lv_obj_set_style_text_color(icon,
                    lp[i].saved ? GUI_COLOR_TEXT_PRI : GUI_COLOR_TEXT_MUTED, 0);
            }
        }

        // Update climate preset button labels
        if (_climateBtnLabels[i]) {
            lv_label_set_text(_climateBtnLabels[i], cp[i].name);
            lv_obj_set_style_text_color(_climateBtnLabels[i],
                cp[i].saved ? GUI_COLOR_TEXT_SEC : GUI_COLOR_TEXT_MUTED, 0);
        }
        if (_climateBtns[i]) {
            lv_obj_t *icon = lv_obj_get_child(_climateBtns[i], 0);
            if (icon) {
                lv_obj_set_style_text_color(icon,
                    cp[i].saved ? GUI_COLOR_TEXT_PRI : GUI_COLOR_TEXT_MUTED, 0);
            }
        }
    }
}
