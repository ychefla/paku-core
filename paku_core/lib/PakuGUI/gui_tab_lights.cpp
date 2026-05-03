/**
 * @file gui_tab_lights.cpp
 * @brief Lights tab — master controls (ALL ON/OFF, brightness, color temp)
 *        and per-zone 2×2 grid with toggle + brightness + color temp sliders.
 */
#include "gui_tab_lights.h"
#include "gui_theme.h"
#include "gui_events.h"
#include "gui_presets.h"
#include "gui_tab_main.h"
#include <cstdio>

// ---------------------------------------------------------------------------
//  Zone data
// ---------------------------------------------------------------------------

#define NUM_ZONES 4

struct ZoneWidgets {
    lv_obj_t *card;
    lv_obj_t *sldBright;
    lv_obj_t *lblBright;
    lv_obj_t *sldCTemp;
    lv_obj_t *lblCTemp;
};

static const char *zoneNames[NUM_ZONES]  = {"Ceiling", "Counter", "Drawers", "Under Bed"};
static const char *zoneIcons[NUM_ZONES]  = {LV_SYMBOL_HOME, LV_SYMBOL_LIST, LV_SYMBOL_IMAGE, LV_SYMBOL_EYE_CLOSE};
static uint8_t     zoneBright[NUM_ZONES] = {80, 65, 40, 100};
static uint16_t    zoneCTemp[NUM_ZONES]  = {4000, 3500, 2700, 5000};
static bool        zoneOn[NUM_ZONES]     = {true, true, false, false};

static ZoneWidgets zw[NUM_ZONES];

// Master controls
static lv_obj_t *_btnAllOn   = nullptr;
static lv_obj_t *_btnAllOff  = nullptr;
static lv_obj_t *_sldMBright = nullptr;
static lv_obj_t *_lblMBright = nullptr;
static lv_obj_t *_sldMCTemp  = nullptr;
static lv_obj_t *_lblMCTemp  = nullptr;

// ---------------------------------------------------------------------------
//  Helpers
// ---------------------------------------------------------------------------

static void style_card(lv_obj_t *obj) {
    lv_obj_set_style_bg_color(obj, GUI_COLOR_CARD, 0);
    lv_obj_set_style_bg_opa(obj, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(obj, GUI_RADIUS, 0);
    lv_obj_set_style_border_width(obj, 0, 0);
    lv_obj_set_style_pad_all(obj, GUI_PAD_MD, 0);
    lv_obj_clear_flag(obj, LV_OBJ_FLAG_SCROLLABLE);
}

/**
 * @brief Map colour-temperature (Kelvin) to an approximate RGB colour.
 *
 * Maps 2700 K → warm amber, through neutral white, to 6500 K → cool blue-white.
 * This is a simplified piecewise-linear approximation — not based on a
 * verified CIE model, but looks reasonable on screen.
 */
static lv_color_t ctemp_to_color(uint16_t kelvin) {
    // Clamp
    if (kelvin < 2700) kelvin = 2700;
    if (kelvin > 6500) kelvin = 6500;

    uint8_t r, g, b;
    if (kelvin <= 4000) {
        // 2700 K (warm amber 255,160,40) → 4000 K (neutral warm 255,220,160)
        float t = (float)(kelvin - 2700) / 1300.0f;
        r = 255;
        g = (uint8_t)(160 + t * 60);    // 160 → 220
        b = (uint8_t)(40  + t * 120);   // 40  → 160
    } else {
        // 4000 K (255,220,160) → 6500 K (cool daylight 200,220,255)
        float t = (float)(kelvin - 4000) / 2500.0f;
        r = (uint8_t)(255 - t * 55);    // 255 → 200
        g = 220;
        b = (uint8_t)(160 + t * 95);    // 160 → 255
    }
    return lv_color_make(r, g, b);
}

/** @brief Update zone card border and slider accent colours to match colour temperature. */
static void update_zone_visuals(int z) {
    lv_color_t col = ctemp_to_color(zoneCTemp[z]);

    // Card border
    if (zw[z].card) {
        lv_obj_set_style_border_color(zw[z].card,
            zoneOn[z] ? col : GUI_COLOR_CARD, 0);
        lv_obj_set_style_border_width(zw[z].card,
            zoneOn[z] ? 2 : 0, 0);
    }

    // Brightness slider accent
    if (zw[z].sldBright) {
        lv_obj_set_style_bg_color(zw[z].sldBright, col, LV_PART_INDICATOR);
        lv_obj_set_style_bg_color(zw[z].sldBright, col, LV_PART_KNOB);
    }

    // Color-temp slider accent
    if (zw[z].sldCTemp) {
        lv_obj_set_style_bg_color(zw[z].sldCTemp, col, LV_PART_INDICATOR);
        lv_obj_set_style_bg_color(zw[z].sldCTemp, col, LV_PART_KNOB);
    }
}

/** @brief Sync master controls (sliders + All On/Off buttons) from per-zone state. */
static void sync_master_controls() {
    // All On/Off button colours
    bool anyOn  = false;
    bool allOff = true;
    for (int i = 0; i < NUM_ZONES; i++) {
        if (zoneOn[i]) { anyOn = true; allOff = false; }
    }
    if (_btnAllOn)  lv_obj_set_style_bg_color(_btnAllOn,  anyOn  ? GUI_COLOR_ACCENT : GUI_COLOR_CARD_HOVER, 0);
    if (_btnAllOff) lv_obj_set_style_bg_color(_btnAllOff, allOff ? GUI_COLOR_ACCENT : GUI_COLOR_CARD_HOVER, 0);

    // If all zones share the same brightness, update master slider
    bool sameBright = true;
    for (int i = 1; i < NUM_ZONES; i++) {
        if (zoneBright[i] != zoneBright[0]) { sameBright = false; break; }
    }
    if (sameBright && _sldMBright) {
        lv_slider_set_value(_sldMBright, zoneBright[0], LV_ANIM_ON);
        if (_lblMBright) {
            char buf[8]; snprintf(buf, sizeof(buf), "%d", zoneBright[0]);
            lv_label_set_text(_lblMBright, buf);
        }
    }

    // If all zones share the same color temp, update master slider
    bool sameCTemp = true;
    for (int i = 1; i < NUM_ZONES; i++) {
        if (zoneCTemp[i] != zoneCTemp[0]) { sameCTemp = false; break; }
    }
    if (sameCTemp && _sldMCTemp) {
        lv_slider_set_value(_sldMCTemp, zoneCTemp[0], LV_ANIM_ON);
        if (_lblMCTemp) {
            char buf[8]; snprintf(buf, sizeof(buf), "%d", zoneCTemp[0]);
            lv_label_set_text(_lblMCTemp, buf);
        }
    }
}

// ---------------------------------------------------------------------------
//  Callbacks
// ---------------------------------------------------------------------------

static void zone_card_toggle_cb(lv_event_t *e) {
    int idx = (int)(intptr_t)lv_event_get_user_data(e);
    zoneOn[idx] = !zoneOn[idx];
    update_zone_visuals(idx);
    if (_cb_light) _cb_light((uint8_t)idx, zoneOn[idx], zoneBright[idx], zoneCTemp[idx]);
}

static void zone_bright_cb(lv_event_t *e) {
    int idx = (int)(intptr_t)lv_event_get_user_data(e);
    int val = lv_slider_get_value(zw[idx].sldBright);
    zoneBright[idx] = (uint8_t)val;
    char buf[8]; snprintf(buf, sizeof(buf), "%d", val);
    lv_label_set_text(zw[idx].lblBright, buf);
    if (_cb_light) _cb_light((uint8_t)idx, zoneOn[idx], zoneBright[idx], zoneCTemp[idx]);
}

static void zone_ctemp_cb(lv_event_t *e) {
    int idx = (int)(intptr_t)lv_event_get_user_data(e);
    int val = lv_slider_get_value(zw[idx].sldCTemp);
    zoneCTemp[idx] = (uint16_t)val;
    char buf[8]; snprintf(buf, sizeof(buf), "%d", val);
    lv_label_set_text(zw[idx].lblCTemp, buf);
    update_zone_visuals(idx);
    if (_cb_light) _cb_light((uint8_t)idx, zoneOn[idx], zoneBright[idx], zoneCTemp[idx]);
}

static void master_bright_cb(lv_event_t *e) {
    int val = lv_slider_get_value(_sldMBright);
    char buf[8]; snprintf(buf, sizeof(buf), "%d", val);
    lv_label_set_text(_lblMBright, buf);
    // Apply to all zones
    for (int i = 0; i < NUM_ZONES; i++) {
        zoneBright[i] = (uint8_t)val;
        if (zw[i].sldBright) lv_slider_set_value(zw[i].sldBright, val, LV_ANIM_OFF);
        if (zw[i].lblBright) {
            char tb[8]; snprintf(tb, sizeof(tb), "%d", val);
            lv_label_set_text(zw[i].lblBright, tb);
        }
        if (_cb_light) _cb_light((uint8_t)i, zoneOn[i], zoneBright[i], zoneCTemp[i]);
    }
}

static void master_ctemp_cb(lv_event_t *e) {
    int val = lv_slider_get_value(_sldMCTemp);
    char buf[8]; snprintf(buf, sizeof(buf), "%d", val);
    lv_label_set_text(_lblMCTemp, buf);
    // Apply to all zones
    for (int i = 0; i < NUM_ZONES; i++) {
        zoneCTemp[i] = (uint16_t)val;
        if (zw[i].sldCTemp) lv_slider_set_value(zw[i].sldCTemp, val, LV_ANIM_OFF);
        if (zw[i].lblCTemp) {
            char tb[8]; snprintf(tb, sizeof(tb), "%d", val);
            lv_label_set_text(zw[i].lblCTemp, tb);
        }
        update_zone_visuals(i);
        if (_cb_light) _cb_light((uint8_t)i, zoneOn[i], zoneBright[i], zoneCTemp[i]);
    }
}

static void all_on_cb(lv_event_t *e) {
    for (int i = 0; i < NUM_ZONES; i++) {
        zoneOn[i] = true;
        update_zone_visuals(i);
        if (_cb_light) _cb_light((uint8_t)i, true, zoneBright[i], zoneCTemp[i]);
    }
    lv_obj_set_style_bg_color(_btnAllOn,  GUI_COLOR_ACCENT, 0);
    lv_obj_set_style_bg_color(_btnAllOff, GUI_COLOR_CARD_HOVER, 0);
}

static void all_off_cb(lv_event_t *e) {
    for (int i = 0; i < NUM_ZONES; i++) {
        zoneOn[i] = false;
        update_zone_visuals(i);
        if (_cb_light) _cb_light((uint8_t)i, false, zoneBright[i], zoneCTemp[i]);
    }
    lv_obj_set_style_bg_color(_btnAllOff, GUI_COLOR_ACCENT, 0);
    lv_obj_set_style_bg_color(_btnAllOn,  GUI_COLOR_CARD_HOVER, 0);
}

// ---------------------------------------------------------------------------
//  Master controls panel
// ---------------------------------------------------------------------------

/** Helper: create a slider with overlaid name (left) and value (right) labels. */
static lv_obj_t *create_overlay_slider(
    lv_obj_t *parent, const char *name,
    int32_t rangeMin, int32_t rangeMax, int32_t initVal,
    lv_obj_t **outSlider, lv_obj_t **outValLabel,
    lv_color_t accentColor)
{
    // Wrapper holds the slider + floating labels
    lv_obj_t *wrap = lv_obj_create(parent);
    lv_obj_set_size(wrap, LV_PCT(100), 40);
    lv_obj_set_style_bg_opa(wrap, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(wrap, 0, 0);
    lv_obj_set_style_pad_all(wrap, 0, 0);
    lv_obj_clear_flag(wrap, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *sld = lv_slider_create(wrap);
    lv_slider_set_range(sld, rangeMin, rangeMax);
    lv_slider_set_value(sld, initVal, LV_ANIM_OFF);
    lv_obj_set_size(sld, LV_PCT(100), LV_PCT(100));
    lv_obj_center(sld);
    lv_obj_set_style_bg_color(sld, GUI_COLOR_CARD_HOVER, LV_PART_MAIN);
    lv_obj_set_style_bg_color(sld, accentColor, LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(sld, accentColor, LV_PART_KNOB);
    lv_obj_set_style_radius(sld, 4, LV_PART_MAIN);
    lv_obj_set_style_radius(sld, 4, LV_PART_INDICATOR);

    // Name overlay (left-aligned on top of slider)
    lv_obj_t *lblName = lv_label_create(wrap);
    lv_label_set_text(lblName, name);
    lv_obj_set_style_text_font(lblName, GUI_FONT_LG, 0);
    lv_obj_set_style_text_color(lblName, lv_color_white(), 0);
    lv_obj_set_style_text_opa(lblName, LV_OPA_COVER, 0);
    lv_obj_align(lblName, LV_ALIGN_LEFT_MID, 8, 0);
    lv_obj_add_flag(lblName, LV_OBJ_FLAG_FLOATING);
    lv_obj_clear_flag(lblName, LV_OBJ_FLAG_CLICKABLE);

    // Value overlay (right-aligned on top of slider)
    lv_obj_t *lblVal = lv_label_create(wrap);
    char buf[8];
    snprintf(buf, sizeof(buf), "%d", (int)initVal);
    lv_label_set_text(lblVal, buf);
    lv_obj_set_style_text_font(lblVal, GUI_FONT_LG, 0);
    lv_obj_set_style_text_color(lblVal, lv_color_white(), 0);
    lv_obj_set_style_text_opa(lblVal, LV_OPA_COVER, 0);
    lv_obj_align(lblVal, LV_ALIGN_RIGHT_MID, -8, 0);
    lv_obj_add_flag(lblVal, LV_OBJ_FLAG_FLOATING);
    lv_obj_clear_flag(lblVal, LV_OBJ_FLAG_CLICKABLE);

    *outSlider  = sld;
    *outValLabel = lblVal;
    return wrap;
}

static lv_obj_t *create_master_panel(lv_obj_t *parent) {
    lv_obj_t *panel = lv_obj_create(parent);
    lv_obj_set_size(panel, LV_PCT(100), LV_SIZE_CONTENT);
    style_card(panel);
    lv_obj_set_style_pad_all(panel, GUI_PAD_SM, 0);
    lv_obj_set_style_border_color(panel, GUI_COLOR_PANEL, 0);
    lv_obj_set_style_border_width(panel, 2, 0);
    lv_obj_set_flex_flow(panel, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_gap(panel, GUI_PAD_SM, 0);

    // ALL OFF / ALL ON buttons row (OFF left, ON right)
    lv_obj_t *btnRow = lv_obj_create(panel);
    lv_obj_set_size(btnRow, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_style_bg_opa(btnRow, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(btnRow, 0, 0);
    lv_obj_set_style_pad_all(btnRow, 0, 0);
    lv_obj_clear_flag(btnRow, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flex_flow(btnRow, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_gap(btnRow, GUI_PAD_SM, 0);

    _btnAllOff = lv_btn_create(btnRow);
    lv_obj_set_flex_grow(_btnAllOff, 1);
    lv_obj_set_height(_btnAllOff, 40);
    lv_obj_set_style_bg_color(_btnAllOff, GUI_COLOR_CARD_HOVER, 0);
    lv_obj_set_style_radius(_btnAllOff, GUI_RADIUS, 0);
    lv_obj_t *lbl2 = lv_label_create(_btnAllOff);
    lv_label_set_text(lbl2, "ALL OFF");
    lv_obj_set_style_text_font(lbl2, GUI_FONT_LG, 0);
    lv_obj_center(lbl2);
    lv_obj_add_event_cb(_btnAllOff, all_off_cb, LV_EVENT_CLICKED, nullptr);

    _btnAllOn = lv_btn_create(btnRow);
    lv_obj_set_flex_grow(_btnAllOn, 1);
    lv_obj_set_height(_btnAllOn, 40);
    lv_obj_set_style_bg_color(_btnAllOn, GUI_COLOR_ACCENT, 0);
    lv_obj_set_style_radius(_btnAllOn, GUI_RADIUS, 0);
    lv_obj_t *lbl1 = lv_label_create(_btnAllOn);
    lv_label_set_text(lbl1, "ALL ON");
    lv_obj_set_style_text_font(lbl1, GUI_FONT_LG, 0);
    lv_obj_center(lbl1);
    lv_obj_add_event_cb(_btnAllOn, all_on_cb, LV_EVENT_CLICKED, nullptr);

    // Master brightness slider with overlay
    create_overlay_slider(panel, "Brightness", 0, 100, 75,
                          &_sldMBright, &_lblMBright, GUI_COLOR_ACCENT);
    lv_obj_add_event_cb(_sldMBright, master_bright_cb, LV_EVENT_VALUE_CHANGED, nullptr);

    // Master color temp slider with overlay
    create_overlay_slider(panel, "Color Temp", 2700, 6500, 4000,
                          &_sldMCTemp, &_lblMCTemp, GUI_COLOR_ACCENT);
    lv_obj_add_event_cb(_sldMCTemp, master_ctemp_cb, LV_EVENT_VALUE_CHANGED, nullptr);

    return panel;
}

// ---------------------------------------------------------------------------
//  Zone grid (2×2)
// ---------------------------------------------------------------------------

static lv_obj_t *create_zone_card(lv_obj_t *grid, int idx) {
    lv_obj_t *card = lv_obj_create(grid);
    style_card(card);
    lv_obj_set_style_pad_all(card, GUI_PAD_SM, 0);
    lv_obj_set_flex_flow(card, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_gap(card, 4, 0);
    lv_obj_add_flag(card, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(card, zone_card_toggle_cb, LV_EVENT_CLICKED, (void *)(intptr_t)idx);

    // Header area — fills remaining space above sliders, acts as toggle target
    lv_obj_t *header = lv_obj_create(card);
    lv_obj_set_size(header, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_flex_grow(header, 1);
    lv_obj_set_style_bg_opa(header, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(header, 0, 0);
    lv_obj_set_style_pad_all(header, 0, 0);
    lv_obj_clear_flag(header, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(header, LV_OBJ_FLAG_EVENT_BUBBLE);
    lv_obj_set_flex_flow(header, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(header, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_gap(header, GUI_PAD_SM, 0);

    lv_obj_t *iconLbl = lv_label_create(header);
    lv_label_set_text(iconLbl, zoneIcons[idx]);
    lv_obj_set_style_text_font(iconLbl, GUI_FONT_LG, 0);
    lv_obj_set_style_text_color(iconLbl, GUI_COLOR_TEXT_PRI, 0);
    lv_obj_clear_flag(iconLbl, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(iconLbl, LV_OBJ_FLAG_EVENT_BUBBLE);

    lv_obj_t *nameLbl = lv_label_create(header);
    lv_label_set_text(nameLbl, zoneNames[idx]);
    lv_obj_set_style_text_font(nameLbl, GUI_FONT_LG, 0);
    lv_obj_set_style_text_color(nameLbl, GUI_COLOR_TEXT_PRI, 0);
    lv_obj_clear_flag(nameLbl, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(nameLbl, LV_OBJ_FLAG_EVENT_BUBBLE);

    // Brightness slider with overlay (at bottom of card)
    lv_color_t zoneCol = ctemp_to_color(zoneCTemp[idx]);
    create_overlay_slider(card, "Bright", 0, 100, zoneBright[idx],
                          &zw[idx].sldBright, &zw[idx].lblBright, zoneCol);
    lv_obj_add_event_cb(zw[idx].sldBright, zone_bright_cb, LV_EVENT_VALUE_CHANGED, (void *)(intptr_t)idx);

    // Color temp slider with overlay (at bottom of card)
    create_overlay_slider(card, "Color", 2700, 6500, zoneCTemp[idx],
                          &zw[idx].sldCTemp, &zw[idx].lblCTemp, zoneCol);
    lv_obj_add_event_cb(zw[idx].sldCTemp, zone_ctemp_cb, LV_EVENT_VALUE_CHANGED, (void *)(intptr_t)idx);

    // Active border
    update_zone_visuals(idx);
    zw[idx].card = card;
    return card;
}

static lv_obj_t *create_zone_grid(lv_obj_t *parent) {
    static lv_coord_t col_dsc[] = {LV_GRID_FR(1), LV_GRID_FR(1), LV_GRID_TEMPLATE_LAST};
    static lv_coord_t row_dsc[] = {LV_GRID_FR(1), LV_GRID_FR(1), LV_GRID_TEMPLATE_LAST};

    lv_obj_t *grid = lv_obj_create(parent);
    lv_obj_set_size(grid, LV_PCT(100), LV_PCT(100));
    lv_obj_set_flex_grow(grid, 1);
    lv_obj_set_style_bg_opa(grid, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(grid, 0, 0);
    lv_obj_set_style_pad_all(grid, 0, 0);
    lv_obj_clear_flag(grid, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_grid_dsc_array(grid, col_dsc, row_dsc);
    lv_obj_set_style_pad_gap(grid, GUI_PAD_SM, 0);

    // Grid layout: top row L→R (Ceiling, Counter), bottom row swapped
    // so Under Bed (idx 3) is bottom-left and Drawers (idx 2) is bottom-right
    static const int colMap[NUM_ZONES] = {0, 1, 1, 0};
    for (int i = 0; i < NUM_ZONES; i++) {
        lv_obj_t *card = create_zone_card(grid, i);
        lv_obj_set_grid_cell(card,
            LV_GRID_ALIGN_STRETCH, colMap[i], 1,
            LV_GRID_ALIGN_STRETCH, i / 2, 1);
    }

    return grid;
}

// ---------------------------------------------------------------------------
//  Save Preset modal
// ---------------------------------------------------------------------------

static lv_obj_t *_saveModal = nullptr;

/** @brief Close the save-preset modal. */
static void save_modal_close_cb(lv_event_t *e) {
    if (_saveModal) {
        lv_obj_del(_saveModal);
        _saveModal = nullptr;
    }
}

/** @brief Slot button pressed — save current light state to that slot. */
static void save_slot_cb(lv_event_t *e) {
    int slot = (int)(intptr_t)lv_event_get_user_data(e);
    if (slot < 0 || slot >= PRESET_COUNT) return;

    LightZonePreset zones[LIGHT_ZONE_COUNT];
    gui_tab_lights_get_state(zones);

    char name[PRESET_NAME_LEN];
    snprintf(name, sizeof(name), "Preset %d", slot + 1);

    gui_presets_save_light((uint8_t)slot, name, zones);
    gui_tab_main_refresh_presets();

    // Close modal
    if (_saveModal) {
        lv_obj_del(_saveModal);
        _saveModal = nullptr;
    }
}

/** @brief Backdrop click — dismiss the modal. */
static void save_backdrop_cb(lv_event_t *e) {
    lv_obj_t *target = lv_event_get_target(e);
    lv_obj_t *current = lv_event_get_current_target(e);
    // Only close if the backdrop itself was clicked (not a child)
    if (target == current) save_modal_close_cb(e);
}

/** @brief Open a modal with 2x2 grid of all 4 slots for saving light preset. */
static void save_preset_cb(lv_event_t *e) {
    if (_saveModal) return;  // already open

    // Semi-transparent backdrop covering the whole screen
    _saveModal = lv_obj_create(lv_scr_act());
    lv_obj_set_size(_saveModal, 800, 480);
    lv_obj_set_pos(_saveModal, 0, 0);
    lv_obj_set_style_bg_color(_saveModal, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(_saveModal, LV_OPA_50, 0);
    lv_obj_set_style_border_width(_saveModal, 0, 0);
    lv_obj_set_style_radius(_saveModal, 0, 0);
    lv_obj_set_style_pad_all(_saveModal, 0, 0);
    lv_obj_clear_flag(_saveModal, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(_saveModal, save_backdrop_cb, LV_EVENT_CLICKED, nullptr);

    // Dialog box
    lv_obj_t *dlg = lv_obj_create(_saveModal);
    lv_obj_set_size(dlg, 480, 340);
    lv_obj_center(dlg);
    lv_obj_set_style_bg_color(dlg, GUI_COLOR_PANEL, 0);
    lv_obj_set_style_bg_opa(dlg, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(dlg, GUI_RADIUS, 0);
    lv_obj_set_style_border_color(dlg, GUI_COLOR_BLUE, 0);
    lv_obj_set_style_border_width(dlg, 2, 0);
    lv_obj_set_style_pad_all(dlg, GUI_PAD_LG, 0);
    lv_obj_clear_flag(dlg, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flex_flow(dlg, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_gap(dlg, GUI_PAD_MD, 0);

    // Title
    lv_obj_t *title = lv_label_create(dlg);
    lv_label_set_text(title, "Save Lighting Preset");
    lv_obj_set_style_text_font(title, GUI_FONT_LG, 0);
    lv_obj_set_style_text_color(title, GUI_COLOR_BLUE, 0);

    // 2x2 grid of slot buttons
    static lv_coord_t col_dsc[] = {LV_GRID_FR(1), LV_GRID_FR(1), LV_GRID_TEMPLATE_LAST};
    static lv_coord_t row_dsc[] = {LV_GRID_FR(1), LV_GRID_FR(1), LV_GRID_TEMPLATE_LAST};

    lv_obj_t *grid = lv_obj_create(dlg);
    lv_obj_set_size(grid, LV_PCT(100), LV_PCT(100));
    lv_obj_set_flex_grow(grid, 1);
    lv_obj_set_style_bg_opa(grid, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(grid, 0, 0);
    lv_obj_set_style_pad_all(grid, 0, 0);
    lv_obj_clear_flag(grid, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_grid_dsc_array(grid, col_dsc, row_dsc);
    lv_obj_set_style_pad_gap(grid, GUI_PAD_MD, 0);

    const LightPreset *presets = gui_presets_get_lights();
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
        lv_label_set_text(icon, presets[i].saved ? LV_SYMBOL_SAVE : LV_SYMBOL_PLUS);
        lv_obj_set_style_text_font(icon, GUI_FONT_XL, 0);
        lv_obj_set_style_text_color(icon, presets[i].saved ? GUI_COLOR_TEXT_PRI : GUI_COLOR_TEXT_MUTED, 0);

        // Label
        lv_obj_t *lbl = lv_label_create(btn);
        char label[24];
        if (presets[i].saved) {
            snprintf(label, sizeof(label), "%s", presets[i].name);
        } else {
            snprintf(label, sizeof(label), "Slot %d", i + 1);
        }
        lv_label_set_text(lbl, label);
        lv_obj_set_style_text_font(lbl, GUI_FONT_MD, 0);
        lv_obj_set_style_text_color(lbl, presets[i].saved ? GUI_COLOR_TEXT_SEC : GUI_COLOR_TEXT_MUTED, 0);

        lv_obj_add_event_cb(btn, save_slot_cb, LV_EVENT_CLICKED, (void *)(intptr_t)i);
    }
}

// ---------------------------------------------------------------------------
//  Public API
// ---------------------------------------------------------------------------

lv_obj_t *gui_tab_lights_create(lv_obj_t *parent) {
    lv_obj_t *cont = lv_obj_create(parent);
    lv_obj_set_size(cont, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_opa(cont, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(cont, 0, 0);
    lv_obj_set_style_pad_all(cont, GUI_PAD_SM, 0);
    lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_gap(cont, GUI_PAD_SM, 0);
    lv_obj_clear_flag(cont, LV_OBJ_FLAG_SCROLLABLE);

    create_master_panel(cont);
    create_zone_grid(cont);

    return cont;
}

void gui_tab_lights_set_zone(uint8_t zone, bool on, uint8_t brightness, uint16_t colorTemp) {
    if (zone >= NUM_ZONES) return;
    zoneOn[zone] = on;
    zoneBright[zone] = brightness;
    zoneCTemp[zone] = colorTemp;

    if (zw[zone].sldBright) lv_slider_set_value(zw[zone].sldBright, brightness, LV_ANIM_ON);
    if (zw[zone].lblBright) {
        char buf[8]; snprintf(buf, sizeof(buf), "%d", brightness);
        lv_label_set_text(zw[zone].lblBright, buf);
    }
    if (zw[zone].sldCTemp) lv_slider_set_value(zw[zone].sldCTemp, colorTemp, LV_ANIM_ON);
    if (zw[zone].lblCTemp) {
        char buf[8]; snprintf(buf, sizeof(buf), "%d", colorTemp);
        lv_label_set_text(zw[zone].lblCTemp, buf);
    }
    update_zone_visuals(zone);
    sync_master_controls();
}

void gui_tab_lights_get_state(LightZonePreset out[LIGHT_ZONE_COUNT]) {
    for (int i = 0; i < LIGHT_ZONE_COUNT; i++) {
        out[i].on         = zoneOn[i];
        out[i].brightness = zoneBright[i];
        out[i].colorTempK = zoneCTemp[i];
    }
}

void gui_tab_lights_open_save_modal() {
    save_preset_cb(nullptr);
}
