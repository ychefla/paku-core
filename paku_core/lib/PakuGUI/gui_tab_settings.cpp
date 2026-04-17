/**
 * @file gui_tab_settings.cpp
 * @brief Settings tab — display brightness, sleep timer, network status,
 *        firmware info, and system actions.
 */
#include "gui_tab_settings.h"
#include "gui_theme.h"
#include "gui_events.h"
#include <cstdio>

// ---------------------------------------------------------------------------
//  Static handles
// ---------------------------------------------------------------------------

static lv_obj_t *_sldBright  = nullptr;
static lv_obj_t *_lblBright  = nullptr;
static lv_obj_t *_ddSleep    = nullptr;
static lv_obj_t *_lblVersion = nullptr;

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
 * @brief Create a settings row with label on left and control on right.
 */
static lv_obj_t *make_row(lv_obj_t *parent) {
    lv_obj_t *row = lv_obj_create(parent);
    lv_obj_set_size(row, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(row, 0, 0);
    lv_obj_set_style_pad_all(row, 0, 0);
    lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    return row;
}

// ---------------------------------------------------------------------------
//  Callbacks
// ---------------------------------------------------------------------------

static void bright_cb(lv_event_t *e) {
    int val = lv_slider_get_value(_sldBright);
    char buf[8]; snprintf(buf, sizeof(buf), "%d%%", val);
    lv_label_set_text(_lblBright, buf);
    if (_cb_backlight) _cb_backlight((uint8_t)val);
}

static void restart_cb(lv_event_t *e) {
    if (_cb_restart) _cb_restart();
}

// ---------------------------------------------------------------------------
//  Public API
// ---------------------------------------------------------------------------

lv_obj_t *gui_tab_settings_create(lv_obj_t *parent) {
    lv_obj_t *cont = lv_obj_create(parent);
    lv_obj_set_size(cont, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_opa(cont, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(cont, 0, 0);
    lv_obj_set_style_pad_all(cont, GUI_PAD_MD, 0);
    lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_gap(cont, GUI_PAD_MD, 0);
    lv_obj_clear_flag(cont, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *card = lv_obj_create(cont);
    lv_obj_set_size(card, LV_PCT(100), LV_SIZE_CONTENT);
    style_card(card);
    lv_obj_set_flex_flow(card, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_gap(card, GUI_PAD_MD, 0);

    // Title
    lv_obj_t *title = lv_label_create(card);
    lv_label_set_text(title, LV_SYMBOL_SETTINGS " Settings");
    lv_obj_set_style_text_font(title, GUI_FONT_LG, 0);
    lv_obj_set_style_text_color(title, GUI_COLOR_TEXT_PRI, 0);

    // --- Display Brightness ---
    {
        lv_obj_t *row = make_row(card);
        lv_obj_t *lbl = lv_label_create(row);
        lv_label_set_text(lbl, "Display Brightness");
        lv_obj_set_style_text_font(lbl, GUI_FONT_MD, 0);
        lv_obj_set_style_text_color(lbl, GUI_COLOR_TEXT_SEC, 0);

        _lblBright = lv_label_create(row);
        lv_label_set_text(_lblBright, "100%");
        lv_obj_set_style_text_font(_lblBright, GUI_FONT_MD, 0);
        lv_obj_set_style_text_color(_lblBright, GUI_COLOR_TEXT_PRI, 0);
    }
    _sldBright = lv_slider_create(card);
    lv_slider_set_range(_sldBright, 10, 100);
    lv_slider_set_value(_sldBright, 100, LV_ANIM_OFF);
    lv_obj_set_width(_sldBright, LV_PCT(100));
    lv_obj_set_style_bg_color(_sldBright, GUI_COLOR_CARD_HOVER, LV_PART_MAIN);
    lv_obj_set_style_bg_color(_sldBright, GUI_COLOR_ACCENT, LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(_sldBright, GUI_COLOR_ACCENT, LV_PART_KNOB);
    lv_obj_add_event_cb(_sldBright, bright_cb, LV_EVENT_VALUE_CHANGED, nullptr);

    // --- Sleep Timer ---
    {
        lv_obj_t *row = make_row(card);
        lv_obj_t *lbl = lv_label_create(row);
        lv_label_set_text(lbl, "Sleep Timer");
        lv_obj_set_style_text_font(lbl, GUI_FONT_MD, 0);
        lv_obj_set_style_text_color(lbl, GUI_COLOR_TEXT_SEC, 0);

        _ddSleep = lv_dropdown_create(row);
        lv_dropdown_set_options(_ddSleep, "Never\n30 sec\n1 min\n5 min\n10 min");
        lv_obj_set_width(_ddSleep, 140);
        lv_obj_set_style_bg_color(_ddSleep, GUI_COLOR_CARD_HOVER, 0);
        lv_obj_set_style_text_font(_ddSleep, GUI_FONT_SM, 0);
    }

    // --- Firmware Version ---
    {
        lv_obj_t *row = make_row(card);
        lv_obj_t *lbl = lv_label_create(row);
        lv_label_set_text(lbl, "Firmware");
        lv_obj_set_style_text_font(lbl, GUI_FONT_MD, 0);
        lv_obj_set_style_text_color(lbl, GUI_COLOR_TEXT_SEC, 0);

        _lblVersion = lv_label_create(row);
        lv_label_set_text(_lblVersion, "---");
        lv_obj_set_style_text_font(_lblVersion, GUI_FONT_MD, 0);
        lv_obj_set_style_text_color(_lblVersion, GUI_COLOR_TEXT_PRI, 0);
    }

    // --- Restart button ---
    lv_obj_t *btnRestart = lv_btn_create(card);
    lv_obj_set_width(btnRestart, LV_PCT(100));
    lv_obj_set_height(btnRestart, GUI_TOUCH_MIN);
    lv_obj_set_style_bg_color(btnRestart, GUI_COLOR_RED, 0);
    lv_obj_set_style_radius(btnRestart, GUI_RADIUS, 0);
    lv_obj_t *rl = lv_label_create(btnRestart);
    lv_label_set_text(rl, LV_SYMBOL_REFRESH " Restart");
    lv_obj_set_style_text_font(rl, GUI_FONT_MD, 0);
    lv_obj_center(rl);
    lv_obj_add_event_cb(btnRestart, restart_cb, LV_EVENT_CLICKED, nullptr);

    return cont;
}

void gui_tab_settings_set_version(const char *ver) {
    if (_lblVersion && ver) {
        lv_label_set_text(_lblVersion, ver);
    }
}
