/**
 * @file gui_tab_power.cpp
 * @brief Power tab — horizontal bar graphs for input and output wattage.
 *
 * Two sections stacked vertically:
 *   Inputs:  Solar (orange), AC Charger (green)
 *   Outputs: AC Inverter (red), 12V DC Load (blue)
 *
 * Each bar shows a gradient-filled bar with the current watt value on the right.
 * The max range is auto-scaled but defaults to 500 W.
 */
#include "gui_tab_power.h"
#include "gui_theme.h"
#include <cstdio>

// ---------------------------------------------------------------------------
//  Constants
// ---------------------------------------------------------------------------

/// Default max wattage for bar scaling
#define DEFAULT_MAX_W 500

// ---------------------------------------------------------------------------
//  Static handles
// ---------------------------------------------------------------------------

// Input bars
static lv_obj_t *_barSolar    = nullptr;
static lv_obj_t *_lblSolar    = nullptr;
static lv_obj_t *_barAC       = nullptr;
static lv_obj_t *_lblAC       = nullptr;

// Output bars
static lv_obj_t *_barInverter = nullptr;
static lv_obj_t *_lblInverter = nullptr;
static lv_obj_t *_barDC       = nullptr;
static lv_obj_t *_lblDC       = nullptr;

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
 * @brief Create a bar row: [label] [===bar===] [value]
 */
static void create_bar_row(lv_obj_t *parent, const char *name,
                            lv_color_t barColor,
                            lv_obj_t **barOut, lv_obj_t **lblOut) {
    lv_obj_t *row = lv_obj_create(parent);
    lv_obj_set_size(row, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(row, 0, 0);
    lv_obj_set_style_pad_all(row, 0, 0);
    lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(row, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_gap(row, GUI_PAD_SM, 0);

    // Name label (fixed width)
    lv_obj_t *lbl = lv_label_create(row);
    lv_label_set_text(lbl, name);
    lv_obj_set_style_text_font(lbl, GUI_FONT_MD, 0);
    lv_obj_set_style_text_color(lbl, barColor, 0);
    lv_obj_set_width(lbl, 120);

    // Bar
    lv_obj_t *bar = lv_bar_create(row);
    lv_obj_set_flex_grow(bar, 1);
    lv_obj_set_height(bar, 28);
    lv_bar_set_range(bar, 0, DEFAULT_MAX_W);
    lv_bar_set_value(bar, 0, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(bar, GUI_COLOR_CARD_HOVER, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(bar, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_radius(bar, 4, LV_PART_MAIN);
    lv_obj_set_style_bg_color(bar, barColor, LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(bar, LV_OPA_COVER, LV_PART_INDICATOR);
    lv_obj_set_style_radius(bar, 4, LV_PART_INDICATOR);
    *barOut = bar;

    // Value label (right side)
    lv_obj_t *val = lv_label_create(row);
    lv_label_set_text(val, "0W");
    lv_obj_set_style_text_font(val, GUI_FONT_LG, 0);
    lv_obj_set_style_text_color(val, GUI_COLOR_TEXT_PRI, 0);
    lv_obj_set_width(val, 80);
    lv_obj_set_style_text_align(val, LV_TEXT_ALIGN_RIGHT, 0);
    *lblOut = val;
}

static lv_obj_t *create_section(lv_obj_t *parent, const char *title,
                                 const char *name1, lv_color_t col1,
                                 lv_obj_t **bar1, lv_obj_t **lbl1,
                                 const char *name2, lv_color_t col2,
                                 lv_obj_t **bar2, lv_obj_t **lbl2) {
    lv_obj_t *card = lv_obj_create(parent);
    lv_obj_set_width(card, LV_PCT(100));
    lv_obj_set_flex_grow(card, 1);
    style_card(card);
    lv_obj_set_flex_flow(card, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_gap(card, GUI_PAD_MD, 0);

    lv_obj_t *hdr = lv_label_create(card);
    lv_label_set_text(hdr, title);
    lv_obj_set_style_text_font(hdr, GUI_FONT_LG, 0);
    lv_obj_set_style_text_color(hdr, GUI_COLOR_TEXT_PRI, 0);

    create_bar_row(card, name1, col1, bar1, lbl1);
    create_bar_row(card, name2, col2, bar2, lbl2);

    return card;
}

// ---------------------------------------------------------------------------
//  Public API
// ---------------------------------------------------------------------------

lv_obj_t *gui_tab_power_create(lv_obj_t *parent) {
    lv_obj_t *cont = lv_obj_create(parent);
    lv_obj_set_size(cont, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_opa(cont, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(cont, 0, 0);
    lv_obj_set_style_pad_all(cont, GUI_PAD_MD, 0);
    lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_gap(cont, GUI_PAD_MD, 0);
    lv_obj_clear_flag(cont, LV_OBJ_FLAG_SCROLLABLE);

    create_section(cont, "Inputs (W)",
                   "Solar",      GUI_COLOR_SOLAR,      &_barSolar,    &_lblSolar,
                   "AC Charger", GUI_COLOR_AC_CHARGER,  &_barAC,       &_lblAC);

    create_section(cont, "Outputs (W)",
                   "AC Inverter", GUI_COLOR_INVERTER,   &_barInverter, &_lblInverter,
                   "12V DC Load", GUI_COLOR_DC_LOAD,    &_barDC,       &_lblDC);

    return cont;
}

/**
 * @brief Update a single bar + label.
 */
static void update_bar(lv_obj_t *bar, lv_obj_t *lbl, float watts) {
    if (!bar || !lbl) return;
    int w = (int)(watts + 0.5f);
    if (w < 0) w = 0;
    lv_bar_set_value(bar, w, LV_ANIM_ON);
    char buf[12];
    snprintf(buf, sizeof(buf), "%dW", w);
    lv_label_set_text(lbl, buf);
}

void gui_tab_power_set_data(float solarW, float acChargerW,
                            float inverterW, float dcLoadW) {
    update_bar(_barSolar,    _lblSolar,    solarW);
    update_bar(_barAC,       _lblAC,       acChargerW);
    update_bar(_barInverter, _lblInverter, inverterW);
    update_bar(_barDC,       _lblDC,       dcLoadW);
}
