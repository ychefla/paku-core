/**
 * @file gui_header.cpp
 * @brief Header bar matching the HTML prototype layout:
 *        Left  [WiFi MQTT BLE Heater]
 *        Centre[OUT IN FRIDGE]
 *        Right [Battery PowerOff Clock]
 */
#include "gui_header.h"
#include "gui_theme.h"
#include "gui_events.h"
#include <cstdio>

// Static widget handles
static lv_obj_t *_header      = nullptr;
static lv_obj_t *_lblWifi     = nullptr;
static lv_obj_t *_lblMqtt     = nullptr;
static lv_obj_t *_lblBle      = nullptr;
static lv_obj_t *_lblHeater   = nullptr;
static lv_obj_t *_lblOutdoor  = nullptr;
static lv_obj_t *_lblIndoor   = nullptr;
static lv_obj_t *_lblFridge   = nullptr;
static lv_obj_t *_lblBattery  = nullptr;
static lv_obj_t *_lblTime     = nullptr;

// Helper: create an invisible flex-row container
static lv_obj_t *_make_group(lv_obj_t *parent, lv_flex_align_t align, lv_coord_t gap) {
    lv_obj_t *g = lv_obj_create(parent);
    lv_obj_set_size(g, LV_SIZE_CONTENT, LV_PCT(100));
    lv_obj_set_style_bg_opa(g, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(g, 0, 0);
    lv_obj_set_style_pad_all(g, 0, 0);
    lv_obj_clear_flag(g, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flex_flow(g, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(g, align, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(g, gap, 0);
    return g;
}

// Power-off button callback — turns screen off via backlight(0)
static void _power_off_cb(lv_event_t *e) {
    if (_cb_backlight) _cb_backlight(0);
}

lv_obj_t *gui_header_create(lv_obj_t *parent) {
    _header = lv_obj_create(parent);
    lv_obj_set_size(_header, 800, GUI_HEADER_H);
    lv_obj_align(_header, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_set_style_bg_color(_header, GUI_COLOR_PANEL, 0);
    lv_obj_set_style_bg_opa(_header, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(_header, 0, 0);
    lv_obj_set_style_radius(_header, 0, 0);
    lv_obj_set_style_pad_all(_header, 4, 0);
    lv_obj_clear_flag(_header, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flex_flow(_header, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(_header, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    // ---- Left group: WiFi, MQTT, BLE, Heater ----
    lv_obj_t *leftGrp = _make_group(_header, LV_FLEX_ALIGN_START, 6);

    _lblWifi = lv_label_create(leftGrp);
    lv_label_set_text(_lblWifi, LV_SYMBOL_WIFI " --");
    lv_obj_set_style_text_font(_lblWifi, GUI_FONT_MD, 0);
    lv_obj_set_style_text_color(_lblWifi, GUI_COLOR_TEXT_MUTED, 0);

    _lblMqtt = lv_label_create(leftGrp);
    lv_label_set_text(_lblMqtt, "MQTT");
    lv_obj_set_style_text_font(_lblMqtt, GUI_FONT_MD, 0);
    lv_obj_set_style_text_color(_lblMqtt, GUI_COLOR_TEXT_MUTED, 0);

    _lblBle = lv_label_create(leftGrp);
    lv_label_set_text(_lblBle, LV_SYMBOL_BLUETOOTH " BLE");
    lv_obj_set_style_text_font(_lblBle, GUI_FONT_MD, 0);
    lv_obj_set_style_text_color(_lblBle, GUI_COLOR_TEXT_MUTED, 0);

    _lblHeater = lv_label_create(leftGrp);
    lv_label_set_text(_lblHeater, "");
    lv_obj_set_style_text_font(_lblHeater, GUI_FONT_MD, 0);
    lv_obj_set_style_text_color(_lblHeater, GUI_COLOR_TEXT_SEC, 0);

    // ---- Centre group: sensor readouts (flex-grow so it shrinks, not right) ----
    lv_obj_t *centerGrp = _make_group(_header, LV_FLEX_ALIGN_CENTER, 10);
    lv_obj_set_flex_grow(centerGrp, 1);
    lv_obj_set_style_clip_corner(centerGrp, true, 0);

    _lblOutdoor = lv_label_create(centerGrp);
    lv_label_set_text(_lblOutdoor, "OUT: --");
    lv_obj_set_style_text_font(_lblOutdoor, GUI_FONT_MD, 0);
    lv_obj_set_style_text_color(_lblOutdoor, GUI_COLOR_TEXT_SEC, 0);

    _lblIndoor = lv_label_create(centerGrp);
    lv_label_set_text(_lblIndoor, "IN: --");
    lv_obj_set_style_text_font(_lblIndoor, GUI_FONT_MD, 0);
    lv_obj_set_style_text_color(_lblIndoor, GUI_COLOR_TEXT_SEC, 0);

    _lblFridge = lv_label_create(centerGrp);
    lv_label_set_text(_lblFridge, "FRIDGE: --");
    lv_obj_set_style_text_font(_lblFridge, GUI_FONT_MD, 0);
    lv_obj_set_style_text_color(_lblFridge, GUI_COLOR_TEXT_SEC, 0);

    // ---- Right group: battery, power-off, clock ----
    lv_obj_t *rightGrp = _make_group(_header, LV_FLEX_ALIGN_END, 6);
    lv_obj_set_style_min_width(rightGrp, 180, 0);

    _lblBattery = lv_label_create(rightGrp);
    lv_label_set_text(_lblBattery, LV_SYMBOL_BATTERY_EMPTY " --%");
    lv_obj_set_style_text_font(_lblBattery, GUI_FONT_MD, 0);
    lv_obj_set_style_text_color(_lblBattery, GUI_COLOR_TEXT_SEC, 0);

    // Power-off button (plain obj to avoid lv_btn theme min-width)
    lv_obj_t *btnPower = lv_obj_create(rightGrp);
    lv_obj_set_size(btnPower, 30, 30);
    lv_obj_set_style_bg_color(btnPower, GUI_COLOR_CARD, 0);
    lv_obj_set_style_bg_opa(btnPower, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(btnPower, 4, 0);
    lv_obj_set_style_border_width(btnPower, 1, 0);
    lv_obj_set_style_border_color(btnPower, lv_color_hex(0x555555), 0);
    lv_obj_set_style_shadow_width(btnPower, 0, 0);
    lv_obj_set_style_pad_all(btnPower, 0, 0);
    lv_obj_add_flag(btnPower, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(btnPower, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(btnPower, _power_off_cb, LV_EVENT_CLICKED, nullptr);
    lv_obj_t *icoP = lv_label_create(btnPower);
    lv_label_set_text(icoP, LV_SYMBOL_POWER);
    lv_obj_set_style_text_color(icoP, GUI_COLOR_TEXT_PRI, 0);
    lv_obj_center(icoP);

    _lblTime = lv_label_create(rightGrp);
    lv_label_set_text(_lblTime, "--:--");
    lv_obj_set_style_text_font(_lblTime, GUI_FONT_LG, 0);
    lv_obj_set_style_text_color(_lblTime, GUI_COLOR_TEXT_PRI, 0);

    return _header;
}

// ---- Updaters ----

void gui_header_set_wifi(bool connected, int rssi) {
    if (!_lblWifi) return;
    if (connected) {
        char buf[20];
        snprintf(buf, sizeof(buf), LV_SYMBOL_WIFI " %d", rssi);
        lv_label_set_text(_lblWifi, buf);
        lv_obj_set_style_text_color(_lblWifi, GUI_COLOR_GREEN, 0);
    } else {
        lv_label_set_text(_lblWifi, LV_SYMBOL_WIFI " --");
        lv_obj_set_style_text_color(_lblWifi, GUI_COLOR_RED, 0);
    }
}

void gui_header_set_mqtt(bool connected) {
    if (!_lblMqtt) return;
    lv_obj_set_style_text_color(_lblMqtt,
        connected ? GUI_COLOR_GREEN : GUI_COLOR_RED, 0);
    lv_label_set_text(_lblMqtt, connected ? "MQTT" : "MQTT" LV_SYMBOL_CLOSE);
}

void gui_header_set_ble(bool active) {
    if (!_lblBle) return;
    lv_obj_set_style_text_color(_lblBle,
        active ? GUI_COLOR_BLUE : GUI_COLOR_TEXT_MUTED, 0);
}

void gui_header_set_time(const char *timeStr) {
    if (!_lblTime) return;
    lv_label_set_text(_lblTime, timeStr);
}

void gui_header_set_heater(int state) {
    if (!_lblHeater) return;
    const char *txt = "";
    lv_color_t col = GUI_COLOR_TEXT_SEC;
    switch (state) {
        case 0:  txt = "";             break;
        case 1:  txt = LV_SYMBOL_CHARGE " START"; col = GUI_COLOR_YELLOW; break;
        case 2:  txt = LV_SYMBOL_CHARGE " ON";    col = GUI_COLOR_ACCENT; break;
        case 3:  txt = LV_SYMBOL_CHARGE " COOL";  col = GUI_COLOR_BLUE;   break;
        default: txt = LV_SYMBOL_CHARGE " ERR";   col = GUI_COLOR_RED;    break;
    }
    lv_label_set_text(_lblHeater, txt);
    lv_obj_set_style_text_color(_lblHeater, col, 0);
}

void gui_header_set_outdoor(float tempC, float humidity) {
    if (!_lblOutdoor) return;
    char buf[32];
    snprintf(buf, sizeof(buf), "OUT: %.1f\u00b0C / %.0f%%", tempC, humidity);
    lv_label_set_text(_lblOutdoor, buf);
}

void gui_header_set_indoor(float tempC, float humidity) {
    if (!_lblIndoor) return;
    char buf[32];
    snprintf(buf, sizeof(buf), "IN: %.1f\u00b0C / %.0f%%", tempC, humidity);
    lv_label_set_text(_lblIndoor, buf);
}

void gui_header_set_fridge(float tempC) {
    if (!_lblFridge) return;
    char buf[24];
    snprintf(buf, sizeof(buf), "FRIDGE: %.1f\u00b0C", tempC);
    lv_label_set_text(_lblFridge, buf);
}

void gui_header_set_battery(int soc) {
    if (!_lblBattery) return;
    char buf[20];
    const char *icon;
    if      (soc >= 75) icon = LV_SYMBOL_BATTERY_FULL;
    else if (soc >= 50) icon = LV_SYMBOL_BATTERY_3;
    else if (soc >= 25) icon = LV_SYMBOL_BATTERY_2;
    else if (soc >= 5)  icon = LV_SYMBOL_BATTERY_1;
    else                icon = LV_SYMBOL_BATTERY_EMPTY;
    snprintf(buf, sizeof(buf), "%s %d%%", icon, soc);
    lv_label_set_text(_lblBattery, buf);

    lv_color_t col = (soc < 15) ? GUI_COLOR_RED :
                     (soc < 30) ? GUI_COLOR_YELLOW : GUI_COLOR_TEXT_SEC;
    lv_obj_set_style_text_color(_lblBattery, col, 0);
}
