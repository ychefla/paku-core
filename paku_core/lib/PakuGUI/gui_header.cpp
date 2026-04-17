/**
 * @file gui_header.cpp
 * @brief Header bar implementation — WiFi/MQTT/BLE indicators, clock, heater badge.
 */
#include "gui_header.h"
#include "gui_theme.h"
#include <cstdio>

// Static widget handles
static lv_obj_t *_header    = nullptr;
static lv_obj_t *_lblWifi   = nullptr;
static lv_obj_t *_lblMqtt   = nullptr;
static lv_obj_t *_lblBle    = nullptr;
static lv_obj_t *_lblTime   = nullptr;
static lv_obj_t *_lblHeater = nullptr;

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

    // --- Left group: status indicators ---
    lv_obj_t *leftGrp = lv_obj_create(_header);
    lv_obj_set_size(leftGrp, LV_SIZE_CONTENT, LV_PCT(100));
    lv_obj_set_style_bg_opa(leftGrp, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(leftGrp, 0, 0);
    lv_obj_set_style_pad_all(leftGrp, 0, 0);
    lv_obj_clear_flag(leftGrp, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flex_flow(leftGrp, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(leftGrp, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(leftGrp, 12, 0);

    // WiFi badge
    _lblWifi = lv_label_create(leftGrp);
    lv_label_set_text(_lblWifi, LV_SYMBOL_WIFI " --");
    lv_obj_set_style_text_font(_lblWifi, GUI_FONT_SM, 0);
    lv_obj_set_style_text_color(_lblWifi, GUI_COLOR_TEXT_MUTED, 0);

    // MQTT badge
    _lblMqtt = lv_label_create(leftGrp);
    lv_label_set_text(_lblMqtt, "MQTT");
    lv_obj_set_style_text_font(_lblMqtt, GUI_FONT_SM, 0);
    lv_obj_set_style_text_color(_lblMqtt, GUI_COLOR_TEXT_MUTED, 0);

    // BLE badge
    _lblBle = lv_label_create(leftGrp);
    lv_label_set_text(_lblBle, LV_SYMBOL_BLUETOOTH " BLE");
    lv_obj_set_style_text_font(_lblBle, GUI_FONT_SM, 0);
    lv_obj_set_style_text_color(_lblBle, GUI_COLOR_TEXT_MUTED, 0);

    // --- Centre: clock ---
    _lblTime = lv_label_create(_header);
    lv_label_set_text(_lblTime, "--:--");
    lv_obj_set_style_text_font(_lblTime, GUI_FONT_LG, 0);
    lv_obj_set_style_text_color(_lblTime, GUI_COLOR_TEXT_PRI, 0);

    // --- Right group: heater status ---
    _lblHeater = lv_label_create(_header);
    lv_label_set_text(_lblHeater, "");
    lv_obj_set_style_text_font(_lblHeater, GUI_FONT_SM, 0);
    lv_obj_set_style_text_color(_lblHeater, GUI_COLOR_TEXT_SEC, 0);

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
        case 1:  txt = "HTR START";    col = GUI_COLOR_YELLOW; break;
        case 2:  txt = "HTR RUN";      col = GUI_COLOR_GREEN;  break;
        case 3:  txt = "HTR COOL";     col = GUI_COLOR_BLUE;   break;
        default: txt = "HTR ERR";      col = GUI_COLOR_RED;    break;
    }
    lv_label_set_text(_lblHeater, txt);
    lv_obj_set_style_text_color(_lblHeater, col, 0);
}
