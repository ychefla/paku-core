/**
 * @file gui_tab_climate.cpp
 * @brief Climate tab — Roof Fan (MaxxFan) and Diesel Heater controls.
 *
 * Two-column layout:
 *   Left:  Fan direction, lid toggle, speed slider (0-100%)
 *   Right: Heater power button, target temp display, +/- temp control, full-power override
 */
#include "gui_tab_climate.h"
#include "gui_theme.h"
#include "gui_events.h"
#include "gui_presets.h"
#include "gui_tab_main.h"
#include "paku_gui.h"   // HeaterMode enum
#include <cstdio>

// ---------------------------------------------------------------------------
//  Static widget handles
// ---------------------------------------------------------------------------

// Fan panel
static lv_obj_t *_btnFanPower = nullptr;
static lv_obj_t *_btnAirDir   = nullptr;
static lv_obj_t *_btnLid      = nullptr;
static lv_obj_t *_sldSpeed    = nullptr;
static lv_obj_t *_lblSpeedName = nullptr;
static lv_obj_t *_lblSpeedVal  = nullptr;

// Heater panel
static lv_obj_t *_btnPower    = nullptr;
static lv_obj_t *_btnModePwr  = nullptr;
static lv_obj_t *_btnModeThm  = nullptr;
static lv_obj_t *_sldTarget   = nullptr;
static lv_obj_t *_sldPower    = nullptr;
static lv_obj_t *_lblActual   = nullptr;
static lv_obj_t *_sldHeaterFan = nullptr;

// Overlay labels for heater sliders
static lv_obj_t *_lblPowerName = nullptr;
static lv_obj_t *_lblPowerVal  = nullptr;
static lv_obj_t *_lblTempName  = nullptr;
static lv_obj_t *_lblTempVal   = nullptr;
static lv_obj_t *_lblHFanName  = nullptr;
static lv_obj_t *_lblHFanVal   = nullptr;

// Wrapper containers for power/temp slider (only one visible at a time)
static lv_obj_t *_wrapPower   = nullptr;
static lv_obj_t *_wrapTemp    = nullptr;

static bool _heaterOn     = false;
static int  _targetTemp   = 21;
static int  _powerLevel   = 5;
static bool _fanDirIn     = true;
static bool _lidOpen      = true;
static bool _fanPower     = false;
static HeaterMode _heaterMode = HEATER_MODE_POWER;

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

static lv_obj_t *make_toggle_btn(lv_obj_t *parent, const char *text, bool active, lv_color_t activeCol) {
    lv_obj_t *btn = lv_btn_create(parent);
    lv_obj_set_height(btn, GUI_TOUCH_MIN);
    lv_obj_set_flex_grow(btn, 1);
    lv_obj_set_style_radius(btn, GUI_RADIUS, 0);
    lv_obj_set_style_bg_color(btn, active ? activeCol : GUI_COLOR_CARD_HOVER, 0);
    lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, 0);

    lv_obj_t *lbl = lv_label_create(btn);
    lv_label_set_text(lbl, text);
    lv_obj_set_style_text_font(lbl, GUI_FONT_XL, 0);
    lv_obj_center(lbl);
    return btn;
}

// ---------------------------------------------------------------------------
//  Callbacks
// ---------------------------------------------------------------------------

static void fan_power_cb(lv_event_t *e) {
    _fanPower = !_fanPower;
    lv_obj_t *lbl = lv_obj_get_child(_btnFanPower, 0);
    lv_label_set_text(lbl, _fanPower ? LV_SYMBOL_POWER " ON" : LV_SYMBOL_POWER " OFF");
    lv_obj_set_style_bg_color(_btnFanPower, _fanPower ? GUI_COLOR_BLUE : GUI_COLOR_CARD_HOVER, 0);
    if (_cb_fan) {
        int spd = _sldSpeed ? lv_slider_get_value(_sldSpeed) : 0;
        _cb_fan(_fanPower, (uint8_t)spd, _fanDirIn, _lidOpen);
    }
}

static void fan_dir_cb(lv_event_t *e) {
    _fanDirIn = !_fanDirIn;
    lv_obj_t *lbl = lv_obj_get_child(_btnAirDir, 0);
    lv_label_set_text(lbl, _fanDirIn ? "Air In " LV_SYMBOL_DOWN : "Air Out " LV_SYMBOL_UP);
    lv_obj_set_style_bg_color(_btnAirDir, GUI_COLOR_BLUE, 0);
    if (_cb_fan) {
        int spd = _sldSpeed ? lv_slider_get_value(_sldSpeed) : 0;
        _cb_fan(_fanPower, (uint8_t)spd, _fanDirIn, _lidOpen);
    }
}

static void lid_cb(lv_event_t *e) {
    _lidOpen = !_lidOpen;
    lv_obj_t *lbl = lv_obj_get_child(_btnLid, 0);
    lv_label_set_text(lbl, _lidOpen ? "Lid Open" : "Lid Closed");
    lv_obj_set_style_bg_color(_btnLid, _lidOpen ? GUI_COLOR_BLUE : GUI_COLOR_CARD_HOVER, 0);
    if (_cb_fan) {
        int spd = _sldSpeed ? lv_slider_get_value(_sldSpeed) : 0;
        _cb_fan(_fanPower, (uint8_t)spd, _fanDirIn, _lidOpen);
    }
}

static void speed_cb(lv_event_t *e) {
    int val = lv_slider_get_value(_sldSpeed);
    char buf[8];
    snprintf(buf, sizeof(buf), "%d%%", val);
    if (_lblSpeedVal) lv_label_set_text(_lblSpeedVal, buf);
    if (_cb_fan) _cb_fan(_fanPower, (uint8_t)val, _fanDirIn, _lidOpen);
}

static void power_cb(lv_event_t *e) {
    _heaterOn = !_heaterOn;
    lv_obj_set_style_bg_color(_btnPower,
        _heaterOn ? GUI_COLOR_ACCENT : GUI_COLOR_CARD_HOVER, 0);
    lv_obj_t *lbl = lv_obj_get_child(_btnPower, 0);
    if (lbl) lv_label_set_text(lbl, _heaterOn ? LV_SYMBOL_POWER " ON" : LV_SYMBOL_POWER " OFF");
    if (_cb_heater) {
        _cb_heater(_heaterOn, _heaterMode, (uint8_t)_powerLevel, (uint8_t)_targetTemp);
    }
}

/** @brief Helper: notify heater of settings change while running. */
static void _notify_heater() {
    if (_heaterOn && _cb_heater) {
        _cb_heater(true, _heaterMode, (uint8_t)_powerLevel, (uint8_t)_targetTemp);
    }
}

/** @brief Switch visible heater slider based on active mode. */
static void _update_mode_panels() {
    if (_wrapPower) {
        if (_heaterMode == HEATER_MODE_POWER)
            lv_obj_clear_flag(_wrapPower, LV_OBJ_FLAG_HIDDEN);
        else
            lv_obj_add_flag(_wrapPower, LV_OBJ_FLAG_HIDDEN);
    }
    if (_wrapTemp) {
        if (_heaterMode == HEATER_MODE_THERMOSTAT)
            lv_obj_clear_flag(_wrapTemp, LV_OBJ_FLAG_HIDDEN);
        else
            lv_obj_add_flag(_wrapTemp, LV_OBJ_FLAG_HIDDEN);
    }
    if (_btnModePwr) lv_obj_set_style_bg_color(_btnModePwr,
        _heaterMode == HEATER_MODE_POWER ? GUI_COLOR_ACCENT : GUI_COLOR_CARD_HOVER, 0);
    if (_btnModeThm) lv_obj_set_style_bg_color(_btnModeThm,
        _heaterMode == HEATER_MODE_THERMOSTAT ? GUI_COLOR_ACCENT : GUI_COLOR_CARD_HOVER, 0);
}

static void mode_power_cb(lv_event_t *e) {
    _heaterMode = HEATER_MODE_POWER;
    _update_mode_panels();
    _notify_heater();
}

static void mode_thermo_cb(lv_event_t *e) {
    _heaterMode = HEATER_MODE_THERMOSTAT;
    _update_mode_panels();
    _notify_heater();
}

static void power_slider_cb(lv_event_t *e) {
    _powerLevel = lv_slider_get_value(_sldPower);
    char buf[8];
    snprintf(buf, sizeof(buf), "%d", _powerLevel);
    if (_lblPowerVal) lv_label_set_text(_lblPowerVal, buf);
    _notify_heater();
}

static void temp_slider_cb(lv_event_t *e) {
    _targetTemp = lv_slider_get_value(_sldTarget);
    char buf[8];
    snprintf(buf, sizeof(buf), "%d\xC2\xB0""C", _targetTemp);
    if (_lblTempVal) lv_label_set_text(_lblTempVal, buf);
    _notify_heater();
}

static void heater_fan_slider_cb(lv_event_t *e) {
    // Heater fan speed — informational for now, not sent as separate command
    int val = lv_slider_get_value(_sldHeaterFan);
    char buf[8];
    snprintf(buf, sizeof(buf), "%d", val);
    if (_lblHFanVal) lv_label_set_text(_lblHFanVal, buf);
}

// ---------------------------------------------------------------------------
//  Fan panel
// ---------------------------------------------------------------------------

static lv_obj_t *create_fan_panel(lv_obj_t *parent) {
    lv_obj_t *card = lv_obj_create(parent);
    lv_obj_set_flex_grow(card, 1);
    lv_obj_set_height(card, LV_PCT(100));
    style_card(card);
    lv_obj_set_flex_flow(card, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_gap(card, GUI_PAD_SM, 0);

    // Title
    lv_obj_t *title = lv_label_create(card);
    lv_label_set_text(title, "MaxxFan");
    lv_obj_set_style_text_font(title, GUI_FONT_LG, 0);
    lv_obj_set_style_text_color(title, GUI_COLOR_BLUE, 0);

    // ---- Row 1: Power on/off (flex_grow=1) ----
    _btnFanPower = lv_btn_create(card);
    lv_obj_set_width(_btnFanPower, LV_PCT(100));
    lv_obj_set_flex_grow(_btnFanPower, 1);
    lv_obj_set_style_radius(_btnFanPower, GUI_RADIUS, 0);
    lv_obj_set_style_bg_color(_btnFanPower, GUI_COLOR_CARD_HOVER, 0);
    lv_obj_set_style_bg_opa(_btnFanPower, LV_OPA_COVER, 0);
    lv_obj_add_event_cb(_btnFanPower, fan_power_cb, LV_EVENT_CLICKED, nullptr);

    lv_obj_t *pwrLbl = lv_label_create(_btnFanPower);
    lv_label_set_text(pwrLbl, LV_SYMBOL_POWER " OFF");
    lv_obj_set_style_text_font(pwrLbl, GUI_FONT_XL, 0);
    lv_obj_center(pwrLbl);

    // ---- Row 2: Direction & Lid toggle (flex_grow=1) ----
    lv_obj_t *toggleRow = lv_obj_create(card);
    lv_obj_set_width(toggleRow, LV_PCT(100));
    lv_obj_set_flex_grow(toggleRow, 1);
    lv_obj_set_style_bg_opa(toggleRow, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(toggleRow, 0, 0);
    lv_obj_set_style_pad_all(toggleRow, 0, 0);
    lv_obj_clear_flag(toggleRow, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flex_flow(toggleRow, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_gap(toggleRow, GUI_PAD_SM, 0);

    _btnAirDir = make_toggle_btn(toggleRow, "Air In " LV_SYMBOL_DOWN, true, GUI_COLOR_BLUE);
    lv_obj_set_height(_btnAirDir, LV_PCT(100));
    lv_obj_add_event_cb(_btnAirDir, fan_dir_cb, LV_EVENT_CLICKED, nullptr);

    _btnLid = make_toggle_btn(toggleRow, "Lid Open", true, GUI_COLOR_BLUE);
    lv_obj_set_height(_btnLid, LV_PCT(100));
    lv_obj_add_event_cb(_btnLid, lid_cb, LV_EVENT_CLICKED, nullptr);

    // ---- Row 3: Fan speed slider (flex_grow=1) ----
    lv_obj_t *speedWrap = lv_obj_create(card);
    lv_obj_set_width(speedWrap, LV_PCT(100));
    lv_obj_set_flex_grow(speedWrap, 1);
    lv_obj_set_style_bg_opa(speedWrap, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(speedWrap, 0, 0);
    lv_obj_set_style_pad_all(speedWrap, 0, 0);
    lv_obj_clear_flag(speedWrap, LV_OBJ_FLAG_SCROLLABLE);

    _sldSpeed = lv_slider_create(speedWrap);
    lv_slider_set_range(_sldSpeed, 0, 100);
    lv_slider_set_value(_sldSpeed, 40, LV_ANIM_OFF);
    lv_obj_set_size(_sldSpeed, LV_PCT(100), LV_PCT(100));
    lv_obj_center(_sldSpeed);
    lv_obj_set_style_bg_color(_sldSpeed, GUI_COLOR_CARD_HOVER, LV_PART_MAIN);
    lv_obj_set_style_bg_color(_sldSpeed, GUI_COLOR_BLUE, LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(_sldSpeed, GUI_COLOR_BLUE, LV_PART_KNOB);
    lv_obj_set_style_radius(_sldSpeed, GUI_RADIUS, LV_PART_MAIN);
    lv_obj_set_style_radius(_sldSpeed, GUI_RADIUS, LV_PART_INDICATOR);
    lv_obj_add_event_cb(_sldSpeed, speed_cb, LV_EVENT_VALUE_CHANGED, nullptr);

    // Overlay: name label (left)
    _lblSpeedName = lv_label_create(speedWrap);
    lv_label_set_text(_lblSpeedName, "Fan Speed");
    lv_obj_set_style_text_font(_lblSpeedName, GUI_FONT_XL, 0);
    lv_obj_set_style_text_color(_lblSpeedName, lv_color_white(), 0);
    lv_obj_set_style_text_opa(_lblSpeedName, LV_OPA_COVER, 0);
    lv_obj_align(_lblSpeedName, LV_ALIGN_LEFT_MID, 12, 0);
    lv_obj_add_flag(_lblSpeedName, LV_OBJ_FLAG_FLOATING);
    lv_obj_clear_flag(_lblSpeedName, LV_OBJ_FLAG_CLICKABLE);

    // Overlay: value label (right)
    _lblSpeedVal = lv_label_create(speedWrap);
    lv_label_set_text(_lblSpeedVal, "40%");
    lv_obj_set_style_text_font(_lblSpeedVal, GUI_FONT_XL, 0);
    lv_obj_set_style_text_color(_lblSpeedVal, lv_color_white(), 0);
    lv_obj_set_style_text_opa(_lblSpeedVal, LV_OPA_COVER, 0);
    lv_obj_align(_lblSpeedVal, LV_ALIGN_RIGHT_MID, -12, 0);
    lv_obj_add_flag(_lblSpeedVal, LV_OBJ_FLAG_FLOATING);
    lv_obj_clear_flag(_lblSpeedVal, LV_OBJ_FLAG_CLICKABLE);

    return card;
}

// ---------------------------------------------------------------------------
//  Heater panel
// ---------------------------------------------------------------------------

static lv_obj_t *create_heater_panel(lv_obj_t *parent) {
    lv_obj_t *card = lv_obj_create(parent);
    lv_obj_set_flex_grow(card, 1);
    lv_obj_set_height(card, LV_PCT(100));
    style_card(card);
    lv_obj_set_flex_flow(card, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_gap(card, GUI_PAD_SM, 0);

    // Title
    lv_obj_t *title = lv_label_create(card);
    lv_label_set_text(title, "Diesel Heater");
    lv_obj_set_style_text_font(title, GUI_FONT_LG, 0);
    lv_obj_set_style_text_color(title, GUI_COLOR_ACCENT, 0);

    // ---- Row 1: Power ON/OFF button (flex_grow=1) ----
    _btnPower = lv_btn_create(card);
    lv_obj_set_width(_btnPower, LV_PCT(100));
    lv_obj_set_flex_grow(_btnPower, 1);
    lv_obj_set_style_radius(_btnPower, GUI_RADIUS, 0);
    lv_obj_set_style_bg_color(_btnPower, GUI_COLOR_CARD_HOVER, 0);
    lv_obj_set_style_bg_opa(_btnPower, LV_OPA_COVER, 0);
    lv_obj_add_event_cb(_btnPower, power_cb, LV_EVENT_CLICKED, nullptr);

    // Status text (left side)
    lv_obj_t *pwrLbl = lv_label_create(_btnPower);
    lv_label_set_text(pwrLbl, LV_SYMBOL_POWER " OFF");
    lv_obj_set_style_text_font(pwrLbl, GUI_FONT_XL, 0);
    lv_obj_align(pwrLbl, LV_ALIGN_LEFT_MID, 12, 0);
    lv_obj_add_flag(pwrLbl, LV_OBJ_FLAG_FLOATING);
    lv_obj_clear_flag(pwrLbl, LV_OBJ_FLAG_CLICKABLE);

    // Actual temperature (right side)
    _lblActual = lv_label_create(_btnPower);
    lv_label_set_text(_lblActual, "--\xC2\xB0""C");
    lv_obj_set_style_text_font(_lblActual, GUI_FONT_XL, 0);
    lv_obj_align(_lblActual, LV_ALIGN_RIGHT_MID, -12, 0);
    lv_obj_add_flag(_lblActual, LV_OBJ_FLAG_FLOATING);
    lv_obj_clear_flag(_lblActual, LV_OBJ_FLAG_CLICKABLE);

    // ---- Row 2: Mode toggle [Power] [Temp] (flex_grow=1) ----
    lv_obj_t *modeRow = lv_obj_create(card);
    lv_obj_set_width(modeRow, LV_PCT(100));
    lv_obj_set_flex_grow(modeRow, 1);
    lv_obj_set_style_bg_opa(modeRow, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(modeRow, 0, 0);
    lv_obj_set_style_pad_all(modeRow, 0, 0);
    lv_obj_clear_flag(modeRow, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flex_flow(modeRow, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_gap(modeRow, GUI_PAD_SM, 0);

    _btnModePwr = make_toggle_btn(modeRow, LV_SYMBOL_CHARGE " Power", true, GUI_COLOR_ACCENT);
    lv_obj_set_height(_btnModePwr, LV_PCT(100));
    lv_obj_add_event_cb(_btnModePwr, mode_power_cb, LV_EVENT_CLICKED, nullptr);

    _btnModeThm = make_toggle_btn(modeRow, LV_SYMBOL_HOME " Temp", false, GUI_COLOR_ACCENT);
    lv_obj_set_height(_btnModeThm, LV_PCT(100));
    lv_obj_add_event_cb(_btnModeThm, mode_thermo_cb, LV_EVENT_CLICKED, nullptr);

    // ---- Row 3: Power level slider (visible in Power mode) ----
    _wrapPower = lv_obj_create(card);
    lv_obj_set_width(_wrapPower, LV_PCT(100));
    lv_obj_set_flex_grow(_wrapPower, 1);
    lv_obj_set_style_bg_opa(_wrapPower, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(_wrapPower, 0, 0);
    lv_obj_set_style_pad_all(_wrapPower, 0, 0);
    lv_obj_clear_flag(_wrapPower, LV_OBJ_FLAG_SCROLLABLE);

    _sldPower = lv_slider_create(_wrapPower);
    lv_slider_set_range(_sldPower, 0, 9);
    lv_slider_set_value(_sldPower, 5, LV_ANIM_OFF);
    lv_obj_set_size(_sldPower, LV_PCT(100), LV_PCT(100));
    lv_obj_center(_sldPower);
    lv_obj_set_style_bg_color(_sldPower, GUI_COLOR_CARD_HOVER, LV_PART_MAIN);
    lv_obj_set_style_bg_color(_sldPower, GUI_COLOR_ACCENT, LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(_sldPower, GUI_COLOR_ACCENT, LV_PART_KNOB);
    lv_obj_set_style_radius(_sldPower, GUI_RADIUS, LV_PART_MAIN);
    lv_obj_set_style_radius(_sldPower, GUI_RADIUS, LV_PART_INDICATOR);
    lv_obj_add_event_cb(_sldPower, power_slider_cb, LV_EVENT_VALUE_CHANGED, nullptr);

    _lblPowerName = lv_label_create(_wrapPower);
    lv_label_set_text(_lblPowerName, "Power Level");
    lv_obj_set_style_text_font(_lblPowerName, GUI_FONT_XL, 0);
    lv_obj_set_style_text_color(_lblPowerName, lv_color_white(), 0);
    lv_obj_align(_lblPowerName, LV_ALIGN_LEFT_MID, 12, 0);
    lv_obj_add_flag(_lblPowerName, LV_OBJ_FLAG_FLOATING);
    lv_obj_clear_flag(_lblPowerName, LV_OBJ_FLAG_CLICKABLE);

    _lblPowerVal = lv_label_create(_wrapPower);
    lv_label_set_text(_lblPowerVal, "5");
    lv_obj_set_style_text_font(_lblPowerVal, GUI_FONT_XL, 0);
    lv_obj_set_style_text_color(_lblPowerVal, lv_color_white(), 0);
    lv_obj_align(_lblPowerVal, LV_ALIGN_RIGHT_MID, -12, 0);
    lv_obj_add_flag(_lblPowerVal, LV_OBJ_FLAG_FLOATING);
    lv_obj_clear_flag(_lblPowerVal, LV_OBJ_FLAG_CLICKABLE);

    // ---- Row 3 alt: Target temp slider (visible in Temp mode, starts hidden) ----
    _wrapTemp = lv_obj_create(card);
    lv_obj_set_width(_wrapTemp, LV_PCT(100));
    lv_obj_set_flex_grow(_wrapTemp, 1);
    lv_obj_set_style_bg_opa(_wrapTemp, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(_wrapTemp, 0, 0);
    lv_obj_set_style_pad_all(_wrapTemp, 0, 0);
    lv_obj_clear_flag(_wrapTemp, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(_wrapTemp, LV_OBJ_FLAG_HIDDEN);

    _sldTarget = lv_slider_create(_wrapTemp);
    lv_slider_set_range(_sldTarget, 10, 30);
    lv_slider_set_value(_sldTarget, 21, LV_ANIM_OFF);
    lv_obj_set_size(_sldTarget, LV_PCT(100), LV_PCT(100));
    lv_obj_center(_sldTarget);
    lv_obj_set_style_bg_color(_sldTarget, GUI_COLOR_CARD_HOVER, LV_PART_MAIN);
    lv_obj_set_style_bg_color(_sldTarget, GUI_COLOR_ACCENT, LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(_sldTarget, GUI_COLOR_ACCENT, LV_PART_KNOB);
    lv_obj_set_style_radius(_sldTarget, GUI_RADIUS, LV_PART_MAIN);
    lv_obj_set_style_radius(_sldTarget, GUI_RADIUS, LV_PART_INDICATOR);
    lv_obj_add_event_cb(_sldTarget, temp_slider_cb, LV_EVENT_VALUE_CHANGED, nullptr);

    _lblTempName = lv_label_create(_wrapTemp);
    lv_label_set_text(_lblTempName, "Target Temp");
    lv_obj_set_style_text_font(_lblTempName, GUI_FONT_XL, 0);
    lv_obj_set_style_text_color(_lblTempName, lv_color_white(), 0);
    lv_obj_align(_lblTempName, LV_ALIGN_LEFT_MID, 12, 0);
    lv_obj_add_flag(_lblTempName, LV_OBJ_FLAG_FLOATING);
    lv_obj_clear_flag(_lblTempName, LV_OBJ_FLAG_CLICKABLE);

    _lblTempVal = lv_label_create(_wrapTemp);
    lv_label_set_text(_lblTempVal, "21\xC2\xB0""C");
    lv_obj_set_style_text_font(_lblTempVal, GUI_FONT_XL, 0);
    lv_obj_set_style_text_color(_lblTempVal, lv_color_white(), 0);
    lv_obj_align(_lblTempVal, LV_ALIGN_RIGHT_MID, -12, 0);
    lv_obj_add_flag(_lblTempVal, LV_OBJ_FLAG_FLOATING);
    lv_obj_clear_flag(_lblTempVal, LV_OBJ_FLAG_CLICKABLE);

    // ---- Row 4: Fan speed slider (always visible) ----
    lv_obj_t *fanWrap = lv_obj_create(card);
    lv_obj_set_width(fanWrap, LV_PCT(100));
    lv_obj_set_flex_grow(fanWrap, 1);
    lv_obj_set_style_bg_opa(fanWrap, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(fanWrap, 0, 0);
    lv_obj_set_style_pad_all(fanWrap, 0, 0);
    lv_obj_clear_flag(fanWrap, LV_OBJ_FLAG_SCROLLABLE);

    _sldHeaterFan = lv_slider_create(fanWrap);
    lv_slider_set_range(_sldHeaterFan, 0, 5);
    lv_slider_set_value(_sldHeaterFan, 0, LV_ANIM_OFF);
    lv_obj_set_size(_sldHeaterFan, LV_PCT(100), LV_PCT(100));
    lv_obj_center(_sldHeaterFan);
    lv_obj_set_style_bg_color(_sldHeaterFan, GUI_COLOR_CARD_HOVER, LV_PART_MAIN);
    lv_obj_set_style_bg_color(_sldHeaterFan, GUI_COLOR_ACCENT, LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(_sldHeaterFan, GUI_COLOR_ACCENT, LV_PART_KNOB);
    lv_obj_set_style_radius(_sldHeaterFan, GUI_RADIUS, LV_PART_MAIN);
    lv_obj_set_style_radius(_sldHeaterFan, GUI_RADIUS, LV_PART_INDICATOR);
    lv_obj_add_event_cb(_sldHeaterFan, heater_fan_slider_cb, LV_EVENT_VALUE_CHANGED, nullptr);

    _lblHFanName = lv_label_create(fanWrap);
    lv_label_set_text(_lblHFanName, "Fan Speed");
    lv_obj_set_style_text_font(_lblHFanName, GUI_FONT_XL, 0);
    lv_obj_set_style_text_color(_lblHFanName, lv_color_white(), 0);
    lv_obj_align(_lblHFanName, LV_ALIGN_LEFT_MID, 12, 0);
    lv_obj_add_flag(_lblHFanName, LV_OBJ_FLAG_FLOATING);
    lv_obj_clear_flag(_lblHFanName, LV_OBJ_FLAG_CLICKABLE);

    _lblHFanVal = lv_label_create(fanWrap);
    lv_label_set_text(_lblHFanVal, "0");
    lv_obj_set_style_text_font(_lblHFanVal, GUI_FONT_XL, 0);
    lv_obj_set_style_text_color(_lblHFanVal, lv_color_white(), 0);
    lv_obj_align(_lblHFanVal, LV_ALIGN_RIGHT_MID, -12, 0);
    lv_obj_add_flag(_lblHFanVal, LV_OBJ_FLAG_FLOATING);
    lv_obj_clear_flag(_lblHFanVal, LV_OBJ_FLAG_CLICKABLE);

    return card;
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

/** @brief Slot button pressed — save current climate state to that slot. */
static void save_slot_cb(lv_event_t *e) {
    int slot = (int)(intptr_t)lv_event_get_user_data(e);
    if (slot < 0 || slot >= PRESET_COUNT) return;

    ClimatePreset cp = {};
    gui_tab_climate_get_state(&cp);

    char name[PRESET_NAME_LEN];
    snprintf(name, sizeof(name), "Preset %d", slot + 1);

    gui_presets_save_climate((uint8_t)slot, name, &cp);
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

/** @brief Open a modal with 2x2 grid of all 4 slots for saving climate preset. */
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
    lv_obj_set_style_border_color(dlg, GUI_COLOR_ACCENT, 0);
    lv_obj_set_style_border_width(dlg, 2, 0);
    lv_obj_set_style_pad_all(dlg, GUI_PAD_LG, 0);
    lv_obj_clear_flag(dlg, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flex_flow(dlg, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_gap(dlg, GUI_PAD_MD, 0);

    // Title
    lv_obj_t *title = lv_label_create(dlg);
    lv_label_set_text(title, "Save Climate Preset");
    lv_obj_set_style_text_font(title, GUI_FONT_LG, 0);
    lv_obj_set_style_text_color(title, GUI_COLOR_ACCENT, 0);

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

    const ClimatePreset *presets = gui_presets_get_climate();
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

lv_obj_t *gui_tab_climate_create(lv_obj_t *parent) {
    lv_obj_t *cont = lv_obj_create(parent);
    lv_obj_set_size(cont, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_opa(cont, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(cont, 0, 0);
    lv_obj_set_style_pad_all(cont, GUI_PAD_MD, 0);
    lv_obj_clear_flag(cont, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_gap(cont, GUI_PAD_MD, 0);

    // Main content row (fan + heater panels side by side)
    lv_obj_t *row = lv_obj_create(cont);
    lv_obj_set_size(row, LV_PCT(100), LV_PCT(100));
    lv_obj_set_flex_grow(row, 1);
    lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(row, 0, 0);
    lv_obj_set_style_pad_all(row, 0, 0);
    lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_gap(row, GUI_PAD_MD, 0);

    create_fan_panel(row);
    create_heater_panel(row);

    return cont;
}

void gui_tab_climate_set_heater(int state, float target, float actual) {
    // state: 0=OFF, 1=STARTING, 2=ON/RUNNING, 3=STOPPING
    _heaterOn = (state == 1 || state == 2);

    // Only update target if a valid value is provided (>= 0)
    if (target >= 0) {
        _targetTemp = (int)target;
        if (_sldTarget) {
            lv_slider_set_value(_sldTarget, _targetTemp, LV_ANIM_ON);
        }
        if (_lblTempVal) {
            char buf[8]; snprintf(buf, sizeof(buf), "%d\xC2\xB0""C", _targetTemp);
            lv_label_set_text(_lblTempVal, buf);
        }
    }

    if (_btnPower) {
        lv_color_t bgCol;
        const char *statusText;
        switch (state) {
            case 1:  bgCol = GUI_COLOR_ACCENT;     statusText = LV_SYMBOL_REFRESH " START";  break;
            case 2:  bgCol = GUI_COLOR_ACCENT;     statusText = LV_SYMBOL_POWER " ON";       break;
            case 3:  bgCol = GUI_COLOR_ACCENT;     statusText = LV_SYMBOL_REFRESH " STOP";   break;
            default: bgCol = GUI_COLOR_CARD_HOVER;  statusText = LV_SYMBOL_POWER " OFF";      break;
        }
        lv_obj_set_style_bg_color(_btnPower, bgCol, 0);

        lv_obj_t *lbl = lv_obj_get_child(_btnPower, 0);
        if (lbl) lv_label_set_text(lbl, statusText);
    }
    if (_lblActual) {
        char buf[16]; snprintf(buf, sizeof(buf), "%.1f\xC2\xB0""C", actual);
        lv_label_set_text(_lblActual, buf);
    }
}

void gui_tab_climate_set_fan(int speed, bool dirIn, bool lidOpen) {
    _fanDirIn = dirIn;
    _lidOpen  = lidOpen;
    _fanPower = (speed > 0);

    if (_btnFanPower) {
        lv_obj_t *lbl = lv_obj_get_child(_btnFanPower, 0);
        if (lbl) lv_label_set_text(lbl, _fanPower ? LV_SYMBOL_POWER " ON" : LV_SYMBOL_POWER " OFF");
        lv_obj_set_style_bg_color(_btnFanPower, _fanPower ? GUI_COLOR_BLUE : GUI_COLOR_CARD_HOVER, 0);
    }
    if (_btnAirDir) {
        lv_obj_t *lbl = lv_obj_get_child(_btnAirDir, 0);
        if (lbl) lv_label_set_text(lbl, dirIn ? "Air In " LV_SYMBOL_DOWN : "Air Out " LV_SYMBOL_UP);
        lv_obj_set_style_bg_color(_btnAirDir, GUI_COLOR_BLUE, 0);
    }
    if (_btnLid) {
        lv_obj_t *lbl = lv_obj_get_child(_btnLid, 0);
        if (lbl) lv_label_set_text(lbl, lidOpen ? "Lid Open" : "Lid Closed");
        lv_obj_set_style_bg_color(_btnLid, lidOpen ? GUI_COLOR_BLUE : GUI_COLOR_CARD_HOVER, 0);
    }
    if (_sldSpeed) {
        lv_slider_set_value(_sldSpeed, speed, LV_ANIM_ON);
    }
    if (_lblSpeedVal) {
        char buf[8]; snprintf(buf, sizeof(buf), "%d%%", speed);
        lv_label_set_text(_lblSpeedVal, buf);
    }
}

void gui_tab_climate_get_state(ClimatePreset *out) {
    if (!out) return;
    out->heaterOn    = _heaterOn;
    out->heaterMode  = (uint8_t)_heaterMode;
    out->powerLevel  = (uint8_t)_powerLevel;
    out->targetTempC = (uint8_t)_targetTemp;
    out->fanPower    = _fanPower;
    out->fanSpeed    = _sldSpeed ? (uint8_t)lv_slider_get_value(_sldSpeed) : 0;
    out->fanDirIn    = _fanDirIn;
    out->lidOpen     = _lidOpen;
    out->saved       = true;
}

void gui_tab_climate_open_save_modal() {
    save_preset_cb(nullptr);
}
