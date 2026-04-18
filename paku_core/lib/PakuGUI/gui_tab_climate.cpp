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
static lv_obj_t *_btnAirDir   = nullptr;
static lv_obj_t *_btnLid      = nullptr;
static lv_obj_t *_sldSpeed    = nullptr;
static lv_obj_t *_lblSpeedName = nullptr;
static lv_obj_t *_lblSpeedVal  = nullptr;

// Heater panel
static lv_obj_t *_btnPower    = nullptr;
static lv_obj_t *_btnModePwr  = nullptr;
static lv_obj_t *_btnModeThm  = nullptr;
static lv_obj_t *_btnModeVent = nullptr;
static lv_obj_t *_lblTarget   = nullptr;
static lv_obj_t *_sldTarget   = nullptr;
static lv_obj_t *_lblPower    = nullptr;
static lv_obj_t *_sldPower    = nullptr;
static lv_obj_t *_lblActual   = nullptr;
static lv_obj_t *_pnlPowerMode = nullptr;
static lv_obj_t *_pnlThermoMode = nullptr;

static bool _heaterOn     = false;
static int  _targetTemp   = 21;
static int  _powerLevel   = 5;
static bool _fanDirIn     = true;
static bool _lidOpen      = true;
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
    lv_obj_set_style_text_font(lbl, GUI_FONT_MD, 0);
    lv_obj_center(lbl);
    return btn;
}

// ---------------------------------------------------------------------------
//  Callbacks
// ---------------------------------------------------------------------------

static void fan_dir_cb(lv_event_t *e) {
    _fanDirIn = !_fanDirIn;
    lv_obj_t *lbl = lv_obj_get_child(_btnAirDir, 0);
    lv_label_set_text(lbl, _fanDirIn ? "Air In " LV_SYMBOL_DOWN : "Air Out " LV_SYMBOL_UP);
    lv_obj_set_style_bg_color(_btnAirDir, GUI_COLOR_BLUE, 0);
    if (_sldSpeed && _cb_fan) {
        int spd = lv_slider_get_value(_sldSpeed);
        _cb_fan(spd > 0, (uint8_t)spd, _fanDirIn, _lidOpen);
    }
}

static void lid_cb(lv_event_t *e) {
    _lidOpen = !_lidOpen;
    lv_obj_t *lbl = lv_obj_get_child(_btnLid, 0);
    lv_label_set_text(lbl, _lidOpen ? "Lid Open" : "Lid Closed");
    lv_obj_set_style_bg_color(_btnLid, _lidOpen ? GUI_COLOR_BLUE : GUI_COLOR_CARD_HOVER, 0);
    if (_sldSpeed && _cb_fan) {
        int spd = lv_slider_get_value(_sldSpeed);
        _cb_fan(spd > 0, (uint8_t)spd, _fanDirIn, _lidOpen);
    }
}

static void speed_cb(lv_event_t *e) {
    int val = lv_slider_get_value(_sldSpeed);
    char buf[8];
    snprintf(buf, sizeof(buf), "%d%%", val);
    if (_lblSpeedVal) lv_label_set_text(_lblSpeedVal, buf);
    if (_cb_fan) _cb_fan(val > 0, (uint8_t)val, _fanDirIn, _lidOpen);
}

static void power_cb(lv_event_t *e) {
    _heaterOn = !_heaterOn;
    lv_obj_set_style_border_color(_btnPower,
        _heaterOn ? GUI_COLOR_RED : GUI_COLOR_TEXT_MUTED, 0);
    lv_obj_set_style_border_width(_btnPower, 3, 0);
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

/** @brief Switch visible heater sub-panel based on active mode. */
static void _update_mode_panels() {
    if (_pnlPowerMode)  lv_obj_set_style_opa(_pnlPowerMode,
        _heaterMode == HEATER_MODE_POWER ? LV_OPA_COVER : LV_OPA_TRANSP, 0);
    if (_pnlThermoMode) lv_obj_set_style_opa(_pnlThermoMode,
        _heaterMode == HEATER_MODE_THERMOSTAT ? LV_OPA_COVER : LV_OPA_TRANSP, 0);
    // Hide/show via flag for layout
    if (_pnlPowerMode) {
        if (_heaterMode == HEATER_MODE_POWER)
            lv_obj_clear_flag(_pnlPowerMode, LV_OBJ_FLAG_HIDDEN);
        else
            lv_obj_add_flag(_pnlPowerMode, LV_OBJ_FLAG_HIDDEN);
    }
    if (_pnlThermoMode) {
        if (_heaterMode == HEATER_MODE_THERMOSTAT)
            lv_obj_clear_flag(_pnlThermoMode, LV_OBJ_FLAG_HIDDEN);
        else
            lv_obj_add_flag(_pnlThermoMode, LV_OBJ_FLAG_HIDDEN);
    }
    // Update mode button colors
    if (_btnModePwr) lv_obj_set_style_bg_color(_btnModePwr,
        _heaterMode == HEATER_MODE_POWER ? GUI_COLOR_ACCENT : GUI_COLOR_CARD_HOVER, 0);
    if (_btnModeThm) lv_obj_set_style_bg_color(_btnModeThm,
        _heaterMode == HEATER_MODE_THERMOSTAT ? GUI_COLOR_ACCENT : GUI_COLOR_CARD_HOVER, 0);
    if (_btnModeVent) lv_obj_set_style_bg_color(_btnModeVent,
        _heaterMode == HEATER_MODE_VENT ? GUI_COLOR_BLUE : GUI_COLOR_CARD_HOVER, 0);
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

static void mode_vent_cb(lv_event_t *e) {
    _heaterMode = HEATER_MODE_VENT;
    _heaterOn = true;  // vent always means "on" (fan running)
    _update_mode_panels();
    if (_btnPower) {
        lv_obj_set_style_border_color(_btnPower, GUI_COLOR_BLUE, 0);
        lv_obj_set_style_border_width(_btnPower, 3, 0);
    }
    if (_cb_heater) {
        _cb_heater(true, HEATER_MODE_VENT, (uint8_t)_powerLevel, 0);
    }
}

static void power_slider_cb(lv_event_t *e) {
    _powerLevel = lv_slider_get_value(_sldPower);
    char buf[8];
    snprintf(buf, sizeof(buf), "%d", _powerLevel);
    if (_lblPower) lv_label_set_text(_lblPower, buf);
    _notify_heater();
}

static void power_minus_cb(lv_event_t *e) {
    if (_powerLevel > 0) {
        _powerLevel--;
        lv_slider_set_value(_sldPower, _powerLevel, LV_ANIM_ON);
        char buf[8]; snprintf(buf, sizeof(buf), "%d", _powerLevel);
        if (_lblPower) lv_label_set_text(_lblPower, buf);
        _notify_heater();
    }
}

static void power_plus_cb(lv_event_t *e) {
    if (_powerLevel < 9) {
        _powerLevel++;
        lv_slider_set_value(_sldPower, _powerLevel, LV_ANIM_ON);
        char buf[8]; snprintf(buf, sizeof(buf), "%d", _powerLevel);
        if (_lblPower) lv_label_set_text(_lblPower, buf);
        _notify_heater();
    }
}

static void temp_slider_cb(lv_event_t *e) {
    _targetTemp = lv_slider_get_value(_sldTarget);
    char buf[8];
    snprintf(buf, sizeof(buf), "%d\xC2\xB0""C", _targetTemp);
    lv_label_set_text(_lblTarget, buf);
    _notify_heater();
}

static void temp_minus_cb(lv_event_t *e) {
    if (_targetTemp > 10) {
        _targetTemp--;
        lv_slider_set_value(_sldTarget, _targetTemp, LV_ANIM_ON);
        char buf[8]; snprintf(buf, sizeof(buf), "%d\xC2\xB0" "C", _targetTemp);
        lv_label_set_text(_lblTarget, buf);
        _notify_heater();
    }
}

static void temp_plus_cb(lv_event_t *e) {
    if (_targetTemp < 30) {
        _targetTemp++;
        lv_slider_set_value(_sldTarget, _targetTemp, LV_ANIM_ON);
        char buf[8]; snprintf(buf, sizeof(buf), "%d\xC2\xB0" "C", _targetTemp);
        lv_label_set_text(_lblTarget, buf);
        _notify_heater();
    }
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
    lv_obj_set_style_pad_gap(card, GUI_PAD_MD, 0);

    // Title
    lv_obj_t *title = lv_label_create(card);
    lv_label_set_text(title, "Roof Fan (MaxxFan)");
    lv_obj_set_style_text_font(title, GUI_FONT_LG, 0);
    lv_obj_set_style_text_color(title, GUI_COLOR_BLUE, 0);

    // Direction & Lid toggle row
    lv_obj_t *toggleRow = lv_obj_create(card);
    lv_obj_set_size(toggleRow, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_style_bg_opa(toggleRow, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(toggleRow, 0, 0);
    lv_obj_set_style_pad_all(toggleRow, 0, 0);
    lv_obj_clear_flag(toggleRow, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flex_flow(toggleRow, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_gap(toggleRow, GUI_PAD_SM, 0);

    _btnAirDir = make_toggle_btn(toggleRow, "Air In " LV_SYMBOL_DOWN, true, GUI_COLOR_BLUE);
    lv_obj_add_event_cb(_btnAirDir, fan_dir_cb, LV_EVENT_CLICKED, nullptr);

    _btnLid = make_toggle_btn(toggleRow, "Lid Open", true, GUI_COLOR_BLUE);
    lv_obj_add_event_cb(_btnLid, lid_cb, LV_EVENT_CLICKED, nullptr);

    // Fan speed — large overlay slider filling remaining space
    lv_obj_t *speedWrap = lv_obj_create(card);
    lv_obj_set_size(speedWrap, LV_PCT(100), LV_SIZE_CONTENT);
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
    lv_obj_set_style_pad_gap(card, GUI_PAD_MD, 0);

    // Title
    lv_obj_t *title = lv_label_create(card);
    lv_label_set_text(title, "Diesel Heater");
    lv_obj_set_style_text_font(title, GUI_FONT_LG, 0);
    lv_obj_set_style_text_color(title, GUI_COLOR_ACCENT, 0);

    // Mode toggle row:  [Power] [Thermostat]
    lv_obj_t *modeRow = lv_obj_create(card);
    lv_obj_set_size(modeRow, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_style_bg_opa(modeRow, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(modeRow, 0, 0);
    lv_obj_set_style_pad_all(modeRow, 0, 0);
    lv_obj_clear_flag(modeRow, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flex_flow(modeRow, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_gap(modeRow, GUI_PAD_SM, 0);

    _btnModePwr = make_toggle_btn(modeRow, LV_SYMBOL_CHARGE " Power", true, GUI_COLOR_ACCENT);
    lv_obj_add_event_cb(_btnModePwr, mode_power_cb, LV_EVENT_CLICKED, nullptr);

    _btnModeThm = make_toggle_btn(modeRow, LV_SYMBOL_HOME " Thermo", false, GUI_COLOR_ACCENT);
    lv_obj_add_event_cb(_btnModeThm, mode_thermo_cb, LV_EVENT_CLICKED, nullptr);

    _btnModeVent = make_toggle_btn(modeRow, LV_SYMBOL_REFRESH " Vent", false, GUI_COLOR_BLUE);
    lv_obj_add_event_cb(_btnModeVent, mode_vent_cb, LV_EVENT_CLICKED, nullptr);

    // Power button + status row
    lv_obj_t *topRow = lv_obj_create(card);
    lv_obj_set_size(topRow, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_style_bg_opa(topRow, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(topRow, 0, 0);
    lv_obj_set_style_pad_all(topRow, 0, 0);
    lv_obj_clear_flag(topRow, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flex_flow(topRow, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(topRow, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    // Power button (circular 80x80)
    _btnPower = lv_btn_create(topRow);
    lv_obj_set_size(_btnPower, 80, 80);
    lv_obj_set_style_radius(_btnPower, 40, 0);
    lv_obj_set_style_bg_color(_btnPower, GUI_COLOR_CARD, 0);
    lv_obj_set_style_border_color(_btnPower, GUI_COLOR_TEXT_MUTED, 0);
    lv_obj_set_style_border_width(_btnPower, 3, 0);
    lv_obj_t *pwrIcon = lv_label_create(_btnPower);
    lv_label_set_text(pwrIcon, LV_SYMBOL_POWER);
    lv_obj_set_style_text_font(pwrIcon, GUI_FONT_XL, 0);
    lv_obj_center(pwrIcon);
    lv_obj_add_event_cb(_btnPower, power_cb, LV_EVENT_CLICKED, nullptr);

    // Actual temperature label (read-only, updated via data push)
    _lblActual = lv_label_create(topRow);
    lv_label_set_text(_lblActual, "Actual: --\xC2\xB0""C");
    lv_obj_set_style_text_font(_lblActual, GUI_FONT_LG, 0);
    lv_obj_set_style_text_color(_lblActual, GUI_COLOR_TEXT_SEC, 0);

    // ── Power Mode sub-panel ──
    _pnlPowerMode = lv_obj_create(card);
    lv_obj_set_size(_pnlPowerMode, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_style_bg_opa(_pnlPowerMode, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(_pnlPowerMode, 0, 0);
    lv_obj_set_style_pad_all(_pnlPowerMode, 0, 0);
    lv_obj_clear_flag(_pnlPowerMode, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flex_flow(_pnlPowerMode, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_gap(_pnlPowerMode, GUI_PAD_SM, 0);

    // Power level label
    lv_obj_t *pwrLbl = lv_label_create(_pnlPowerMode);
    lv_label_set_text(pwrLbl, "Power Level");
    lv_obj_set_style_text_font(pwrLbl, GUI_FONT_SM, 0);
    lv_obj_set_style_text_color(pwrLbl, GUI_COLOR_TEXT_SEC, 0);

    _lblPower = lv_label_create(_pnlPowerMode);
    lv_label_set_text(_lblPower, "5");
    lv_obj_set_style_text_font(_lblPower, GUI_FONT_XXL, 0);
    lv_obj_set_style_text_color(_lblPower, GUI_COLOR_TEXT_PRI, 0);

    // Power control row: [-] slider [+]
    lv_obj_t *pwrCtrl = lv_obj_create(_pnlPowerMode);
    lv_obj_set_size(pwrCtrl, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_style_bg_opa(pwrCtrl, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(pwrCtrl, 0, 0);
    lv_obj_set_style_pad_all(pwrCtrl, 0, 0);
    lv_obj_clear_flag(pwrCtrl, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flex_flow(pwrCtrl, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(pwrCtrl, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_gap(pwrCtrl, GUI_PAD_SM, 0);

    lv_obj_t *pMinus = lv_btn_create(pwrCtrl);
    lv_obj_set_size(pMinus, 56, 56);
    lv_obj_set_style_radius(pMinus, 12, 0);
    lv_obj_set_style_bg_color(pMinus, GUI_COLOR_CARD_HOVER, 0);
    lv_obj_t *pMinLbl = lv_label_create(pMinus);
    lv_label_set_text(pMinLbl, LV_SYMBOL_MINUS);
    lv_obj_set_style_text_font(pMinLbl, GUI_FONT_LG, 0);
    lv_obj_center(pMinLbl);
    lv_obj_add_event_cb(pMinus, power_minus_cb, LV_EVENT_CLICKED, nullptr);

    _sldPower = lv_slider_create(pwrCtrl);
    lv_slider_set_range(_sldPower, 0, 9);
    lv_slider_set_value(_sldPower, 5, LV_ANIM_OFF);
    lv_obj_set_flex_grow(_sldPower, 1);
    lv_obj_set_style_bg_color(_sldPower, GUI_COLOR_CARD_HOVER, LV_PART_MAIN);
    lv_obj_set_style_bg_color(_sldPower, GUI_COLOR_ACCENT, LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(_sldPower, GUI_COLOR_ACCENT, LV_PART_KNOB);
    lv_obj_add_event_cb(_sldPower, power_slider_cb, LV_EVENT_VALUE_CHANGED, nullptr);

    lv_obj_t *pPlus = lv_btn_create(pwrCtrl);
    lv_obj_set_size(pPlus, 56, 56);
    lv_obj_set_style_radius(pPlus, 12, 0);
    lv_obj_set_style_bg_color(pPlus, GUI_COLOR_CARD_HOVER, 0);
    lv_obj_t *pPlusLbl = lv_label_create(pPlus);
    lv_label_set_text(pPlusLbl, LV_SYMBOL_PLUS);
    lv_obj_set_style_text_font(pPlusLbl, GUI_FONT_LG, 0);
    lv_obj_center(pPlusLbl);
    lv_obj_add_event_cb(pPlus, power_plus_cb, LV_EVENT_CLICKED, nullptr);

    // ── Thermostat Mode sub-panel ──
    _pnlThermoMode = lv_obj_create(card);
    lv_obj_set_size(_pnlThermoMode, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_style_bg_opa(_pnlThermoMode, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(_pnlThermoMode, 0, 0);
    lv_obj_set_style_pad_all(_pnlThermoMode, 0, 0);
    lv_obj_clear_flag(_pnlThermoMode, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flex_flow(_pnlThermoMode, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_gap(_pnlThermoMode, GUI_PAD_SM, 0);
    lv_obj_add_flag(_pnlThermoMode, LV_OBJ_FLAG_HIDDEN); // starts hidden

    // Target temp label
    lv_obj_t *tgtLbl = lv_label_create(_pnlThermoMode);
    lv_label_set_text(tgtLbl, "Target Temp");
    lv_obj_set_style_text_font(tgtLbl, GUI_FONT_SM, 0);
    lv_obj_set_style_text_color(tgtLbl, GUI_COLOR_TEXT_SEC, 0);

    _lblTarget = lv_label_create(_pnlThermoMode);
    lv_label_set_text(_lblTarget, "21\xC2\xB0""C");
    lv_obj_set_style_text_font(_lblTarget, GUI_FONT_XXL, 0);
    lv_obj_set_style_text_color(_lblTarget, GUI_COLOR_TEXT_PRI, 0);

    // Temp control row: [-] slider [+]
    lv_obj_t *ctrlRow = lv_obj_create(_pnlThermoMode);
    lv_obj_set_size(ctrlRow, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_style_bg_opa(ctrlRow, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(ctrlRow, 0, 0);
    lv_obj_set_style_pad_all(ctrlRow, 0, 0);
    lv_obj_clear_flag(ctrlRow, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flex_flow(ctrlRow, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(ctrlRow, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_gap(ctrlRow, GUI_PAD_SM, 0);

    lv_obj_t *btnMinus = lv_btn_create(ctrlRow);
    lv_obj_set_size(btnMinus, 56, 56);
    lv_obj_set_style_radius(btnMinus, 12, 0);
    lv_obj_set_style_bg_color(btnMinus, GUI_COLOR_CARD_HOVER, 0);
    lv_obj_t *minLbl = lv_label_create(btnMinus);
    lv_label_set_text(minLbl, LV_SYMBOL_MINUS);
    lv_obj_set_style_text_font(minLbl, GUI_FONT_LG, 0);
    lv_obj_center(minLbl);
    lv_obj_add_event_cb(btnMinus, temp_minus_cb, LV_EVENT_CLICKED, nullptr);

    _sldTarget = lv_slider_create(ctrlRow);
    lv_slider_set_range(_sldTarget, 10, 30);
    lv_slider_set_value(_sldTarget, 21, LV_ANIM_OFF);
    lv_obj_set_flex_grow(_sldTarget, 1);
    lv_obj_set_style_bg_color(_sldTarget, GUI_COLOR_CARD_HOVER, LV_PART_MAIN);
    lv_obj_set_style_bg_color(_sldTarget, GUI_COLOR_ACCENT, LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(_sldTarget, GUI_COLOR_ACCENT, LV_PART_KNOB);
    lv_obj_add_event_cb(_sldTarget, temp_slider_cb, LV_EVENT_VALUE_CHANGED, nullptr);

    lv_obj_t *btnPlus = lv_btn_create(ctrlRow);
    lv_obj_set_size(btnPlus, 56, 56);
    lv_obj_set_style_radius(btnPlus, 12, 0);
    lv_obj_set_style_bg_color(btnPlus, GUI_COLOR_CARD_HOVER, 0);
    lv_obj_t *plusLbl = lv_label_create(btnPlus);
    lv_label_set_text(plusLbl, LV_SYMBOL_PLUS);
    lv_obj_set_style_text_font(plusLbl, GUI_FONT_LG, 0);
    lv_obj_center(plusLbl);
    lv_obj_add_event_cb(btnPlus, temp_plus_cb, LV_EVENT_CLICKED, nullptr);

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
    _heaterOn = (state > 0);

    // Only update target if a valid value is provided (>= 0)
    if (target >= 0) {
        _targetTemp = (int)target;
        if (_lblTarget) {
            char buf[8]; snprintf(buf, sizeof(buf), "%d\xC2\xB0""C", _targetTemp);
            lv_label_set_text(_lblTarget, buf);
        }
        if (_sldTarget) {
            lv_slider_set_value(_sldTarget, _targetTemp, LV_ANIM_ON);
        }
    }

    if (_btnPower) {
        lv_obj_set_style_border_color(_btnPower,
            _heaterOn ? GUI_COLOR_RED : GUI_COLOR_TEXT_MUTED, 0);
    }
    if (_lblActual) {
        char buf[16]; snprintf(buf, sizeof(buf), "Actual: %.1f\xC2\xB0""C", actual);
        lv_label_set_text(_lblActual, buf);
    }
}

void gui_tab_climate_set_fan(int speed, bool dirIn, bool lidOpen) {
    _fanDirIn = dirIn;
    _lidOpen  = lidOpen;

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
    out->fanPower    = _sldSpeed ? (lv_slider_get_value(_sldSpeed) > 0) : false;
    out->fanSpeed    = _sldSpeed ? (uint8_t)lv_slider_get_value(_sldSpeed) : 0;
    out->fanDirIn    = _fanDirIn;
    out->lidOpen     = _lidOpen;
    out->saved       = true;
}

void gui_tab_climate_open_save_modal() {
    save_preset_cb(nullptr);
}
