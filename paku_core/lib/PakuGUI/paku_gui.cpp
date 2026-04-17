/**
 * @file paku_gui.cpp
 * @brief Main GUI controller — sidebar navigation, tab switching, and
 *        delegation to individual tab/header modules.
 *
 * This file implements the public API declared in paku_gui.h.
 * It builds the top-level layout (sidebar + header + content area),
 * creates every tab panel, and forwards data-push calls to the
 * appropriate sub-module.
 *
 * Layout (800×480):
 * ┌──────────┬────────────────────────────────────────────────────┐
 * │          │              Header  (40 px)                       │
 * │ Sidebar  ├────────────────────────────────────────────────────┤
 * │ (80 px)  │                                                    │
 * │          │        Active Tab Content  (720 × 440)             │
 * │          │                                                    │
 * └──────────┴────────────────────────────────────────────────────┘
 */

#include "paku_gui.h"
#include "gui_theme.h"
#include "waveshare_hal.h"

// Tab create & update headers
#include "gui_header.h"
#include "gui_tab_main.h"
#include "gui_presets.h"
#include "gui_tab_climate.h"
#include "gui_tab_lights.h"
#include "gui_tab_sensors.h"
#include "gui_tab_power.h"
#include "gui_tab_settings.h"

// ---------------------------------------------------------------------------
//  Static state
// ---------------------------------------------------------------------------

/// Tab content panels (one per tab — only the active one is visible)
static lv_obj_t *_tabPanels[GUI_TAB_COUNT] = {};

/// Sidebar icon buttons (one per tab)
static lv_obj_t *_sidebarBtns[GUI_TAB_COUNT] = {};

/// Currently active tab index
static GuiTab _activeTab = GUI_TAB_MAIN;

// ---------------------------------------------------------------------------
//  Sidebar icon definitions
// ---------------------------------------------------------------------------

/// LVGL symbols for each tab.
/// NOTE: LVGL 8.3 built-in symbol set is limited. Using best-fit substitutes.
/// A custom icon font can replace these later (see gui_theme.h).
static const char *_tabIcons[GUI_TAB_COUNT] = {
    LV_SYMBOL_HOME,      // Main / Presets
    LV_SYMBOL_CHARGE,    // Climate  (thermometer substitute)
    LV_SYMBOL_IMAGE,     // Lights   (lightbulb substitute)
    LV_SYMBOL_EYE_OPEN,  // Sensors  (chart substitute)
    LV_SYMBOL_POWER,     // Power
    LV_SYMBOL_SETTINGS,  // Settings
};

/// Short labels under icons (optional — hidden on small sidebar)
static const char *_tabLabels[GUI_TAB_COUNT] = {
    "Home", "Climate", "Lights", "Sensors", "Power", "Settings",
};

// ---------------------------------------------------------------------------
//  Forward declarations
// ---------------------------------------------------------------------------

static void switch_to_tab(GuiTab tab);
static void sidebar_btn_cb(lv_event_t *e);
static void sidebar_long_press_cb(lv_event_t *e);
static lv_obj_t *create_sidebar(lv_obj_t *parent);
static lv_obj_t *create_content_area(lv_obj_t *parent);

// ---------------------------------------------------------------------------
//  Sidebar construction
// ---------------------------------------------------------------------------

/**
 * @brief Build the vertical sidebar with icon buttons.
 */
static lv_obj_t *create_sidebar(lv_obj_t *parent) {
    lv_obj_t *sidebar = lv_obj_create(parent);
    lv_obj_set_size(sidebar, GUI_SIDEBAR_W, 480);
    lv_obj_align(sidebar, LV_ALIGN_TOP_LEFT, 0, 0);

    // Visual style
    lv_obj_set_style_bg_color(sidebar, GUI_COLOR_PANEL, 0);
    lv_obj_set_style_bg_opa(sidebar, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(sidebar, 0, 0);
    lv_obj_set_style_border_width(sidebar, 0, 0);
    lv_obj_set_style_pad_all(sidebar, 0, 0);
    lv_obj_set_style_pad_gap(sidebar, 0, 0);
    lv_obj_clear_flag(sidebar, LV_OBJ_FLAG_SCROLLABLE);

    // Flex layout — column, centred horizontally
    lv_obj_set_flex_flow(sidebar, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(sidebar, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);

    for (int i = 0; i < GUI_TAB_COUNT; i++) {
        lv_obj_t *btn = lv_btn_create(sidebar);
        lv_obj_set_size(btn, GUI_SIDEBAR_W, 80);  // 480/6 = 80 per button
        lv_obj_set_style_radius(btn, 0, 0);
        lv_obj_set_style_border_width(btn, 0, 0);
        lv_obj_set_style_shadow_width(btn, 0, 0);
        lv_obj_set_style_pad_all(btn, 4, 0);

        // Default (inactive) style
        lv_obj_set_style_bg_color(btn, GUI_COLOR_PANEL, 0);
        lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, 0);

        // Pressed style
        lv_obj_set_style_bg_color(btn, GUI_COLOR_CARD, LV_STATE_PRESSED);

        // Layout: icon + label stacked
        lv_obj_set_flex_flow(btn, LV_FLEX_FLOW_COLUMN);
        lv_obj_set_flex_align(btn, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER,
                              LV_FLEX_ALIGN_CENTER);

        lv_obj_t *icon = lv_label_create(btn);
        lv_label_set_text(icon, _tabIcons[i]);
        lv_obj_set_style_text_font(icon, GUI_FONT_XL, 0);
        lv_obj_set_style_text_color(icon, GUI_COLOR_TEXT_SEC, 0);

        lv_obj_t *lbl = lv_label_create(btn);
        lv_label_set_text(lbl, _tabLabels[i]);
        lv_obj_set_style_text_font(lbl, GUI_FONT_SM, 0);
        lv_obj_set_style_text_color(lbl, GUI_COLOR_TEXT_MUTED, 0);

        // Store tab index in user_data for the click callback
        lv_obj_set_user_data(btn, (void *)(intptr_t)i);
        lv_obj_add_event_cb(btn, sidebar_btn_cb, LV_EVENT_CLICKED, nullptr);
        lv_obj_add_event_cb(btn, sidebar_long_press_cb, LV_EVENT_LONG_PRESSED, nullptr);

        _sidebarBtns[i] = btn;
    }

    return sidebar;
}

// ---------------------------------------------------------------------------
//  Content area
// ---------------------------------------------------------------------------

/**
 * @brief Create a content container positioned to the right of the sidebar
 *        and below the header.  Tab panels are created as children.
 */
static lv_obj_t *create_content_area(lv_obj_t *parent) {
    lv_obj_t *cont = lv_obj_create(parent);
    lv_obj_set_pos(cont, GUI_SIDEBAR_W, GUI_HEADER_H);
    lv_obj_set_size(cont, GUI_CONTENT_W, GUI_CONTENT_H);
    lv_obj_set_style_bg_color(cont, GUI_COLOR_BG, 0);
    lv_obj_set_style_bg_opa(cont, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(cont, 0, 0);
    lv_obj_set_style_border_width(cont, 0, 0);
    lv_obj_set_style_pad_all(cont, 0, 0);
    lv_obj_clear_flag(cont, LV_OBJ_FLAG_SCROLLABLE);

    // Create each tab panel (fills the content area)
    _tabPanels[GUI_TAB_MAIN]     = gui_tab_main_create(cont);
    _tabPanels[GUI_TAB_CLIMATE]  = gui_tab_climate_create(cont);
    _tabPanels[GUI_TAB_LIGHTS]   = gui_tab_lights_create(cont);
    _tabPanels[GUI_TAB_SENSORS]  = gui_tab_sensors_create(cont);
    _tabPanels[GUI_TAB_POWER]    = gui_tab_power_create(cont);
    _tabPanels[GUI_TAB_SETTINGS] = gui_tab_settings_create(cont);

    return cont;
}

// ---------------------------------------------------------------------------
//  Tab switching
// ---------------------------------------------------------------------------

/**
 * @brief Switch the visible tab and update sidebar highlight.
 */
static void switch_to_tab(GuiTab tab) {
    if (tab >= GUI_TAB_COUNT) return;

    // Hide all panels, show only the selected one
    for (int i = 0; i < GUI_TAB_COUNT; i++) {
        if (_tabPanels[i]) {
            if ((GuiTab)i == tab) {
                lv_obj_clear_flag(_tabPanels[i], LV_OBJ_FLAG_HIDDEN);
            } else {
                lv_obj_add_flag(_tabPanels[i], LV_OBJ_FLAG_HIDDEN);
            }
        }
    }

    // Update sidebar button styles
    for (int i = 0; i < GUI_TAB_COUNT; i++) {
        if (!_sidebarBtns[i]) continue;

        lv_obj_t *btn = _sidebarBtns[i];
        if ((GuiTab)i == tab) {
            // Active tab — accent left border + orange icon
            lv_obj_set_style_bg_color(btn, GUI_COLOR_CARD, 0);
            lv_obj_set_style_border_side(btn, LV_BORDER_SIDE_LEFT, 0);
            lv_obj_set_style_border_width(btn, GUI_BORDER_W, 0);
            lv_obj_set_style_border_color(btn, GUI_COLOR_ACCENT, 0);

            // Recolour icon (first child) to accent
            lv_obj_t *icon = lv_obj_get_child(btn, 0);
            if (icon) lv_obj_set_style_text_color(icon, GUI_COLOR_ACCENT, 0);

            // Recolour label (second child) to primary text
            lv_obj_t *lbl = lv_obj_get_child(btn, 1);
            if (lbl) lv_obj_set_style_text_color(lbl, GUI_COLOR_TEXT_PRI, 0);
        } else {
            // Inactive tab — plain panel background
            lv_obj_set_style_bg_color(btn, GUI_COLOR_PANEL, 0);
            lv_obj_set_style_border_width(btn, 0, 0);

            lv_obj_t *icon = lv_obj_get_child(btn, 0);
            if (icon) lv_obj_set_style_text_color(icon, GUI_COLOR_TEXT_SEC, 0);

            lv_obj_t *lbl = lv_obj_get_child(btn, 1);
            if (lbl) lv_obj_set_style_text_color(lbl, GUI_COLOR_TEXT_MUTED, 0);
        }
    }

    _activeTab = tab;
}

/**
 * @brief Sidebar button click handler.
 */
static void sidebar_btn_cb(lv_event_t *e) {
    lv_obj_t *btn = lv_event_get_target(e);
    int idx = (int)(intptr_t)lv_obj_get_user_data(btn);
    switch_to_tab((GuiTab)idx);
}

/**
 * @brief Sidebar button long-press handler.
 *
 * Opens the save-preset modal when the user long-presses the Climate
 * or Lights sidebar button while that tab is already active.
 */
static void sidebar_long_press_cb(lv_event_t *e) {
    lv_obj_t *btn = lv_event_get_target(e);
    int idx = (int)(intptr_t)lv_obj_get_user_data(btn);

    // Only trigger when the tab is already active
    if ((GuiTab)idx != _activeTab) return;

    if ((GuiTab)idx == GUI_TAB_CLIMATE) {
        gui_tab_climate_open_save_modal();
    } else if ((GuiTab)idx == GUI_TAB_LIGHTS) {
        gui_tab_lights_open_save_modal();
    }
}

// ============================================================================
//  Public API implementation — Lifecycle
// ============================================================================

void gui_init() {
    // Initialise hardware (display, touch, LVGL drivers)
    waveshare_hal_init();

    // Load presets from NVS before building UI
    gui_presets_init();

    // Set screen background
    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, GUI_COLOR_BG, 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    // Build the three top-level regions
    create_sidebar(scr);

    // Header sits to the right of the sidebar, at the top
    lv_obj_t *header = gui_header_create(scr);
    lv_obj_set_pos(header, GUI_SIDEBAR_W, 0);
    lv_obj_set_size(header, GUI_CONTENT_W, GUI_HEADER_H);

    // Content area fills the remaining space
    create_content_area(scr);

    // Show the first tab
    switch_to_tab(GUI_TAB_MAIN);
}

void gui_update() {
    waveshare_hal_loop();
}

// ============================================================================
//  Public API implementation — Data delegates
// ============================================================================

void gui_set_wifi_status(bool connected, int rssi) {
    gui_header_set_wifi(connected, rssi);
}

void gui_set_mqtt_status(bool connected) {
    gui_header_set_mqtt(connected);
}

void gui_set_ble_status(bool active) {
    gui_header_set_ble(active);
}

void gui_set_time(const char *timeStr) {
    gui_header_set_time(timeStr);
}

void gui_set_heater_data(int state, float targetTemp, float actualTemp) {
    gui_header_set_heater(state);
    gui_tab_climate_set_heater(state, targetTemp, actualTemp);
}

void gui_set_fan_data(int speed, bool dirIn, bool lidOpen) {
    gui_tab_climate_set_fan(speed, dirIn, lidOpen);
}

void gui_push_temperature(uint8_t series, float value) {
    gui_tab_sensors_push_temp(series, value);
}

void gui_push_humidity(uint8_t series, float value) {
    gui_tab_sensors_push_hum(series, value);
}

void gui_set_power_data(float solarW, float acChargerW,
                        float inverterW, float dcLoadW) {
    gui_tab_power_set_data(solarW, acChargerW, inverterW, dcLoadW);
}

void gui_set_light_zone(uint8_t zone, bool on, uint8_t brightness,
                         uint16_t colorTemp) {
    gui_tab_lights_set_zone(zone, on, brightness, colorTemp);
}

void gui_set_firmware_version(const char *ver) {
    gui_tab_settings_set_version(ver);
}
