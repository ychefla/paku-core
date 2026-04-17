/**
 * @file paku_gui.h
 * @brief Public API for the Paku touchscreen GUI.
 *
 * This is the ONLY header that main.cpp needs to include.  All rendering,
 * layout, and LVGL details are hidden behind this interface so the GUI
 * can evolve independently from the core firmware.
 *
 * Data flow: main.cpp pushes data into the GUI via gui_set_*() functions.
 * The GUI is never polled — it renders whatever was last pushed.
 */
#pragma once

#include <Arduino.h>

// ============================================================================
//  Lifecycle
// ============================================================================

/**
 * @brief Initialise the GUI.
 *
 * Must be called AFTER waveshare_hal_init() has set up the display & LVGL.
 * Creates the sidebar, header, and all tab content panels.
 */
void gui_init();

/**
 * @brief Periodic GUI update — call every loop() iteration.
 *
 * Delegates to waveshare_hal_loop() which runs lv_timer_handler().
 */
void gui_update();

// ============================================================================
//  Status bar data
// ============================================================================

/**
 * @brief Update WiFi status shown in the header bar.
 * @param connected  true if WiFi is associated.
 * @param rssi       Signal strength in dBm (only used when connected).
 */
void gui_set_wifi_status(bool connected, int rssi = 0);

/**
 * @brief Update MQTT broker connection status.
 */
void gui_set_mqtt_status(bool connected);

/**
 * @brief Update BLE scanning status.
 */
void gui_set_ble_status(bool active);

/**
 * @brief Update the clock display in the header.
 * @param timeStr  "HH:MM" formatted string.
 */
void gui_set_time(const char *timeStr);

// ============================================================================
//  Climate tab data
// ============================================================================

/**
 * @brief Push heater telemetry to the Climate tab.
 * @param state       Heater state (0=off, 1=starting, 2=running, 3=cooling, etc.)
 * @param targetTemp  Target temperature in °C.
 * @param actualTemp  Current coolant/exhaust temperature in °C.
 */
void gui_set_heater_data(int state, float targetTemp, float actualTemp);

/**
 * @brief Push fan status to the Climate tab.
 * @param speed   Fan speed 0-100%.
 * @param dirIn   true = Air-In direction.
 * @param lidOpen true = lid/vent is open.
 */
void gui_set_fan_data(int speed, bool dirIn, bool lidOpen);

// ============================================================================
//  Sensor tab data
// ============================================================================

/**
 * @brief Push a temperature reading to the Sensors tab graph.
 * @param series  Index 0-3 (Indoor, Outdoor, Fridge, Reppu).
 * @param value   Temperature in °C.
 */
void gui_push_temperature(uint8_t series, float value);

/**
 * @brief Push a humidity reading to the Sensors tab graph.
 * @param series  Index 0-2 (Indoor, Outdoor, Reppu).
 * @param value   Humidity in %.
 */
void gui_push_humidity(uint8_t series, float value);

// ============================================================================
//  Power tab data
// ============================================================================

/**
 * @brief Push power measurements to the Power tab.
 * @param solarW      Solar input in Watts.
 * @param acChargerW  AC charger input in Watts.
 * @param inverterW   AC inverter output in Watts.
 * @param dcLoadW     12 V DC base load in Watts.
 */
void gui_set_power_data(float solarW, float acChargerW,
                        float inverterW, float dcLoadW);

// ============================================================================
//  Lights tab data
// ============================================================================

/**
 * @brief Push a single zone's light state.
 * @param zone        Zone 0-3 (Living, Kitchen, Bedroom, Outside).
 * @param on          true = zone is on.
 * @param brightness  0-100%.
 * @param colorTemp   Mireds or Kelvin (display only).
 */
void gui_set_light_zone(uint8_t zone, bool on, uint8_t brightness, uint16_t colorTemp);

// ============================================================================
//  Settings tab helpers
// ============================================================================

/**
 * @brief Update the firmware version string shown in Settings.
 */
void gui_set_firmware_version(const char *ver);

// ============================================================================
//  Action callbacks — GUI → firmware (outbound events)
//
//  Register handlers with gui_on_*() before calling gui_init().
//  All callbacks are invoked from the LVGL/loop() task.
// ============================================================================

/**
 * Light zone changed.
 * @param zone        Zone index 0-3 (Living, Kitchen, Bedroom, Outside).
 * @param on          Power state.
 * @param brightness  0-100%.
 * @param colorTempK  Color temperature in Kelvin (2700-6500).
 */
typedef void (*GuiLightCb)(uint8_t zone, bool on, uint8_t brightness, uint16_t colorTempK);
void gui_on_light_changed(GuiLightCb cb);

/**
 * Fan (MaxxFan) state changed.
 * @param power   Fan is running.
 * @param speed   0-100%.
 * @param dirIn   true = Air-In / intake.
 * @param lidOpen true = lid open.
 */
typedef void (*GuiFanCb)(bool power, uint8_t speed, bool dirIn, bool lidOpen);
void gui_on_fan_changed(GuiFanCb cb);

/**
 * Heater power or target temperature changed.
 * @param on          Heater requested on.
 * @param powerLevel  1-10 (derived from targetTempC).
 * @param targetTempC Target temperature in °C.
 */
typedef void (*GuiHeaterCb)(bool on, uint8_t powerLevel, uint8_t targetTempC);
void gui_on_heater_changed(GuiHeaterCb cb);

/**
 * Display backlight brightness changed.
 * @param percent 10-100%.
 */
typedef void (*GuiBacklightCb)(uint8_t percent);
void gui_on_backlight_changed(GuiBacklightCb cb);

/**
 * Restart button pressed (after confirmation, if any).
 */
typedef void (*GuiRestartCb)();
void gui_on_restart(GuiRestartCb cb);

/**
 * Lighting preset activated.
 * Preset indices: 0=All Off, 1=All On, 2=Kitchen Only, 3=Night Light.
 */
typedef void (*GuiLightPresetCb)(uint8_t presetIdx);
void gui_on_light_preset(GuiLightPresetCb cb);

/**
 * Climate preset activated.
 * Preset indices: 0=All Off, 1=Heat, 2=Max Out Flow, 3=Night Setting.
 */
typedef void (*GuiClimatePresetCb)(uint8_t presetIdx);
void gui_on_climate_preset(GuiClimatePresetCb cb);
