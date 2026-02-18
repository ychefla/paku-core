/**
 * @file display_ui.h
 * @brief Multi-screen display UI for LilyGo T-Display S3
 * 
 * Provides a user interface with multiple screens that can be navigated using buttons:
 * - Button 1 (PIN 0): Turn display on/off
 * - Button 2 (PIN 14): Switch between screens
 * 
 * Available screens:
 * 1. Status Screen - WiFi, MQTT, system status
 * 2. Sensor Screen - RuuviTag sensor data
 * 3. Wired Sensor Screen - DS18B20 temperature data
 * 4. System Info Screen - Device ID, firmware version, uptime
 * 5. Network Screen - IP address, signal strength, connection details
 */

#pragma once

#include "Arduino.h"
#include "device_config.h"

#if HAS_DISPLAY
#include "TFT_eSPI.h"
#include "pin_config.h"

// Display screen enumeration
enum DisplayScreen {
    SCREEN_STATUS = 0,      // WiFi, MQTT status
    SCREEN_SENSORS,         // RuuviTag sensor data
    SCREEN_WIRED,           // Wired sensor data (DS18B20)
    SCREEN_SYSTEM,          // System info (Device ID, firmware, uptime)
    SCREEN_NETWORK,         // Network details (IP, signal)
#ifdef HEATER_ENABLED
    SCREEN_HEATER,          // Hydronic heater status + control
#endif
#if HAS_FAN_IR
    SCREEN_FAN,             // MaxxFan IR control
#endif
#if HAS_MILIGHT
    SCREEN_LIGHT,           // MiLight/MIBO light control
#endif
    SCREEN_COUNT            // Total number of screens
};

/**
 * @brief Display UI Manager class
 * Handles button input, screen switching, and display rendering
 */
class DisplayUI {
public:
    /**
     * @brief Initialize the display UI system
     * @param display Pointer to TFT_eSPI display object
     */
    void begin(TFT_eSPI* display);
    
    /**
     * @brief Update button states and handle screen updates
     * Should be called frequently in main loop
     */
    void update();
    
    /**
     * @brief Force a screen refresh
     */
    void refresh();
    
    /**
     * @brief Turn display on
     */
    void displayOn();
    
    /**
     * @brief Turn display off
     */
    void displayOff();
    
    /**
     * @brief Toggle display on/off
     */
    void toggleDisplay();
    
    /**
     * @brief Switch to next screen
     */
    void nextScreen();
    
    /**
     * @brief Switch to previous screen
     */
    void prevScreen();
    
    /**
     * @brief Check if display is currently on
     */
    bool isDisplayOn() const { return displayEnabled; }
    
    /**
     * @brief Get current screen
     */
    DisplayScreen getCurrentScreen() const { return currentScreen; }
    
    /**
     * @brief Set WiFi status text
     */
    void setWiFiStatus(const String& status) { wifiStatus = status; }
    
    /**
     * @brief Set MQTT connection status
     */
    void setMQTTConnected(bool connected) { mqttConnected = connected; }
    
    /**
     * @brief Set device ID
     */
    void setDeviceId(const char* id) { deviceId = id; }
    
private:
    TFT_eSPI* tft;
    
    DisplayScreen currentScreen;
    bool displayEnabled;
    unsigned long lastUpdate;
    unsigned long updateInterval;   // Screen refresh interval in ms
    
    String wifiStatus;
    bool mqttConnected;
    const char* deviceId;
    
    // Interrupt flags
    static volatile bool button1Pressed;
    static volatile bool button2Pressed;
    static volatile unsigned long button1LastPress;
    static volatile unsigned long button2LastPress;
    
    // Long-press detection for Button 2
    bool button2Down;                    ///< True while button is held
    unsigned long button2DownTime;       ///< millis() when press started
    bool button2LongHandled;             ///< Long-press already acted on
    static constexpr unsigned long LONG_PRESS_MS = 800;  ///< Hold threshold
    
    // Button callback functions (must be static)
    static void onButton1Click();
    static void onButton2Click();
    
    // ISR handlers (must be static and IRAM_ATTR)
    static void IRAM_ATTR button1ISR();
    static void IRAM_ATTR button2ISR();
    
    // Screen rendering functions
    void renderStatusScreen();
    void renderSensorScreen();
    void renderWiredSensorScreen();
    void renderSystemScreen();
    void renderNetworkScreen();
#ifdef HEATER_ENABLED
    void renderHeaterScreen();
    void toggleHeater();  ///< Start or stop heater (called on long-press)
#endif
#if HAS_FAN_IR
    void renderFanScreen();
    void toggleFan();     ///< Toggle fan power (called on long-press)
#endif
#if HAS_MILIGHT
    void renderLightScreen();
    void toggleLight();    ///< Toggle light on/off (called on long-press)
    void nextLightZone();  ///< Cycle to next zone (called on short-press)
#endif
    
    // Helper functions
    void drawHeader(const char* title);
    void drawFooter();
    void clearScreen();
    String getUptimeString();
    
    // Static instance pointer for button callbacks
    static DisplayUI* instance;
};

// Global display UI instance
extern DisplayUI displayUI;

#endif // HAS_DISPLAY
