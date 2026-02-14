/**
 * @file display_ui.cpp
 * @brief Implementation of multi-screen display UI for LilyGo T-Display S3
 */

#include "display_ui.h"

#if HAS_DISPLAY

#include "PubSubClient.h"
#include <WiFi.h>

// External references from main.cpp
extern char deviceId[];  // Array, not pointer
extern PubSubClient client;
extern String wifi_status;

#if HAS_BLE
#include "ruuvi_scanner.h"  // Includes ruuvi.h and provides RuuviTag, MAX_RUUVI_TAGS, getFreshTags
#endif

#if HAS_WIRED_SENSORS
#include "wired_sensors.h"
extern WiredSensors wiredSensors;
extern bool wiredSensorsEnabled;
#endif

#ifdef HEATER_ENABLED
#include "heater_addon.h"
#endif

// Static member initialization
DisplayUI* DisplayUI::instance = nullptr;
DisplayUI displayUI;

// Interrupt flags
volatile bool DisplayUI::button1Pressed = false;
volatile bool DisplayUI::button2Pressed = false;
volatile unsigned long DisplayUI::button1LastPress = 0;
volatile unsigned long DisplayUI::button2LastPress = 0;

// ISR handlers - must be in IRAM for fast access
void IRAM_ATTR DisplayUI::button1ISR() {
    unsigned long now = millis();
    // Simple debounce: ignore if pressed within last 200ms
    if (now - button1LastPress > 200) {
        button1Pressed = true;
        button1LastPress = now;
    }
}

void IRAM_ATTR DisplayUI::button2ISR() {
    unsigned long now = millis();
    // Simple debounce: ignore if pressed within last 200ms
    if (now - button2LastPress > 200) {
        button2Pressed = true;
        button2LastPress = now;
    }
}

// Button callback implementations
void DisplayUI::onButton1Click() {
    Serial.println("[DEBUG] Button 1 clicked!");
    if (instance) {
        instance->toggleDisplay();
    } else {
        Serial.println("[ERROR] DisplayUI instance is null!");
    }
}

void DisplayUI::onButton2Click() {
    Serial.println("[DEBUG] Button 2 clicked!");
    if (instance) {
        if (instance->isDisplayOn()) {
            // Prevent rapid screen switching (min 300ms between switches)
            static unsigned long lastSwitch = 0;
            unsigned long now = millis();
            if (now - lastSwitch > 300) {
                instance->nextScreen();
                lastSwitch = now;
            } else {
                Serial.println("[DEBUG] Button press ignored - too fast");
            }
        } else {
            Serial.println("[DEBUG] Display is off, ignoring button 2");
        }
    } else {
        Serial.println("[ERROR] DisplayUI instance is null!");
    }
}

void DisplayUI::begin(TFT_eSPI* display) {
    tft = display;
    instance = this;
    
    // Configure button pins as inputs with pullup
    pinMode(PIN_BUTTON_1, INPUT_PULLUP);
    pinMode(PIN_BUTTON_2, INPUT_PULLUP);
    
    // Attach interrupts (FALLING = button press, since active low)
    attachInterrupt(digitalPinToInterrupt(PIN_BUTTON_1), button1ISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(PIN_BUTTON_2), button2ISR, FALLING);
    
    // Initialize state
    currentScreen = SCREEN_STATUS;
    displayEnabled = true;
    lastUpdate = 0;
    updateInterval = 3000;  // Update every 3 seconds to reduce power consumption
    wifiStatus = "Disconnected";
    mqttConnected = false;
    button2Down = false;
    button2DownTime = 0;
    button2LongHandled = false;
    
    Serial.println("Display UI initialized (interrupt-based buttons)");
    Serial.println("Button 1 (PIN 0): Toggle display on/off");
    Serial.println("Button 2 (PIN 14): Switch screens");
    Serial.printf("Instance pointer: %p\n", instance);
}

void DisplayUI::update() {
    if (!tft) return;
    
    // Check interrupt flags and handle button presses
    if (button1Pressed) {
        button1Pressed = false;  // Clear flag
        onButton1Click();
    }
    
    if (button2Pressed) {
        button2Pressed = false;  // Clear flag
        // Mark button as down — we'll decide short vs long on release
        if (!button2Down) {
            button2Down = true;
            button2DownTime = millis();
            button2LongHandled = false;
            Serial.printf("[BTN2] Down at %lu, screen=%d\n", button2DownTime, currentScreen);
        }
    }
    
    // Long-press detection: check while button is held
    if (button2Down) {
        bool pinState = digitalRead(PIN_BUTTON_2);  // Active low
        unsigned long held = millis() - button2DownTime;
        
        if (pinState == LOW && held >= LONG_PRESS_MS && !button2LongHandled) {
            // Long press detected!
            button2LongHandled = true;
            Serial.printf("[BTN2] LONG PRESS detected (%lu ms), screen=%d, display=%d\n",
                          held, currentScreen, displayEnabled);
#ifdef HEATER_ENABLED
            if (displayEnabled && currentScreen == SCREEN_HEATER) {
                toggleHeater();
                lastUpdate = 0;  // Force immediate redraw
            } else {
                Serial.printf("[BTN2] Long press ignored: need SCREEN_HEATER(%d) but on %d\n",
                              SCREEN_HEATER, currentScreen);
            }
#endif
        } else if (pinState == HIGH) {
            // Button released
            Serial.printf("[BTN2] Released after %lu ms (longHandled=%d)\n",
                          held, button2LongHandled);
            if (!button2LongHandled && held < LONG_PRESS_MS) {
                // Short press — switch screen
                onButton2Click();
            }
            button2Down = false;
        }
    }
    
    // Update display if enabled and interval has passed
    if (displayEnabled && (millis() - lastUpdate >= updateInterval)) {
        refresh();
        lastUpdate = millis();
    }
}

void DisplayUI::refresh() {
    if (!tft || !displayEnabled) return;
    
    // Power optimization: Skip refresh if not enough time has passed
    unsigned long now = millis();
    if (now - lastUpdate < updateInterval) {
        return;
    }
    
    switch (currentScreen) {
        case SCREEN_STATUS:
            renderStatusScreen();
            break;
        case SCREEN_SENSORS:
            renderSensorScreen();
            break;
        case SCREEN_WIRED:
            renderWiredSensorScreen();
            break;
        case SCREEN_SYSTEM:
            renderSystemScreen();
            break;
        case SCREEN_NETWORK:
            renderNetworkScreen();
            break;
#ifdef HEATER_ENABLED
        case SCREEN_HEATER:
            renderHeaterScreen();
            break;
#endif
        default:
            currentScreen = SCREEN_STATUS;
            renderStatusScreen();
            break;
    }
}

void DisplayUI::displayOn() {
    displayEnabled = true;
    
    // Turn on backlight at 40% brightness to reduce heat and save power
    // (TFT is easily readable at this level; increase if needed)
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5,0,0)
    ledcWrite(0, 100);  // ~40% brightness
#else
    ledcWrite(PIN_LCD_BL, 100);  // ~40% brightness
#endif
    
    // Force immediate refresh
    lastUpdate = millis() - updateInterval;
    Serial.println("Display ON (40% brightness)");
}

void DisplayUI::displayOff() {
    displayEnabled = false;
    
    // Turn off backlight
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5,0,0)
    ledcWrite(0, 0);
#else
    ledcWrite(PIN_LCD_BL, 0);
#endif
    
    // Clear screen
    tft->fillScreen(TFT_BLACK);
    Serial.println("Display OFF");
}

void DisplayUI::toggleDisplay() {
    if (displayEnabled) {
        displayOff();
    } else {
        displayOn();
    }
}

void DisplayUI::nextScreen() {
    currentScreen = (DisplayScreen)((currentScreen + 1) % SCREEN_COUNT);
    Serial.print("Switched to screen: ");
    Serial.println(currentScreen);
    
    // Force immediate refresh by resetting timer
    lastUpdate = 0;
}

void DisplayUI::prevScreen() {
    currentScreen = (DisplayScreen)((currentScreen - 1 + SCREEN_COUNT) % SCREEN_COUNT);
    refresh();
}

void DisplayUI::clearScreen() {
    tft->fillScreen(TFT_BLACK);
}

void DisplayUI::drawHeader(const char* title) {
    tft->setTextSize(3);  // Increased from 2
    tft->setTextColor(TFT_CYAN, TFT_BLACK);
    tft->setCursor(0, 0);
    tft->println(title);
    tft->drawLine(0, 28, 320, 28, TFT_CYAN);  // Adjusted line position
}

void DisplayUI::drawFooter() {
    // Draw navigation hint at bottom
    tft->drawLine(0, 145, 320, 145, TFT_DARKGREY);
    tft->setTextSize(2);  // Increased from 1
    tft->setTextColor(TFT_DARKGREY, TFT_BLACK);
    tft->setCursor(5, 150);
    tft->printf("[%d/%d]", currentScreen + 1, SCREEN_COUNT);
}

String DisplayUI::getUptimeString() {
    unsigned long uptimeMs = millis();
    unsigned long seconds = uptimeMs / 1000;
    unsigned long minutes = seconds / 60;
    unsigned long hours = minutes / 60;
    unsigned long days = hours / 24;
    
    hours = hours % 24;
    minutes = minutes % 60;
    seconds = seconds % 60;
    
    char buffer[32];
    if (days > 0) {
        snprintf(buffer, sizeof(buffer), "%lud %02lu:%02lu:%02lu", days, hours, minutes, seconds);
    } else {
        snprintf(buffer, sizeof(buffer), "%02lu:%02lu:%02lu", hours, minutes, seconds);
    }
    return String(buffer);
}

void DisplayUI::renderStatusScreen() {
    clearScreen();
    drawHeader("Status");
    
    tft->setTextSize(2);
    tft->setTextColor(TFT_WHITE, TFT_BLACK);
    int y = 32;
    
    // WiFi Status
    tft->setCursor(5, y);
    tft->print("WiFi: ");
    if (WiFi.status() == WL_CONNECTED) {
        tft->setTextColor(TFT_GREEN, TFT_BLACK);
        tft->println("OK");
    } else {
        tft->setTextColor(TFT_RED, TFT_BLACK);
        tft->println("NO");
    }
    
    // MQTT Status
    y += 18;
    tft->setTextColor(TFT_WHITE, TFT_BLACK);
    tft->setCursor(5, y);
    tft->print("MQTT: ");
    if (client.connected()) {
        tft->setTextColor(TFT_GREEN, TFT_BLACK);
        tft->println("OK");
    } else {
        tft->setTextColor(TFT_RED, TFT_BLACK);
        tft->println("NO");
    }
    
    // Uptime
    y += 18;
    tft->setTextColor(TFT_WHITE, TFT_BLACK);
    tft->setCursor(5, y);
    tft->print("Up: ");
    tft->println(getUptimeString());
    
    // Device ID
    y += 18;
    tft->setCursor(5, y);
    tft->print("ID: ");
    tft->println(deviceId);
    
    // Memory info
    y += 18;
    tft->setCursor(5, y);
    tft->print("Mem: ");
    tft->print(ESP.getFreeHeap() / 1024);
    tft->println("KB");
    
    // CPU Frequency
    y += 18;
    tft->setCursor(5, y);
    tft->print("CPU: ");
    tft->print(ESP.getCpuFreqMHz());
    tft->println("MHz");
    
    drawFooter();
}

void DisplayUI::renderSensorScreen() {
    clearScreen();
    drawHeader("Sensors");
    
    tft->setTextSize(2);
    tft->setTextColor(TFT_WHITE, TFT_BLACK);
    
#if HAS_BLE
    const RuuviTag* freshTags[MAX_RUUVI_TAGS];
    uint8_t freshCount = getFreshTags(freshTags, MAX_RUUVI_TAGS, millis());
    
    if (freshCount == 0) {
        tft->setCursor(5, 70);
        tft->setTextColor(TFT_YELLOW, TFT_BLACK);
        tft->println("No sensors");
        tft->setTextColor(TFT_DARKGREY, TFT_BLACK);
        tft->setCursor(5, 90);
        tft->println("Scanning...");
    } else {
        int y = 32;
        for (uint8_t i = 0; i < freshCount && i < 5; i++) {  // Show max 5 tags
            const RuuviTag* tag = freshTags[i];
            if (tag->hasData && tag->lastData.valid) {
                tft->setCursor(5, y);
                tft->setTextColor(TFT_CYAN, TFT_BLACK);
                tft->print(tag->location);
                tft->println(":");
                
                y += 18;
                tft->setCursor(8, y);
                tft->setTextColor(TFT_WHITE, TFT_BLACK);
                tft->printf("%.1fC %.0f%% %.0fhPa", 
                    tag->lastData.temperature,
                    tag->lastData.humidity,
                    tag->lastData.pressure);
                y += 20;
            }
        }
    }
#else
    tft->setCursor(5, 70);
    tft->setTextColor(TFT_YELLOW, TFT_BLACK);
    tft->println("BLE N/A");
#endif
    
    drawFooter();
}

void DisplayUI::renderWiredSensorScreen() {
    clearScreen();
    drawHeader("Wired");
    
    tft->setTextSize(2);
    tft->setTextColor(TFT_WHITE, TFT_BLACK);
    
#if HAS_WIRED_SENSORS
    if (wiredSensorsEnabled) {
        int y = 32;
        tft->setCursor(5, y);
        tft->print("Type: ");
        tft->println(wiredSensors.getSensorType());
        
        y += 18;
        tft->setCursor(5, y);
        tft->print("Count: ");
        tft->println(wiredSensors.getSensorCount());
        
        // Read current values
        float temp1 = 0.0f, temp2 = 0.0f;
        if (wiredSensors.read(temp1, temp2)) {
            y += 22;
            tft->setCursor(5, y);
            tft->setTextColor(TFT_CYAN, TFT_BLACK);
            tft->println("Sensor 1:");
            
            y += 18;
            tft->setCursor(10, y);
            tft->setTextSize(3);
            tft->setTextColor(TFT_GREEN, TFT_BLACK);
            tft->printf("%.1fC", temp1);
            
            if (wiredSensors.getSensorCount() > 1) {
                y += 26;
                tft->setTextSize(2);
                tft->setCursor(5, y);
                tft->setTextColor(TFT_CYAN, TFT_BLACK);
                tft->println("Sensor 2:");
                
                y += 18;
                tft->setCursor(10, y);
                tft->setTextSize(3);
                tft->setTextColor(TFT_GREEN, TFT_BLACK);
                tft->printf("%.1fC", temp2);
            }
        } else {
            y += 22;
            tft->setCursor(5, y);
            tft->setTextColor(TFT_RED, TFT_BLACK);
            tft->println("Read error");
        }
    } else {
        tft->setCursor(5, 70);
        tft->setTextColor(TFT_YELLOW, TFT_BLACK);
        tft->println("No sensors");
        tft->setCursor(5, 90);
        tft->setTextColor(TFT_DARKGREY, TFT_BLACK);
        tft->println("Check wiring");
    }
#else
    tft->setCursor(5, 70);
    tft->setTextColor(TFT_YELLOW, TFT_BLACK);
    tft->println("Disabled");
#endif
    
    drawFooter();
}

void DisplayUI::renderSystemScreen() {
    clearScreen();
    drawHeader("System");
    
    tft->setTextSize(2);
    tft->setTextColor(TFT_WHITE, TFT_BLACK);
    
    int y = 32;
    
    // Device ID
    tft->setCursor(5, y);
    tft->print("ID: ");
    tft->setTextColor(TFT_CYAN, TFT_BLACK);
    tft->println(deviceId);
    
    // Firmware Version
    y += 18;
    tft->setTextColor(TFT_WHITE, TFT_BLACK);
    tft->setCursor(5, y);
    tft->print("FW: ");
    tft->setTextColor(TFT_CYAN, TFT_BLACK);
    #ifdef FIRMWARE_VERSION
    tft->println(FIRMWARE_VERSION);
    #else
    tft->println("N/A");
    #endif
    
    // Chip Model
    y += 18;
    tft->setTextColor(TFT_WHITE, TFT_BLACK);
    tft->setCursor(5, y);
    tft->print("Chip: ");
    tft->println(ESP.getChipModel());
    
    // Flash Size
    y += 18;
    tft->setCursor(5, y);
    tft->print("Flash: ");
    tft->print(ESP.getFlashChipSize() / (1024 * 1024));
    tft->println("MB");
    
    // PSRAM
    y += 18;
    tft->setCursor(5, y);
    tft->print("PSRAM: ");
    if (ESP.getPsramSize() > 0) {
        tft->print(ESP.getPsramSize() / (1024 * 1024));
        tft->println("MB");
    } else {
        tft->println("N/A");
    }
    
    drawFooter();
}

void DisplayUI::renderNetworkScreen() {
    clearScreen();
    drawHeader("Network");
    
    tft->setTextSize(2);
    tft->setTextColor(TFT_WHITE, TFT_BLACK);
    
    int y = 32;
    
    if (WiFi.status() == WL_CONNECTED) {
        // SSID
        tft->setCursor(5, y);
        tft->print("SSID: ");
        tft->setTextColor(TFT_CYAN, TFT_BLACK);
        tft->println(WiFi.SSID());
        
        // IP Address
        y += 18;
        tft->setTextColor(TFT_WHITE, TFT_BLACK);
        tft->setCursor(5, y);
        tft->print("IP: ");
        tft->setTextColor(TFT_CYAN, TFT_BLACK);
        tft->println(WiFi.localIP());
        
        // Signal Strength
        y += 18;
        tft->setTextColor(TFT_WHITE, TFT_BLACK);
        tft->setCursor(5, y);
        tft->print("Signal: ");
        int rssi = WiFi.RSSI();
        if (rssi > -50) {
            tft->setTextColor(TFT_GREEN, TFT_BLACK);
        } else if (rssi > -70) {
            tft->setTextColor(TFT_YELLOW, TFT_BLACK);
        } else {
            tft->setTextColor(TFT_RED, TFT_BLACK);
        }
        tft->print(rssi);
        tft->println(" dBm");
        
        // MAC Address
        y += 18;
        tft->setTextColor(TFT_WHITE, TFT_BLACK);
        tft->setCursor(5, y);
        tft->print("MAC: ");
        tft->setTextColor(TFT_DARKGREY, TFT_BLACK);
        tft->println(WiFi.macAddress());
        
    } else {
        tft->setCursor(5, 70);
        tft->setTextColor(TFT_RED, TFT_BLACK);
        tft->println("No WiFi");
    }
    
    drawFooter();
}

// ===========================================================================
// Heater screen (only when compiled with -D HEATER_ENABLED)
// ===========================================================================
#ifdef HEATER_ENABLED

void DisplayUI::renderHeaterScreen() {
    clearScreen();
    drawHeader("Heater");

    AutotermUart&  uart   = heater_addon_getUart();
    HeaterSafety&  safety = heater_addon_getSafety();
    const auto&    st     = uart.getStatus();

    tft->setTextSize(2);
    int y = 32;

    // --- Row 1: Heater state (large, colored) ---
    tft->setCursor(5, y);
    tft->setTextColor(TFT_WHITE, TFT_BLACK);
    tft->print("State: ");

    if (safety.isTripped()) {
        tft->setTextColor(TFT_RED, TFT_BLACK);
        tft->println("TRIPPED");
    } else if (!uart.isOnline()) {
        tft->setTextColor(TFT_DARKGREY, TFT_BLACK);
        tft->println("OFFLINE");
    } else if (st.valid) {
        switch (st.state) {
            case AutotermState::Running:
                tft->setTextColor(TFT_GREEN, TFT_BLACK);
                break;
            case AutotermState::Starting:
            case AutotermState::Warming:
                tft->setTextColor(TFT_YELLOW, TFT_BLACK);
                break;
            case AutotermState::ShuttingDown:
            case AutotermState::Cooling:
                tft->setTextColor(TFT_ORANGE, TFT_BLACK);
                break;
            default:
                tft->setTextColor(TFT_WHITE, TFT_BLACK);
                break;
        }
        tft->println(autotermStateName(st.state));
    } else {
        tft->setTextColor(TFT_DARKGREY, TFT_BLACK);
        tft->println("---");
    }

    // --- Row 2: Coolant temp ---
    y += 20;
    tft->setCursor(5, y);
    tft->setTextColor(TFT_WHITE, TFT_BLACK);
    tft->print("Coolant: ");
    float coolant = safety.getCoolantTemp();
    if (isnan(coolant)) {
        tft->setTextColor(TFT_DARKGREY, TFT_BLACK);
        tft->println("N/A");
    } else {
        if (coolant > 85.0f) {
            tft->setTextColor(TFT_RED, TFT_BLACK);
        } else if (coolant > 60.0f) {
            tft->setTextColor(TFT_YELLOW, TFT_BLACK);
        } else {
            tft->setTextColor(TFT_GREEN, TFT_BLACK);
        }
        tft->printf("%.1fC\n", coolant);
    }

    // --- Row 3: Flow rate ---
    y += 20;
    tft->setCursor(5, y);
    tft->setTextColor(TFT_WHITE, TFT_BLACK);
    tft->print("Flow: ");
    float flow = safety.getFlowRate();
    if (flow > 0.5f) {
        tft->setTextColor(TFT_GREEN, TFT_BLACK);
    } else {
        tft->setTextColor(TFT_RED, TFT_BLACK);
    }
    tft->printf("%.1f L/m\n", flow);

    // --- Row 4: Battery voltage ---
    y += 20;
    tft->setCursor(5, y);
    tft->setTextColor(TFT_WHITE, TFT_BLACK);
    tft->print("Battery: ");
    if (st.valid) {
        if (st.voltage < 11.5f) {
            tft->setTextColor(TFT_RED, TFT_BLACK);
        } else if (st.voltage < 12.2f) {
            tft->setTextColor(TFT_YELLOW, TFT_BLACK);
        } else {
            tft->setTextColor(TFT_GREEN, TFT_BLACK);
        }
        tft->printf("%.1fV\n", st.voltage);
    } else {
        tft->setTextColor(TFT_DARKGREY, TFT_BLACK);
        tft->println("---");
    }

    // --- Row 5: Safety trip detail (if tripped) ---
    if (safety.isTripped()) {
        y += 20;
        tft->setCursor(5, y);
        tft->setTextColor(TFT_RED, TFT_BLACK);
        uint8_t reason = safety.getTripReason();
        if (reason & TRIP_COOLANT_OVERHEAT) tft->print("T1 ");
        if (reason & TRIP_FLOW_LOSS)        tft->print("F1 ");
        if (reason & TRIP_UART_TIMEOUT)     tft->print("U1 ");
        if (reason & TRIP_HEATER_ERROR)     tft->print("E1 ");
    }

    // --- Footer: show long-press hint on this screen ---
    tft->drawLine(0, 145, 320, 145, TFT_DARKGREY);
    tft->setTextSize(2);
    tft->setTextColor(TFT_DARKGREY, TFT_BLACK);
    tft->setCursor(5, 150);
    tft->printf("[%d/%d]", currentScreen + 1, SCREEN_COUNT);

    // Right-aligned hint
    bool heaterRunning = st.valid &&
        (st.state == AutotermState::Starting ||
         st.state == AutotermState::Warming  ||
         st.state == AutotermState::Running);
    tft->setCursor(150, 150);
    if (safety.isTripped()) {
        tft->setTextColor(TFT_RED, TFT_BLACK);
        tft->print("Hold:Reset");
    } else if (heaterRunning) {
        tft->setTextColor(TFT_ORANGE, TFT_BLACK);
        tft->print("Hold:Stop");
    } else {
        tft->setTextColor(TFT_GREEN, TFT_BLACK);
        tft->print("Hold:Start");
    }
}

void DisplayUI::toggleHeater() {
    AutotermUart&  uart   = heater_addon_getUart();
    HeaterSafety&  safety = heater_addon_getSafety();
    const auto&    st     = uart.getStatus();

    if (safety.isTripped()) {
        // Long-press on tripped screen = try to clear trip
        if (safety.clearTrip()) {
            Serial.println("[Display] Safety trip cleared via button");
        } else {
            Serial.println("[Display] Cannot clear trip — conditions not met");
        }
        return;
    }

    bool heaterRunning = st.valid &&
        (st.state == AutotermState::Starting ||
         st.state == AutotermState::Warming  ||
         st.state == AutotermState::Running);

    if (heaterRunning) {
#ifdef HEATER_EMULATE
        // Route through command handler so emulation state is updated
        static const char stopCmd[] = "{\"cmd\":\"stop\"}";
        heater_addon_command(stopCmd, strlen(stopCmd));
#else
        uart.shutdown();
#endif
        Serial.println("[Display] Heater STOP via button");
    } else {
#ifdef HEATER_EMULATE
        static const char startCmd[] = "{\"cmd\":\"start\",\"power\":5}";
        heater_addon_command(startCmd, strlen(startCmd));
#else
        uart.start(MODE_BY_POWER, 0xFF, 5);
#endif
        Serial.println("[Display] Heater START via button (power 5)");
    }
}

#endif // HEATER_ENABLED

#endif // HAS_DISPLAY
