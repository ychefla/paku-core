# ESP32 Touchscreen GUI Requirements

## Document Information
- **Created:** Based on smart-home-prototype.html analysis
- **Target Hardware:** Waveshare ESP32-S3-Touch-LCD-4.3" and 5" displays  
- **Resolution:** 800x480 pixels
- **Touch Controller:** GT911 capacitive touch
- **Framework Target:** LVGL with ESP32 Arduino framework

## System Overview

### Hardware Requirements
- **Display Specifications**
  - Resolution: 800x480 pixels
  - Interface: RGB565 (16-bit parallel)
  - Touch: GT911 capacitive touch controller
  - Supported boards: Waveshare ESP32-S3-Touch-LCD-4.3 & 5.0
  - Display driver: Arduino_GFX or similar RGB parallel interface library

### Functional Requirements

#### FR-001: Navigation System
- **Requirement:** Vertical sidebar navigation with 6 tabs (icon-only navigation)
- **Implementation:** 
  - Sidebar width: 100px
  - Tab icons: FontAwesome 6.4.0+ icons (24px) - icons only, no text labels
  - Active state: Orange accent (#ff9800) with 4px left border
  - Tabs: Main (house icon), Climate, Lights, Sensors, Power, Settings

#### FR-002: System Header
- **Requirement:** Top status bar showing system information
- **Implementation:**
  - Height: 40px
  - Left section: WiFi/MQTT status indicators with color coding
  - Center section: Current time (HH:MM format)
  - Right section: Heater status and screen off button
  - Status colors: Green (connected), Orange (connecting), Red (error)

#### FR-003: Main Dashboard (Tab 1)
- **Requirement:** Quick preset access for lighting and climate
- **Implementation:**
  - Two-column layout: Lighting presets | Climate presets
  - Each section: 2x2 grid (4 preset buttons)
  - Lighting presets: "All Off", "All On", "Kitchen Only", "Night Light"
  - Climate presets: "All Off", "Heat", "Max Out Flow", "Night Setting" 
  - Button size: Expandable to fill grid
  - Icons: 48px FontAwesome icons
  - Active state: Orange border and background tint

#### FR-004: Climate Control (Tab 2)
- **Requirement:** Diesel heater and roof fan control
- **Implementation:**
  - Two-panel layout: Fan control | Heater control
  - **Fan Controls:**
    - Direction toggle: "Air In" / "Air Out" buttons
    - Lid control: "Lid Open" / "Lid Closed" toggle
    - Speed slider: 0-100% with real-time percentage display
  - **Heater Controls:**
    - Large power button: 80px circular button with on/off states
    - Temperature target: Large display (56px) with degree symbol
    - Temperature adjust: +/- buttons with slider control (10-30°C range)
    - Full power override: Emergency override toggle button

#### FR-005: Lighting Control (Tab 3)
- **Requirement:** Master and individual zone lighting control
- **Implementation:**
  - **Master Controls Section:**
    - ALL ON / ALL OFF buttons (full width, side-by-side)
    - Master brightness slider (0-100%)
    - Master color temperature slider (2700K-6500K)
  - **Individual Zone Grid (2x2):**
    - Zones: Living Area, Kitchen, Bedroom, Outside
    - Per-zone controls: Power toggle switch, brightness mini-slider, color temp mini-slider
    - Visual feedback: Green border when zone is active
    - Real-time value display for each zone setting

#### FR-006: Environmental Sensors (Tab 4)
- **Requirement:** Temperature and humidity trend visualization
- **Implementation:**
  - **Temperature Graph:**
    - Data series: Indoor, Outdoor, Fridge, Reppu (mobile sensor)
    - Color coding: Red (#ff5252), Blue (#448aff), Green (#69f0ae), Yellow (#ffd740)
    - Grid lines with dashed stroke pattern
    - Legend showing current values
  - **Humidity Graph:**
    - Data series: Indoor, Outdoor, Reppu
    - Color coding: Cyan (#00e5ff), Purple (#b388ff), Pink (#ff8a80)
    - Percentage-based Y-axis scaling

#### FR-007: Power Management (Tab 5)
- **Requirement:** Power input/output monitoring with graphs
- **Implementation:**
  - **Input Monitoring:**
    - Solar power visualization (orange area chart)
    - AC charger status (green area chart)
    - Stacked area chart showing combined inputs
    - Real-time wattage display for each source
  - **Output Monitoring:**  
    - AC inverter consumption (red area chart)
    - 12V DC base load (blue area chart)
    - Stacked area chart showing combined outputs
    - Real-time wattage display for each consumer

#### FR-008: Settings/Configuration (Tab 6)
- **Requirement:** Comprehensive system configuration and management interface
- **Implementation:**
  - **Display Controls:** Brightness slider (10-100%), theme selection, sleep timer settings
  - **Network Information:** WiFi status, signal strength, IP address, MQTT broker connection
  - **Device Information:** Firmware version, memory usage, uptime, CPU temperature
  - **System Actions:** Export logs, clear cache, restart, and factory reset with confirmation dialogs
  - **Layout:** Multi-card layout with grouped settings, scrollable content for full functionality
  - **Scrolling:** Vertical scrolling enabled to accommodate all configuration options

#### FR-009: Preset Management
- **Requirement:** Save and recall custom presets for lighting and climate
- **Implementation:**
  - Modal dialog overlay for preset saving
  - Context-sensitive preset options (lighting vs climate)
  - Preset categories with predefined slots
  - Visual confirmation of save actions
  - Accessible from save buttons in Climate and Lights tabs

### Non-Functional Requirements

#### NFR-001: Performance
- Target refresh rate: 30 FPS minimum for smooth interactions
- Touch response time: \<100ms from touch to visual feedback
- Tab switching animation: 300ms fade-in transition
- Memory usage: Optimize for ESP32-S3 constraints
- **Scrolling support:** Vertical scrolling enabled for tabs with extensive content

#### NFR-002: Visual Design
- **Dark Theme:** 
  - Background: #121212 (base), #1e1e1e (panels), #2c2c2c (cards)
  - Accent colors: Orange (#ff9800), Blue (#03a9f4), Green (#4caf50)
  - Text hierarchy: White primary, #aaaaaa secondary
- **Typography:** 'Segoe UI' fallback font stack
- **Touch targets:** Minimum 44px for accessibility
- **Visual feedback:** Color state changes, border highlights, shadows
- **Scrolling interface:** Custom scrollbars with accent color theming for overflow content

#### NFR-003: Hardware Integration
- **Touch calibration:** Support for GT911 touch controller calibration
- **Display initialization:** Proper RGB565 display setup with Arduino_GFX
- **Performance optimization:** Hardware-accelerated graphics where possible
- **Power management:** Display brightness control and sleep mode support

#### NFR-004: Data Integration
- **MQTT connectivity:** Real-time data updates from paku-iot backend
- **Sensor data:** Integration with BLE sensors (Ruuvi tags) for environmental data
- **Heater control:** Integration with hydronic-heater system for climate control
- **State persistence:** Save current settings to ESP32 NVS (Non-Volatile Storage)

### Technical Implementation Notes

#### LVGL Considerations
- Use LVGL's built-in slider, button, and chart widgets
- Implement custom drawing for complex graphs using LVGL draw functions
- Utilize LVGL's theme system for consistent color management
- Leverage LVGL's animation framework for smooth transitions
- **Scrolling support:** Implement LVGL scroll containers for content overflow management
- **Touch handling:** Support touch scrolling with momentum and boundary detection

#### Memory Management
- Pre-allocate buffers for graph data to avoid dynamic allocation
- Use LVGL's memory management for widget creation/destruction
- Consider dual-buffer rendering for smooth graphics updates
- Optimize image assets (icons) for minimal memory footprint

#### Touch Interaction
- Implement proper touch event handling for sliders and buttons
- Support gesture recognition for potential future swipe navigation
- Add touch feedback animations (button press states)
- Handle multi-touch scenarios gracefully

#### Communication Protocol
- Define MQTT message format for bidirectional device communication
- Implement JSON parsing for configuration updates
- Handle connection state management (WiFi/MQTT reconnection)
- Support OTA updates through existing paku-core infrastructure

### Testing Requirements

#### Hardware Testing
- Verify touch calibration across full display area
- Test display performance under various lighting conditions
- Validate power consumption in active and sleep modes
- Confirm thermal behavior during extended operation

#### Integration Testing
- Test real-time data flow from paku-iot services
- Verify control commands reach target devices (heater, lights)
- Validate preset save/load functionality
- Test system behavior during network disconnections

#### User Experience Testing  
- Verify intuitive navigation flow between tabs
- Test slider precision and responsiveness
- Validate visual feedback for all interactive elements
- Confirm readability under various viewing angles