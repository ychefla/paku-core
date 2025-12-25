# LilyGo T-Display S3 User Interface Guide

## Overview

The paku-core firmware now includes a comprehensive multi-screen user interface for the LilyGo T-Display S3, with intuitive button controls for easy navigation and power management.

## Hardware

### Display
- **Model**: ST7789V TFT LCD
- **Resolution**: 320x170 pixels
- **Interface**: 8-bit parallel

### Buttons
- **Button 1** (GPIO 0): Display ON/OFF toggle
- **Button 2** (GPIO 14): Screen navigation

## Features

### Button Controls

#### Button 1 - Display Power
- **Single Click**: Toggle display on/off
- **Behavior**: 
  - When OFF: Display goes black and backlight turns off to save power
  - When ON: Display turns on and shows the current screen
  - WiFi, MQTT, and sensors continue to operate regardless of display state

#### Button 2 - Screen Navigation
- **Single Click**: Switch to next screen (only when display is on)
- **Behavior**: Cycles through all available screens in order

### Available Screens

The UI provides 5 different screens with relevant system information:

#### 1. Status Screen (Default)
Displays overall system status:
- WiFi connection status (Connected/Disconnected with color coding)
- MQTT connection status (Connected/Disconnected with color coding)
- System uptime (days, hours, minutes, seconds)
- Device ID
- Free heap memory (KB)
- CPU frequency (MHz)

#### 2. BLE Sensors Screen
Shows real-time data from RuuviTag BLE sensors:
- Lists all active RuuviTag sensors with their configured locations
- For each sensor:
  - Temperature (°C)
  - Humidity (%)
  - Pressure (hPa)
- Shows "No BLE sensors found" if no tags are detected
- Maximum of 6 sensors displayed simultaneously

#### 3. Wired Sensors Screen
Displays data from wired temperature sensors:
- Sensor type (e.g., DS18B20)
- Number of detected sensors
- Temperature readings for each sensor (°C)
- Large, easy-to-read temperature values
- Error indication if sensors fail to read

#### 4. System Info Screen
Provides device and firmware information:
- Device ID (derived from MAC address)
- Firmware version
- ESP32 chip model
- Flash memory size (MB)
- PSRAM size (MB) if available

#### 5. Network Screen
Shows detailed network connection information:
- Connected SSID
- Local IP address
- WiFi signal strength (dBm) with color-coded quality:
  - Green: > -50 dBm (Excellent)
  - Yellow: -50 to -70 dBm (Good)
  - Red: < -70 dBm (Poor)
- MAC address

## Visual Design

### Color Scheme
- **Headers**: Cyan (TFT_CYAN)
- **Normal Text**: White (TFT_WHITE)
- **Success States**: Green (TFT_GREEN)
- **Warning/Info**: Yellow (TFT_YELLOW)
- **Error States**: Red (TFT_RED)
- **Secondary Info**: Dark Grey (TFT_DARKGREY)
- **Highlights**: Cyan (TFT_CYAN)

### Layout
- **Header**: Screen title with cyan underline
- **Content Area**: Sensor data and system information
- **Footer**: Navigation hints showing current screen number and button functions

### Screen Footer
Each screen displays navigation information at the bottom:
```
BTN1:On/Off  BTN2:Next [1/5]
```
Shows which button does what and current screen position.

## Usage Examples

### Basic Navigation
1. **Power on device** → Display shows logo, then switches to Status Screen
2. **Press Button 2** → Switches to BLE Sensors Screen
3. **Press Button 2** → Switches to Wired Sensors Screen
4. **Press Button 2** → Switches to System Info Screen
5. **Press Button 2** → Switches to Network Screen
6. **Press Button 2** → Cycles back to Status Screen

### Power Management
1. **Press Button 1** → Display turns off (screen goes black)
2. **Press Button 1 again** → Display turns on, showing the same screen as before

## Technical Details

### Update Frequency
- Screen content refreshes every 1 second when display is on
- Button inputs are polled continuously for responsive interaction
- Display does not update when powered off to save CPU cycles

### Button Debouncing
- Debounce time: 50ms
- Click detection time: 400ms
- Prevents accidental double-clicks and ensures reliable input

### Memory Usage
- Display buffer: Managed by TFT_eSPI library
- UI state: Minimal (~100 bytes)
- Button handlers: Static callbacks for efficient operation

## Implementation

### File Structure
```
paku_core/src/
├── display_ui.h         # UI class definition and interface
├── display_ui.cpp       # UI implementation
└── main.cpp            # Integration with main firmware
```

### Integration Points
The UI system integrates with:
- **WiFi Manager**: Displays connection status
- **MQTT Client**: Shows broker connection state
- **RuuviTag Scanner**: Renders BLE sensor data
- **Wired Sensors**: Displays DS18B20 temperature readings
- **System Info**: Reports device and memory statistics

### Dependencies
- **TFT_eSPI**: Display driver library
- **OneButton**: Button handling library
- **Device Config**: System configuration and feature flags

## Configuration

### Enabling/Disabling UI
The UI is automatically included when `HAS_DISPLAY` is defined in device_config.h:

```cpp
#if HAS_DISPLAY
// UI code is compiled
#endif
```

### Customizing Update Interval
In `display_ui.cpp`, modify:

```cpp
updateInterval = 1000;  // Update every 1 second (default)
```

### Customizing Button Behavior
Button timings can be adjusted in `display_ui.cpp`:

```cpp
button1.setDebounceTicks(50);   // Debounce delay
button1.setClickTicks(400);     // Click detection time
```

## Troubleshooting

### Display Not Updating
- Check that `HAS_DISPLAY` is defined
- Verify button connections to GPIO 0 and GPIO 14
- Ensure display initialization succeeded in setup()

### Buttons Not Responding
- Verify GPIO pins are not used by other peripherals
- Check button wiring (should be active-low with pullup)
- Increase debounce time if experiencing noise

### Screen Flickering
- Increase update interval to reduce refresh rate
- Check power supply stability
- Verify TFT_eSPI configuration matches hardware

### Missing Sensor Data
- **BLE Sensors**: Ensure `HAS_BLE` is defined and RuuviTags are in range
- **Wired Sensors**: Verify `HAS_WIRED_SENSORS` is defined and sensors are connected
- Check sensor initialization in setup() logs

## Future Enhancements

Potential improvements for future versions:
- Touch screen support (hardware dependent)
- Custom screen layouts via configuration
- Graphical trend displays for sensor data
- Screen saver mode with timeout
- Adjustable brightness control via long-press
- Configuration menu for WiFi and MQTT settings

## Development Notes

### Adding New Screens
To add a new screen:

1. Add enum value to `DisplayScreen` in `display_ui.h`:
   ```cpp
   enum DisplayScreen {
       // ... existing screens
       SCREEN_NEW,
       SCREEN_COUNT
   };
   ```

2. Declare rendering function in `display_ui.h`:
   ```cpp
   void renderNewScreen();
   ```

3. Implement rendering in `display_ui.cpp`:
   ```cpp
   void DisplayUI::renderNewScreen() {
       clearScreen();
       drawHeader("New Screen");
       // ... render content
       drawFooter();
   }
   ```

4. Add case to `refresh()` switch statement:
   ```cpp
   case SCREEN_NEW:
       renderNewScreen();
       break;
   ```

### Button Event Handlers
The OneButton library supports additional events:
- `attachDoubleClick()` - Double click detection
- `attachLongPressStart()` - Long press detection
- `attachLongPressStop()` - Long press release

These can be added for advanced interactions.

## References

- [TFT_eSPI Library](https://github.com/Bodmer/TFT_eSPI)
- [OneButton Library](https://github.com/mathertel/OneButton)
- [LilyGo T-Display S3 Documentation](https://github.com/Xinyuan-LilyGO/T-Display-S3)
- [ESP32-S3 Technical Reference](https://www.espressif.com/sites/default/files/documentation/esp32-s3_technical_reference_manual_en.pdf)
