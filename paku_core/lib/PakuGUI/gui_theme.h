/**
 * @file gui_theme.h
 * @brief Paku GUI colour palette, sizing constants, and font aliases.
 *
 * All visual constants are collected here so the design can be tweaked
 * without touching individual screen files.
 *
 * Colour values are taken from the HTML prototype (smart-home-prototype.html).
 */
#pragma once

#include <lvgl.h>

// ============================================================================
//  Colour palette  (dark theme — matching HTML prototype)
// ============================================================================

/// Base background colour (#121212)
#define GUI_COLOR_BG           lv_color_hex(0x121212)
/// Panel / sidebar background (#1e1e1e)
#define GUI_COLOR_PANEL        lv_color_hex(0x1e1e1e)
/// Card / widget background (#2c2c2c)
#define GUI_COLOR_CARD         lv_color_hex(0x2c2c2c)
/// Elevated card / hover (#333333)
#define GUI_COLOR_CARD_HOVER   lv_color_hex(0x333333)

/// Primary accent – orange (#ff9800)
#define GUI_COLOR_ACCENT       lv_color_hex(0xff9800)
/// Secondary accent – blue (#03a9f4)
#define GUI_COLOR_BLUE         lv_color_hex(0x03a9f4)
/// Success / active green (#4caf50)
#define GUI_COLOR_GREEN        lv_color_hex(0x4caf50)
/// Warning / connecting – yellow (#ffeb3b)
#define GUI_COLOR_YELLOW       lv_color_hex(0xffeb3b)
/// Error / danger – red (#f44336)
#define GUI_COLOR_RED          lv_color_hex(0xf44336)

/// Primary text (white)
#define GUI_COLOR_TEXT_PRI     lv_color_hex(0xffffff)
/// Secondary text (#aaaaaa)
#define GUI_COLOR_TEXT_SEC     lv_color_hex(0xaaaaaa)
/// Muted text (#666666)
#define GUI_COLOR_TEXT_MUTED   lv_color_hex(0x666666)

// --- Sensor graph series colours ---
#define GUI_COLOR_TEMP_INDOOR  lv_color_hex(0xff5252)
#define GUI_COLOR_TEMP_OUTDOOR lv_color_hex(0x448aff)
#define GUI_COLOR_TEMP_FRIDGE  lv_color_hex(0x69f0ae)
#define GUI_COLOR_TEMP_REPPU   lv_color_hex(0xffd740)

#define GUI_COLOR_HUM_INDOOR   lv_color_hex(0x00e5ff)
#define GUI_COLOR_HUM_OUTDOOR  lv_color_hex(0xb388ff)
#define GUI_COLOR_HUM_REPPU    lv_color_hex(0xff8a80)

// --- Power bar colours ---
#define GUI_COLOR_SOLAR        lv_color_hex(0xff9800)
#define GUI_COLOR_AC_CHARGER   lv_color_hex(0x4caf50)
#define GUI_COLOR_INVERTER     lv_color_hex(0xf44336)
#define GUI_COLOR_DC_LOAD      lv_color_hex(0x03a9f4)

// ============================================================================
//  Layout dimensions (pixels)
// ============================================================================

/// Sidebar width (vertical tab bar on the left)
#define GUI_SIDEBAR_W          80
/// Header bar height
#define GUI_HEADER_H           40
/// Content area = screen minus sidebar and header
#define GUI_CONTENT_W          (800 - GUI_SIDEBAR_W)
#define GUI_CONTENT_H          (480 - GUI_HEADER_H)

/// Standard padding / margins
#define GUI_PAD_SM             6
#define GUI_PAD_MD             12
#define GUI_PAD_LG             20

/// Card corner radius
#define GUI_RADIUS             8
/// Border width for active indicators
#define GUI_BORDER_W           3

/// Minimum touch-target size (px)
#define GUI_TOUCH_MIN          44

// ============================================================================
//  Font aliases  (Montserrat – enabled in lv_conf.h)
// ============================================================================

#define GUI_FONT_SM            &lv_font_montserrat_12
#define GUI_FONT_MD            &lv_font_montserrat_16
#define GUI_FONT_LG            &lv_font_montserrat_20
#define GUI_FONT_XL            &lv_font_montserrat_28
#define GUI_FONT_XXL           &lv_font_montserrat_36

// Default font for body text
#define GUI_FONT_DEFAULT       GUI_FONT_MD

// ============================================================================
//  Tab identifiers (maps to sidebar order)
// ============================================================================
enum GuiTab : uint8_t {
    GUI_TAB_MAIN     = 0,
    GUI_TAB_CLIMATE  = 1,
    GUI_TAB_LIGHTS   = 2,
    GUI_TAB_SENSORS  = 3,
    GUI_TAB_POWER    = 4,
    GUI_TAB_SETTINGS = 5,
    GUI_TAB_COUNT    = 6,
};
