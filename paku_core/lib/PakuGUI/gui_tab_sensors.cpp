/**
 * @file gui_tab_sensors.cpp
 * @brief Sensors tab — temperature and humidity trend charts using LVGL chart widget.
 *
 * Two charts stacked vertically:
 *   1. Temperature (°C) — 4 series: Indoor, Outdoor, Fridge, Reppu
 *   2. Humidity (%) — 3 series: Indoor, Outdoor, Reppu
 *
 * Each chart stores up to MAX_POINTS data points (ring buffer).
 */
#include "gui_tab_sensors.h"
#include "gui_theme.h"
#include <cstdio>

// ---------------------------------------------------------------------------
//  Constants
// ---------------------------------------------------------------------------

/// Maximum data points in each chart series (approx. 10 min at 5 s interval)
#define MAX_POINTS 120

/// Number of temperature series
#define TEMP_SERIES_COUNT 4
/// Number of humidity series
#define HUM_SERIES_COUNT  3

// ---------------------------------------------------------------------------
//  Static handles
// ---------------------------------------------------------------------------

static lv_obj_t           *_chartTemp     = nullptr;
static lv_chart_series_t  *_serTemp[TEMP_SERIES_COUNT] = {};
static lv_obj_t           *_legendTemp[TEMP_SERIES_COUNT] = {};

static lv_obj_t           *_chartHum      = nullptr;
static lv_chart_series_t  *_serHum[HUM_SERIES_COUNT]  = {};
static lv_obj_t           *_legendHum[HUM_SERIES_COUNT]  = {};

static const char *tempNames[TEMP_SERIES_COUNT] = {"Indoor", "Outdoor", "Fridge", "Reppu"};
static const lv_color_t tempColors[TEMP_SERIES_COUNT] = {
    GUI_COLOR_TEMP_INDOOR,
    GUI_COLOR_TEMP_OUTDOOR,
    GUI_COLOR_TEMP_FRIDGE,
    GUI_COLOR_TEMP_REPPU,
};

static const char *humNames[HUM_SERIES_COUNT] = {"Indoor", "Outdoor", "Reppu"};
static const lv_color_t humColors[HUM_SERIES_COUNT] = {
    GUI_COLOR_HUM_INDOOR,
    GUI_COLOR_HUM_OUTDOOR,
    GUI_COLOR_HUM_REPPU,
};

// Most-recent values for legend display
static float _lastTemp[TEMP_SERIES_COUNT] = {22.4f, 8.1f, 4.5f, 15.2f};
static float _lastHum[HUM_SERIES_COUNT]   = {45.0f, 82.0f, 55.0f};

// ---------------------------------------------------------------------------
//  Helpers
// ---------------------------------------------------------------------------

static void style_card(lv_obj_t *obj) {
    lv_obj_set_style_bg_color(obj, GUI_COLOR_CARD, 0);
    lv_obj_set_style_bg_opa(obj, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(obj, GUI_RADIUS, 0);
    lv_obj_set_style_border_width(obj, 0, 0);
    lv_obj_set_style_pad_all(obj, GUI_PAD_SM, 0);
    lv_obj_clear_flag(obj, LV_OBJ_FLAG_SCROLLABLE);
}

static void update_temp_legend() {
    for (int i = 0; i < TEMP_SERIES_COUNT; i++) {
        if (_legendTemp[i]) {
            char buf[24];
            snprintf(buf, sizeof(buf), "%s: %.1f°", tempNames[i], _lastTemp[i]);
            lv_label_set_text(_legendTemp[i], buf);
        }
    }
}

static void update_hum_legend() {
    for (int i = 0; i < HUM_SERIES_COUNT; i++) {
        if (_legendHum[i]) {
            char buf[24];
            snprintf(buf, sizeof(buf), "%s: %.0f%%", humNames[i], _lastHum[i]);
            lv_label_set_text(_legendHum[i], buf);
        }
    }
}

// ---------------------------------------------------------------------------
//  Chart creation
// ---------------------------------------------------------------------------

static lv_obj_t *create_chart_card(lv_obj_t *parent, const char *title,
                                    lv_obj_t **chartOut,
                                    int seriesCount,
                                    const char *names[],
                                    const lv_color_t colors[],
                                    lv_chart_series_t *serArr[],
                                    lv_obj_t *legendArr[],
                                    int yMin, int yMax) {
    lv_obj_t *card = lv_obj_create(parent);
    lv_obj_set_width(card, LV_PCT(100));
    lv_obj_set_flex_grow(card, 1);
    style_card(card);
    lv_obj_set_flex_flow(card, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_gap(card, 4, 0);

    // Title
    lv_obj_t *hdr = lv_label_create(card);
    lv_label_set_text(hdr, title);
    lv_obj_set_style_text_font(hdr, GUI_FONT_MD, 0);
    lv_obj_set_style_text_color(hdr, GUI_COLOR_TEXT_PRI, 0);

    // Chart + legend row
    lv_obj_t *row = lv_obj_create(card);
    lv_obj_set_size(row, LV_PCT(100), LV_PCT(100));
    lv_obj_set_flex_grow(row, 1);
    lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(row, 0, 0);
    lv_obj_set_style_pad_all(row, 0, 0);
    lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_gap(row, GUI_PAD_SM, 0);

    // Chart
    lv_obj_t *chart = lv_chart_create(row);
    *chartOut = chart;
    lv_obj_set_flex_grow(chart, 1);
    lv_obj_set_height(chart, LV_PCT(100));
    lv_chart_set_type(chart, LV_CHART_TYPE_LINE);
    lv_chart_set_point_count(chart, MAX_POINTS);
    lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, yMin * 10, yMax * 10);
    lv_chart_set_div_line_count(chart, 5, 0);
    lv_obj_set_style_bg_color(chart, GUI_COLOR_CARD, 0);
    lv_obj_set_style_border_width(chart, 0, 0);
    lv_obj_set_style_line_color(chart, GUI_COLOR_TEXT_MUTED, LV_PART_MAIN);
    lv_obj_set_style_line_opa(chart, LV_OPA_30, LV_PART_MAIN);
    lv_obj_set_style_size(chart, 0, LV_PART_INDICATOR);   // hide point dots

    // Create series
    for (int i = 0; i < seriesCount; i++) {
        serArr[i] = lv_chart_add_series(chart, colors[i], LV_CHART_AXIS_PRIMARY_Y);
        // Pre-fill with LV_CHART_POINT_NONE so chart is blank
        lv_chart_set_all_value(chart, serArr[i], LV_CHART_POINT_NONE);
    }

    // Legend column
    lv_obj_t *legend = lv_obj_create(row);
    lv_obj_set_size(legend, 140, LV_PCT(100));
    lv_obj_set_style_bg_opa(legend, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(legend, 0, 0);
    lv_obj_set_style_pad_all(legend, 4, 0);
    lv_obj_clear_flag(legend, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flex_flow(legend, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_gap(legend, 2, 0);

    for (int i = 0; i < seriesCount; i++) {
        legendArr[i] = lv_label_create(legend);
        lv_obj_set_style_text_font(legendArr[i], GUI_FONT_SM, 0);
        lv_obj_set_style_text_color(legendArr[i], colors[i], 0);
        lv_label_set_text(legendArr[i], names[i]);
    }

    return card;
}

// ---------------------------------------------------------------------------
//  Public API
// ---------------------------------------------------------------------------

lv_obj_t *gui_tab_sensors_create(lv_obj_t *parent) {
    lv_obj_t *cont = lv_obj_create(parent);
    lv_obj_set_size(cont, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_opa(cont, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(cont, 0, 0);
    lv_obj_set_style_pad_all(cont, GUI_PAD_MD, 0);
    lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_gap(cont, GUI_PAD_MD, 0);
    lv_obj_clear_flag(cont, LV_OBJ_FLAG_SCROLLABLE);

    // Temperature chart  (Y range: -10 to 40 °C, stored as ×10)
    create_chart_card(cont, "Temperatures (\xC2\xB0""C)",
                      &_chartTemp, TEMP_SERIES_COUNT,
                      tempNames, tempColors, _serTemp, _legendTemp,
                      -10, 40);
    update_temp_legend();

    // Humidity chart  (Y range: 0 to 100%)
    create_chart_card(cont, "Humidity (%)",
                      &_chartHum, HUM_SERIES_COUNT,
                      humNames, humColors, _serHum, _legendHum,
                      0, 100);
    update_hum_legend();

    return cont;
}

void gui_tab_sensors_push_temp(uint8_t series, float value) {
    if (series >= TEMP_SERIES_COUNT || !_chartTemp) return;
    _lastTemp[series] = value;
    // Store as ×10 for integer chart values
    lv_chart_set_next_value(_chartTemp, _serTemp[series], (lv_coord_t)(value * 10));
    update_temp_legend();
}

void gui_tab_sensors_push_hum(uint8_t series, float value) {
    if (series >= HUM_SERIES_COUNT || !_chartHum) return;
    _lastHum[series] = value;
    lv_chart_set_next_value(_chartHum, _serHum[series], (lv_coord_t)(value * 10));
    update_hum_legend();
}
