/**
 * @file        mag_plot.hpp
 * @brief       Live magnetometer time / Hz magnitude plotter
 * @author      Mat RB
 * @date        2026-08-19
 */

#ifndef MAG_PLOT_HPP
#define MAG_PLOT_HPP

#include <stddef.h>
#include <stdint.h>

#include "config.hpp"
#include "display/display_api.hpp"
#include "display/layout/mag_live_menu.hpp"
#include "esp_err.h"

class MagPlot
{
public:
    MagPlot() = default;
    ~MagPlot();

    void reset(void);

    void setViewMode(uint8_t mode);
    void setAxisMode(uint8_t mode);
    void setUnitMode(uint8_t mode);
    uint8_t viewMode(void) const { return m_view_mode; }
    uint8_t axisMode(void) const { return m_axis_mode; }
    uint8_t unitMode(void) const { return m_unit_mode; }

    void pushSample(float x_gauss, float y_gauss, float z_gauss, uint32_t t_ms);

    esp_err_t draw(Display& display, uint32_t now_ms);

private:
    struct Sample {
        float    x = 0.0f;
        float    y = 0.0f;
        float    z = 0.0f;
        uint32_t t_ms = 0U;
    };

    esp_err_t ensureFb(void);
    void      fbClear(uint16_t color);
    void      fbPixel(int16_t x, int16_t y, uint16_t color);
    void      fbLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color);
    void      fbVBar(uint16_t x, uint16_t y_top, uint16_t y_bot, uint16_t color);
    void      drawYTicks(Display& display, float mean_g, float half_span_g);
    void      drawHzLabels(Display& display, float max_mag_g);
    void      markLabelsDirty(void);

    esp_err_t drawTime(Display& display, uint32_t now_ms);
    esp_err_t drawHz(Display& display);

    bool copyAxisSeries(float* out, size_t n) const;

    Sample   m_buf[proto_cfg::kMagMaxSamples] = {};
    uint16_t m_count = 0U;
    uint16_t m_head  = 0U;

    uint8_t m_view_mode = display_layout::eMagViewTime;
    uint8_t m_axis_mode = display_layout::eMagAxisAll;
    uint8_t m_unit_mode = display_layout::eMagUnitUT;

    /** Redraw axis/unit labels only when view/axis/unit changes. */
    bool  m_labels_dirty = true;
    float m_scale_mean = 0.0f;
    float m_scale_half_span = proto_cfg::kMagMinHalfSpanGauss;
    float m_scale_max_mag = proto_cfg::kMagMinHalfSpanGauss;

    uint16_t* m_fb = nullptr;
    uint16_t  m_fb_w = 0U;
    uint16_t  m_fb_h = 0U;
};

#endif /* MAG_PLOT_HPP */
