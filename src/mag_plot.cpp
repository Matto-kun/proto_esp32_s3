/**
 * @file        mag_plot.cpp
 * @brief       Live magnetometer trace plotter implementation
 * @author      Mat RB
 * @date        2026-08-19
 */

#include "mag_plot.hpp"

#include <math.h>
#include <stdint.h>

#include "esp_heap_caps.h"

namespace {

static float clampF(float v, float lo, float hi)
{
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

static int16_t valueToY(float value, float mean, float half_span,
                        uint16_t plot_y0, uint16_t plot_y1)
{
    const float plot_h = static_cast<float>(plot_y1 - plot_y0 + 1U);
    const float half_plot = (plot_h * 0.5f) - static_cast<float>(proto_cfg::kMagPlotPadPx);
    const float center = static_cast<float>(plot_y0) + (plot_h * 0.5f);
    const float norm = (value - mean) / half_span;
    const float y = center - (norm * half_plot);
    const float y_clamped = clampF(y,
                                   static_cast<float>(plot_y0 + proto_cfg::kMagPlotPadPx),
                                   static_cast<float>(plot_y1 - proto_cfg::kMagPlotPadPx));
    return static_cast<int16_t>(y_clamped + 0.5f);
}

static int16_t timeToX(uint32_t t, uint32_t t_min, uint32_t t_max,
                       uint16_t plot_x0, uint16_t plot_x1)
{
    if (t_max <= t_min) {
        return static_cast<int16_t>((plot_x0 + plot_x1) / 2U);
    }
    const float frac = static_cast<float>(t - t_min) /
                       static_cast<float>(t_max - t_min);
    const float x = static_cast<float>(plot_x0) +
                    (frac * static_cast<float>(plot_x1 - plot_x0));
    return static_cast<int16_t>(x + 0.5f);
}

} /* anonymous */

MagPlot::~MagPlot()
{
    if (m_fb != nullptr) {
        heap_caps_free(m_fb);
        m_fb = nullptr;
    }
}

esp_err_t MagPlot::ensureFb(void)
{
    const uint16_t w = static_cast<uint16_t>(
        proto_cfg::kMagPlotX1 - proto_cfg::kMagPlotX0 + 1U);
    const uint16_t h = static_cast<uint16_t>(
        proto_cfg::kMagPlotY1 - proto_cfg::kMagPlotY0 + 1U);

    if ((m_fb != nullptr) && (m_fb_w == w) && (m_fb_h == h)) {
        return ESP_OK;
    }

    if (m_fb != nullptr) {
        heap_caps_free(m_fb);
        m_fb = nullptr;
    }

    const size_t bytes = static_cast<size_t>(w) * static_cast<size_t>(h) * sizeof(uint16_t);
    /* Prefer SPIRAM; LibDriver copies into a 4 KB DMA chunk for SPI TX. */
    m_fb = static_cast<uint16_t*>(
        heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (m_fb == nullptr) {
        m_fb = static_cast<uint16_t*>(heap_caps_malloc(bytes, MALLOC_CAP_DEFAULT));
    }
    if (m_fb == nullptr) {
        m_fb_w = 0U;
        m_fb_h = 0U;
        return ESP_ERR_NO_MEM;
    }

    m_fb_w = w;
    m_fb_h = h;
    return ESP_OK;
}

void MagPlot::fbClear(uint16_t color)
{
    const size_t n = static_cast<size_t>(m_fb_w) * static_cast<size_t>(m_fb_h);
    for (size_t i = 0U; i < n; i++) {
        m_fb[i] = color;
    }
}

void MagPlot::fbPixel(int16_t x, int16_t y, uint16_t color)
{
    const int16_t x0 = static_cast<int16_t>(proto_cfg::kMagPlotX0);
    const int16_t y0 = static_cast<int16_t>(proto_cfg::kMagPlotY0);
    const int16_t x1 = static_cast<int16_t>(proto_cfg::kMagPlotX1);
    const int16_t y1 = static_cast<int16_t>(proto_cfg::kMagPlotY1);

    if ((x < x0) || (y < y0) || (x > x1) || (y > y1)) {
        return;
    }

    const uint16_t lx = static_cast<uint16_t>(x - x0);
    const uint16_t ly = static_cast<uint16_t>(y - y0);
    /* Column-major for LibDriver st7789_draw_picture_16bits. */
    m_fb[static_cast<size_t>(lx) * m_fb_h + ly] = color;
}

void MagPlot::fbLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color)
{
    if (x0 > x1) {
        const int16_t tx = x0;
        x0 = x1;
        x1 = tx;
        const int16_t ty = y0;
        y0 = y1;
        y1 = ty;
    }

    if (y0 == y1) {
        for (int16_t x = x0; x <= x1; x++) {
            fbPixel(x, y0, color);
        }
        return;
    }

    if (x0 == x1) {
        const int16_t ya = (y0 < y1) ? y0 : y1;
        const int16_t yb = (y0 < y1) ? y1 : y0;
        for (int16_t y = ya; y <= yb; y++) {
            fbPixel(x0, y, color);
        }
        return;
    }

    int16_t dx = static_cast<int16_t>(x1 - x0);
    int16_t sx = 1;
    int16_t dy = (y0 < y1) ? static_cast<int16_t>(y1 - y0) : static_cast<int16_t>(y0 - y1);
    int16_t sy = (y0 < y1) ? 1 : -1;
    dy = static_cast<int16_t>(-dy);
    int16_t err = static_cast<int16_t>(dx + dy);
    int16_t cx = x0;
    int16_t cy = y0;

    while (true) {
        fbPixel(cx, cy, color);
        if ((cx == x1) && (cy == y1)) {
            break;
        }
        const int16_t e2 = static_cast<int16_t>(2 * err);
        if (e2 >= dy) {
            err = static_cast<int16_t>(err + dy);
            cx  = static_cast<int16_t>(cx + sx);
        }
        if (e2 <= dx) {
            err = static_cast<int16_t>(err + dx);
            cy  = static_cast<int16_t>(cy + sy);
        }
    }
}

void MagPlot::reset(void)
{
    m_count = 0U;
    m_head  = 0U;
}

void MagPlot::pushSample(float x_gauss, float y_gauss, float z_gauss, uint32_t t_ms)
{
    m_buf[m_head].x    = x_gauss;
    m_buf[m_head].y    = y_gauss;
    m_buf[m_head].z    = z_gauss;
    m_buf[m_head].t_ms = t_ms;

    m_head = static_cast<uint16_t>((m_head + 1U) % proto_cfg::kMagMaxSamples);
    if (m_count < proto_cfg::kMagMaxSamples) {
        m_count++;
    }
}

esp_err_t MagPlot::draw(Display& display, uint32_t now_ms)
{
    const esp_err_t fb_err = ensureFb();
    if (fb_err != ESP_OK) {
        return fb_err;
    }

    const uint32_t window_start = (now_ms > proto_cfg::kMagPlotWindowMs)
                                      ? (now_ms - proto_cfg::kMagPlotWindowMs)
                                      : 0U;

    const uint16_t plot_x0 = proto_cfg::kMagPlotX0;
    const uint16_t plot_y0 = proto_cfg::kMagPlotY0;
    const uint16_t plot_x1 = proto_cfg::kMagPlotX1;
    const uint16_t plot_y1 = proto_cfg::kMagPlotY1;

    fbClear(proto_cfg::kMagPlotBgColor);

    if (m_count == 0U) {
        return display.drawPicture16(plot_x0, plot_y0, plot_x1, plot_y1, m_fb);
    }

    const uint16_t start_idx = (m_count < proto_cfg::kMagMaxSamples)
                                   ? 0U
                                   : m_head;

    uint16_t win_count = 0U;
    uint32_t t_min = UINT32_MAX;
    uint32_t t_max = 0U;

    for (uint16_t i = 0U; i < m_count; i++) {
        const uint16_t idx = static_cast<uint16_t>((start_idx + i) % proto_cfg::kMagMaxSamples);
        const Sample& s = m_buf[idx];
        if (s.t_ms < window_start) {
            continue;
        }
        if (s.t_ms < t_min) {
            t_min = s.t_ms;
        }
        if (s.t_ms > t_max) {
            t_max = s.t_ms;
        }
        win_count++;
    }

    if (win_count == 0U) {
        return display.drawPicture16(plot_x0, plot_y0, plot_x1, plot_y1, m_fb);
    }

    if (t_max < now_ms) {
        t_max = now_ms;
    }

    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float sum_z = 0.0f;
    uint16_t n = 0U;

    for (uint16_t i = 0U; i < m_count; i++) {
        const uint16_t idx = static_cast<uint16_t>((start_idx + i) % proto_cfg::kMagMaxSamples);
        const Sample& s = m_buf[idx];
        if (s.t_ms < window_start) {
            continue;
        }
        sum_x += s.x;
        sum_y += s.y;
        sum_z += s.z;
        n++;
    }

    if (n == 0U) {
        return display.drawPicture16(plot_x0, plot_y0, plot_x1, plot_y1, m_fb);
    }

    const float mean_x = sum_x / static_cast<float>(n);
    const float mean_y = sum_y / static_cast<float>(n);
    const float mean_z = sum_z / static_cast<float>(n);

    float max_dev = 0.0f;
    for (uint16_t i = 0U; i < m_count; i++) {
        const uint16_t idx = static_cast<uint16_t>((start_idx + i) % proto_cfg::kMagMaxSamples);
        const Sample& s = m_buf[idx];
        if (s.t_ms < window_start) {
            continue;
        }
        const float dx = fabsf(s.x - mean_x);
        const float dy = fabsf(s.y - mean_y);
        const float dz = fabsf(s.z - mean_z);
        if (dx > max_dev) {
            max_dev = dx;
        }
        if (dy > max_dev) {
            max_dev = dy;
        }
        if (dz > max_dev) {
            max_dev = dz;
        }
    }

    float half_span = max_dev * proto_cfg::kMagScaleMargin;
    if (half_span < proto_cfg::kMagMinHalfSpanGauss) {
        half_span = proto_cfg::kMagMinHalfSpanGauss;
    }

    const int16_t center_y = valueToY(mean_x, mean_x, half_span, plot_y0, plot_y1);
    fbLine(static_cast<int16_t>(plot_x0), center_y,
           static_cast<int16_t>(plot_x1), center_y,
           proto_cfg::kMagPlotMeanLineColor);

    int16_t prev_x = -1;
    int16_t prev_yx = -1;
    int16_t prev_yy = -1;
    int16_t prev_yz = -1;
    bool have_prev = false;

    for (uint16_t i = 0U; i < m_count; i++) {
        const uint16_t idx = static_cast<uint16_t>((start_idx + i) % proto_cfg::kMagMaxSamples);
        const Sample& s = m_buf[idx];
        if (s.t_ms < window_start) {
            continue;
        }

        const int16_t px = timeToX(s.t_ms, t_min, t_max, plot_x0, plot_x1);
        const int16_t pyx = valueToY(s.x, mean_x, half_span, plot_y0, plot_y1);
        const int16_t pyy = valueToY(s.y, mean_y, half_span, plot_y0, plot_y1);
        const int16_t pyz = valueToY(s.z, mean_z, half_span, plot_y0, plot_y1);

        if (have_prev) {
            fbLine(prev_x, prev_yx, px, pyx, proto_cfg::kMagColorX);
            fbLine(prev_x, prev_yy, px, pyy, proto_cfg::kMagColorY);
            fbLine(prev_x, prev_yz, px, pyz, proto_cfg::kMagColorZ);
        } else {
            fbPixel(px, pyx, proto_cfg::kMagColorX);
            fbPixel(px, pyy, proto_cfg::kMagColorY);
            fbPixel(px, pyz, proto_cfg::kMagColorZ);
            fbPixel(static_cast<int16_t>(px + 1), pyx, proto_cfg::kMagColorX);
            fbPixel(static_cast<int16_t>(px + 1), pyy, proto_cfg::kMagColorY);
            fbPixel(static_cast<int16_t>(px + 1), pyz, proto_cfg::kMagColorZ);
        }

        prev_x   = px;
        prev_yx  = pyx;
        prev_yy  = pyy;
        prev_yz  = pyz;
        have_prev = true;
    }

    return display.drawPicture16(plot_x0, plot_y0, plot_x1, plot_y1, m_fb);
}
