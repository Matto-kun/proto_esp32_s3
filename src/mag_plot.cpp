/**
 * @file        mag_plot.cpp
 * @brief       Live magnetometer time / Hz plot implementation
 * @author      Mat RB
 * @date        2026-08-19
 */

#include "mag_plot.hpp"

#include "mag_fft.hpp"

#include <math.h>
#include <stdio.h>
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

static void formatTick(char* buf, size_t buflen, float value_unit)
{
    const float av = fabsf(value_unit);
    if (av >= 100.0f) {
        (void)snprintf(buf, buflen, "%.0f", static_cast<double>(value_unit));
    } else if (av >= 10.0f) {
        (void)snprintf(buf, buflen, "%.1f", static_cast<double>(value_unit));
    } else {
        (void)snprintf(buf, buflen, "%.2f", static_cast<double>(value_unit));
    }
}

} /* anonymous */

MagPlot::~MagPlot()
{
    if (m_fb != nullptr) {
        heap_caps_free(m_fb);
        m_fb = nullptr;
    }
}

void MagPlot::setViewMode(uint8_t mode)
{
    if (mode < display_layout::kMagViewModeCount) {
        if (m_view_mode != mode) {
            m_view_mode = mode;
            markLabelsDirty();
        }
    }
}

void MagPlot::setAxisMode(uint8_t mode)
{
    if (mode < display_layout::kMagAxisModeCount) {
        if (m_axis_mode != mode) {
            m_axis_mode = mode;
            markLabelsDirty();
        }
    }
}

void MagPlot::setUnitMode(uint8_t mode)
{
    if (mode < display_layout::kMagUnitModeCount) {
        if (m_unit_mode != mode) {
            m_unit_mode = mode;
            markLabelsDirty();
        }
    }
}

void MagPlot::markLabelsDirty(void)
{
    m_labels_dirty = true;
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

void MagPlot::fbVBar(uint16_t x, uint16_t y_top, uint16_t y_bot, uint16_t color)
{
    if (y_top > y_bot) {
        const uint16_t t = y_top;
        y_top = y_bot;
        y_bot = t;
    }
    fbLine(static_cast<int16_t>(x), static_cast<int16_t>(y_top),
           static_cast<int16_t>(x), static_cast<int16_t>(y_bot), color);
}

void MagPlot::drawYTicks(Display& display, float mean_g, float half_span_g)
{
    const uint16_t label_x1 = static_cast<uint16_t>(proto_cfg::kMagPlotX0 - 1U);
    (void)display.fillRect(0U, proto_cfg::kMagPlotY0, label_x1, proto_cfg::kMagPlotY1,
                           proto_cfg::kMagPlotBgColor);

    const float top_u = display_layout::mag_gauss_to_unit(mean_g + half_span_g, m_unit_mode);
    const float mid_u = display_layout::mag_gauss_to_unit(mean_g, m_unit_mode);
    const float bot_u = display_layout::mag_gauss_to_unit(mean_g - half_span_g, m_unit_mode);

    char buf[12];
    formatTick(buf, sizeof(buf), top_u);
    (void)display.drawString(2U, proto_cfg::kMagPlotY0 + 2U, buf, kColorWhite);

    formatTick(buf, sizeof(buf), mid_u);
    const uint16_t mid_y = static_cast<uint16_t>(
        (proto_cfg::kMagPlotY0 + proto_cfg::kMagPlotY1) / 2U);
    (void)display.drawString(2U, static_cast<uint16_t>(mid_y - 6U), buf, kColorGray);

    formatTick(buf, sizeof(buf), bot_u);
    (void)display.drawString(2U, static_cast<uint16_t>(proto_cfg::kMagPlotY1 - 14U),
                             buf, kColorWhite);

    (void)display.drawString(2U, static_cast<uint16_t>(proto_cfg::kMagPlotY0 + 16U),
                             display_layout::mag_unit_label(m_unit_mode), kColorYellow);
}

void MagPlot::drawHzLabels(Display& display, float max_mag_g)
{
    const uint16_t label_x1 = static_cast<uint16_t>(proto_cfg::kMagPlotX0 - 1U);
    (void)display.fillRect(0U, proto_cfg::kMagPlotY0, label_x1, proto_cfg::kMagPlotY1,
                           proto_cfg::kMagPlotBgColor);

    const float top_u = display_layout::mag_gauss_to_unit(max_mag_g, m_unit_mode);
    char buf[12];
    formatTick(buf, sizeof(buf), top_u);
    (void)display.drawString(2U, proto_cfg::kMagPlotY0 + 2U, buf, kColorWhite);
    (void)display.drawString(2U, static_cast<uint16_t>(proto_cfg::kMagPlotY0 + 16U),
                             display_layout::mag_unit_label(m_unit_mode), kColorYellow);
    (void)display.drawString(2U, static_cast<uint16_t>(proto_cfg::kMagPlotY1 - 14U),
                             "0", kColorGray);

    /* Frequency span stays in the left margin (outside the blit region). */
    const float nyquist = proto_cfg::kMagSampleRateHz * 0.5f;
    char hz[16];
    (void)snprintf(hz, sizeof(hz), "~%.0fHz", static_cast<double>(nyquist));
    (void)display.drawString(2U, static_cast<uint16_t>(proto_cfg::kMagPlotY1 - 28U),
                             hz, kColorCyan);
}

void MagPlot::reset(void)
{
    m_count = 0U;
    m_head  = 0U;
    markLabelsDirty();
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

bool MagPlot::copyAxisSeries(float* out, size_t n) const
{
    if ((out == nullptr) || (n != proto_cfg::kMagFftN) || (m_count < n)) {
        return false;
    }

    const uint16_t start = static_cast<uint16_t>(
        (m_head + proto_cfg::kMagMaxSamples - n) % proto_cfg::kMagMaxSamples);

    const bool use_abs = (m_view_mode == display_layout::eMagViewHz) &&
                         (m_axis_mode == display_layout::eMagAxisAll);

    for (size_t i = 0U; i < n; i++) {
        const uint16_t idx = static_cast<uint16_t>((start + i) % proto_cfg::kMagMaxSamples);
        const Sample& s = m_buf[idx];
        if (use_abs) {
            out[i] = sqrtf((s.x * s.x) + (s.y * s.y) + (s.z * s.z));
        } else if (m_axis_mode == display_layout::eMagAxisY) {
            out[i] = s.y;
        } else if (m_axis_mode == display_layout::eMagAxisZ) {
            out[i] = s.z;
        } else if (m_axis_mode == display_layout::eMagAxisX) {
            out[i] = s.x;
        } else {
            /* TIME ALL should not call this for FFT; default X. */
            out[i] = s.x;
        }
    }
    return true;
}

esp_err_t MagPlot::drawTime(Display& display, uint32_t now_ms)
{
    const uint32_t window_start = (now_ms > proto_cfg::kMagPlotWindowMs)
                                      ? (now_ms - proto_cfg::kMagPlotWindowMs)
                                      : 0U;

    const uint16_t plot_x0 = proto_cfg::kMagPlotX0;
    const uint16_t plot_y0 = proto_cfg::kMagPlotY0;
    const uint16_t plot_x1 = proto_cfg::kMagPlotX1;
    const uint16_t plot_y1 = proto_cfg::kMagPlotY1;

    fbClear(proto_cfg::kMagPlotBgColor);

    auto maybeDrawTimeLabels = [this, &display](float mean, float half_span) {
        if (m_labels_dirty) {
            m_scale_mean = mean;
            m_scale_half_span = half_span;
            drawYTicks(display, m_scale_mean, m_scale_half_span);
            m_labels_dirty = false;
        }
    };

    if (m_count == 0U) {
        const esp_err_t blit = display.drawPicture16(plot_x0, plot_y0, plot_x1, plot_y1, m_fb);
        if (blit != ESP_OK) {
            return blit;
        }
        maybeDrawTimeLabels(0.0f, proto_cfg::kMagMinHalfSpanGauss);
        return ESP_OK;
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
        const esp_err_t blit = display.drawPicture16(plot_x0, plot_y0, plot_x1, plot_y1, m_fb);
        if (blit != ESP_OK) {
            return blit;
        }
        maybeDrawTimeLabels(0.0f, proto_cfg::kMagMinHalfSpanGauss);
        return ESP_OK;
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
        const esp_err_t blit = display.drawPicture16(plot_x0, plot_y0, plot_x1, plot_y1, m_fb);
        if (blit != ESP_OK) {
            return blit;
        }
        maybeDrawTimeLabels(0.0f, proto_cfg::kMagMinHalfSpanGauss);
        return ESP_OK;
    }

    const float mean_x = sum_x / static_cast<float>(n);
    const float mean_y = sum_y / static_cast<float>(n);
    const float mean_z = sum_z / static_cast<float>(n);

    const bool show_x = (m_axis_mode == display_layout::eMagAxisAll) ||
                        (m_axis_mode == display_layout::eMagAxisX);
    const bool show_y = (m_axis_mode == display_layout::eMagAxisAll) ||
                        (m_axis_mode == display_layout::eMagAxisY);
    const bool show_z = (m_axis_mode == display_layout::eMagAxisAll) ||
                        (m_axis_mode == display_layout::eMagAxisZ);

    float mean = 0.0f;
    if (m_axis_mode == display_layout::eMagAxisAll) {
        mean = (mean_x + mean_y + mean_z) / 3.0f;
    } else if (m_axis_mode == display_layout::eMagAxisX) {
        mean = mean_x;
    } else if (m_axis_mode == display_layout::eMagAxisY) {
        mean = mean_y;
    } else {
        mean = mean_z;
    }

    float max_dev = 0.0f;
    for (uint16_t i = 0U; i < m_count; i++) {
        const uint16_t idx = static_cast<uint16_t>((start_idx + i) % proto_cfg::kMagMaxSamples);
        const Sample& s = m_buf[idx];
        if (s.t_ms < window_start) {
            continue;
        }
        if (show_x) {
            const float dx = fabsf(s.x - mean);
            if (dx > max_dev) {
                max_dev = dx;
            }
        }
        if (show_y) {
            const float dy = fabsf(s.y - mean);
            if (dy > max_dev) {
                max_dev = dy;
            }
        }
        if (show_z) {
            const float dz = fabsf(s.z - mean);
            if (dz > max_dev) {
                max_dev = dz;
            }
        }
    }

    float half_span = max_dev * proto_cfg::kMagScaleMargin;
    if (half_span < proto_cfg::kMagMinHalfSpanGauss) {
        half_span = proto_cfg::kMagMinHalfSpanGauss;
    }

    /* Freeze scale with labels so plot and ticks stay aligned until axis/unit change. */
    if (m_labels_dirty) {
        m_scale_mean = mean;
        m_scale_half_span = half_span;
    }
    const float plot_mean = m_scale_mean;
    const float plot_span = m_scale_half_span;

    const int16_t center_y = valueToY(plot_mean, plot_mean, plot_span, plot_y0, plot_y1);
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
        const int16_t pyx = valueToY(s.x, plot_mean, plot_span, plot_y0, plot_y1);
        const int16_t pyy = valueToY(s.y, plot_mean, plot_span, plot_y0, plot_y1);
        const int16_t pyz = valueToY(s.z, plot_mean, plot_span, plot_y0, plot_y1);

        if (have_prev) {
            if (show_x) {
                fbLine(prev_x, prev_yx, px, pyx, proto_cfg::kMagColorX);
            }
            if (show_y) {
                fbLine(prev_x, prev_yy, px, pyy, proto_cfg::kMagColorY);
            }
            if (show_z) {
                fbLine(prev_x, prev_yz, px, pyz, proto_cfg::kMagColorZ);
            }
        } else {
            if (show_x) {
                fbPixel(px, pyx, proto_cfg::kMagColorX);
            }
            if (show_y) {
                fbPixel(px, pyy, proto_cfg::kMagColorY);
            }
            if (show_z) {
                fbPixel(px, pyz, proto_cfg::kMagColorZ);
            }
        }

        prev_x   = px;
        prev_yx  = pyx;
        prev_yy  = pyy;
        prev_yz  = pyz;
        have_prev = true;
    }

    const esp_err_t blit = display.drawPicture16(plot_x0, plot_y0, plot_x1, plot_y1, m_fb);
    if (blit != ESP_OK) {
        return blit;
    }
    if (m_labels_dirty) {
        drawYTicks(display, m_scale_mean, m_scale_half_span);
        m_labels_dirty = false;
    }
    return ESP_OK;
}

esp_err_t MagPlot::drawHz(Display& display)
{
    const uint16_t plot_x0 = proto_cfg::kMagPlotX0;
    const uint16_t plot_y0 = proto_cfg::kMagPlotY0;
    const uint16_t plot_x1 = proto_cfg::kMagPlotX1;
    const uint16_t plot_y1 = proto_cfg::kMagPlotY1;

    fbClear(proto_cfg::kMagPlotBgColor);

    float series[proto_cfg::kMagFftN];
    float bins[proto_cfg::kMagFftBins];

    auto maybeDrawHzLabels = [this, &display](float max_mag) {
        if (m_labels_dirty) {
            m_scale_max_mag = max_mag;
            drawHzLabels(display, m_scale_max_mag);
            m_labels_dirty = false;
        }
    };

    if (!copyAxisSeries(series, proto_cfg::kMagFftN)) {
        const esp_err_t blit = display.drawPicture16(plot_x0, plot_y0, plot_x1, plot_y1, m_fb);
        if (blit != ESP_OK) {
            return blit;
        }
        maybeDrawHzLabels(proto_cfg::kMagMinHalfSpanGauss);
        return ESP_OK;
    }

    const esp_err_t fft_err = mag_fft_magnitude(series, proto_cfg::kMagFftN, bins);
    if (fft_err != ESP_OK) {
        return fft_err;
    }

    float max_mag = proto_cfg::kMagMinHalfSpanGauss;
    for (uint16_t k = 0U; k < proto_cfg::kMagFftBins; k++) {
        if (bins[k] > max_mag) {
            max_mag = bins[k];
        }
    }
    max_mag *= proto_cfg::kMagScaleMargin;

    if (m_labels_dirty) {
        m_scale_max_mag = max_mag;
    }
    const float plot_max = m_scale_max_mag;

    const uint16_t plot_w = static_cast<uint16_t>(plot_x1 - plot_x0 + 1U);
    const uint16_t plot_h = static_cast<uint16_t>(plot_y1 - plot_y0 + 1U);
    const uint16_t usable_h = static_cast<uint16_t>(plot_h - 4U);
    const uint16_t baseline = plot_y1;

    uint16_t bar_color = proto_cfg::kMagColorAbs;
    if (m_axis_mode == display_layout::eMagAxisX) {
        bar_color = proto_cfg::kMagColorX;
    } else if (m_axis_mode == display_layout::eMagAxisY) {
        bar_color = proto_cfg::kMagColorY;
    } else if (m_axis_mode == display_layout::eMagAxisZ) {
        bar_color = proto_cfg::kMagColorZ;
    }

    for (uint16_t k = 0U; k < proto_cfg::kMagFftBins; k++) {
        const float frac = static_cast<float>(k) /
                           static_cast<float>(proto_cfg::kMagFftBins - 1U);
        const uint16_t x = static_cast<uint16_t>(
            plot_x0 + static_cast<uint16_t>(frac * static_cast<float>(plot_w - 1U)));
        const float h_frac = clampF(bins[k] / plot_max, 0.0f, 1.0f);
        const uint16_t bar_h = static_cast<uint16_t>(h_frac * static_cast<float>(usable_h));
        const uint16_t y_top = static_cast<uint16_t>(baseline - bar_h);
        fbVBar(x, y_top, baseline, bar_color);
    }

    const esp_err_t blit = display.drawPicture16(plot_x0, plot_y0, plot_x1, plot_y1, m_fb);
    if (blit != ESP_OK) {
        return blit;
    }
    maybeDrawHzLabels(plot_max);
    return ESP_OK;
}

esp_err_t MagPlot::draw(Display& display, uint32_t now_ms)
{
    const esp_err_t fb_err = ensureFb();
    if (fb_err != ESP_OK) {
        return fb_err;
    }

    if (m_view_mode == display_layout::eMagViewHz) {
        return drawHz(display);
    }
    return drawTime(display, now_ms);
}
