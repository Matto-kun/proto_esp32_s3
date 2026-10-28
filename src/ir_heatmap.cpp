/**
 * @file        ir_heatmap.cpp
 * @brief       Inferno heatmap: upsample filters, 8x8 unsharp, steady labels
 * @author      Mat RB
 * @date        2026-08-26
 */

#include "ir_heatmap.hpp"

#include <stdio.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {

static constexpr uint8_t kInfernoStops = 32U;

static const uint8_t kInfernoRgb[kInfernoStops][3] = {
    {  0,   0,   4}, { 13,   8,  25}, { 26,  11,  52}, { 40,  11,  71},
    { 56,   9,  87}, { 70,  10,  94}, { 84,  15,  98}, { 97,  22,  98},
    {111,  29,  96}, {124,  36,  92}, {138,  43,  86}, {151,  51,  80},
    {164,  59,  72}, {177,  68,  63}, {189,  78,  53}, {201,  89,  42},
    {211, 101,  30}, {220, 114,  17}, {227, 129,   6}, {232, 145,   6},
    {236, 161,  18}, {239, 177,  35}, {242, 193,  52}, {245, 209,  71},
    {247, 224,  92}, {249, 237, 115}, {251, 247, 141}, {252, 254, 168},
    {251, 255, 191}, {248, 254, 210}, {245, 251, 224}, {252, 255, 164},
};

static uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b)
{
    return static_cast<uint16_t>(
        (static_cast<uint16_t>(r & 0xF8U) << 8U) |
        (static_cast<uint16_t>(g & 0xFCU) << 3U) |
        (static_cast<uint16_t>(b) >> 3U));
}

static uint16_t infernoColor(float t01)
{
    if (t01 <= 0.0f) {
        return rgb565(kInfernoRgb[0][0], kInfernoRgb[0][1], kInfernoRgb[0][2]);
    }
    if (t01 >= 1.0f) {
        return rgb565(kInfernoRgb[kInfernoStops - 1U][0],
                      kInfernoRgb[kInfernoStops - 1U][1],
                      kInfernoRgb[kInfernoStops - 1U][2]);
    }

    const float scaled = t01 * static_cast<float>(kInfernoStops - 1U);
    const uint8_t i0 = static_cast<uint8_t>(scaled);
    uint8_t i1 = static_cast<uint8_t>(i0 + 1U);
    if (i1 >= kInfernoStops) {
        i1 = static_cast<uint8_t>(kInfernoStops - 1U);
    }
    const float frac = scaled - static_cast<float>(i0);

    const float r = static_cast<float>(kInfernoRgb[i0][0]) +
                    (frac * static_cast<float>(
                        static_cast<int16_t>(kInfernoRgb[i1][0]) -
                        static_cast<int16_t>(kInfernoRgb[i0][0])));
    const float g = static_cast<float>(kInfernoRgb[i0][1]) +
                    (frac * static_cast<float>(
                        static_cast<int16_t>(kInfernoRgb[i1][1]) -
                        static_cast<int16_t>(kInfernoRgb[i0][1])));
    const float b = static_cast<float>(kInfernoRgb[i0][2]) +
                    (frac * static_cast<float>(
                        static_cast<int16_t>(kInfernoRgb[i1][2]) -
                        static_cast<int16_t>(kInfernoRgb[i0][2])));

    return rgb565(static_cast<uint8_t>(r + 0.5f),
                  static_cast<uint8_t>(g + 0.5f),
                  static_cast<uint8_t>(b + 0.5f));
}

static float gridAt(const float* grid, uint8_t col, uint8_t row)
{
    return grid[static_cast<size_t>(row) * proto_cfg::kIrGridSize + col];
}

static float sampleBilinearGrid(const float* grid, float gx, float gy)
{
    constexpr float kMax = static_cast<float>(proto_cfg::kIrGridSize - 1U);
    if (gx < 0.0f) {
        gx = 0.0f;
    }
    if (gy < 0.0f) {
        gy = 0.0f;
    }
    if (gx > kMax) {
        gx = kMax;
    }
    if (gy > kMax) {
        gy = kMax;
    }

    const uint8_t x0 = static_cast<uint8_t>(gx);
    const uint8_t y0 = static_cast<uint8_t>(gy);
    uint8_t x1 = static_cast<uint8_t>(x0 + 1U);
    uint8_t y1 = static_cast<uint8_t>(y0 + 1U);
    if (x1 >= proto_cfg::kIrGridSize) {
        x1 = x0;
    }
    if (y1 >= proto_cfg::kIrGridSize) {
        y1 = y0;
    }

    const float fx = gx - static_cast<float>(x0);
    const float fy = gy - static_cast<float>(y0);

    const float t00 = gridAt(grid, x0, y0);
    const float t10 = gridAt(grid, x1, y0);
    const float t01 = gridAt(grid, x0, y1);
    const float t11 = gridAt(grid, x1, y1);

    const float t0 = t00 + ((t10 - t00) * fx);
    const float t1 = t01 + ((t11 - t01) * fx);
    return t0 + ((t1 - t0) * fy);
}

static float sampleNearestGrid(const float* grid, float gx, float gy)
{
    constexpr float kMax = static_cast<float>(proto_cfg::kIrGridSize - 1U);
    if (gx < 0.0f) {
        gx = 0.0f;
    }
    if (gy < 0.0f) {
        gy = 0.0f;
    }
    if (gx > kMax) {
        gx = kMax;
    }
    if (gy > kMax) {
        gy = kMax;
    }
    const uint8_t col = static_cast<uint8_t>(gx + 0.5f);
    const uint8_t row = static_cast<uint8_t>(gy + 0.5f);
    const uint8_t c = (col >= proto_cfg::kIrGridSize)
                          ? static_cast<uint8_t>(proto_cfg::kIrGridSize - 1U)
                          : col;
    const uint8_t r = (row >= proto_cfg::kIrGridSize)
                          ? static_cast<uint8_t>(proto_cfg::kIrGridSize - 1U)
                          : row;
    return gridAt(grid, c, r);
}

static uint32_t nowMs(void)
{
    return static_cast<uint32_t>(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

static void mapOrientCoords(float gx, float gy, float grid_max,
                            uint8_t rot, bool flip_on,
                            float& out_gx, float& out_gy)
{
    if (flip_on) {
        gx = grid_max - gx;
    }

    switch (rot) {
        case display_layout::eIrRot90:
            out_gx = gy;
            out_gy = grid_max - gx;
            break;
        case display_layout::eIrRot180:
            out_gx = grid_max - gx;
            out_gy = grid_max - gy;
            break;
        case display_layout::eIrRot270:
            out_gx = grid_max - gy;
            out_gy = gx;
            break;
        case display_layout::eIrRot0:
        default:
            out_gx = gx;
            out_gy = gy;
            break;
    }
}

} /* anonymous */

IrHeatmap::~IrHeatmap()
{
    if (m_fb != nullptr) {
        heap_caps_free(m_fb);
        m_fb = nullptr;
    }
}

void IrHeatmap::reset(void)
{
    m_filter = display_layout::eIrFilterBilin;
    m_level  = 1U;
    m_rot    = display_layout::eIrRot180;
    m_flip   = false;
    m_scale_inited = false;
    m_ema_min = 0.0f;
    m_ema_max = 0.0f;
    m_scale_lo = 0.0f;
    m_scale_hi = proto_cfg::kIrMinSpanC;
    m_label_inited = false;
    m_ema_therm = 0.0f;
    m_last_label_ms = 0U;
    m_label_text[0] = '\0';
}

void IrHeatmap::setFilterMode(uint8_t mode)
{
    if (mode < display_layout::kIrFilterModeCount) {
        m_filter = mode;
    }
}

void IrHeatmap::setSharpLevel(uint8_t level)
{
    if (level < display_layout::kIrSharpLevelCount) {
        m_level = level;
    }
}

void IrHeatmap::setRotMode(uint8_t rot)
{
    if (rot < display_layout::kIrRotModeCount) {
        m_rot = rot;
    }
}

void IrHeatmap::setFlip(bool flip_on)
{
    m_flip = flip_on;
}

esp_err_t IrHeatmap::ensureFb(void)
{
    const uint16_t w = static_cast<uint16_t>(
        proto_cfg::kIrHeatX1 - proto_cfg::kIrHeatX0 + 1U);
    const uint16_t h = static_cast<uint16_t>(
        proto_cfg::kIrHeatY1 - proto_cfg::kIrHeatY0 + 1U);

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

void IrHeatmap::fbPixel(uint16_t lx, uint16_t ly, uint16_t color)
{
    if ((m_fb == nullptr) || (lx >= m_fb_w) || (ly >= m_fb_h)) {
        return;
    }
    m_fb[static_cast<size_t>(lx) * static_cast<size_t>(m_fb_h) + ly] = color;
}

void IrHeatmap::updateScale(float frame_min, float frame_max, float therm_c)
{
    const float a = proto_cfg::kIrScaleAlpha;
    if (!m_scale_inited) {
        m_ema_min = frame_min;
        m_ema_max = frame_max;
        m_scale_inited = true;
    } else {
        m_ema_min = ((1.0f - a) * m_ema_min) + (a * frame_min);
        m_ema_max = ((1.0f - a) * m_ema_max) + (a * frame_max);
    }

    const float lo = therm_c - proto_cfg::kIrBelowAmbientC;
    const float hi = therm_c + proto_cfg::kIrAboveAmbientC;

    float scale_lo = (m_ema_min < lo) ? m_ema_min : lo;
    float scale_hi = (m_ema_max > hi) ? m_ema_max : hi;

    const float span = scale_hi - scale_lo;
    if (span < proto_cfg::kIrMinSpanC) {
        const float half = proto_cfg::kIrMinSpanC * 0.5f;
        scale_lo = therm_c - half;
        scale_hi = therm_c + half;
    }

    m_scale_lo = scale_lo;
    m_scale_hi = scale_hi;
}

void IrHeatmap::applyUnsharp8(const float* src, float* dst, float amount) const
{
    if (amount <= 0.0f) {
        for (uint8_t i = 0U; i < amg8833_regs::kPixelCount; i++) {
            dst[i] = src[i];
        }
        return;
    }

    for (uint8_t row = 0U; row < proto_cfg::kIrGridSize; row++) {
        for (uint8_t col = 0U; col < proto_cfg::kIrGridSize; col++) {
            float sum = 0.0f;
            uint8_t n = 0U;
            for (int8_t dy = -1; dy <= 1; dy++) {
                const int16_t ry = static_cast<int16_t>(row) + dy;
                if ((ry < 0) || (ry >= proto_cfg::kIrGridSize)) {
                    continue;
                }
                for (int8_t dx = -1; dx <= 1; dx++) {
                    const int16_t cx = static_cast<int16_t>(col) + dx;
                    if ((cx < 0) || (cx >= proto_cfg::kIrGridSize)) {
                        continue;
                    }
                    sum += gridAt(src, static_cast<uint8_t>(cx), static_cast<uint8_t>(ry));
                    n++;
                }
            }
            const float blur = sum / static_cast<float>(n);
            const float src_v = gridAt(src, col, row);
            dst[static_cast<size_t>(row) * proto_cfg::kIrGridSize + col] =
                src_v + (amount * (src_v - blur));
        }
    }
}

void IrHeatmap::fillFbFromGrid(const float* grid, float inv_span)
{
    const float grid_max = static_cast<float>(proto_cfg::kIrGridSize - 1U);
    const float x_den = (m_fb_w > 1U) ? static_cast<float>(m_fb_w - 1U) : 1.0f;
    const float y_den = (m_fb_h > 1U) ? static_cast<float>(m_fb_h - 1U) : 1.0f;
    const bool nearest = (m_filter == display_layout::eIrFilterNear);

    for (uint16_t y = 0U; y < m_fb_h; y++) {
        const float gy = (static_cast<float>(y) * grid_max) / y_den;
        for (uint16_t x = 0U; x < m_fb_w; x++) {
            const float gx = (static_cast<float>(x) * grid_max) / x_den;
            float src_gx = gx;
            float src_gy = gy;
            mapOrientCoords(gx, gy, grid_max, m_rot, m_flip, src_gx, src_gy);
            const float t = nearest ? sampleNearestGrid(grid, src_gx, src_gy)
                                    : sampleBilinearGrid(grid, src_gx, src_gy);
            float norm = (t - m_scale_lo) * inv_span;
            if (norm < 0.0f) {
                norm = 0.0f;
            }
            if (norm > 1.0f) {
                norm = 1.0f;
            }
            fbPixel(x, y, infernoColor(norm));
        }
    }
}

void IrHeatmap::maybeDrawLabels(Display& display, uint32_t now_ms)
{
    if (m_label_inited &&
        ((now_ms - m_last_label_ms) < proto_cfg::kIrLabelPeriodMs)) {
        return;
    }

    char line[40];
    (void)snprintf(line, sizeof(line), "T %.1f  %.1f..%.1f",
                   static_cast<double>(m_ema_therm),
                   static_cast<double>(m_scale_lo),
                   static_cast<double>(m_scale_hi));

    if (m_label_inited && (strncmp(line, m_label_text, sizeof(m_label_text)) == 0)) {
        m_last_label_ms = now_ms;
        return;
    }

    (void)strncpy(m_label_text, line, sizeof(m_label_text) - 1U);
    m_label_text[sizeof(m_label_text) - 1U] = '\0';
    m_last_label_ms = now_ms;
    m_label_inited = true;

    const uint16_t y1 = static_cast<uint16_t>(proto_cfg::kIrStatusBandH - 1U);
    (void)display.fillRect(0U, 0U, 239U, y1, proto_cfg::kIrStatusBgColor);
    (void)display.drawString(2U, 2U, m_label_text, proto_cfg::kIrStatusFgColor);
}

esp_err_t IrHeatmap::render(Display& display,
                            const amg8833_frame_t& frame,
                            float thermistor_c)
{
    const esp_err_t fb_err = ensureFb();
    if (fb_err != ESP_OK) {
        return fb_err;
    }

    float frame_min = frame.pixels[0];
    float frame_max = frame.pixels[0];
    for (uint8_t i = 0U; i < amg8833_regs::kPixelCount; i++) {
        m_work[i] = frame.pixels[i];
        if (frame.pixels[i] < frame_min) {
            frame_min = frame.pixels[i];
        }
        if (frame.pixels[i] > frame_max) {
            frame_max = frame.pixels[i];
        }
    }

    updateScale(frame_min, frame_max, thermistor_c);

    const float la = proto_cfg::kIrLabelAlpha;
    if (!m_label_inited) {
        m_ema_therm = thermistor_c;
    } else {
        m_ema_therm = ((1.0f - la) * m_ema_therm) + (la * thermistor_c);
    }

    const float amount = (m_level < display_layout::kIrSharpLevelCount)
                             ? proto_cfg::kIrSharpenAmount[m_level]
                             : 0.0f;
    const float* grid = m_work;
    if (amount > 0.0f) {
        applyUnsharp8(m_work, m_sharp, amount);
        grid = m_sharp;
    }

    const float span = m_scale_hi - m_scale_lo;
    const float inv_span = (span > 0.001f) ? (1.0f / span) : 1.0f;
    fillFbFromGrid(grid, inv_span);

    const esp_err_t blit = display.drawPicture16(
        proto_cfg::kIrHeatX0, proto_cfg::kIrHeatY0,
        proto_cfg::kIrHeatX1, proto_cfg::kIrHeatY1,
        m_fb);
    if (blit != ESP_OK) {
        return blit;
    }

    maybeDrawLabels(display, nowMs());
    return ESP_OK;
}
