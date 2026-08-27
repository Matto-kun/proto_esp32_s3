/**
 * @file        ir_heatmap.cpp
 * @brief       Inferno colormap heatmap for AMG8833 frames
 * @author      Mat RB
 * @date        2026-08-26
 */

#include "ir_heatmap.hpp"

#include <stdio.h>

#include "esp_heap_caps.h"

namespace {

/** Matplotlib Inferno stops (approx RGB), 32 entries. */
static constexpr uint8_t kInfernoStops = 32U;

static const uint8_t kInfernoRgb[kInfernoStops][3] = {
    {  0,   0,   4}, { 13,   8,  25}, { 26,  11,  52}, { 40,  11,  71},
    { 56,   9,  87}, { 70,  10,  94}, { 84,  15,  98}, { 97,  22,  98},
    {111,  29,  96}, {124,  36,  92}, {138,  43,  86}, {151,  51,  80},
    {164,  59,  72}, {177,  68,  63}, {189,  78,  53}, {201,  89,  42},
    {211, 101,  30}, {220, 114,  17}, {227, 129,   6}, {232, 145,   6},
    {236, 161,  18}, {239, 177,  35}, {242, 193,  52}, {245, 209,  71},
    {247, 224,  92}, {249, 237, 115}, {251, 247, 141}, {252, 254, 168},
    {251, 255, 191}, {248, 254, 210},     {245, 251, 224}, {252, 255, 164},
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
    /* Keep FB allocated; clear on next render. */
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
    /* Column-major: img[x * H + y] (LibDriver ST7789). */
    m_fb[static_cast<size_t>(lx) * static_cast<size_t>(m_fb_h) + ly] = color;
}

esp_err_t IrHeatmap::render(Display& display,
                            const amg8833_frame_t& frame,
                            float thermistor_c)
{
    const esp_err_t fb_err = ensureFb();
    if (fb_err != ESP_OK) {
        return fb_err;
    }

    float t_min = frame.pixels[0];
    float t_max = frame.pixels[0];
    for (uint8_t i = 1U; i < amg8833_regs::kPixelCount; i++) {
        if (frame.pixels[i] < t_min) {
            t_min = frame.pixels[i];
        }
        if (frame.pixels[i] > t_max) {
            t_max = frame.pixels[i];
        }
    }

    float span = t_max - t_min;
    if (span < 0.25f) {
        span = 0.25f;
        t_min = t_max - span;
    }

    const uint16_t cell_w = static_cast<uint16_t>(m_fb_w / proto_cfg::kIrGridSize);
    const uint16_t cell_h = static_cast<uint16_t>(m_fb_h / proto_cfg::kIrGridSize);

    for (uint8_t row = 0U; row < proto_cfg::kIrGridSize; row++) {
        for (uint8_t col = 0U; col < proto_cfg::kIrGridSize; col++) {
            const float t = frame.pixels[
                static_cast<size_t>(row) * proto_cfg::kIrGridSize + col];
            float norm = (t - t_min) / span;
            if (norm < 0.0f) {
                norm = 0.0f;
            }
            if (norm > 1.0f) {
                norm = 1.0f;
            }
            const uint16_t color = infernoColor(norm);

            const uint16_t x0 = static_cast<uint16_t>(col * cell_w);
            const uint16_t y0 = static_cast<uint16_t>(row * cell_h);
            uint16_t x1 = static_cast<uint16_t>(x0 + cell_w);
            uint16_t y1 = static_cast<uint16_t>(y0 + cell_h);
            if (col == (proto_cfg::kIrGridSize - 1U)) {
                x1 = m_fb_w;
            }
            if (row == (proto_cfg::kIrGridSize - 1U)) {
                y1 = m_fb_h;
            }

            for (uint16_t x = x0; x < x1; x++) {
                for (uint16_t y = y0; y < y1; y++) {
                    fbPixel(x, y, color);
                }
            }
        }
    }

    const esp_err_t blit = display.drawPicture16(
        proto_cfg::kIrHeatX0, proto_cfg::kIrHeatY0,
        proto_cfg::kIrHeatX1, proto_cfg::kIrHeatY1,
        m_fb);
    if (blit != ESP_OK) {
        return blit;
    }

    char line[40];
    (void)snprintf(line, sizeof(line), "T %.1f  %.1f..%.1f",
                   static_cast<double>(thermistor_c),
                   static_cast<double>(t_min),
                   static_cast<double>(t_max));

    (void)display.fillRect(0U, 0U, 239U, 14U, proto_cfg::kIrStatusBgColor);
    (void)display.drawString(2U, 2U, line, proto_cfg::kIrStatusFgColor);
    return ESP_OK;
}
