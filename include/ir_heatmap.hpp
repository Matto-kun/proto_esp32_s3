/**
 * @file        ir_heatmap.hpp
 * @brief       AMG8833 Inferno heatmap (filter/level + steady labels)
 * @author      Mat RB
 * @date        2026-08-26
 */

#ifndef IR_HEATMAP_HPP
#define IR_HEATMAP_HPP

#include <stdint.h>

#include "config.hpp"
#include "display/display_api.hpp"
#include "display/layout/ir_live_menu.hpp"
#include "esp_err.h"
#include "sensors/amg8833_api.hpp"

class IrHeatmap
{
public:
    IrHeatmap() = default;
    ~IrHeatmap();

    void reset(void);

    void setFilterMode(uint8_t mode);
    void setSharpLevel(uint8_t level);
    void setRotMode(uint8_t rot);
    void setFlip(bool flip_on);
    uint8_t filterMode(void) const { return m_filter; }
    uint8_t sharpLevel(void) const { return m_level; }
    uint8_t rotMode(void) const { return m_rot; }
    bool    flipOn(void) const { return m_flip; }

    esp_err_t render(Display& display,
                     const amg8833_frame_t& frame,
                     float thermistor_c);

private:
    esp_err_t ensureFb(void);
    void      fbPixel(uint16_t lx, uint16_t ly, uint16_t color);
    void      updateScale(float frame_min, float frame_max, float therm_c);
    void      maybeDrawLabels(Display& display, uint32_t now_ms);
    void      applyUnsharp8(const float* src, float* dst, float amount) const;
    void      fillFbFromGrid(const float* grid, float inv_span);

    uint16_t* m_fb = nullptr;
    uint16_t  m_fb_w = 0U;
    uint16_t  m_fb_h = 0U;

    float m_work[amg8833_regs::kPixelCount] = {};
    float m_sharp[amg8833_regs::kPixelCount] = {};

    uint8_t m_filter = display_layout::eIrFilterBilin;
    uint8_t m_level  = 1U;
    uint8_t m_rot    = display_layout::eIrRot180;
    bool    m_flip   = false;

    bool  m_scale_inited = false;
    float m_ema_min = 0.0f;
    float m_ema_max = 0.0f;
    float m_scale_lo = 0.0f;
    float m_scale_hi = 8.0f;

    bool     m_label_inited = false;
    float    m_ema_therm = 0.0f;
    uint32_t m_last_label_ms = 0U;
    char     m_label_text[40] = {};
};

#endif /* IR_HEATMAP_HPP */
