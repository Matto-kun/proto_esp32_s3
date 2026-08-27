/**
 * @file        ir_heatmap.hpp
 * @brief       AMG8833 8x8 Inferno heatmap blit (PSRAM RGB565)
 * @author      Mat RB
 * @date        2026-08-26
 */

#ifndef IR_HEATMAP_HPP
#define IR_HEATMAP_HPP

#include <stdint.h>

#include "config.hpp"
#include "display/display_api.hpp"
#include "esp_err.h"
#include "sensors/amg8833_api.hpp"

class IrHeatmap
{
public:
    IrHeatmap() = default;
    ~IrHeatmap();

    void reset(void);

    /**
     * @brief Upscale 8x8 frame with Inferno colormap and blit once.
     * @note  Auto-scales to frame min/max. Overlays thermistor + range.
     */
    esp_err_t render(Display& display,
                     const amg8833_frame_t& frame,
                     float thermistor_c);

private:
    esp_err_t ensureFb(void);
    void      fbPixel(uint16_t lx, uint16_t ly, uint16_t color);

    uint16_t* m_fb = nullptr;
    uint16_t  m_fb_w = 0U;
    uint16_t  m_fb_h = 0U;
};

#endif /* IR_HEATMAP_HPP */
