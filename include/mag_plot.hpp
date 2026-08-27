/**
 * @file        mag_plot.hpp
 * @brief       Live X/Y/Z magnetometer trace plotter (app module)
 * @author      Mat RB
 * @date        2026-08-19
 *
 * @details
 *   Ring buffer of Gauss samples with shared mean-centered Y auto-scale
 *   over a configurable time window. Composes three traces into a host
 *   RGB565 framebuffer (column-major for LibDriver ST7789 picture blit)
 *   and uploads once per draw via Display::drawPicture16.
 */

#ifndef MAG_PLOT_HPP
#define MAG_PLOT_HPP

#include <stdint.h>

#include "config.hpp"
#include "display/display_api.hpp"
#include "esp_err.h"

class MagPlot
{
public:
    MagPlot() = default;
    ~MagPlot();

    void reset(void);

    /** Append one sample; oldest entries drop when buffer is full. */
    void pushSample(float x_gauss, float y_gauss, float z_gauss, uint32_t t_ms);

    /**
     * Compose plot into host FB and blit once.
     * @return ESP_ERR_NO_MEM if framebuffer allocation fails.
     */
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

    Sample   m_buf[proto_cfg::kMagMaxSamples] = {};
    uint16_t m_count = 0U;
    uint16_t m_head  = 0U;

    /** Column-major RGB565: fb[lx * H + ly]. Async SPI queue deferred. */
    uint16_t* m_fb = nullptr;
    uint16_t  m_fb_w = 0U;
    uint16_t  m_fb_h = 0U;
};

#endif /* MAG_PLOT_HPP */
