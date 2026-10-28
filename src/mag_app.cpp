/**
 * @file        mag_app.cpp
 * @brief       Lazy magnetometer + live plot implementation
 * @author      Mat RB
 * @date        2026-08-19
 */

#include "mag_app.hpp"

#include "config.hpp"
#include "mag_fft.hpp"
#include "mag_plot.hpp"

#include "display/layout/mag_live_menu.hpp"
#include "i2c_api.hpp"
#include "sensors/hmc5983_api.hpp"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "MAG_APP";

static I2C       s_bus(proto_cfg::makeI2cCfg());
static Hmc5983   s_mag(s_bus, proto_cfg::makeHmc5983Cfg());
static MagPlot   s_plot;
static bool      s_live = false;
static uint32_t  s_last_sample_ms = 0U;

static uint32_t nowMs(void)
{
    return static_cast<uint32_t>(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

static void clearPlotArea(Display& display)
{
    (void)display.fillRect(0U, proto_cfg::kMagPlotY0,
                           proto_cfg::kMagPlotX1, proto_cfg::kMagPlotY1,
                           proto_cfg::kMagPlotBgColor);
}

esp_err_t mag_app_start(Display& display)
{
    if (s_live && s_mag.isInited()) {
        mag_app_reset_plot(display);
        return ESP_OK;
    }

    esp_err_t err = mag_fft_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "FFT init failed: %s", esp_err_to_name(err));
        return err;
    }

    err = s_bus.init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C init failed: %s", esp_err_to_name(err));
        mag_fft_deinit();
        return err;
    }

    err = s_mag.init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "HMC5983 init failed: %s", esp_err_to_name(err));
        (void)s_bus.deinit();
        mag_fft_deinit();
        return err;
    }

    s_live = true;
    s_last_sample_ms = 0U;
    s_plot.reset();
    s_plot.setViewMode(display_layout::eMagViewTime);
    s_plot.setAxisMode(display_layout::eMagAxisAll);
    s_plot.setUnitMode(display_layout::eMagUnitUT);
    clearPlotArea(display);
    ESP_LOGI(TAG, "Magnetometer started");
    return ESP_OK;
}

void mag_app_stop(Display& display)
{
    if (s_mag.isInited()) {
        (void)s_mag.deinit();
    }
    if (s_bus.isInited()) {
        (void)s_bus.deinit();
    }
    mag_fft_deinit();
    s_live = false;
    s_last_sample_ms = 0U;
    s_plot.reset();
    clearPlotArea(display);
    ESP_LOGI(TAG, "Magnetometer stopped");
}

void mag_app_reset_plot(Display& display)
{
    s_plot.reset();
    s_last_sample_ms = 0U;
    clearPlotArea(display);
}

bool mag_app_is_live(void)
{
    return s_live && s_mag.isInited();
}

uint8_t mag_app_view_mode(void)
{
    return s_plot.viewMode();
}

uint8_t mag_app_axis_mode(void)
{
    return s_plot.axisMode();
}

uint8_t mag_app_unit_mode(void)
{
    return s_plot.unitMode();
}

uint8_t mag_app_cycle_view(void)
{
    const uint8_t next = static_cast<uint8_t>(
        (s_plot.viewMode() + 1U) % display_layout::kMagViewModeCount);
    s_plot.setViewMode(next);
    return next;
}

uint8_t mag_app_cycle_axis(void)
{
    const uint8_t next = static_cast<uint8_t>(
        (s_plot.axisMode() + 1U) % display_layout::kMagAxisModeCount);
    s_plot.setAxisMode(next);
    return next;
}

uint8_t mag_app_cycle_unit(void)
{
    const uint8_t next = static_cast<uint8_t>(
        (s_plot.unitMode() + 1U) % display_layout::kMagUnitModeCount);
    s_plot.setUnitMode(next);
    return next;
}

void mag_app_poll(Display& display)
{
    if (!mag_app_is_live()) {
        return;
    }

    const uint32_t t = nowMs();
    if ((s_last_sample_ms != 0U) &&
        ((t - s_last_sample_ms) < proto_cfg::kMagSamplePeriodMs)) {
        return;
    }

    hmc5983_gauss_t gauss = {};
    const esp_err_t err = s_mag.readGauss(gauss);
    if (err == ESP_OK) {
        s_plot.pushSample(gauss.x, gauss.y, gauss.z, t);
        (void)s_plot.draw(display, t);
        s_last_sample_ms = t;
    } else if (err != ESP_ERR_NOT_FINISHED) {
        ESP_LOGW(TAG, "readGauss failed: %s", esp_err_to_name(err));
        s_last_sample_ms = t;
    }
}
