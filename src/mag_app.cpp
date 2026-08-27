/**
 * @file        mag_app.cpp
 * @brief       Lazy magnetometer + live plot implementation
 * @author      Mat RB
 * @date        2026-08-19
 */

#include "mag_app.hpp"

#include "config.hpp"
#include "mag_plot.hpp"

#include "i2c_api.hpp"
#include "sensors/hmc5883l_api.hpp"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "MAG_APP";

static I2C       s_bus(proto_cfg::makeI2cCfg());
static Hmc5883l  s_mag(s_bus, proto_cfg::makeHmcCfg());
static MagPlot   s_plot;
static bool      s_live = false;
static uint32_t  s_last_sample_ms = 0U;

static uint32_t nowMs(void)
{
    return static_cast<uint32_t>(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

static void clearPlotArea(Display& display)
{
    (void)display.fillRect(proto_cfg::kMagPlotX0, proto_cfg::kMagPlotY0,
                           proto_cfg::kMagPlotX1, proto_cfg::kMagPlotY1,
                           proto_cfg::kMagPlotBgColor);
}

esp_err_t mag_app_start(Display& display)
{
    if (s_live && s_mag.isInited()) {
        mag_app_reset_plot(display);
        return ESP_OK;
    }

    esp_err_t err = s_bus.init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C init failed: %s", esp_err_to_name(err));
        return err;
    }

    err = s_mag.init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "HMC5883L init failed: %s", esp_err_to_name(err));
        (void)s_bus.deinit();
        return err;
    }

    s_live = true;
    s_last_sample_ms = 0U;
    s_plot.reset();
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

void mag_app_poll(Display& display, uint8_t sensor_sel)
{
    (void)sensor_sel;

    if (!mag_app_is_live()) {
        return;
    }

    const uint32_t t = nowMs();
    if ((s_last_sample_ms != 0U) &&
        ((t - s_last_sample_ms) < proto_cfg::kMagSamplePeriodMs)) {
        return;
    }

    hmc5883l_gauss_t gauss = {};
    if (s_mag.readGauss(gauss) == ESP_OK) {
        s_plot.pushSample(gauss.x, gauss.y, gauss.z, t);
        (void)s_plot.draw(display, t);
    } else {
        ESP_LOGW(TAG, "readGauss failed");
    }
    s_last_sample_ms = t;
}
