/**
 * @file        ir_app.cpp
 * @brief       Lazy AMG8833 + live heatmap implementation
 * @author      Mat RB
 * @date        2026-08-26
 */

#include "ir_app.hpp"

#include "config.hpp"
#include "ir_heatmap.hpp"

#include "i2c_api.hpp"
#include "sensors/amg8833_api.hpp"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "IR_APP";

static I2C       s_bus(proto_cfg::makeI2cCfg());
static Amg8833   s_cam(s_bus, proto_cfg::makeAmgCfg());
static IrHeatmap s_heat;
static bool      s_live = false;
static uint32_t  s_last_sample_ms = 0U;

static uint32_t nowMs(void)
{
    return static_cast<uint32_t>(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

static void clearHeatArea(Display& display)
{
    (void)display.fillRect(proto_cfg::kIrHeatX0, proto_cfg::kIrHeatY0,
                           proto_cfg::kIrHeatX1, proto_cfg::kIrHeatY1,
                           proto_cfg::kIrStatusBgColor);
}

esp_err_t ir_app_start(Display& display)
{
    if (s_live && s_cam.isInited()) {
        s_last_sample_ms = 0U;
        clearHeatArea(display);
        return ESP_OK;
    }

    esp_err_t err = s_bus.init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C init failed: %s", esp_err_to_name(err));
        return err;
    }

    err = s_cam.init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "AMG8833 init failed: %s", esp_err_to_name(err));
        (void)s_bus.deinit();
        return err;
    }

    s_live = true;
    s_last_sample_ms = 0U;
    s_heat.reset();
    clearHeatArea(display);
    ESP_LOGI(TAG, "IR cam started");
    return ESP_OK;
}

void ir_app_stop(Display& display)
{
    if (s_cam.isInited()) {
        (void)s_cam.deinit();
    }
    if (s_bus.isInited()) {
        (void)s_bus.deinit();
    }
    s_live = false;
    s_last_sample_ms = 0U;
    s_heat.reset();
    clearHeatArea(display);
    ESP_LOGI(TAG, "IR cam stopped");
}

bool ir_app_is_live(void)
{
    return s_live && s_cam.isInited();
}

void ir_app_poll(Display& display)
{
    if (!ir_app_is_live()) {
        return;
    }

    const uint32_t now = nowMs();
    if ((s_last_sample_ms != 0U) &&
        ((now - s_last_sample_ms) < proto_cfg::kIrSamplePeriodMs)) {
        return;
    }
    s_last_sample_ms = now;

    float therm = 0.0f;
    amg8833_frame_t frame = {};
    if (s_cam.readThermistor(therm) != ESP_OK) {
        return;
    }
    if (s_cam.readFrame(frame) != ESP_OK) {
        return;
    }

    (void)s_heat.render(display, frame, therm);
}
