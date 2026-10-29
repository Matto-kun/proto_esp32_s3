/**
 * @file        sensor_bringup_main.cpp
 * @brief       HMC5983 + AMG8833 serial bring-up on I2C1 (SDA=39, SCL=38)
 * @details     Trial pins from operator. No display, no BLE.
 */

#include "config.hpp"

#include "i2c_api.hpp"
#include "sensors/amg8833_api.hpp"
#include "sensors/hmc5983_api.hpp"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "SENSOR_BRINGUP";
static constexpr uint32_t kLoopMs = 500U;

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "I2C1 SDA=%d SCL=%d @ %lu Hz",
             (int)proto_cfg::kPinI2cSda, (int)proto_cfg::kPinI2cScl,
             (unsigned long)proto_cfg::kI2cClkHz);
    ESP_LOGI(TAG, "Expect HMC5983 @ 0x1E, AMG8833 @ 0x69");

    I2C bus(proto_cfg::makeI2cCfg());
    if (bus.init() != ESP_OK) {
        ESP_LOGE(TAG, "I2C bus init FAILED");
        return;
    }
    ESP_LOGI(TAG, "I2C bus OK");

    Hmc5983 mag(bus, proto_cfg::makeHmc5983Cfg());
    const esp_err_t mag_err = mag.init();
    if (mag_err != ESP_OK) {
        ESP_LOGE(TAG, "HMC5983 init FAILED: %s", esp_err_to_name(mag_err));
    } else {
        ESP_LOGI(TAG, "HMC5983 init OK");
    }

    Amg8833 cam(bus, proto_cfg::makeAmgCfg());
    esp_err_t ir_err = cam.init();
    if (ir_err != ESP_OK) {
        ESP_LOGW(TAG, "AMG8833 @0x69 failed (%s); try 0x68",
                 esp_err_to_name(ir_err));
        amg8833_cfg_t alt_cfg = proto_cfg::makeAmgCfg();
        alt_cfg.addr7 = amg8833_regs::kAddr7Alt;
        Amg8833 cam_alt(bus, alt_cfg);
        ir_err = cam_alt.init();
        if (ir_err != ESP_OK) {
            ESP_LOGE(TAG, "AMG8833 init FAILED on 0x69 and 0x68");
        } else {
            ESP_LOGI(TAG, "AMG8833 init OK @ 0x68");
            while (true) {
                if (mag_err == ESP_OK) {
                    hmc5983_raw_t raw = {};
                    hmc5983_gauss_t g = {};
                    if (mag.readRaw(raw) == ESP_OK &&
                        mag.readGauss(g) == ESP_OK) {
                        ESP_LOGI(TAG,
                                 "MAG raw=%d,%d,%d g=%.3f,%.3f,%.3f",
                                 (int)raw.x, (int)raw.y, (int)raw.z,
                                 (double)g.x, (double)g.y, (double)g.z);
                    } else {
                        ESP_LOGW(TAG, "MAG read failed");
                    }
                }
                float therm = 0.0f;
                amg8833_frame_t frame = {};
                if (cam_alt.readThermistor(therm) == ESP_OK &&
                    cam_alt.readFrame(frame) == ESP_OK) {
                    ESP_LOGI(TAG, "IR therm=%.2fC px0=%.1f",
                             (double)therm, (double)frame.pixels[0]);
                } else {
                    ESP_LOGW(TAG, "IR read failed");
                }
                vTaskDelay(pdMS_TO_TICKS(kLoopMs));
            }
        }
        if (mag_err != ESP_OK) {
            return;
        }
        return;
    }
    ESP_LOGI(TAG, "AMG8833 init OK @ 0x%02X", (unsigned)cam.addr7());

    if (mag_err != ESP_OK) {
        ESP_LOGW(TAG, "Continuing with IR only");
    }

    while (true) {
        if (mag_err == ESP_OK) {
            hmc5983_raw_t raw = {};
            hmc5983_gauss_t g = {};
            if (mag.readRaw(raw) == ESP_OK && mag.readGauss(g) == ESP_OK) {
                ESP_LOGI(TAG, "MAG raw=%d,%d,%d g=%.3f,%.3f,%.3f",
                         (int)raw.x, (int)raw.y, (int)raw.z,
                         (double)g.x, (double)g.y, (double)g.z);
            } else {
                ESP_LOGW(TAG, "MAG read failed");
            }
        }
        float therm = 0.0f;
        amg8833_frame_t frame = {};
        if (cam.readThermistor(therm) == ESP_OK &&
            cam.readFrame(frame) == ESP_OK) {
            ESP_LOGI(TAG, "IR therm=%.2fC px0=%.1f",
                     (double)therm, (double)frame.pixels[0]);
        } else {
            ESP_LOGW(TAG, "IR read failed");
        }
        vTaskDelay(pdMS_TO_TICKS(kLoopMs));
    }
}
