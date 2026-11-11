/**
 * @file        main.cpp
 * @brief       QMC5883P logger: microSD plus Wi-Fi SoftAP
 */

#include "config.hpp"

#include "app_mag.hpp"
#include "app_sd_log.hpp"
#include "app_wifi.hpp"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "APP_MAIN";

static void park(void)
{
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "DRDY GPIO %d stored, Status.RDY gates samples",
             static_cast<int>(proto_cfg::kPinMagDrdy));

    if (app_sd_init() != ESP_OK) {
        ESP_LOGE(TAG, "microSD mount FAILED");
        park();
    }
    ESP_LOGI(TAG, "microSD mounted at /sdcard");

    if (app_mag_init() != ESP_OK) {
        ESP_LOGE(TAG, "QMC5883P init FAILED");
        ESP_LOGW(TAG, "SoftAP still starts so the phone can connect");
    } else {
        ESP_LOGI(TAG, "QMC5883P id 0x80 at address 0x%02X",
                 static_cast<unsigned>(proto_cfg::kQmcAddr7));
    }

    if (app_wifi_start() != ESP_OK) {
        ESP_LOGE(TAG, "Wi-Fi start FAILED");
        park();
    }
}
