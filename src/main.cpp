/**
 * @file        main.cpp
 * @brief       BLE bring-up: peripheral (default) or central via BLE_TEST_ROLE
 * @author      Mat RB
 * @date        2026-10-28
 *
 * @details
 *   Peripheral: advertise as ESP32-S3-BLE, status heartbeats, log RX writes.
 *   Central: scan for that name / default service UUID, subscribe, write RX.
 *   Build central with: build_flags = -DBLE_TEST_ROLE_CENTRAL=1
 */

#include "config.hpp"

#include "ble_api.hpp"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdio.h>
#include <string.h>

static const char* TAG = "BLE_TEST";

static void logPayload(const char* label, const uint8_t* data, size_t len)
{
    if (data == nullptr || len == 0U) {
        ESP_LOGI(TAG, "%s: (empty)", label);
        return;
    }
    char printable[80];
    const size_t n =
        (len < sizeof(printable) - 1U) ? len : (sizeof(printable) - 1U);
    for (size_t i = 0; i < n; ++i) {
        const char c = static_cast<char>(data[i]);
        printable[i] = (c >= 32 && c < 127) ? c : '.';
    }
    printable[n] = '\0';
    ESP_LOGI(TAG, "%s (%u): \"%s\"", label, (unsigned)len, printable);
}

static void onConn(bool connected, uint16_t handle)
{
    ESP_LOGI(TAG, "connected=%d handle=%u", (int)connected, (unsigned)handle);
}

static void onRx(const uint8_t* data, size_t len)
{
    logPayload("RX", data, len);
}

static void onStatusNotify(const uint8_t* data, size_t len)
{
    logPayload("status", data, len);
}

static void onTxNotify(const uint8_t* data, size_t len)
{
    logPayload("TX", data, len);
}

#if BLE_TEST_ROLE_CENTRAL

static void runCentral(void)
{
    ble_cfg_t cfg = proto_cfg::makeBleCfg();
    BleCentral cent(cfg);
    cent.setConnCallback(onConn);
    cent.setStatusNotifyCallback(onStatusNotify);
    cent.setTxNotifyCallback(onTxNotify);

    if (cent.init() != ESP_OK) {
        ESP_LOGE(TAG, "BleCentral init failed");
        return;
    }

    ESP_LOGI(TAG, "central mode; peer filter=\"%s\"", cfg.peer_name_filter);

    uint32_t writes = 0;
    while (true) {
        if (!cent.isConnected()) {
            ESP_LOGI(TAG, "scanning...");
            (void)cent.startScan();
            vTaskDelay(pdMS_TO_TICKS(proto_cfg::kBleCentralRescanMs));
            continue;
        }
        if (cent.isReady()) {
            char msg[32];
            snprintf(msg, sizeof(msg), "ping %lu", (unsigned long)writes);
            if (cent.writeRx(msg) == ESP_OK) {
                ESP_LOGI(TAG, "wrote \"%s\"", msg);
                ++writes;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(proto_cfg::kBleCentralWriteMs));
    }
}

#else

static void runPeripheral(void)
{
    ble_cfg_t cfg = proto_cfg::makeBleCfg();
    BlePeripheral peri(cfg);
    peri.setConnCallback(onConn);
    peri.setRxCallback(onRx);

    if (peri.init() != ESP_OK) {
        ESP_LOGE(TAG, "BlePeripheral init failed");
        return;
    }

    ESP_LOGI(TAG, "peripheral mode; name=\"%s\"", cfg.device_name);

    uint32_t beat = 0;
    bool sent_connected = false;
    while (true) {
        if (peri.isConnected()) {
            if (!sent_connected) {
                (void)peri.notifyStatus("connected");
                sent_connected = true;
            }
            char msg[48];
            snprintf(msg, sizeof(msg), "heartbeat %lu", (unsigned long)beat);
            (void)peri.notifyStatus(msg);
            ++beat;
        } else {
            sent_connected = false;
        }
        vTaskDelay(pdMS_TO_TICKS(proto_cfg::kBleHeartbeatMs));
    }
}

#endif

extern "C" void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(200));
#if BLE_TEST_ROLE_CENTRAL
    ESP_LOGI(TAG, "BLE_TEST_ROLE=central");
    runCentral();
#else
    ESP_LOGI(TAG, "BLE_TEST_ROLE=peripheral");
    runPeripheral();
#endif
}
