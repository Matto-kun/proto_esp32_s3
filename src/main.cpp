/**
 * @file        main.cpp
 * @brief       Two-level menu: root BLE/WIFI/SENSOR + SENSOR submenu
 * @author      Mat RB
 * @date        2026-08-12
 *
 * @details
 *   Tunable parameters in include/config.hpp (pins from pinout_gpio.json).
 *
 *   Bottom GPIO = advance (cycle selection)
 *   Top    GPIO = accept (mode icon, enter SENSOR, back; forward is no-op)
 *
 *   Magnetometer/plot runs only after Accept MAG (mag_app module).
 */

#include "config.hpp"
#include "mag_app.hpp"

#include "display/display_api.hpp"
#include "display/layout/menu_icons.hpp"
#include "display/layout/screen_geom.hpp"
#include "gpio_api.hpp"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "MENU";

enum UiLevel : uint8_t {
    eUiRoot   = 0,
    eUiSensor = 1,
};

static void clearIconBand(Display& display)
{
    (void)display.fillRect(
        display_layout::kIconBand.x0, display_layout::kIconBand.y0,
        display_layout::kIconBand.x1, display_layout::kIconBand.y1,
        kColorBlack);
}

extern "C" void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_NONE);

    display_board_t board = {};
    board.host     = eSpiHost2;
    board.mosi_pin = proto_cfg::kPinMosi;
    board.sclk_pin = proto_cfg::kPinSclk;
    board.dc_pin   = proto_cfg::kPinDc;
    board.rst_pin  = proto_cfg::kPinRst;
    board.bl_pin   = proto_cfg::kPinBl;
    board.clock_hz = proto_cfg::kSpiClockHz;

    Display display(board);

    ESP_LOGI(TAG, "Panel power settle %lu ms",
             (unsigned long)proto_cfg::kPanelPowerSettleMs);
    vTaskDelay(pdMS_TO_TICKS(proto_cfg::kPanelPowerSettleMs));

    if (display.init() != ESP_OK) {
        ESP_LOGE(TAG, "Display init failed");
        return;
    }

    gpio_cfg_t btn_cfg = {};
    btn_cfg.direction   = eGpioDirInput;
    btn_cfg.pull        = eGpioPullUp;
    btn_cfg.debounce_ms = proto_cfg::kBtnDebounceMs;

    btn_cfg.pin_num = proto_cfg::kBtnAdvance;
    GPIO btn_adv(btn_cfg);
    btn_cfg.pin_num = proto_cfg::kBtnAccept;
    GPIO btn_ok(btn_cfg);

    if (btn_adv.init() != ESP_OK || btn_ok.init() != ESP_OK) {
        ESP_LOGE(TAG, "Button init failed");
        return;
    }

    UiLevel level = eUiRoot;
    uint8_t root_sel   = display_layout::eMenuBle;
    uint8_t sensor_sel = display_layout::eSensorIrCam;
    bool prev_adv = true;
    bool prev_ok  = true;

    (void)display.clear(kColorBlack);
    (void)display_layout::draw_menu_bar(display, root_sel,
                                        kColorDkGray, kColorGreen,
                                        kColorWhite, kColorYellow);
    ESP_LOGI(TAG, "Menu bar drawn");

    while (true) {
        bool adv = true;
        bool ok  = true;
        (void)btn_adv.read(adv);
        (void)btn_ok.read(ok);

        if (prev_adv && !adv) {
            if (level == eUiRoot) {
                root_sel = static_cast<uint8_t>(
                    (root_sel + 1U) % display_layout::kMenuSelectCount);
                (void)display_layout::draw_menu_bar(display, root_sel,
                                                    kColorDkGray, kColorGreen,
                                                    kColorWhite, kColorYellow);
                ESP_LOGI(TAG, "Root advance -> %u", (unsigned)root_sel);
            } else {
                sensor_sel = static_cast<uint8_t>(
                    (sensor_sel + 1U) % display_layout::kSensorMenuSelectCount);
                (void)display_layout::draw_sensor_menu_bar(display, sensor_sel,
                                                           kColorDkGray, kColorGreen,
                                                           kColorWhite, kColorYellow);
                ESP_LOGI(TAG, "Sensor advance -> %u", (unsigned)sensor_sel);
            }
        }

        if (prev_ok && !ok) {
            if (level == eUiRoot) {
                if (root_sel == display_layout::eMenuFwd) {
                    ESP_LOGI(TAG, "Forward (no-op, menu full)");
                } else if (root_sel == display_layout::eMenuSensor) {
                    level = eUiSensor;
                    sensor_sel = display_layout::eSensorIrCam;
                    if (mag_app_is_live()) {
                        mag_app_stop(display);
                    }
                    clearIconBand(display);
                    (void)display_layout::draw_sensor_menu_bar(display, sensor_sel,
                                                               kColorDkGray, kColorGreen,
                                                               kColorWhite, kColorYellow);
                    ESP_LOGI(TAG, "Enter SENSOR submenu");
                } else {
                    (void)display_layout::draw_mode_icon(display, root_sel,
                                                         kColorCyan, kColorBlack);
                    ESP_LOGI(TAG, "Accept root mode %u", (unsigned)root_sel);
                }
            } else if (sensor_sel == display_layout::eSensorFwd) {
                ESP_LOGI(TAG, "Forward (no-op, menu full)");
            } else if (sensor_sel == display_layout::eSensorBack) {
                if (mag_app_is_live()) {
                    mag_app_stop(display);
                    (void)display_layout::draw_sensor_menu_bar(display, sensor_sel,
                                                               kColorDkGray, kColorGreen,
                                                               kColorWhite, kColorYellow);
                    ESP_LOGI(TAG, "Plot stopped; sensor submenu idle");
                } else {
                    level = eUiRoot;
                    clearIconBand(display);
                    (void)display_layout::draw_menu_bar(display, root_sel,
                                                        kColorDkGray, kColorGreen,
                                                        kColorWhite, kColorYellow);
                    ESP_LOGI(TAG, "Back to root menu");
                }
            } else if (sensor_sel == display_layout::eSensorMag) {
                if (mag_app_start(display) != ESP_OK) {
                    (void)display.fillRect(proto_cfg::kMagPlotX0, proto_cfg::kMagPlotY0,
                                           proto_cfg::kMagPlotX1, proto_cfg::kMagPlotY1,
                                           kColorBlack);
                    (void)display.drawString(8U, 8U, "MAG ERR", kColorRed);
                    (void)display_layout::draw_sensor_menu_bar(display, sensor_sel,
                                                               kColorDkGray, kColorGreen,
                                                               kColorWhite, kColorYellow);
                } else {
                    ESP_LOGI(TAG, "MAG live plot started");
                }
            } else {
                if (mag_app_is_live()) {
                    mag_app_stop(display);
                }
                (void)display_layout::draw_sensor_mode_icon(display, sensor_sel,
                                                            kColorCyan, kColorBlack);
                ESP_LOGI(TAG, "Accept sensor mode %u", (unsigned)sensor_sel);
            }
        }

        if (mag_app_is_live()) {
            mag_app_poll(display, sensor_sel);
        }

        prev_adv = adv;
        prev_ok  = ok;
        vTaskDelay(pdMS_TO_TICKS(proto_cfg::kMainLoopPeriodMs));
    }
}
