/**
 * @file        app_wifi.hpp
 * @brief       SoftAP and one TCP client for CBOR frames
 */

#ifndef APP_WIFI_HPP
#define APP_WIFI_HPP

#include "esp_err.h"

/**
 * @brief Start the SoftAP and the TCP listen task.
 * @return ESP_OK when the radio is up. The listen log comes from the task.
 */
esp_err_t app_wifi_start(void);

#endif /* APP_WIFI_HPP */
