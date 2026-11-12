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

/**
 * @brief Tell if this raw sample should enter the TCP queue.
 * @param odr_hz Active output rate in hertz.
 * @return True when the sample is kept.
 * @note At 200 Hz and below, every raw sample is kept.
 *       Above 200 Hz the keep count holds one sample per group.
 *       The group size is the rate divided by 50. FFT frames do not use this check.
 */
bool app_wifi_keep_raw(float odr_hz);

#endif /* APP_WIFI_HPP */
