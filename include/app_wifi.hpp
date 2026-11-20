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
 * @param full_rate True when the TCP task keeps every raw sample.
 * @return True when the sample is kept.
 * @note When full_rate is true, every raw sample is kept.
 *       At 200 Hz or below, every raw sample is kept.
 *       Above 200 Hz with full_rate false, the task keeps about 50 samples each second.
 *       FFT frames do not use this check.
 */
bool app_wifi_keep_raw(float odr_hz, bool full_rate);

#endif /* APP_WIFI_HPP */
