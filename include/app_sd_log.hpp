/**
 * @file        app_sd_log.hpp
 * @brief       SPI microSD mount and session CSV files
 */

#ifndef APP_SD_LOG_HPP
#define APP_SD_LOG_HPP

#include <stdint.h>

#include "esp_err.h"

/**
 * @brief Mount the card. Do not format the card when the mount fails.
 * @return ESP_OK when /sdcard is mounted.
 */
esp_err_t app_sd_init(void);

/**
 * @brief Open /sdcard/sNNNN/mag.csv and fft.csv.
 * @return ESP_OK when both files are open.
 */
esp_err_t app_sd_start_session(void);

/**
 * @brief Append one magnetometer row. Unit is uT.
 */
esp_err_t app_sd_write_mag(uint32_t t_ms, float x_ut, float y_ut, float z_ut);

/**
 * @brief Append one FFT window. bins is the length of each array.
 */
esp_err_t app_sd_write_fft(uint32_t t_ms,
                           float rate_hz,
                           const float* mx,
                           const float* my,
                           const float* mz,
                           const float* mabs,
                           uint16_t bins);

/**
 * @brief Flush and close the session. Log the CSV paths and the sample count.
 */
void app_sd_stop_session(void);

#endif /* APP_SD_LOG_HPP */
