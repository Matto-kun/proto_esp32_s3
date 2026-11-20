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
 * @return ESP_OK when /sdcard is mounted. The mount error when the card is absent.
 * @note A failed mount frees the SPI bus. Later card calls do nothing and return ESP_OK.
 */
esp_err_t app_sd_init(void);

/**
 * @brief Open /sdcard/sNNNN/mag.csv and fft.csv.
 * @return ESP_OK when both files are open, or when the card is absent.
 */
esp_err_t app_sd_start_session(void);

/**
 * @brief Append one magnetometer row. Unit is uT.
 * @param rate_hz Active output rate in hertz.
 * @note Above 200 Hz the writer stores several lines, then writes that block.
 *       That path does not flush the file. Call app_sd_sync to flush it.
 */
esp_err_t app_sd_write_mag(uint32_t t_ms, float x_ut, float y_ut, float z_ut, float rate_hz);

/**
 * @brief Flush the mag block and both CSV files to the card.
 * @return ESP_OK when both files are durable, or when the card is absent.
 * @note The call does not close the files.
 */
esp_err_t app_sd_sync(void);

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
 * @brief Sync the session, then close it. Log the CSV paths and the sample count.
 */
void app_sd_stop_session(void);

#endif /* APP_SD_LOG_HPP */
