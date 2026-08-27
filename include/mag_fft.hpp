/**
 * @file        mag_fft.hpp
 * @brief       Hann-windowed magnitude spectrum via esp-dsp FFT
 * @author      Mat RB
 * @date        2026-08-26
 */

#ifndef MAG_FFT_HPP
#define MAG_FFT_HPP

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

/**
 * @brief One-time FFT table init (call from mag_app_start).
 */
esp_err_t mag_fft_init(void);

/**
 * @brief Release FFT tables (call from mag_app_stop).
 */
void mag_fft_deinit(void);

/**
 * @brief Compute |FFT| of a real signal (length must be kMagFftN).
 * @param input   Real samples in Gauss (or same units throughout).
 * @param n       Must equal proto_cfg::kMagFftN.
 * @param out_mag Length >= n/2; out_mag[k] = magnitude of bin k.
 * @return ESP_OK on success.
 */
esp_err_t mag_fft_magnitude(const float* input, size_t n, float* out_mag);

#endif /* MAG_FFT_HPP */
