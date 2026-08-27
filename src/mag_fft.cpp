/**
 * @file        mag_fft.cpp
 * @brief       Hann + dsps_fft2r magnitude spectrum
 * @author      Mat RB
 * @date        2026-08-26
 */

#include "mag_fft.hpp"

#include "config.hpp"

#include "esp_log.h"
#include "dsps_fft2r.h"
#include "dsps_wind.h"

#include <math.h>
#include <string.h>

static const char* TAG = "MAG_FFT";

static bool  s_inited = false;
static float s_window[proto_cfg::kMagFftN];
static float s_fft[proto_cfg::kMagFftN * 2U];

esp_err_t mag_fft_init(void)
{
    if (s_inited) {
        return ESP_OK;
    }

    const esp_err_t err = dsps_fft2r_init_fc32(nullptr, CONFIG_DSP_MAX_FFT_SIZE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "dsps_fft2r_init_fc32 failed: %s", esp_err_to_name(err));
        return err;
    }

    dsps_wind_hann_f32(s_window, proto_cfg::kMagFftN);
    s_inited = true;
    return ESP_OK;
}

void mag_fft_deinit(void)
{
    if (!s_inited) {
        return;
    }
    dsps_fft2r_deinit_fc32();
    s_inited = false;
}

esp_err_t mag_fft_magnitude(const float* input, size_t n, float* out_mag)
{
    if (!s_inited) {
        return ESP_ERR_INVALID_STATE;
    }
    if ((input == nullptr) || (out_mag == nullptr) || (n != proto_cfg::kMagFftN)) {
        return ESP_ERR_INVALID_ARG;
    }

    for (size_t i = 0U; i < n; i++) {
        s_fft[i * 2U + 0U] = input[i] * s_window[i];
        s_fft[i * 2U + 1U] = 0.0f;
    }

    esp_err_t err = dsps_fft2r_fc32(s_fft, static_cast<int>(n));
    if (err != ESP_OK) {
        return err;
    }
    err = dsps_bit_rev_fc32(s_fft, static_cast<int>(n));
    if (err != ESP_OK) {
        return err;
    }

    const float scale0 = 1.0f / static_cast<float>(n);
    const float scale  = 2.0f / static_cast<float>(n);
    const size_t bins  = n / 2U;

    for (size_t k = 0U; k < bins; k++) {
        const float re = s_fft[k * 2U + 0U];
        const float im = s_fft[k * 2U + 1U];
        const float mag = hypotf(re, im);
        out_mag[k] = mag * ((k == 0U) ? scale0 : scale);
    }

    return ESP_OK;
}
