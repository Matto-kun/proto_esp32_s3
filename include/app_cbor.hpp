/**
 * @file        app_cbor.hpp
 * @brief       CBOR encode and decode for the logger TCP frames
 */

#ifndef APP_CBOR_HPP
#define APP_CBOR_HPP

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "app_mag.hpp"

struct cbor_buf_t {
    uint8_t* data = nullptr;
    size_t cap = 0;
    size_t len = 0;
};

/**
 * @brief Decode one control map (config, start, or stop).
 * @return ESP_OK when the map has a known cmd string.
 */
esp_err_t app_cbor_decode_cmd(const uint8_t* data, size_t len, app_cmd_t& out);

/** @brief Encode the config reply map. */
esp_err_t app_cbor_encode_cfg(cbor_buf_t& buf, const app_cfg_rsp_t& rsp);

/** @brief Encode one mag sample map. Unit is uT. */
esp_err_t app_cbor_encode_mag(cbor_buf_t& buf, const mag_sample_msg_t& sample);

/** @brief Encode one 3-axis FFT map. */
esp_err_t app_cbor_encode_fft(cbor_buf_t& buf, const mag_fft_msg_t& fft);

#endif /* APP_CBOR_HPP */
