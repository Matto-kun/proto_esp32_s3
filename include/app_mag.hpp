/**
 * @file        app_mag.hpp
 * @brief       QMC5883P sample task, FFT window, and command mailbox
 */

#ifndef APP_MAG_HPP
#define APP_MAG_HPP

#include <stdint.h>
#include <stdbool.h>

#include "esp_err.h"
#include "config.hpp"

enum class eAppCmd : uint8_t {
    eCmdNone = 0,
    eCmdConfig,
    eCmdStart,
    eCmdStop,
};

struct app_cmd_t {
    eAppCmd kind = eAppCmd::eCmdNone;
    bool list_present = false;
    bool has_mag = false;
    bool has_rate = false;
    float rate_hz = 0.0f;
};

struct app_cfg_rsp_t {
    bool ok = false;
    uint8_t addr7 = 0;
    char chip_id[4] = {'8', '0', '\0'};
    float odr_hz = 0.0f;
    float gain_ga = 0.0f;
    uint8_t samples = 0;
    uint16_t fft_n = 0;
};

struct mag_sample_msg_t {
    uint32_t t_ms = 0;
    float x_ut = 0.0f;
    float y_ut = 0.0f;
    float z_ut = 0.0f;
};

struct mag_fft_msg_t {
    uint32_t t_ms = 0;
    float rate_hz = 0.0f;
    float df_hz = 0.0f;
    uint16_t n = 0;
    float mx[proto_cfg::kMagFftBins] = {};
    float my[proto_cfg::kMagFftBins] = {};
    float mz[proto_cfg::kMagFftBins] = {};
    float mabs[proto_cfg::kMagFftBins] = {};
};

/**
 * @brief Start I2C, the QMC5883P, the FFT tables, and the sample task.
 * @return ESP_OK when the chip id is 0x80.
 */
esp_err_t app_mag_init(void);

/**
 * @brief Post a command and wait until the sample task applies it.
 * @param cmd Command from the TCP client.
 * @param rsp Filled with the applied sensor config.
 * @return ESP_OK when a reply arrives. ESP_ERR_TIMEOUT when the task does not answer.
 */
esp_err_t app_mag_request(const app_cmd_t& cmd, app_cfg_rsp_t& rsp);

/** @brief Take one queued sample. ESP_ERR_TIMEOUT when the queue is empty. */
esp_err_t app_mag_pop_sample(mag_sample_msg_t& out);

/** @brief Take one queued FFT frame. ESP_ERR_TIMEOUT when the queue is empty. */
esp_err_t app_mag_pop_fft(mag_fft_msg_t& out);

#endif /* APP_MAG_HPP */
