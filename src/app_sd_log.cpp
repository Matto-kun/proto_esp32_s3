/**
 * @file        app_sd_log.cpp
 * @brief       Session CSV writer for mag samples and FFT rows
 */

#include "app_sd_log.hpp"

#include "config.hpp"
#include "sd_api.hpp"

#include <errno.h>
#include <stdio.h>
#include <sys/stat.h>

#include "esp_log.h"

static const char* TAG = "APP_SD";
static constexpr size_t kDirLen = 32U;
static constexpr size_t kPathLen = 48U;

static SdCard s_card(proto_cfg::makeSdCfg());
static FILE* s_mag = nullptr;
static FILE* s_fft = nullptr;
static char s_dir[kDirLen] = {};
static uint32_t s_samples = 0U;

static esp_err_t readSeq(uint32_t& seq)
{
    seq = 1U;
    FILE* file = fopen("/sdcard/SEQ.TXT", "r");
    if (file == nullptr) {
        return ESP_OK;
    }
    unsigned long value = 1U;
    if (fscanf(file, "%lu", &value) != 1) {
        value = 1U;
    }
    fclose(file);
    if (value == 0U || value > 9999U) {
        value = 1U;
    }
    seq = static_cast<uint32_t>(value);
    return ESP_OK;
}

static void writeSeq(uint32_t seq)
{
    FILE* file = fopen("/sdcard/SEQ.TXT", "w");
    if (file == nullptr) {
        ESP_LOGW(TAG, "SEQ.TXT write failed");
        return;
    }
    fprintf(file, "%lu\n", static_cast<unsigned long>(seq));
    fclose(file);
}

esp_err_t app_sd_init(void)
{
    const esp_err_t err = s_card.init();
    if (err != ESP_OK) {
        return err;
    }
    return ESP_OK;
}

esp_err_t app_sd_start_session(void)
{
    if (s_mag != nullptr) {
        return ESP_OK;
    }
    uint32_t seq = 1U;
    (void)readSeq(seq);
    snprintf(s_dir, sizeof(s_dir), "/sdcard/s%04lu", static_cast<unsigned long>(seq));
    if (mkdir(s_dir, 0775) != 0 && errno != EEXIST) {
        ESP_LOGE(TAG, "mkdir %s failed", s_dir);
        return ESP_FAIL;
    }
    writeSeq(seq + 1U);

    char path[kPathLen];
    snprintf(path, sizeof(path), "%s/mag.csv", s_dir);
    s_mag = fopen(path, "w");
    if (s_mag == nullptr) {
        ESP_LOGE(TAG, "open %s failed", path);
        return ESP_FAIL;
    }
    fprintf(s_mag, "t_ms,x_uT,y_uT,z_uT\n");

    snprintf(path, sizeof(path), "%s/fft.csv", s_dir);
    s_fft = fopen(path, "w");
    if (s_fft == nullptr) {
        ESP_LOGE(TAG, "open %s failed", path);
        fclose(s_mag);
        s_mag = nullptr;
        return ESP_FAIL;
    }
    fprintf(s_fft, "t_ms,bin,freq_hz,mx,my,mz,mabs\n");
    s_samples = 0U;
    ESP_LOGI(TAG, "session %s open", s_dir);
    return ESP_OK;
}

esp_err_t app_sd_write_mag(uint32_t t_ms, float x_ut, float y_ut, float z_ut)
{
    if (s_mag == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    const int wrote = fprintf(s_mag, "%lu,%.4f,%.4f,%.4f\n",
                              static_cast<unsigned long>(t_ms),
                              static_cast<double>(x_ut),
                              static_cast<double>(y_ut),
                              static_cast<double>(z_ut));
    if (wrote < 0) {
        return ESP_FAIL;
    }
    s_samples++;
    if ((s_samples % 16U) == 0U) {
        fflush(s_mag);
    }
    return ESP_OK;
}

esp_err_t app_sd_write_fft(uint32_t t_ms,
                           float rate_hz,
                           const float* mx,
                           const float* my,
                           const float* mz,
                           const float* mabs,
                           uint16_t bins)
{
    if (s_fft == nullptr || mx == nullptr || my == nullptr ||
        mz == nullptr || mabs == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    const float df = rate_hz / static_cast<float>(proto_cfg::kMagFftN);
    for (uint16_t bin = 0U; bin < bins; ++bin) {
        const float freq = df * static_cast<float>(bin);
        const int wrote = fprintf(
            s_fft, "%lu,%u,%.5f,%.5f,%.5f,%.5f,%.5f\n",
            static_cast<unsigned long>(t_ms),
            static_cast<unsigned>(bin),
            static_cast<double>(freq),
            static_cast<double>(mx[bin]),
            static_cast<double>(my[bin]),
            static_cast<double>(mz[bin]),
            static_cast<double>(mabs[bin]));
        if (wrote < 0) {
            return ESP_FAIL;
        }
    }
    return ESP_OK;
}

void app_sd_stop_session(void)
{
    if (s_mag != nullptr) {
        fflush(s_mag);
        fclose(s_mag);
        s_mag = nullptr;
    }
    if (s_fft != nullptr) {
        fflush(s_fft);
        fclose(s_fft);
        s_fft = nullptr;
    }
    if (s_dir[0] == '\0') {
        return;
    }
    ESP_LOGI(TAG, "session %s/mag.csv %s/fft.csv samples=%lu",
             s_dir, s_dir, static_cast<unsigned long>(s_samples));
}
