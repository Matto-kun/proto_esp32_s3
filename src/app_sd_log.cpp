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
#include <unistd.h>

#include "esp_log.h"

static const char* TAG = "APP_SD";
static constexpr size_t kDirLen = 32U;
static constexpr size_t kPathLen = 48U;
static constexpr size_t kMagCsvLineCap = 64U;
static constexpr size_t kMagCsvBurstLines = 16U;
static constexpr size_t kMagCsvBurstBytes = kMagCsvLineCap * kMagCsvBurstLines;

static SdCard s_card(proto_cfg::makeSdCfg());
static bool s_card_present = false;
static FILE* s_mag = nullptr;
static FILE* s_fft = nullptr;
static char s_dir[kDirLen] = {};
static uint32_t s_samples = 0U;
// Internal DRAM (BSS). Do not allocate this buffer on the heap or in PSRAM.
static char s_mag_burst[kMagCsvBurstBytes];
static size_t s_mag_burst_used = 0U;
static uint8_t s_mag_burst_lines = 0U;

// Write the RAM block to the FILE buffer. app_sd_sync flushes that buffer.
static esp_err_t writeMagBurstFile(void)
{
    if (s_mag_burst_used == 0U) {
        return ESP_OK;
    }
    if (s_mag == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    const size_t wrote = fwrite(s_mag_burst, 1U, s_mag_burst_used, s_mag);
    if (wrote != s_mag_burst_used) {
        ESP_LOGW(TAG, "mag burst write failed");
        return ESP_FAIL;
    }
    s_mag_burst_used = 0U;
    s_mag_burst_lines = 0U;
    return ESP_OK;
}

static esp_err_t fsyncFile(FILE* file)
{
    if (file == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    const int fd = fileno(file);
    if (fd < 0) {
        return ESP_FAIL;
    }
    if (fsync(fd) != 0) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

static esp_err_t writeMagBurst(uint32_t t_ms, float x_ut, float y_ut, float z_ut, float rate_hz)
{
    const bool full = s_mag_burst_lines >= kMagCsvBurstLines;
    const bool tight = (kMagCsvBurstBytes - s_mag_burst_used) < kMagCsvLineCap;
    if (full || tight) {
        const esp_err_t err = writeMagBurstFile();
        if (err != ESP_OK) {
            return err;
        }
    }

    const size_t room = kMagCsvBurstBytes - s_mag_burst_used;
    const int wrote = snprintf(
        s_mag_burst + s_mag_burst_used,
        room,
        "%lu,%.4f,%.4f,%.4f,%.4f\n",
        static_cast<unsigned long>(t_ms),
        static_cast<double>(x_ut),
        static_cast<double>(y_ut),
        static_cast<double>(z_ut),
        static_cast<double>(rate_hz));
    if (wrote < 0 || static_cast<size_t>(wrote) >= room) {
        return ESP_FAIL;
    }
    s_mag_burst_used += static_cast<size_t>(wrote);
    s_mag_burst_lines++;
    if (s_mag_burst_lines >= kMagCsvBurstLines) {
        return writeMagBurstFile();
    }
    return ESP_OK;
}

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
        s_card_present = false;
        ESP_LOGW(TAG, "microSD not found: %s", esp_err_to_name(err));
        return err;
    }
    s_card_present = true;
    return ESP_OK;
}

esp_err_t app_sd_start_session(void)
{
    if (!s_card_present) {
        return ESP_OK;
    }
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
    fprintf(s_mag, "t_ms,x_uT,y_uT,z_uT,rate_hz\n");

    snprintf(path, sizeof(path), "%s/fft.csv", s_dir);
    s_fft = fopen(path, "w");
    if (s_fft == nullptr) {
        ESP_LOGE(TAG, "open %s failed", path);
        fclose(s_mag);
        s_mag = nullptr;
        return ESP_FAIL;
    }
    fprintf(s_fft, "t_ms,bin,freq_hz,mx,my,mz,mabs\n");
    s_mag_burst_used = 0U;
    s_mag_burst_lines = 0U;
    s_samples = 0U;
    ESP_LOGI(TAG, "session %s open", s_dir);
    return ESP_OK;
}

esp_err_t app_sd_write_mag(uint32_t t_ms, float x_ut, float y_ut, float z_ut, float rate_hz)
{
    if (!s_card_present) {
        return ESP_OK;
    }
    if (s_mag == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    if (rate_hz > proto_cfg::kMagRate200Hz) {
        const esp_err_t err = writeMagBurst(t_ms, x_ut, y_ut, z_ut, rate_hz);
        if (err != ESP_OK) {
            return err;
        }
        s_samples++;
        return ESP_OK;
    }
    if (s_mag_burst_used > 0U) {
        const esp_err_t err = writeMagBurstFile();
        if (err != ESP_OK) {
            return err;
        }
    }
    const int wrote = fprintf(s_mag, "%lu,%.4f,%.4f,%.4f,%.4f\n",
                              static_cast<unsigned long>(t_ms),
                              static_cast<double>(x_ut),
                              static_cast<double>(y_ut),
                              static_cast<double>(z_ut),
                              static_cast<double>(rate_hz));
    if (wrote < 0) {
        return ESP_FAIL;
    }
    s_samples++;
    if ((s_samples % kMagCsvBurstLines) == 0U) {
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
    if (!s_card_present) {
        return ESP_OK;
    }
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

esp_err_t app_sd_sync(void)
{
    if (!s_card_present) {
        return ESP_OK;
    }
    if (s_mag == nullptr || s_fft == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    const esp_err_t burst_err = writeMagBurstFile();
    if (burst_err != ESP_OK) {
        return burst_err;
    }
    if (fflush(s_mag) != 0) {
        ESP_LOGW(TAG, "mag fflush failed");
        return ESP_FAIL;
    }
    if (fflush(s_fft) != 0) {
        ESP_LOGW(TAG, "fft fflush failed");
        return ESP_FAIL;
    }
    if (fsyncFile(s_mag) != ESP_OK) {
        ESP_LOGW(TAG, "mag fsync failed");
        return ESP_FAIL;
    }
    if (fsyncFile(s_fft) != ESP_OK) {
        ESP_LOGW(TAG, "fft fsync failed");
        return ESP_FAIL;
    }
    return ESP_OK;
}

void app_sd_stop_session(void)
{
    if (!s_card_present) {
        return;
    }
    if (s_mag != nullptr || s_fft != nullptr) {
        if (app_sd_sync() != ESP_OK) {
            ESP_LOGW(TAG, "sync before close failed");
        }
    }
    if (s_mag != nullptr) {
        fclose(s_mag);
        s_mag = nullptr;
    }
    s_mag_burst_used = 0U;
    s_mag_burst_lines = 0U;
    if (s_fft != nullptr) {
        fclose(s_fft);
        s_fft = nullptr;
    }
    if (s_dir[0] == '\0') {
        return;
    }
    ESP_LOGI(TAG, "session %s/mag.csv %s/fft.csv samples=%lu",
             s_dir, s_dir, static_cast<unsigned long>(s_samples));
}
