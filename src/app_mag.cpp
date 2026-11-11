/**
 * @file        app_mag.cpp
 * @brief       Sample the QMC5883P, log it, and queue frames
 */

#include "app_mag.hpp"

#include "app_sd_log.hpp"
#include "i2c_api.hpp"
#include "mag_fft.hpp"
#include "sensors/qmc5883p_api.hpp"

#include <math.h>
#include <stdio.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

static const char* TAG = "APP_MAG";

static constexpr UBaseType_t kSampleQueueLen = 32U;
static constexpr UBaseType_t kFftQueueLen = 2U;
static constexpr UBaseType_t kCmdQueueLen = 4U;

struct odr_row_t {
    float hz;
    qmc5883p_regs::Odr odr;
};

static const odr_row_t kOdrTable[] = {
    {10.0f, qmc5883p_regs::eQmcOdr10Hz},
    {50.0f, qmc5883p_regs::eQmcOdr50Hz},
    {100.0f, qmc5883p_regs::eQmcOdr100Hz},
    {200.0f, qmc5883p_regs::eQmcOdr200Hz},
};

static I2C s_bus(proto_cfg::makeMagI2cCfg());
static Qmc5883p s_mag(s_bus, proto_cfg::makeQmc5883pCfg());
static qmc5883p_cfg_t s_cfg = proto_cfg::makeQmc5883pCfg();

static QueueHandle_t s_sample_q = nullptr;
static QueueHandle_t s_fft_q = nullptr;
static QueueHandle_t s_req_q = nullptr;
static QueueHandle_t s_rsp_q = nullptr;

static bool s_running = false;
static bool s_sensor_ok = false;
static uint16_t s_filled = 0U;
static float s_win_x[proto_cfg::kMagFftN];
static float s_win_y[proto_cfg::kMagFftN];
static float s_win_z[proto_cfg::kMagFftN];
static float s_win_abs[proto_cfg::kMagFftN];

static float odrHz(qmc5883p_regs::Odr odr)
{
    for (size_t i = 0U; i < (sizeof(kOdrTable) / sizeof(kOdrTable[0])); ++i) {
        if (kOdrTable[i].odr == odr) {
            return kOdrTable[i].hz;
        }
    }
    return 200.0f;
}

static qmc5883p_regs::Odr nearestOdr(float hz)
{
    size_t best = 0U;
    float best_delta = fabsf(hz - kOdrTable[0].hz);
    for (size_t i = 1U; i < (sizeof(kOdrTable) / sizeof(kOdrTable[0])); ++i) {
        const float delta = fabsf(hz - kOdrTable[i].hz);
        if (delta < best_delta) {
            best = i;
            best_delta = delta;
        }
    }
    return kOdrTable[best].odr;
}

static uint32_t timeoutForHz(float hz)
{
    if (hz < 0.5f) {
        hz = 0.5f;
    }
    float timeout = (1000.0f / hz) * 4.0f;
    if (timeout < static_cast<float>(proto_cfg::kQmcPollTimeoutMs)) {
        timeout = static_cast<float>(proto_cfg::kQmcPollTimeoutMs);
    }
    if (timeout > 4000.0f) {
        timeout = 4000.0f;
    }
    return static_cast<uint32_t>(timeout);
}

static float gainGa(qmc5883p_regs::Range range)
{
    switch (range) {
    case qmc5883p_regs::eQmcRange30Ga:
        return 30.0f;
    case qmc5883p_regs::eQmcRange12Ga:
        return 12.0f;
    case qmc5883p_regs::eQmcRange8Ga:
        return 8.0f;
    case qmc5883p_regs::eQmcRange2Ga:
        return 2.0f;
    default:
        return 8.0f;
    }
}

static uint8_t osr1Count(qmc5883p_regs::Osr1 osr1)
{
    switch (osr1) {
    case qmc5883p_regs::eQmcOsr1_8:
        return 8U;
    case qmc5883p_regs::eQmcOsr1_4:
        return 4U;
    case qmc5883p_regs::eQmcOsr1_2:
        return 2U;
    case qmc5883p_regs::eQmcOsr1_1:
        return 1U;
    default:
        return 8U;
    }
}

static void fillRsp(app_cfg_rsp_t& rsp, bool ok)
{
    rsp = {};
    rsp.ok = ok;
    rsp.addr7 = s_cfg.addr7;
    rsp.chip_id[0] = '8';
    rsp.chip_id[1] = '0';
    rsp.chip_id[2] = '\0';
    rsp.odr_hz = odrHz(s_cfg.odr);
    rsp.gain_ga = gainGa(s_cfg.range);
    rsp.samples = osr1Count(s_cfg.osr1);
    rsp.fft_n = proto_cfg::kMagFftN;
}

static void postSample(const mag_sample_msg_t& msg)
{
    if (xQueueSend(s_sample_q, &msg, 0) == pdTRUE) {
        return;
    }
    mag_sample_msg_t drop = {};
    (void)xQueueReceive(s_sample_q, &drop, 0);
    (void)xQueueSend(s_sample_q, &msg, 0);
}

static void postFft(const mag_fft_msg_t& msg)
{
    if (xQueueSend(s_fft_q, &msg, 0) == pdTRUE) {
        return;
    }
    mag_fft_msg_t drop = {};
    (void)xQueueReceive(s_fft_q, &drop, 0);
    (void)xQueueSend(s_fft_q, &msg, 0);
}

static void emitFft(uint32_t t_ms)
{
    const float rate = odrHz(s_cfg.odr);
    mag_fft_msg_t msg = {};
    msg.t_ms = t_ms;
    msg.rate_hz = rate;
    msg.n = proto_cfg::kMagFftN;
    msg.df_hz = rate / static_cast<float>(proto_cfg::kMagFftN);

    if (mag_fft_magnitude(s_win_x, proto_cfg::kMagFftN, msg.mx) != ESP_OK) {
        ESP_LOGW(TAG, "FFT X failed");
        return;
    }
    if (mag_fft_magnitude(s_win_y, proto_cfg::kMagFftN, msg.my) != ESP_OK) {
        ESP_LOGW(TAG, "FFT Y failed");
        return;
    }
    if (mag_fft_magnitude(s_win_z, proto_cfg::kMagFftN, msg.mz) != ESP_OK) {
        ESP_LOGW(TAG, "FFT Z failed");
        return;
    }
    if (mag_fft_magnitude(s_win_abs, proto_cfg::kMagFftN, msg.mabs) != ESP_OK) {
        ESP_LOGW(TAG, "FFT abs failed");
        return;
    }

    (void)app_sd_write_fft(t_ms, rate, msg.mx, msg.my, msg.mz, msg.mabs,
                           proto_cfg::kMagFftBins);
    postFft(msg);
}

static void applyCmd(const app_cmd_t& cmd, app_cfg_rsp_t& rsp)
{
    if (!s_sensor_ok && cmd.kind != eAppCmd::eCmdStop) {
        fillRsp(rsp, false);
        return;
    }
    if (cmd.kind == eAppCmd::eCmdStart) {
        if (s_running) {
            fillRsp(rsp, true);
            return;
        }
        if (app_sd_start_session() != ESP_OK) {
            fillRsp(rsp, false);
            return;
        }
        s_filled = 0U;
        s_running = true;
        fillRsp(rsp, true);
        return;
    }

    if (cmd.kind == eAppCmd::eCmdStop) {
        s_running = false;
        app_sd_stop_session();
        fillRsp(rsp, true);
        return;
    }

    if (cmd.kind != eAppCmd::eCmdConfig) {
        fillRsp(rsp, false);
        return;
    }
    if (cmd.list_present && !cmd.has_mag) {
        fillRsp(rsp, false);
        return;
    }
    if (!cmd.has_rate) {
        fillRsp(rsp, true);
        return;
    }

    qmc5883p_cfg_t next = s_cfg;
    next.odr = nearestOdr(cmd.rate_hz);
    next.poll_timeout_ms = timeoutForHz(odrHz(next.odr));
    if (s_mag.reconfigure(next) != ESP_OK) {
        fillRsp(rsp, false);
        return;
    }
    s_cfg = next;
    s_filled = 0U;
    fillRsp(rsp, true);
}

static void replyCmd(const app_cmd_t& cmd)
{
    app_cfg_rsp_t rsp = {};
    applyCmd(cmd, rsp);
    (void)xQueueSend(s_rsp_q, &rsp, portMAX_DELAY);
}

static void serviceNow(void)
{
    app_cmd_t cmd = {};
    while (xQueueReceive(s_req_q, &cmd, 0) == pdTRUE) {
        replyCmd(cmd);
    }
}

static void pushWindow(uint32_t t_ms, float x_ut, float y_ut, float z_ut)
{
    if (s_filled >= proto_cfg::kMagFftN) {
        s_filled = 0U;
    }
    s_win_x[s_filled] = x_ut;
    s_win_y[s_filled] = y_ut;
    s_win_z[s_filled] = z_ut;
    s_win_abs[s_filled] = sqrtf((x_ut * x_ut) + (y_ut * y_ut) + (z_ut * z_ut));
    s_filled++;
    if (s_filled < proto_cfg::kMagFftN) {
        return;
    }
    emitFft(t_ms);
    s_filled = 0U;
}

static void readOnce(void)
{
    esp_task_wdt_reset();
    qmc5883p_gauss_t gauss = {};
    const esp_err_t err = s_mag.readGauss(gauss);
    esp_task_wdt_reset();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "readGauss failed: %s", esp_err_to_name(err));
        return;
    }

    const uint32_t t_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000LL);
    const float x_ut = gauss.x * proto_cfg::kGaussToUt;
    const float y_ut = gauss.y * proto_cfg::kGaussToUt;
    const float z_ut = gauss.z * proto_cfg::kGaussToUt;

    if (app_sd_write_mag(t_ms, x_ut, y_ut, z_ut) != ESP_OK) {
        ESP_LOGW(TAG, "mag.csv write failed");
    }

    mag_sample_msg_t msg = {};
    msg.t_ms = t_ms;
    msg.x_ut = x_ut;
    msg.y_ut = y_ut;
    msg.z_ut = z_ut;
    postSample(msg);
    pushWindow(t_ms, x_ut, y_ut, z_ut);
}

static void sampleTask(void* arg)
{
    (void)arg;
    const esp_err_t wdt_err = esp_task_wdt_add(nullptr);
    if (wdt_err != ESP_OK && wdt_err != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "wdt add: %s", esp_err_to_name(wdt_err));
    }

    while (true) {
        if (!s_running) {
            app_cmd_t cmd = {};
            if (xQueueReceive(s_req_q, &cmd, pdMS_TO_TICKS(200)) == pdTRUE) {
                replyCmd(cmd);
            }
            esp_task_wdt_reset();
            continue;
        }
        serviceNow();
        if (!s_running) {
            continue;
        }
        readOnce();
    }
}

static esp_err_t makeQueues(void)
{
    s_sample_q = xQueueCreate(kSampleQueueLen, sizeof(mag_sample_msg_t));
    s_fft_q = xQueueCreate(kFftQueueLen, sizeof(mag_fft_msg_t));
    s_req_q = xQueueCreate(kCmdQueueLen, sizeof(app_cmd_t));
    s_rsp_q = xQueueCreate(2U, sizeof(app_cfg_rsp_t));
    if (s_sample_q == nullptr || s_fft_q == nullptr ||
        s_req_q == nullptr || s_rsp_q == nullptr) {
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

static void logI2cScan(void)
{
    char line[160];
    size_t used = 0U;
    line[0] = '\0';

    for (uint8_t addr = 0x08U; addr < 0x78U; addr++) {
        if (s_bus.probe(addr, 20U) != ESP_OK) {
            continue;
        }
        const int wrote = snprintf(line + used, sizeof(line) - used, " 0x%02X", addr);
        if (wrote < 0 || (used + static_cast<size_t>(wrote)) >= sizeof(line)) {
            break;
        }
        used += static_cast<size_t>(wrote);
    }

    const i2c_cfg_t& cfg = s_bus.getConfig();
    if (used == 0U) {
        const int sda_level = gpio_get_level(static_cast<gpio_num_t>(cfg.sda_pin));
        const int scl_level = gpio_get_level(static_cast<gpio_num_t>(cfg.scl_pin));
        ESP_LOGE(TAG, "I2C scan: no ACK on SDA %d SCL %d levels SDA=%d SCL=%d",
                 (int)cfg.sda_pin, (int)cfg.scl_pin, sda_level, scl_level);
        return;
    }
    ESP_LOGE(TAG, "I2C scan ACK:%s", line);
}

static esp_err_t startSampleTask(void)
{
    const esp_err_t q_err = makeQueues();
    if (q_err != ESP_OK) {
        return q_err;
    }
    if (xTaskCreate(sampleTask, "mag_sample", 12288, nullptr, 6, nullptr) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

esp_err_t app_mag_init(void)
{
    esp_err_t sensor_err = ESP_OK;
    const esp_err_t bus_err = s_bus.init();
    if (bus_err != ESP_OK) {
        ESP_LOGE(TAG, "I2C init failed: %s", esp_err_to_name(bus_err));
        sensor_err = bus_err;
    } else {
        const esp_err_t mag_err = s_mag.init();
        if (mag_err != ESP_OK) {
            ESP_LOGE(TAG, "QMC5883P init failed: %s", esp_err_to_name(mag_err));
            logI2cScan();
            sensor_err = mag_err;
        }
    }
    if (sensor_err == ESP_OK) {
        const esp_err_t fft_err = mag_fft_init();
        if (fft_err != ESP_OK) {
            sensor_err = fft_err;
        }
    }

    s_sensor_ok = (sensor_err == ESP_OK);
    const esp_err_t task_err = startSampleTask();
    if (task_err != ESP_OK) {
        return task_err;
    }
    return sensor_err;
}

esp_err_t app_mag_request(const app_cmd_t& cmd, app_cfg_rsp_t& rsp)
{
    if (s_req_q == nullptr || s_rsp_q == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    app_cfg_rsp_t stale = {};
    while (xQueueReceive(s_rsp_q, &stale, 0) == pdTRUE) {
    }
    if (xQueueSend(s_req_q, &cmd, pdMS_TO_TICKS(500)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    const TickType_t start = xTaskGetTickCount();
    const TickType_t limit = pdMS_TO_TICKS(4000);
    while ((xTaskGetTickCount() - start) < limit) {
        if (xQueueReceive(s_rsp_q, &rsp, pdMS_TO_TICKS(200)) == pdTRUE) {
            return ESP_OK;
        }
        esp_task_wdt_reset();
    }
    return ESP_ERR_TIMEOUT;
}

esp_err_t app_mag_pop_sample(mag_sample_msg_t& out)
{
    if (s_sample_q == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xQueueReceive(s_sample_q, &out, 0) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

esp_err_t app_mag_pop_fft(mag_fft_msg_t& out)
{
    if (s_fft_q == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xQueueReceive(s_fft_q, &out, 0) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}
