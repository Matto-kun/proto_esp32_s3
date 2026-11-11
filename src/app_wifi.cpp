/**
 * @file        app_wifi.cpp
 * @brief       SoftAP TCP server. One client. Length-prefixed CBOR.
 */

#include "app_wifi.hpp"

#include "app_cbor.hpp"
#include "app_mag.hpp"
#include "config.hpp"

#include <errno.h>
#include <string.h>

#include "esp_check.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_task_wdt.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "nvs_flash.h"

static const char* TAG = "APP_WIFI";
static constexpr size_t kMaxCmd = 256U;
static constexpr size_t kTxCap = 3072U;

static uint8_t s_rx[kMaxCmd];
static uint8_t s_tx_mem[kTxCap];

static esp_err_t sendAll(int fd, const uint8_t* data, size_t len)
{
    size_t sent = 0U;
    while (sent < len) {
        const int rc = send(fd, data + sent, len - sent, 0);
        if (rc < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                esp_task_wdt_reset();
                continue;
            }
            return ESP_FAIL;
        }
        if (rc == 0) {
            return ESP_FAIL;
        }
        sent += static_cast<size_t>(rc);
    }
    return ESP_OK;
}

static esp_err_t sendFrame(int fd, const cbor_buf_t& buf)
{
    uint8_t hdr[4];
    const uint32_t len = static_cast<uint32_t>(buf.len);
    hdr[0] = static_cast<uint8_t>(len >> 24U);
    hdr[1] = static_cast<uint8_t>(len >> 16U);
    hdr[2] = static_cast<uint8_t>(len >> 8U);
    hdr[3] = static_cast<uint8_t>(len);
    ESP_RETURN_ON_ERROR(sendAll(fd, hdr, sizeof(hdr)), TAG, "hdr");
    return sendAll(fd, buf.data, buf.len);
}

static esp_err_t drainQueues(int fd)
{
    cbor_buf_t buf = {s_tx_mem, kTxCap, 0U};
    mag_sample_msg_t sample = {};
    while (app_mag_pop_sample(sample) == ESP_OK) {
        if (app_cbor_encode_mag(buf, sample) != ESP_OK) {
            return ESP_FAIL;
        }
        if (sendFrame(fd, buf) != ESP_OK) {
            return ESP_FAIL;
        }
    }
    mag_fft_msg_t fft = {};
    while (app_mag_pop_fft(fft) == ESP_OK) {
        if (app_cbor_encode_fft(buf, fft) != ESP_OK) {
            return ESP_FAIL;
        }
        if (sendFrame(fd, buf) != ESP_OK) {
            return ESP_FAIL;
        }
    }
    return ESP_OK;
}

static esp_err_t readExact(int fd, uint8_t* dst, size_t len)
{
    size_t got = 0U;
    while (got < len) {
        const int rc = recv(fd, dst + got, len - got, 0);
        if (rc < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                esp_task_wdt_reset();
                continue;
            }
            return ESP_FAIL;
        }
        if (rc == 0) {
            return ESP_FAIL;
        }
        got += static_cast<size_t>(rc);
    }
    return ESP_OK;
}

static void handlePayload(int fd, const uint8_t* data, size_t len)
{
    app_cmd_t cmd = {};
    if (app_cbor_decode_cmd(data, len, cmd) != ESP_OK) {
        ESP_LOGW(TAG, "bad command");
        return;
    }
    app_cfg_rsp_t rsp = {};
    if (app_mag_request(cmd, rsp) != ESP_OK) {
        ESP_LOGW(TAG, "command timeout");
        return;
    }
    if (cmd.kind != eAppCmd::eCmdConfig) {
        return;
    }
    cbor_buf_t buf = {s_tx_mem, kTxCap, 0U};
    if (app_cbor_encode_cfg(buf, rsp) != ESP_OK) {
        return;
    }
    (void)sendFrame(fd, buf);
}

static void serveClient(int fd)
{
    timeval rcv = {};
    rcv.tv_sec = 1;
    rcv.tv_usec = 0;
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &rcv, sizeof(rcv));
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &rcv, sizeof(rcv));

    while (true) {
        esp_task_wdt_reset();
        if (drainQueues(fd) != ESP_OK) {
            break;
        }
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);
        timeval tv = {};
        tv.tv_sec = 0;
        tv.tv_usec = 200000;
        const int sel = select(fd + 1, &rfds, nullptr, nullptr, &tv);
        if (sel < 0) {
            if (errno == EINTR) {
                continue;
            }
            break;
        }
        if (sel == 0) {
            continue;
        }
        uint8_t hdr[4];
        if (readExact(fd, hdr, sizeof(hdr)) != ESP_OK) {
            break;
        }
        const uint32_t len = (static_cast<uint32_t>(hdr[0]) << 24U) |
                             (static_cast<uint32_t>(hdr[1]) << 16U) |
                             (static_cast<uint32_t>(hdr[2]) << 8U) |
                             static_cast<uint32_t>(hdr[3]);
        if (len == 0U || len > kMaxCmd) {
            ESP_LOGW(TAG, "bad frame len %lu", static_cast<unsigned long>(len));
            break;
        }
        if (readExact(fd, s_rx, len) != ESP_OK) {
            break;
        }
        handlePayload(fd, s_rx, len);
    }
}

static void parkTask(void)
{
    while (true) {
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

static void tcpTask(void* arg)
{
    (void)arg;
    const esp_err_t wdt_err = esp_task_wdt_add(nullptr);
    if (wdt_err != ESP_OK && wdt_err != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "wdt add: %s", esp_err_to_name(wdt_err));
    }

    const int listen_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (listen_fd < 0) {
        ESP_LOGE(TAG, "TCP listen FAILED");
        parkTask();
    }
    int yes = 1;
    setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

    sockaddr_in addr = {};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(proto_cfg::kWifiTcpPort);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(listen_fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
        ESP_LOGE(TAG, "TCP listen FAILED");
        parkTask();
    }
    if (listen(listen_fd, 1) != 0) {
        ESP_LOGE(TAG, "TCP listen FAILED");
        parkTask();
    }
    ESP_LOGI(TAG, "TCP listen on port %u",
             static_cast<unsigned>(proto_cfg::kWifiTcpPort));

    while (true) {
        esp_task_wdt_reset();
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(listen_fd, &rfds);
        timeval tv = {};
        tv.tv_sec = 1;
        tv.tv_usec = 0;
        const int sel = select(listen_fd + 1, &rfds, nullptr, nullptr, &tv);
        if (sel <= 0) {
            continue;
        }
        const int client = accept(listen_fd, nullptr, nullptr);
        if (client < 0) {
            continue;
        }
        ESP_LOGI(TAG, "TCP client connected");
        serveClient(client);
        close(client);
        ESP_LOGI(TAG, "TCP client closed");
    }
}

static esp_err_t startRadio(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_RETURN_ON_ERROR(nvs_flash_erase(), TAG, "nvs erase");
        err = nvs_flash_init();
    }
    ESP_RETURN_ON_ERROR(err, TAG, "nvs init");
    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "netif");
    ESP_RETURN_ON_ERROR(esp_event_loop_create_default(), TAG, "event loop");
    if (esp_netif_create_default_wifi_ap() == nullptr) {
        return ESP_FAIL;
    }

    wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&wifi_cfg), TAG, "wifi init");
    ESP_RETURN_ON_ERROR(esp_wifi_set_storage(WIFI_STORAGE_RAM), TAG, "wifi storage");
    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_AP), TAG, "wifi mode");

    const size_t ssid_len = strlen(proto_cfg::kWifiApSsid);
    const size_t pass_len = strlen(proto_cfg::kWifiApPass);
    if (ssid_len == 0U || ssid_len > 32U || pass_len < 8U || pass_len > 63U) {
        return ESP_ERR_INVALID_ARG;
    }

    wifi_config_t ap = {};
    memcpy(ap.ap.ssid, proto_cfg::kWifiApSsid, ssid_len);
    ap.ap.ssid_len = static_cast<uint8_t>(ssid_len);
    memcpy(ap.ap.password, proto_cfg::kWifiApPass, pass_len);
    ap.ap.channel = proto_cfg::kWifiApChannel;
    ap.ap.max_connection = proto_cfg::kWifiApMaxClients;
    ap.ap.authmode = WIFI_AUTH_WPA2_PSK;
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_AP, &ap), TAG, "wifi config");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "wifi start");
    ESP_LOGI(TAG, "SoftAP %s up", proto_cfg::kWifiApSsid);
    return ESP_OK;
}

esp_err_t app_wifi_start(void)
{
    ESP_RETURN_ON_ERROR(startRadio(), TAG, "radio");
    if (xTaskCreate(tcpTask, "mag_tcp", 8192, nullptr, 5, nullptr) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}
