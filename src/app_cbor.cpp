/**
 * @file        app_cbor.cpp
 * @brief       Definite-length CBOR for logger commands and frames
 */

#include "app_cbor.hpp"

#include <string.h>

#include "esp_check.h"
#include "esp_log.h"

static const char* TAG = "APP_CBOR";

struct cbor_rd_t {
    const uint8_t* p;
    size_t n;
    size_t i;
};

static esp_err_t putByte(cbor_buf_t& buf, uint8_t value)
{
    if (buf.data == nullptr || buf.len >= buf.cap) {
        return ESP_ERR_NO_MEM;
    }
    buf.data[buf.len] = value;
    buf.len++;
    return ESP_OK;
}

static esp_err_t putType(cbor_buf_t& buf, uint8_t major, uint32_t value)
{
    const uint8_t prefix = static_cast<uint8_t>(major << 5U);
    if (value < 24U) {
        return putByte(buf, static_cast<uint8_t>(prefix | value));
    }
    if (value <= 0xFFU) {
        ESP_RETURN_ON_ERROR(putByte(buf, static_cast<uint8_t>(prefix | 24U)), TAG, "type");
        return putByte(buf, static_cast<uint8_t>(value));
    }
    if (value <= 0xFFFFU) {
        ESP_RETURN_ON_ERROR(putByte(buf, static_cast<uint8_t>(prefix | 25U)), TAG, "type");
        ESP_RETURN_ON_ERROR(putByte(buf, static_cast<uint8_t>(value >> 8U)), TAG, "type");
        return putByte(buf, static_cast<uint8_t>(value));
    }
    ESP_RETURN_ON_ERROR(putByte(buf, static_cast<uint8_t>(prefix | 26U)), TAG, "type");
    ESP_RETURN_ON_ERROR(putByte(buf, static_cast<uint8_t>(value >> 24U)), TAG, "type");
    ESP_RETURN_ON_ERROR(putByte(buf, static_cast<uint8_t>(value >> 16U)), TAG, "type");
    ESP_RETURN_ON_ERROR(putByte(buf, static_cast<uint8_t>(value >> 8U)), TAG, "type");
    return putByte(buf, static_cast<uint8_t>(value));
}

static esp_err_t putText(cbor_buf_t& buf, const char* text)
{
    const size_t len = strlen(text);
    ESP_RETURN_ON_ERROR(putType(buf, 3U, static_cast<uint32_t>(len)), TAG, "text");
    for (size_t i = 0; i < len; ++i) {
        ESP_RETURN_ON_ERROR(putByte(buf, static_cast<uint8_t>(text[i])), TAG, "text");
    }
    return ESP_OK;
}

static esp_err_t putU32(cbor_buf_t& buf, uint32_t value)
{
    return putType(buf, 0U, value);
}

static esp_err_t putBool(cbor_buf_t& buf, bool value)
{
    return putByte(buf, value ? 0xF5U : 0xF4U);
}

static esp_err_t putF64(cbor_buf_t& buf, double value)
{
    uint64_t bits = 0U;
    memcpy(&bits, &value, sizeof(bits));
    ESP_RETURN_ON_ERROR(putByte(buf, 0xFBU), TAG, "f64");
    for (int shift = 56; shift >= 0; shift -= 8) {
        ESP_RETURN_ON_ERROR(
            putByte(buf, static_cast<uint8_t>(bits >> shift)),
            TAG, "f64");
    }
    return ESP_OK;
}

static esp_err_t putF32Array(cbor_buf_t& buf, const float* values, size_t count)
{
    ESP_RETURN_ON_ERROR(putType(buf, 4U, static_cast<uint32_t>(count)), TAG, "array");
    for (size_t i = 0; i < count; ++i) {
        ESP_RETURN_ON_ERROR(putF64(buf, static_cast<double>(values[i])), TAG, "array");
    }
    return ESP_OK;
}

static esp_err_t putKey(cbor_buf_t& buf, const char* key)
{
    return putText(buf, key);
}

static esp_err_t pull(cbor_rd_t& rd, uint8_t& out)
{
    if (rd.i >= rd.n) {
        return ESP_ERR_INVALID_SIZE;
    }
    out = rd.p[rd.i];
    rd.i++;
    return ESP_OK;
}

static esp_err_t readArg(cbor_rd_t& rd, uint8_t ai, uint64_t& value)
{
    if (ai < 24U) {
        value = ai;
        return ESP_OK;
    }
    if (ai == 24U) {
        uint8_t b = 0U;
        ESP_RETURN_ON_ERROR(pull(rd, b), TAG, "arg");
        value = b;
        return ESP_OK;
    }
    if (ai == 25U) {
        uint8_t hi = 0U;
        uint8_t lo = 0U;
        ESP_RETURN_ON_ERROR(pull(rd, hi), TAG, "arg");
        ESP_RETURN_ON_ERROR(pull(rd, lo), TAG, "arg");
        value = (static_cast<uint64_t>(hi) << 8U) | lo;
        return ESP_OK;
    }
    if (ai == 26U) {
        uint64_t acc = 0U;
        for (int i = 0; i < 4; ++i) {
            uint8_t b = 0U;
            ESP_RETURN_ON_ERROR(pull(rd, b), TAG, "arg");
            acc = (acc << 8U) | b;
        }
        value = acc;
        return ESP_OK;
    }
    if (ai == 27U) {
        uint64_t acc = 0U;
        for (int i = 0; i < 8; ++i) {
            uint8_t b = 0U;
            ESP_RETURN_ON_ERROR(pull(rd, b), TAG, "arg");
            acc = (acc << 8U) | b;
        }
        value = acc;
        return ESP_OK;
    }
    return ESP_ERR_NOT_SUPPORTED;
}

static esp_err_t skipPayload(cbor_rd_t& rd, uint64_t len)
{
    if (len > (rd.n - rd.i)) {
        return ESP_ERR_INVALID_SIZE;
    }
    rd.i += static_cast<size_t>(len);
    return ESP_OK;
}

static esp_err_t skipItem(cbor_rd_t& rd);

static esp_err_t skipItem(cbor_rd_t& rd)
{
    uint8_t type = 0U;
    ESP_RETURN_ON_ERROR(pull(rd, type), TAG, "skip");
    const uint8_t major = static_cast<uint8_t>(type >> 5U);
    const uint8_t ai = static_cast<uint8_t>(type & 0x1FU);

    if (major == 7U) {
        if (ai < 24U) {
            return ESP_OK;
        }
        if (ai == 25U) {
            return skipPayload(rd, 2U);
        }
        if (ai == 26U) {
            return skipPayload(rd, 4U);
        }
        if (ai == 27U) {
            return skipPayload(rd, 8U);
        }
        return ESP_ERR_NOT_SUPPORTED;
    }

    uint64_t value = 0U;
    ESP_RETURN_ON_ERROR(readArg(rd, ai, value), TAG, "skip");
    if (major <= 1U) {
        return ESP_OK;
    }
    if (major == 2U || major == 3U) {
        return skipPayload(rd, value);
    }
    if (major == 4U) {
        for (uint64_t i = 0U; i < value; ++i) {
            ESP_RETURN_ON_ERROR(skipItem(rd), TAG, "skip array");
        }
        return ESP_OK;
    }
    if (major == 5U) {
        for (uint64_t i = 0U; i < value; ++i) {
            ESP_RETURN_ON_ERROR(skipItem(rd), TAG, "skip key");
            ESP_RETURN_ON_ERROR(skipItem(rd), TAG, "skip val");
        }
        return ESP_OK;
    }
    if (major == 6U) {
        return skipItem(rd);
    }
    return ESP_ERR_NOT_SUPPORTED;
}

static esp_err_t readTextAt(cbor_rd_t& rd, char* dst, size_t cap)
{
    uint8_t type = 0U;
    ESP_RETURN_ON_ERROR(pull(rd, type), TAG, "text");
    const uint8_t major = static_cast<uint8_t>(type >> 5U);
    const uint8_t ai = static_cast<uint8_t>(type & 0x1FU);
    if (major != 3U) {
        return ESP_ERR_INVALID_ARG;
    }
    uint64_t len = 0U;
    ESP_RETURN_ON_ERROR(readArg(rd, ai, len), TAG, "text");
    if (len >= cap) {
        return ESP_ERR_INVALID_SIZE;
    }
    ESP_RETURN_ON_ERROR(skipPayload(rd, len), TAG, "text");
    memcpy(dst, rd.p + rd.i - static_cast<size_t>(len), static_cast<size_t>(len));
    dst[len] = '\0';
    return ESP_OK;
}

static esp_err_t readNumber(cbor_rd_t& rd, double& out)
{
    uint8_t type = 0U;
    ESP_RETURN_ON_ERROR(pull(rd, type), TAG, "num");
    const uint8_t major = static_cast<uint8_t>(type >> 5U);
    const uint8_t ai = static_cast<uint8_t>(type & 0x1FU);
    if (major == 0U) {
        uint64_t value = 0U;
        ESP_RETURN_ON_ERROR(readArg(rd, ai, value), TAG, "num");
        out = static_cast<double>(value);
        return ESP_OK;
    }
    if (major != 7U) {
        return ESP_ERR_INVALID_ARG;
    }
    if (ai == 26U) {
        uint32_t bits = 0U;
        for (int i = 0; i < 4; ++i) {
            uint8_t b = 0U;
            ESP_RETURN_ON_ERROR(pull(rd, b), TAG, "f32");
            bits = (bits << 8U) | b;
        }
        float value = 0.0f;
        memcpy(&value, &bits, sizeof(value));
        out = static_cast<double>(value);
        return ESP_OK;
    }
    if (ai == 27U) {
        uint64_t bits = 0U;
        for (int i = 0; i < 8; ++i) {
            uint8_t b = 0U;
            ESP_RETURN_ON_ERROR(pull(rd, b), TAG, "f64");
            bits = (bits << 8U) | b;
        }
        double value = 0.0;
        memcpy(&value, &bits, sizeof(value));
        out = value;
        return ESP_OK;
    }
    return ESP_ERR_NOT_SUPPORTED;
}

static esp_err_t readSensorList(cbor_rd_t& rd, app_cmd_t& out)
{
    uint8_t type = 0U;
    ESP_RETURN_ON_ERROR(pull(rd, type), TAG, "sensors");
    const uint8_t major = static_cast<uint8_t>(type >> 5U);
    const uint8_t ai = static_cast<uint8_t>(type & 0x1FU);
    if (major != 4U) {
        return ESP_ERR_INVALID_ARG;
    }
    uint64_t count = 0U;
    ESP_RETURN_ON_ERROR(readArg(rd, ai, count), TAG, "sensors");
    out.list_present = true;
    out.has_mag = false;
    for (uint64_t i = 0U; i < count; ++i) {
        char name[16];
        ESP_RETURN_ON_ERROR(readTextAt(rd, name, sizeof(name)), TAG, "sensor name");
        if (strcmp(name, "mag") == 0) {
            out.has_mag = true;
        }
    }
    return ESP_OK;
}

static eAppCmd cmdFromText(const char* text)
{
    if (strcmp(text, "config") == 0) {
        return eAppCmd::eCmdConfig;
    }
    if (strcmp(text, "start") == 0) {
        return eAppCmd::eCmdStart;
    }
    if (strcmp(text, "stop") == 0) {
        return eAppCmd::eCmdStop;
    }
    return eAppCmd::eCmdNone;
}

esp_err_t app_cbor_decode_cmd(const uint8_t* data, size_t len, app_cmd_t& out)
{
    if (data == nullptr || len == 0U) {
        return ESP_ERR_INVALID_ARG;
    }
    out = {};
    cbor_rd_t rd = {data, len, 0U};
    uint8_t type = 0U;
    ESP_RETURN_ON_ERROR(pull(rd, type), TAG, "map");
    const uint8_t major = static_cast<uint8_t>(type >> 5U);
    const uint8_t ai = static_cast<uint8_t>(type & 0x1FU);
    if (major != 5U) {
        return ESP_ERR_INVALID_ARG;
    }
    uint64_t pairs = 0U;
    ESP_RETURN_ON_ERROR(readArg(rd, ai, pairs), TAG, "map");

    bool saw_cmd = false;
    for (uint64_t i = 0U; i < pairs; ++i) {
        char key[16];
        ESP_RETURN_ON_ERROR(readTextAt(rd, key, sizeof(key)), TAG, "key");
        if (strcmp(key, "cmd") == 0) {
            char verb[16];
            ESP_RETURN_ON_ERROR(readTextAt(rd, verb, sizeof(verb)), TAG, "cmd");
            out.kind = cmdFromText(verb);
            saw_cmd = out.kind != eAppCmd::eCmdNone;
            continue;
        }
        if (strcmp(key, "rate_hz") == 0) {
            double rate = 0.0;
            ESP_RETURN_ON_ERROR(readNumber(rd, rate), TAG, "rate");
            out.has_rate = true;
            out.rate_hz = static_cast<float>(rate);
            continue;
        }
        if (strcmp(key, "sensors") == 0) {
            ESP_RETURN_ON_ERROR(readSensorList(rd, out), TAG, "sensors");
            continue;
        }
        ESP_RETURN_ON_ERROR(skipItem(rd), TAG, "skip");
    }
    if (!saw_cmd) {
        return ESP_ERR_INVALID_ARG;
    }
    return ESP_OK;
}

esp_err_t app_cbor_encode_cfg(cbor_buf_t& buf, const app_cfg_rsp_t& rsp)
{
    buf.len = 0U;
    ESP_RETURN_ON_ERROR(putType(buf, 5U, 10U), TAG, "cfg map");
    ESP_RETURN_ON_ERROR(putKey(buf, "rsp"), TAG, "rsp");
    ESP_RETURN_ON_ERROR(putText(buf, "config"), TAG, "rsp");
    ESP_RETURN_ON_ERROR(putKey(buf, "ok"), TAG, "ok");
    ESP_RETURN_ON_ERROR(putBool(buf, rsp.ok), TAG, "ok");
    ESP_RETURN_ON_ERROR(putKey(buf, "sensor"), TAG, "sensor");
    ESP_RETURN_ON_ERROR(putText(buf, "mag"), TAG, "sensor");
    ESP_RETURN_ON_ERROR(putKey(buf, "addr7"), TAG, "addr");
    ESP_RETURN_ON_ERROR(putU32(buf, rsp.addr7), TAG, "addr");
    ESP_RETURN_ON_ERROR(putKey(buf, "chip_id"), TAG, "id");
    ESP_RETURN_ON_ERROR(putText(buf, rsp.chip_id), TAG, "id");
    ESP_RETURN_ON_ERROR(putKey(buf, "odr_hz"), TAG, "odr");
    ESP_RETURN_ON_ERROR(putF64(buf, static_cast<double>(rsp.odr_hz)), TAG, "odr");
    ESP_RETURN_ON_ERROR(putKey(buf, "gain_ga"), TAG, "gain");
    ESP_RETURN_ON_ERROR(putF64(buf, static_cast<double>(rsp.gain_ga)), TAG, "gain");
    ESP_RETURN_ON_ERROR(putKey(buf, "samples"), TAG, "samples");
    ESP_RETURN_ON_ERROR(putU32(buf, rsp.samples), TAG, "samples");
    ESP_RETURN_ON_ERROR(putKey(buf, "unit"), TAG, "unit");
    ESP_RETURN_ON_ERROR(putText(buf, "uT"), TAG, "unit");
    ESP_RETURN_ON_ERROR(putKey(buf, "fft_n"), TAG, "fft");
    return putU32(buf, rsp.fft_n);
}

esp_err_t app_cbor_encode_mag(cbor_buf_t& buf, const mag_sample_msg_t& sample)
{
    buf.len = 0U;
    ESP_RETURN_ON_ERROR(putType(buf, 5U, 6U), TAG, "mag map");
    ESP_RETURN_ON_ERROR(putKey(buf, "t_ms"), TAG, "t");
    ESP_RETURN_ON_ERROR(putU32(buf, sample.t_ms), TAG, "t");
    ESP_RETURN_ON_ERROR(putKey(buf, "sensor"), TAG, "sensor");
    ESP_RETURN_ON_ERROR(putText(buf, "mag"), TAG, "sensor");
    ESP_RETURN_ON_ERROR(putKey(buf, "unit"), TAG, "unit");
    ESP_RETURN_ON_ERROR(putText(buf, "uT"), TAG, "unit");
    ESP_RETURN_ON_ERROR(putKey(buf, "x"), TAG, "x");
    ESP_RETURN_ON_ERROR(putF64(buf, sample.x_ut), TAG, "x");
    ESP_RETURN_ON_ERROR(putKey(buf, "y"), TAG, "y");
    ESP_RETURN_ON_ERROR(putF64(buf, sample.y_ut), TAG, "y");
    ESP_RETURN_ON_ERROR(putKey(buf, "z"), TAG, "z");
    return putF64(buf, sample.z_ut);
}

esp_err_t app_cbor_encode_fft(cbor_buf_t& buf, const mag_fft_msg_t& fft)
{
    const size_t bins = proto_cfg::kMagFftBins;
    buf.len = 0U;
    ESP_RETURN_ON_ERROR(putType(buf, 5U, 10U), TAG, "fft map");
    ESP_RETURN_ON_ERROR(putKey(buf, "t_ms"), TAG, "t");
    ESP_RETURN_ON_ERROR(putU32(buf, fft.t_ms), TAG, "t");
    ESP_RETURN_ON_ERROR(putKey(buf, "sensor"), TAG, "sensor");
    ESP_RETURN_ON_ERROR(putText(buf, "mag_fft"), TAG, "sensor");
    ESP_RETURN_ON_ERROR(putKey(buf, "unit"), TAG, "unit");
    ESP_RETURN_ON_ERROR(putText(buf, "uT"), TAG, "unit");
    ESP_RETURN_ON_ERROR(putKey(buf, "rate_hz"), TAG, "rate");
    ESP_RETURN_ON_ERROR(putF64(buf, fft.rate_hz), TAG, "rate");
    ESP_RETURN_ON_ERROR(putKey(buf, "n"), TAG, "n");
    ESP_RETURN_ON_ERROR(putU32(buf, fft.n), TAG, "n");
    ESP_RETURN_ON_ERROR(putKey(buf, "df_hz"), TAG, "df");
    ESP_RETURN_ON_ERROR(putF64(buf, fft.df_hz), TAG, "df");
    ESP_RETURN_ON_ERROR(putKey(buf, "mx"), TAG, "mx");
    ESP_RETURN_ON_ERROR(putF32Array(buf, fft.mx, bins), TAG, "mx");
    ESP_RETURN_ON_ERROR(putKey(buf, "my"), TAG, "my");
    ESP_RETURN_ON_ERROR(putF32Array(buf, fft.my, bins), TAG, "my");
    ESP_RETURN_ON_ERROR(putKey(buf, "mz"), TAG, "mz");
    ESP_RETURN_ON_ERROR(putF32Array(buf, fft.mz, bins), TAG, "mz");
    ESP_RETURN_ON_ERROR(putKey(buf, "mabs"), TAG, "mabs");
    return putF32Array(buf, fft.mabs, bins);
}
