/**
 * @file        config.hpp
 * @brief       Tunable parameters for proto_esp32_s3 firmware
 * @author      Mat RB
 * @date        2026-08-19
 *
 * @details
 *   Board pins from proto_esp32_s3/docs/pinout_gpio.json.
 *   Adjust plot and sensor settings here without editing main.cpp.
 */

#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <stdint.h>

#include "display/display_api.hpp"
#include "display/layout/screen_geom.hpp"
#include "i2c_api.hpp"
#include "uart_api.hpp"
#include "ble_api.hpp"
#include "sensors/hmc5983_api.hpp"
#include "sensors/amg8833_api.hpp"
#include "sensors/bmi160_api.hpp"
#include "sensors/l76k_api.hpp"

namespace proto_cfg {

/* --- BLE bring-up (default peripheral; set BLE_TEST_ROLE=central to flip) --- */
#ifndef BLE_TEST_ROLE_CENTRAL
#define BLE_TEST_ROLE_CENTRAL 0
#endif

static constexpr const char* kBleDeviceName = kBleDefaultDeviceName;
static constexpr const char* kBlePeerNameFilter = kBleDefaultDeviceName;
static constexpr uint32_t kBleHeartbeatMs = 2000U;
static constexpr uint32_t kBleScanMs = 15000U;
static constexpr uint32_t kBleCentralWriteMs = 3000U;
static constexpr uint32_t kBleCentralRescanMs = 5000U;

static inline ble_cfg_t makeBleCfg(void)
{
    ble_cfg_t cfg = {};
    cfg.device_name = kBleDeviceName;
    cfg.auto_advertise = true;
    cfg.scan_ms = kBleScanMs;
    cfg.peer_name_filter = kBlePeerNameFilter;
    return cfg;
}


/* --- Display / SPI (pinout_gpio.json) --- */
static constexpr int8_t kPinMosi = 35;
static constexpr int8_t kPinSclk = 36;
static constexpr int8_t kPinDc   = 40;
static constexpr int8_t kPinRst  = 41;
static constexpr int8_t kPinBl   = 42;
static constexpr uint32_t kSpiClockHz = 40000000U;

/** Settle after SoC boot before ST7789 init (panel shares SoC power). */
static constexpr uint32_t kPanelPowerSettleMs = 150U;

/* --- Buttons --- */
static constexpr uint8_t kBtnAdvance = 5U;
static constexpr uint8_t kBtnAccept  = 6U;
static constexpr uint8_t kBtnDebounceMs = 20U;
static constexpr uint32_t kMainLoopPeriodMs = 5U;

/* --- UART / L76K GNSS (pinout_gpio.json uart, MCU pins) --- */
static constexpr int8_t  kPinGnssTx  = 2;
static constexpr int8_t  kPinGnssRx  = 1;
static constexpr int8_t  kPinGnssPps = 42;
static constexpr uint32_t kGnssBaudHz = 9600U;
static constexpr uint32_t kGnssPollMs = 2500U;

static inline uart_cfg_t makeGnssUartCfg(void)
{
    uart_cfg_t cfg = {};
    cfg.port         = eUartPort1;
    cfg.tx_pin       = kPinGnssTx;
    cfg.rx_pin       = kPinGnssRx;
    cfg.baud_hz      = kGnssBaudHz;
    cfg.rx_buf_bytes = 2048;
    cfg.tx_buf_bytes = 512;
    return cfg;
}

static inline l76k_cfg_t makeL76kCfg(void)
{
    l76k_cfg_t cfg = {};
    cfg.pps_pin = kPinGnssPps;
    /* Module streams NMEA by default; PMTK cfg can break Quectel LC76G. */
    cfg.apply_nmea_cfg = false;
    return cfg;
}

/* --- I2C2 / BMI160 (pinout_gpio.json i2c2; order confirmed by bring-up) --- */
static constexpr int8_t  kPinImuSdaA = 9;
static constexpr int8_t  kPinImuSclA = 11;
static constexpr int8_t  kPinImuSdaB = 11;
static constexpr int8_t  kPinImuSclB = 9;
static constexpr uint32_t kI2c2ClkHz = 400000U;
static constexpr uint32_t kImuPollMs = 200U;

/* Confirmed bring-up: SDA=9 SCL=11 addr7=0x69 (SDO high). */
static constexpr int8_t  kPinImuSda = kPinImuSdaA;
static constexpr int8_t  kPinImuScl = kPinImuSclA;
static constexpr uint8_t kImuAddr7  = bmi160_regs::kAddr7SdoHigh;

static inline i2c_cfg_t makeImuI2cCfg(int8_t sda, int8_t scl)
{
    i2c_cfg_t cfg = {};
    cfg.port    = eI2cPort1;
    cfg.sda_pin = sda;
    cfg.scl_pin = scl;
    cfg.clk_hz  = kI2c2ClkHz;
    return cfg;
}

static inline bmi160_cfg_t makeBmi160Cfg(uint8_t addr7)
{
    bmi160_cfg_t cfg = {};
    cfg.addr7 = addr7;
    return cfg;
}

/* --- I2C / HMC5983 + AMG8833 (pinout_gpio.json i2c) --- */
static constexpr int8_t kPinI2cSda = 37;
static constexpr int8_t kPinI2cScl = 36;
static constexpr uint32_t kI2cClkHz = 400000U;

static inline i2c_cfg_t makeI2cCfg(void)
{
    i2c_cfg_t cfg = {};
    cfg.port    = eI2cPort0;
    cfg.sda_pin = kPinI2cSda;
    cfg.scl_pin = kPinI2cScl;
    cfg.clk_hz  = kI2cClkHz;
    return cfg;
}

static inline hmc5983_cfg_t makeHmc5983Cfg(void)
{
    hmc5983_cfg_t cfg = {};
    cfg.mode = hmc5983_regs::eHmc5983ModeContinuous;
    cfg.odr  = hmc5983_regs::eHmc5983Odr220Hz;
    cfg.samples = hmc5983_regs::eHmc5983Samples1;
    cfg.gain    = hmc5983_regs::eHmc5983Gain13Ga;
    cfg.temp_sensor = true;
    cfg.meas = hmc5983_regs::eHmc5983MeasNormal;
    return cfg;
}

static inline amg8833_cfg_t makeAmgCfg(void)
{
    amg8833_cfg_t cfg = {};
    cfg.addr7 = amg8833_regs::kAddr7;
    cfg.fps   = amg8833_regs::eAmgFps10Hz;
    cfg.moving_average = true;
    return cfg;
}

/* --- Magnetometer plot --- */
static constexpr uint32_t kMagPlotWindowMs    = 1000U;
/** ~220 Hz poll to match HMC5983 max ODR. */
static constexpr uint32_t kMagSamplePeriodMs  = 4U;
static constexpr float    kMagSampleRateHz =
    1000.0f / static_cast<float>(kMagSamplePeriodMs);
static constexpr float    kMagScaleMargin     = 1.25f;
static constexpr float    kMagMinHalfSpanGauss = 0.05f;
static constexpr uint8_t  kMagPlotPadPx       = 2U;

/** Power-of-two FFT length for Hz magnitude spectrum. */
static constexpr uint16_t kMagFftN = 128U;
static constexpr uint16_t kMagFftBins = kMagFftN / 2U;

static constexpr uint16_t kMagColorX = kColorRed;
static constexpr uint16_t kMagColorY = kColorGreen;
static constexpr uint16_t kMagColorZ = kColorBlue;
static constexpr uint16_t kMagColorAbs = kColorYellow;
static constexpr uint16_t kMagPlotBgColor = kColorBlack;
static constexpr uint16_t kMagPlotMeanLineColor = kColorDkGray;

static constexpr uint16_t kMagPlotX0 = 40U;
static constexpr uint16_t kMagPlotY0 = 0U;
static constexpr uint16_t kMagPlotX1 = display_layout::kScreenW - 1U;
static constexpr uint16_t kMagPlotY1 =
    static_cast<uint16_t>(display_layout::kBarY - 1U);

/** Ring must hold FFT history (and at least ~1 s for TIME window). */
static constexpr uint16_t kMagMaxSamples = kMagFftN;

static_assert(kMagMaxSamples >= kMagFftN, "mag ring smaller than FFT N");
static_assert(kMagMaxSamples <= 256U, "mag ring buffer too large");

/* --- AMG8833 IR heatmap --- */
/** Match AMG8833 10 Hz frame rate. */
static constexpr uint32_t kIrSamplePeriodMs = 100U;
static constexpr uint8_t  kIrGridSize      = 8U;

/** Status text band above heatmap (blit must not overwrite). */
static constexpr uint16_t kIrStatusBandH = 16U;
static constexpr uint16_t kIrHeatX0 = 0U;
static constexpr uint16_t kIrHeatY0 = kIrStatusBandH;
static constexpr uint16_t kIrHeatX1 = display_layout::kScreenW - 1U;
static constexpr uint16_t kIrHeatY1 =
    static_cast<uint16_t>(display_layout::kBarY - 1U);

/** Autoscale: EMA + min span + thermistor-relative window. */
static constexpr float kIrScaleAlpha      = 0.15f;
static constexpr float kIrMinSpanC        = 8.0f;
static constexpr float kIrBelowAmbientC   = 2.0f;
static constexpr float kIrAboveAmbientC   = 10.0f;

/** Top labels: 2 Hz refresh; redraw only when string changes. */
static constexpr uint32_t kIrLabelPeriodMs = 500U;
static constexpr float    kIrLabelAlpha   = 0.25f;

/** Unsharp amount for levels 0..3 (applied on 8x8 before upsample). */
static constexpr float kIrSharpenAmount[4] = {0.0f, 0.6f, 1.2f, 1.8f};

static constexpr uint16_t kIrStatusFgColor = kColorWhite;
static constexpr uint16_t kIrStatusBgColor = kColorBlack;

} /* namespace proto_cfg */

#endif /* CONFIG_HPP */
