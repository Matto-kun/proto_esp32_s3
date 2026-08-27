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
#include "sensors/hmc5883l_api.hpp"

namespace proto_cfg {

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
static constexpr uint32_t kMainLoopPeriodMs = 20U;

/* --- I2C / HMC5883L (pinout_gpio.json imu) --- */
static constexpr int8_t kPinI2cSda = 9;
static constexpr int8_t kPinI2cScl = 11;
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

static inline hmc5883l_cfg_t makeHmcCfg(void)
{
    hmc5883l_cfg_t cfg = {};
    cfg.mode = hmc5883l_regs::eHmcModeContinuous;
    cfg.odr  = hmc5883l_regs::eHmcOdr15Hz;
    cfg.samples = hmc5883l_regs::eHmcSamples8;
    cfg.gain    = hmc5883l_regs::eHmcGain13Ga;
    return cfg;
}

/* --- Magnetometer plot --- */
static constexpr uint32_t kMagPlotWindowMs    = 1000U;
static constexpr uint32_t kMagSamplePeriodMs  = 50U;
static constexpr float    kMagScaleMargin     = 1.25f;
static constexpr float    kMagMinHalfSpanGauss = 0.05f;
static constexpr uint8_t  kMagPlotPadPx       = 2U;

static constexpr uint16_t kMagColorX = kColorRed;
static constexpr uint16_t kMagColorY = kColorGreen;
static constexpr uint16_t kMagColorZ = kColorBlue;
static constexpr uint16_t kMagPlotBgColor = kColorBlack;
static constexpr uint16_t kMagPlotMeanLineColor = kColorDkGray;

static constexpr uint16_t kMagPlotX0 = 0U;
static constexpr uint16_t kMagPlotY0 = 0U;
static constexpr uint16_t kMagPlotX1 = display_layout::kScreenW - 1U;
static constexpr uint16_t kMagPlotY1 =
    static_cast<uint16_t>(display_layout::kBarY - 1U);

static constexpr uint16_t kMagMaxSamples =
    static_cast<uint16_t>((kMagPlotWindowMs / kMagSamplePeriodMs) + 2U);

static_assert(kMagMaxSamples <= 64U, "mag ring buffer too large");

} /* namespace proto_cfg */

#endif /* CONFIG_HPP */
