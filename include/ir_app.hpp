/**
 * @file        ir_app.hpp
 * @brief       Lazy AMG8833 + live Inferno heatmap (app module)
 * @author      Mat RB
 * @date        2026-08-26
 */

#ifndef IR_APP_HPP
#define IR_APP_HPP

#include <stdint.h>

#include "display/display_api.hpp"
#include "esp_err.h"

esp_err_t ir_app_start(Display& display);
void      ir_app_stop(Display& display);
bool      ir_app_is_live(void);
void      ir_app_poll(Display& display);

uint8_t ir_app_filter_mode(void);
uint8_t ir_app_sharp_level(void);
uint8_t ir_app_rot_mode(void);
bool    ir_app_flip_on(void);
uint8_t ir_app_cycle_filter(void);
uint8_t ir_app_cycle_level(void);
uint8_t ir_app_cycle_rot(void);
bool    ir_app_cycle_flip(void);

#endif /* IR_APP_HPP */
