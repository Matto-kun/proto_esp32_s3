/**
 * @file        ir_app.hpp
 * @brief       Lazy AMG8833 + live Inferno heatmap (app module)
 * @author      Mat RB
 * @date        2026-08-26
 */

#ifndef IR_APP_HPP
#define IR_APP_HPP

#include "display/display_api.hpp"
#include "esp_err.h"

esp_err_t ir_app_start(Display& display);
void      ir_app_stop(Display& display);
bool      ir_app_is_live(void);
void      ir_app_poll(Display& display);

#endif /* IR_APP_HPP */
