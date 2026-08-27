/**
 * @file        mag_app.hpp
 * @brief       Lazy magnetometer + live plot (app module)
 * @author      Mat RB
 * @date        2026-08-19
 */

#ifndef MAG_APP_HPP
#define MAG_APP_HPP

#include <stdint.h>

#include "display/display_api.hpp"
#include "esp_err.h"

esp_err_t mag_app_start(Display& display);
void      mag_app_stop(Display& display);
void      mag_app_reset_plot(Display& display);
bool      mag_app_is_live(void);
void      mag_app_poll(Display& display);

uint8_t mag_app_view_mode(void);
uint8_t mag_app_axis_mode(void);
uint8_t mag_app_unit_mode(void);
uint8_t mag_app_cycle_view(void);
uint8_t mag_app_cycle_axis(void);
uint8_t mag_app_cycle_unit(void);

#endif /* MAG_APP_HPP */
