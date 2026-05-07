#pragma once

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t clock_settings_init(void);

void clock_settings_save_format(uint8_t format);
uint8_t clock_settings_load_format(uint8_t default_format);

void clock_settings_save_mode(uint8_t mode);
uint8_t clock_settings_load_mode(uint8_t default_mode);

void clock_settings_save_brightness(uint8_t brightness_level);
uint8_t clock_settings_load_brightness(uint8_t default_brightness_level);

#ifdef __cplusplus
}
#endif