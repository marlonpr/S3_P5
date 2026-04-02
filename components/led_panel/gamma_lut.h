#pragma once
#include <stdint.h>

extern uint8_t gamma_lut[256];
extern float   gamma_inv[256];

void gamma_init();