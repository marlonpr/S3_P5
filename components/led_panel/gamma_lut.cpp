#include "gamma_lut.h"
#include <math.h>

uint8_t gamma_lut[256];
float   gamma_inv[256];

void gamma_init()
{
    const float gamma = 2.2f;

    for (int i = 0; i < 256; ++i)
    {
        float x = i / 255.0f;

        gamma_lut[i] = (uint8_t)(powf(x, 1.0f/gamma) * 255.0f + 0.5f);
        gamma_inv[i] = powf(x, gamma);
    }
}