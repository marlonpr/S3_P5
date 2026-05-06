// draw_text.h
#pragma once
#include "hub75.h"
#include "font5x7.h"
#include <stdint.h>

// ─────────────────────────────────────────────────────────────
//  draw_char
//
//  Draws a single ASCII character at (x, y) in the given color.
//  Returns the x-advance so callers can chain characters.
//
//  x, y     : top-left pixel of the glyph
//  ch       : ASCII character (0x20–0x7E); anything else → '?'
//  r, g, b  : foreground color (0-255)
//  bg_r/g/b : background color; set alpha=-1 style via bg_enable=false
//             to leave background transparent (default)
// ─────────────────────────────────────────────────────────────
inline int draw_char(Hub75Driver& drv,
                     int x, int y,
                     char ch,
                     uint8_t r, uint8_t g, uint8_t b,
                     bool     bg_enable = false,
                     uint8_t  bg_r = 0, uint8_t bg_g = 0, uint8_t bg_b = 0)
{
    // Clamp to printable range
    if (ch < FONT5x7_FIRST || ch > FONT5x7_LAST) ch = '?';

    const uint8_t* glyph = FONT5x7[ch - FONT5x7_FIRST];

    for (int col = 0; col < FONT5x7_CHAR_W; col++) {
        uint8_t bits = glyph[col];   // each bit = one row, LSB = topmost row
        for (int row = 0; row < FONT5x7_CHAR_H; row++) {
            if (bits & (1 << row)) {
                drv.set_pixel(x + col, y + row, r, g, b);
            } else if (bg_enable) {
                drv.set_pixel(x + col, y + row, bg_r, bg_g, bg_b);
            }
        }
    }

    // Render the 1-pixel spacing column (background only, if requested)
    if (bg_enable) {
        for (int row = 0; row < FONT5x7_CHAR_H; row++)
            drv.set_pixel(x + FONT5x7_CHAR_W, y + row, bg_r, bg_g, bg_b);
    }

    return FONT5x7_CHAR_W + FONT5x7_SPACING;  // x-advance
}

// ─────────────────────────────────────────────────────────────
//  draw_string
//
//  Draws a null-terminated string starting at (x, y).
//  Returns the final x position after the last character.
// ─────────────────────────────────────────────────────────────
inline int draw_string(Hub75Driver& drv,
                       int x, int y,
                       const char* str,
                       uint8_t r, uint8_t g, uint8_t b,
                       bool    bg_enable = false,
                       uint8_t bg_r = 0, uint8_t bg_g = 0, uint8_t bg_b = 0)
{
    while (*str) {
        x += draw_char(drv, x, y, *str++, r, g, b, bg_enable, bg_r, bg_g, bg_b);
    }
    return x;
}