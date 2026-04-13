#pragma once
#include "esp_heap_caps.h"
#include "hub75.h"
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "../src/platforms/platform_dma.h"

class Hub75FastRenderer
{
public:

struct RGB { uint8_t r, g, b; };

// ======================================================
// INIT
// ======================================================
void begin(Hub75Driver* drv)
{
    driver_       = drv;
    planes_       = driver_->bit_depth();
    width_        = driver_->dma_width();
    scan_rows_    = driver_->num_rows();
    panel_height_ = driver_->get_height();
    panel_width_  = driver_->get_width();
    lut_          = driver_->get_lut();

    // Transform parameters
    rotation_           = driver_->get_rotation();
    needs_layout_remap_ = driver_->needs_layout_remap();
    needs_scan_remap_   = driver_->needs_scan_remap();
    layout_             = driver_->get_layout();
    scan_wiring_        = driver_->get_scan_wiring();
    layout_rows_        = driver_->get_layout_rows();
    layout_cols_        = driver_->get_layout_cols();
    virtual_width_      = driver_->get_virtual_width();
    virtual_height_     = driver_->get_virtual_height();

    // Cache rotated bounds — used every set_pixel call
    rot_w_ = hub75::RotationTransform::get_rotated_width(virtual_width_, virtual_height_, rotation_);
    rot_h_ = hub75::RotationTransform::get_rotated_height(virtual_width_, virtual_height_, rotation_);

    // Cache identity flag — used every set_pixel and draw call
    identity_ = (rotation_ == Hub75Rotation::ROTATE_0)
                && !needs_layout_remap_
                && !needs_scan_remap_;

    // Row-pair RGB framebuffer
    rowpairs_ = (RGB*)heap_caps_malloc(
        sizeof(RGB) * width_ * scan_rows_ * 2,
        MALLOC_CAP_INTERNAL);
    memset(rowpairs_, 0, sizeof(RGB) * width_ * scan_rows_ * 2);

    // BUG FIX: lut_[] returns uint16_t values scaled by bit_depth.
    // We need to reduce to 8-bit before indexing bit_luts.
    // lut_ maps 0-255 input → 0-255 output (same range, gamma corrected).
    // The lut_ values ARE 8-bit (0-255) for standard gamma modes.
    // Verify with: assert(lut_[255] <= 255) — if not, adjust lut_shift_ below.
    //
    // If your LUT is 16-bit scaled (0-32767), set lut_shift_ = 7.
    // If your LUT is 8-bit (0-255), set lut_shift_ = 0.
	
    lut_shift_ = (lut_[255] > 255) ? 7 : 0;
	
	//lut_shift_ = (planes_ > 8) ? (planes_ - 8) : 0;


    generate_bit_luts();
	
	scratch_ = (uint64_t*)heap_caps_malloc(width_ * sizeof(uint64_t), MALLOC_CAP_INTERNAL);

}

// ======================================================
// FRAMEBUFFER ACCESS (native DMA coords — internal use)
// ======================================================
inline RGB& pixel_native(int px, int row, bool is_lower)
{
    int index = row * width_ * 2 + (is_lower ? width_ : 0) + px;
    return rowpairs_[index];
}

// ======================================================
// SET PIXEL (user/rotated coords → transform → store)
// ======================================================
inline void set_pixel(int x, int y, uint8_t r, uint8_t g, uint8_t b)
{
    // Use cached bounds — avoids recomputing every call
    if (x < 0 || x >= rot_w_ || y < 0 || y >= rot_h_) return;

    uint16_t px = (uint16_t)x;
    uint16_t row;
    bool is_lower;

    if (identity_) {
        if ((uint16_t)y < (uint16_t)scan_rows_) {
            row = (uint16_t)y;
            is_lower = false;
        } else {
            row = (uint16_t)y - (uint16_t)scan_rows_;
            is_lower = true;
        }
    } else {
        auto t = hub75::PlatformDma::transform_coordinate(
            px, (uint16_t)y,
            rotation_, needs_layout_remap_, needs_scan_remap_,
            layout_, scan_wiring_,
            panel_width_, panel_height_,
            layout_rows_, layout_cols_,
            virtual_width_, virtual_height_,
            width_, scan_rows_);
        px       = t.x;
        row      = t.row;
        is_lower = t.is_lower;
    }

    pixel_native(px, row, is_lower) = { r, g, b };
}

// ======================================================
// DRAW GRADIENT (example effect)
// ======================================================
void draw_gradient(int frame)
{
    if (identity_) {
        // FAST PATH: direct pointer arithmetic, no coordinate math
        RGB* ptr = rowpairs_;
        for (int row = 0; row < scan_rows_; row++) {
            for (int x = 0; x < width_; x++) {
                uint8_t v = (uint8_t)((x + frame) & 255);
                ptr[x]         = { v, 0, (uint8_t)(255 - v) };  // upper
                ptr[x + width_]= { v, 0, (uint8_t)(255 - v) };  // lower
            }
            ptr += width_ * 2;
        }
    } else {
        // SLOW PATH: full coordinate transform
        for (int y = 0; y < virtual_height_; y++) {
            for (int x = 0; x < virtual_width_; x++) {
                uint8_t v = (uint8_t)((x + frame) & 255);
                set_pixel(x, y, v, 0, 255 - v);
            }
        }
    }
}

// ======================================================
// DRAW BITMAP
// ======================================================
void draw_bitmap(int x_off, int y_off, int w, int h, const RGB* data)
{
    for (int y = 0; y < h; y++) {
        int ty = y + y_off;
        if (ty < 0 || ty >= rot_h_) continue;
        for (int x = 0; x < w; x++) {
            int tx = x + x_off;
            if (tx < 0 || tx >= rot_w_) continue;
            const RGB& p = data[y * w + x];
            set_pixel(tx, ty, p.r, p.g, p.b);
        }
    }
}

// ======================================================
// DRAW TEXT
// ======================================================
void draw_char(int x, int y, char c, uint8_t r, uint8_t g, uint8_t b)
{
    if (c < 32 || c > 126) return;
    const uint8_t* glyph = font5x7_[c - 32];
    for (int col = 0; col < 5; col++) {
        uint8_t line = glyph[col];
        for (int row = 0; row < 7; row++) {
            if (line & (1 << row)) {
                set_pixel(x + col, y + row, r, g, b);
            }
        }
    }
}

void draw_text(int x, int y, const char* str, uint8_t r, uint8_t g, uint8_t b)
{
    while (*str) {
        draw_char(x, y, *str++, r, g, b);
        x += 6; // 5px glyph + 1px spacing
    }
}

// Scrolling text — call each frame, increment offset externally
// offset counts pixels; wraps automatically at full string width
void draw_text_scroll(int y, const char* str, int offset,
                      uint8_t r, uint8_t g, uint8_t b)
{
    int len = (int)strlen(str);
    int total_w = len * 6;
    // Normalise offset so it always wraps cleanly
    int off = offset % total_w;
    // Draw twice to fill the seam when text wraps
    draw_text(rot_w_ - off,          y, str, r, g, b);
    draw_text(rot_w_ - off + total_w, y, str, r, g, b);
}

// ======================================================
// CLEAR
// ======================================================
void clear()
{
    memset(rowpairs_, 0, sizeof(RGB) * width_ * scan_rows_ * 2);
}

// ======================================================
// RENDER PLANES
// FIX: loop order is bit-outer, x-inner → cache-friendly DMA writes
// FIX: lut_shift_ corrects OOB indexing into bit_luts
// ======================================================
// render_planes — two clean phases:
void render_planes()
{
    RowBitPlaneBuffer* buffers = driver_->get_back_buffer();
    if (!buffers || !lut_ || !bit_luts_[0] || !scratch_) return;

    const size_t stride = width_ * sizeof(uint16_t);

    for (int row = 0; row < scan_rows_; row++)
    {
        // PHASE 1: x-loop only — 6 LUT lookups per pixel, ONCE
        // rowpairs_ and scratch_ are both sequential → cache friendly
        const RGB* up_row   = &rowpairs_[row * width_ * 2];
        const RGB* down_row = up_row + width_;

        for (int x = 0; x < width_; x++)
        {
            uint8_t ru = (uint8_t)(lut_[up_row[x].r]   >> lut_shift_);
            uint8_t gu = (uint8_t)(lut_[up_row[x].g]   >> lut_shift_);
            uint8_t bu = (uint8_t)(lut_[up_row[x].b]   >> lut_shift_);
            uint8_t rd = (uint8_t)(lut_[down_row[x].r] >> lut_shift_);
            uint8_t gd = (uint8_t)(lut_[down_row[x].g] >> lut_shift_);
            uint8_t bd = (uint8_t)(lut_[down_row[x].b] >> lut_shift_);

            scratch_[x] = bit_luts_[0][ru] | bit_luts_[1][gu] | bit_luts_[2][bu]
                        | bit_luts_[3][rd] | bit_luts_[4][gd] | bit_luts_[5][bd];
        }

        // PHASE 2: bit-outer, x-inner — dst fixed per bit → cache friendly
        // scratch_ is hot in cache from phase 1
        for (int bit = 0; bit < planes_; bit++)
        {
            uint16_t* dst = (uint16_t*)(buffers[row].data + bit * stride);
            const int shift = bit * 8;

            for (int x = 0; x < width_; x++)
            {
                dst[x] = (dst[x] & ~0x003F) | (uint8_t)(scratch_[x] >> shift);
            }
        }
    }
}

private:

// ======================================================
// BIT LUT GENERATION
// Encodes all 8 bits of a color component into a 64-bit word,
// one bit per byte, at the channel's pin position (0-5).
// render_planes extracts each plane's byte with a single shift.
// ======================================================

void generate_bit_luts()
{
    for (int ch = 0; ch < 6; ch++) {
        bit_luts_[ch] = (uint64_t*)heap_caps_malloc(
            256 * sizeof(uint64_t), MALLOC_CAP_INTERNAL);
        for (int val = 0; val < 256; val++) {
            uint64_t entry = 0;
            for (int bit = 0; bit < planes_; bit++) {  // ← was hardcoded 8
                if ((val >> bit) & 1) {
                    entry |= ((uint64_t)1 << (bit * 8 + ch));
                }
            }
            bit_luts_[ch][val] = entry;
        }
    }
}

/*
void generate_bit_luts()
{
    for (int ch = 0; ch < 6; ch++) {
        bit_luts_[ch] = (uint64_t*)heap_caps_malloc(
            256 * sizeof(uint64_t), MALLOC_CAP_INTERNAL);
        for (int val = 0; val < 256; val++) {
            uint64_t entry = 0;
            for (int bit = 0; bit < 8; bit++) {
                if ((val >> bit) & 1) {
                    entry |= ((uint64_t)1 << (bit * 8 + ch));
                }
            }
            bit_luts_[ch][val] = entry;
        }
    }
}
*/

// ======================================================
// FONT DATA (5x7, printable ASCII 32-126)
// Each glyph is 5 bytes (columns), bit0 = top row
// ======================================================
static constexpr uint8_t font5x7_[][5] = {
    {0x00,0x00,0x00,0x00,0x00}, // ' '
    {0x00,0x00,0x5F,0x00,0x00}, // '!'
    {0x00,0x07,0x00,0x07,0x00}, // '"'
    {0x14,0x7F,0x14,0x7F,0x14}, // '#'
    {0x24,0x2A,0x7F,0x2A,0x12}, // '$'
    {0x23,0x13,0x08,0x64,0x62}, // '%'
    {0x36,0x49,0x55,0x22,0x50}, // '&'
    {0x00,0x05,0x03,0x00,0x00}, // '''
    {0x00,0x1C,0x22,0x41,0x00}, // '('
    {0x00,0x41,0x22,0x1C,0x00}, // ')'
    {0x14,0x08,0x3E,0x08,0x14}, // '*'
    {0x08,0x08,0x3E,0x08,0x08}, // '+'
    {0x00,0x50,0x30,0x00,0x00}, // ','
    {0x08,0x08,0x08,0x08,0x08}, // '-'
    {0x00,0x60,0x60,0x00,0x00}, // '.'
    {0x20,0x10,0x08,0x04,0x02}, // '/'
    {0x3E,0x51,0x49,0x45,0x3E}, // '0'
    {0x00,0x42,0x7F,0x40,0x00}, // '1'
    {0x42,0x61,0x51,0x49,0x46}, // '2'
    {0x21,0x41,0x45,0x4B,0x31}, // '3'
    {0x18,0x14,0x12,0x7F,0x10}, // '4'
    {0x27,0x45,0x45,0x45,0x39}, // '5'
    {0x3C,0x4A,0x49,0x49,0x30}, // '6'
    {0x01,0x71,0x09,0x05,0x03}, // '7'
    {0x36,0x49,0x49,0x49,0x36}, // '8'
    {0x06,0x49,0x49,0x29,0x1E}, // '9'
    {0x00,0x36,0x36,0x00,0x00}, // ':'
    {0x00,0x56,0x36,0x00,0x00}, // ';'
    {0x08,0x14,0x22,0x41,0x00}, // '<'
    {0x14,0x14,0x14,0x14,0x14}, // '='
    {0x00,0x41,0x22,0x14,0x08}, // '>'
    {0x02,0x01,0x51,0x09,0x06}, // '?'
    {0x32,0x49,0x79,0x41,0x3E}, // '@'
    {0x7E,0x11,0x11,0x11,0x7E}, // 'A'
    {0x7F,0x49,0x49,0x49,0x36}, // 'B'
    {0x3E,0x41,0x41,0x41,0x22}, // 'C'
    {0x7F,0x41,0x41,0x22,0x1C}, // 'D'
    {0x7F,0x49,0x49,0x49,0x41}, // 'E'
    {0x7F,0x09,0x09,0x09,0x01}, // 'F'
    {0x3E,0x41,0x49,0x49,0x7A}, // 'G'
    {0x7F,0x08,0x08,0x08,0x7F}, // 'H'
    {0x00,0x41,0x7F,0x41,0x00}, // 'I'
    {0x20,0x40,0x41,0x3F,0x01}, // 'J'
    {0x7F,0x08,0x14,0x22,0x41}, // 'K'
    {0x7F,0x40,0x40,0x40,0x40}, // 'L'
    {0x7F,0x02,0x0C,0x02,0x7F}, // 'M'
    {0x7F,0x04,0x08,0x10,0x7F}, // 'N'
    {0x3E,0x41,0x41,0x41,0x3E}, // 'O'
    {0x7F,0x09,0x09,0x09,0x06}, // 'P'
    {0x3E,0x41,0x51,0x21,0x5E}, // 'Q'
    {0x7F,0x09,0x19,0x29,0x46}, // 'R'
    {0x46,0x49,0x49,0x49,0x31}, // 'S'
    {0x01,0x01,0x7F,0x01,0x01}, // 'T'
    {0x3F,0x40,0x40,0x40,0x3F}, // 'U'
    {0x1F,0x20,0x40,0x20,0x1F}, // 'V'
    {0x3F,0x40,0x38,0x40,0x3F}, // 'W'
    {0x63,0x14,0x08,0x14,0x63}, // 'X'
    {0x07,0x08,0x70,0x08,0x07}, // 'Y'
    {0x61,0x51,0x49,0x45,0x43}, // 'Z'
    {0x00,0x7F,0x41,0x41,0x00}, // '['
    {0x02,0x04,0x08,0x10,0x20}, // '\'
    {0x00,0x41,0x41,0x7F,0x00}, // ']'
    {0x04,0x02,0x01,0x02,0x04}, // '^'
    {0x40,0x40,0x40,0x40,0x40}, // '_'
    {0x00,0x01,0x02,0x04,0x00}, // '`'
    {0x20,0x54,0x54,0x54,0x78}, // 'a'
    {0x7F,0x48,0x44,0x44,0x38}, // 'b'
    {0x38,0x44,0x44,0x44,0x20}, // 'c'
    {0x38,0x44,0x44,0x48,0x7F}, // 'd'
    {0x38,0x54,0x54,0x54,0x18}, // 'e'
    {0x08,0x7E,0x09,0x01,0x02}, // 'f'
    {0x0C,0x52,0x52,0x52,0x3E}, // 'g'
    {0x7F,0x08,0x04,0x04,0x78}, // 'h'
    {0x00,0x44,0x7D,0x40,0x00}, // 'i'
    {0x20,0x40,0x44,0x3D,0x00}, // 'j'
    {0x7F,0x10,0x28,0x44,0x00}, // 'k'
    {0x00,0x41,0x7F,0x40,0x00}, // 'l'
    {0x7C,0x04,0x18,0x04,0x78}, // 'm'
    {0x7C,0x08,0x04,0x04,0x78}, // 'n'
    {0x38,0x44,0x44,0x44,0x38}, // 'o'
    {0x7C,0x14,0x14,0x14,0x08}, // 'p'
    {0x08,0x14,0x14,0x18,0x7C}, // 'q'
    {0x7C,0x08,0x04,0x04,0x08}, // 'r'
    {0x48,0x54,0x54,0x54,0x20}, // 's'
    {0x04,0x3F,0x44,0x40,0x20}, // 't'
    {0x3C,0x40,0x40,0x20,0x7C}, // 'u'
    {0x1C,0x20,0x40,0x20,0x1C}, // 'v'
    {0x3C,0x40,0x30,0x40,0x3C}, // 'w'
    {0x44,0x28,0x10,0x28,0x44}, // 'x'
    {0x0C,0x50,0x50,0x50,0x3C}, // 'y'
    {0x44,0x64,0x54,0x4C,0x44}, // 'z'
    {0x00,0x08,0x36,0x41,0x00}, // '{'
    {0x00,0x00,0x7F,0x00,0x00}, // '|'
    {0x00,0x41,0x36,0x08,0x00}, // '}'
    {0x10,0x08,0x08,0x10,0x08}, // '~'
};

// ======================================================
// MEMBERS
// ======================================================
Hub75Driver*      driver_      = nullptr;
const uint16_t*   lut_         = nullptr;
uint64_t*         bit_luts_[6] = {};
int               lut_shift_   = 0;   // 0 if lut is 8-bit, 7 if 16-bit scaled

int      planes_       = 0;
int      width_        = 0;
int      scan_rows_    = 0;
int      panel_height_ = 0;
int      panel_width_  = 0;

// Cached display geometry
uint16_t rot_w_        = 0;
uint16_t rot_h_        = 0;
bool     identity_     = false;

// Transform parameters
Hub75Rotation     rotation_           = Hub75Rotation::ROTATE_0;
bool              needs_layout_remap_ = false;
bool              needs_scan_remap_   = false;
Hub75PanelLayout  layout_             = {};
Hub75ScanWiring   scan_wiring_        = {};
int               layout_rows_        = 0;
int               layout_cols_        = 0;
uint16_t          virtual_width_      = 0;
uint16_t          virtual_height_     = 0;

RGB* rowpairs_ = nullptr;

uint64_t* scratch_ = nullptr;
};