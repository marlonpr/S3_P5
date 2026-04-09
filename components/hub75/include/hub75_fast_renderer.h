#pragma once
#include "esp_heap_caps.h"
#include "hub75.h"
#include <string.h>

class Hub75FastRenderer
{
public:

// ======================================================
// INIT
// ======================================================
void begin(Hub75Driver* drv)
{
    driver_ = drv;

    planes_       = driver_->bit_depth();
    width_        = driver_->dma_width();
    scan_rows_    = driver_->num_rows();
    panel_height_ = driver_->get_height();
    lut_          = driver_->get_lut();

    //----------------------------------
    // Row-pair RGB framebuffer
    //----------------------------------
    rowpairs_ = (RGB*)heap_caps_malloc(
        sizeof(RGB) * width_ * scan_rows_ * 2,
        MALLOC_CAP_INTERNAL);

    memset(rowpairs_, 0, sizeof(RGB) * width_ * scan_rows_ * 2);

    //----------------------------------
    // LUT caches (upper + lower)
    //----------------------------------
    rU_ = (uint16_t*)heap_caps_malloc(width_ * sizeof(uint16_t), MALLOC_CAP_INTERNAL);
    gU_ = (uint16_t*)heap_caps_malloc(width_ * sizeof(uint16_t), MALLOC_CAP_INTERNAL);
    bU_ = (uint16_t*)heap_caps_malloc(width_ * sizeof(uint16_t), MALLOC_CAP_INTERNAL);

    rL_ = (uint16_t*)heap_caps_malloc(width_ * sizeof(uint16_t), MALLOC_CAP_INTERNAL);
    gL_ = (uint16_t*)heap_caps_malloc(width_ * sizeof(uint16_t), MALLOC_CAP_INTERNAL);
    bL_ = (uint16_t*)heap_caps_malloc(width_ * sizeof(uint16_t), MALLOC_CAP_INTERNAL);
}

// ======================================================
// RGB STRUCT
// ======================================================
struct RGB
{
    uint8_t r,g,b;
};

// ======================================================
// FRAMEBUFFER ACCESS
// ======================================================
inline RGB& pixel(int x,int y)
{
    int rowpair = y % scan_rows_;
    bool lower  = y >= scan_rows_;

    int index =
        rowpair * width_ * 2 +
        (lower ? width_ : 0) +
        x;

    return rowpairs_[index];
}

inline void set_pixel(int x,int y,uint8_t r,uint8_t g,uint8_t b)
{
    if (x < 0 || x >= width_) return;
    if (y < 0 || y >= panel_height_) return;

    pixel(x,y) = {r,g,b};
}

// ======================================================
// CLEAR FRAMEBUFFER
// ======================================================
void clear()
{
    memset(rowpairs_, 0, sizeof(RGB) * width_ * scan_rows_ * 2);
}

// ======================================================
// EFFECT EXAMPLE
// ======================================================
void build_gradient(int frame)
{
    for (int r = 0; r < scan_rows_; r++)
    {
        for (int x = 0; x < width_; x++)
        {
            uint8_t v = (x + frame) & 255;

            pixel(x, r)                 = { v, 0, (uint8_t)(255-v) };          // upper
            pixel(x, r + scan_rows_)    = { (uint8_t)(255-v), 0, v };          // lower
        }
    }
}

// ======================================================
// RENDER PLANES (ROW-PAIR NATIVE)
// ======================================================
void render_planes()
{
    RowBitPlaneBuffer* buffers = driver_->get_back_buffer();
    if (!buffers) return;

    const size_t stride = width_ * sizeof(uint16_t);

    for (int row = 0; row < scan_rows_; row++)
    {
        //----------------------------------
        // Build LUTs for this row pair
        //----------------------------------
        for (int x = 0; x < width_; x++)
        {
            RGB& up   = pixel(x, row);           
            RGB& down = pixel(x, row + scan_rows_);

            rU_[x] = lut_[up.r];
            gU_[x] = lut_[up.g];
            bU_[x] = lut_[up.b];

            rL_[x] = lut_[down.r];
            gL_[x] = lut_[down.g];
            bL_[x] = lut_[down.b];
        }

        //----------------------------------
        // Plane-major write
        //----------------------------------
        for (int bit = 0; bit < planes_; bit++)
        {
            uint16_t* dst = (uint16_t*)(buffers[row].data + bit * stride);

            for (int x = 0; x < width_; x++)
            {
                uint16_t rgb =
                    (((rU_[x] >> bit) & 1) << 0) |
                    (((gU_[x] >> bit) & 1) << 1) |
                    (((bU_[x] >> bit) & 1) << 2) |
                    (((rL_[x] >> bit) & 1) << 3) |
                    (((gL_[x] >> bit) & 1) << 4) |
                    (((bL_[x] >> bit) & 1) << 5);

                dst[x] = (dst[x] & ~0x003F) | rgb;
            }
        }
    }
}

private:
Hub75Driver* driver_ = nullptr;
const uint16_t* lut_ = nullptr;

int planes_       = 0;
int width_        = 0;
int scan_rows_    = 0;
int panel_height_ = 0;

// Row-pair framebuffer
RGB* rowpairs_ = nullptr;

// LUT caches (upper and lower)
uint16_t* rU_ = nullptr;
uint16_t* gU_ = nullptr;
uint16_t* bU_ = nullptr;

uint16_t* rL_ = nullptr;
uint16_t* gL_ = nullptr;
uint16_t* bL_ = nullptr;

};