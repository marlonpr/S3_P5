#pragma once
#include "esp_heap_caps.h"
#include "hub75.h"
#include <string.h>

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
    panel_width_  = driver_->get_width();         // needed by transform
    lut_          = driver_->get_lut();

    // --- transform parameters (mirror GdmaDma fields) ---
    rotation_          = driver_->get_rotation();
    needs_layout_remap_= driver_->needs_layout_remap();
    needs_scan_remap_  = driver_->needs_scan_remap();
    layout_            = driver_->get_layout();
    scan_wiring_       = driver_->get_scan_wiring();
    layout_rows_       = driver_->get_layout_rows();
    layout_cols_       = driver_->get_layout_cols();
    virtual_width_     = driver_->get_virtual_width();
    virtual_height_    = driver_->get_virtual_height();

    rowpairs_ = (RGB*)heap_caps_malloc(
        sizeof(RGB) * width_ * scan_rows_ * 2,
        MALLOC_CAP_INTERNAL);
    memset(rowpairs_, 0, sizeof(RGB) * width_ * scan_rows_ * 2);

    rU_ = (uint16_t*)heap_caps_malloc(width_ * sizeof(uint16_t), MALLOC_CAP_INTERNAL);
    gU_ = (uint16_t*)heap_caps_malloc(width_ * sizeof(uint16_t), MALLOC_CAP_INTERNAL);
    bU_ = (uint16_t*)heap_caps_malloc(width_ * sizeof(uint16_t), MALLOC_CAP_INTERNAL);
    rL_ = (uint16_t*)heap_caps_malloc(width_ * sizeof(uint16_t), MALLOC_CAP_INTERNAL);
    gL_ = (uint16_t*)heap_caps_malloc(width_ * sizeof(uint16_t), MALLOC_CAP_INTERNAL);
    bL_ = (uint16_t*)heap_caps_malloc(width_ * sizeof(uint16_t), MALLOC_CAP_INTERNAL);
}

// ======================================================
// FRAMEBUFFER ACCESS  (native DMA coords — internal use)
// ======================================================
inline RGB& pixel_native(int px, int row, bool is_lower)
{
    int index = row * width_ * 2 + (is_lower ? width_ : 0) + px;
    return rowpairs_[index];
}

// ======================================================
// SET PIXEL  (user/rotated coords → transform → store)
// ======================================================
inline void set_pixel(int x, int y, uint8_t r, uint8_t g, uint8_t b)
{
    const uint16_t rot_w = hub75::RotationTransform::get_rotated_width(virtual_width_, virtual_height_, rotation_);
    const uint16_t rot_h = hub75::RotationTransform::get_rotated_height(virtual_width_, virtual_height_, rotation_);
    if (x < 0 || x >= rot_w || y < 0 || y >= rot_h) return;

    uint16_t px = (uint16_t)x;
    uint16_t row;
    bool is_lower;

    const bool identity = (rotation_ == Hub75Rotation::ROTATE_0)
                          && !needs_layout_remap_
                          && !needs_scan_remap_;

    if (identity) {
        if ((uint16_t)y < scan_rows_) { row = (uint16_t)y; is_lower = false; }
        else                          { row = (uint16_t)y - scan_rows_; is_lower = true; }
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
// CLEAR
// ======================================================
void clear()
{
    memset(rowpairs_, 0, sizeof(RGB) * width_ * scan_rows_ * 2);
}

// ======================================================
// RENDER PLANES  (unchanged — already in native coords)
// ======================================================
void render_planes()
{
    RowBitPlaneBuffer* buffers = driver_->get_back_buffer();
    if (!buffers) return;

    const size_t stride = width_ * sizeof(uint16_t);

    for (int row = 0; row < scan_rows_; row++)
    {
        // Build LUT caches for this row-pair
        for (int x = 0; x < width_; x++)
        {
            RGB& up   = pixel_native(x, row, false);
            RGB& down = pixel_native(x, row, true);

            rU_[x] = lut_[up.r];   gU_[x] = lut_[up.g];   bU_[x] = lut_[up.b];
            rL_[x] = lut_[down.r]; gL_[x] = lut_[down.g]; bL_[x] = lut_[down.b];
        }

        // Plane-major write (cache-friendly)
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
Hub75Driver*      driver_      = nullptr;
const uint16_t*   lut_         = nullptr;

int      planes_       = 0;
int      width_        = 0;
int      scan_rows_    = 0;
int      panel_height_ = 0;
int      panel_width_  = 0;

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

RGB*      rowpairs_ = nullptr;
uint16_t* rU_ = nullptr; uint16_t* gU_ = nullptr; uint16_t* bU_ = nullptr;
uint16_t* rL_ = nullptr; uint16_t* gL_ = nullptr; uint16_t* bL_ = nullptr;
};