#pragma once
#include "esp_heap_caps.h"
#include "hub75.h"
#include <stdio.h>
#include <string.h>

class Hub75FastRenderer
{
public:

    void begin(Hub75Driver* drv)
    {
        driver_       = drv;

        planes_       = driver_->bit_depth();
        width_        = driver_->dma_width();
        scan_rows_    = driver_->num_rows();
        panel_height_ = driver_->get_height();
        lut_          = driver_->get_lut();

        row_mask_   = scan_rows_ - 1;
        lower_mask_ = panel_height_ >> 1;
    }
	
	// --------------------------------------------------
	// FAST CLEAR (animation safe)
	// --------------------------------------------------	
	// in Hub75FastRenderer (private)
	uint8_t* clear_template_ = nullptr;
	size_t   clear_template_size_ = 0;

	// call once after begin()
	void build_clear_template()
	{
	    if (!driver_) return;
	    RowBitPlaneBuffer* buffers = driver_->get_back_buffer();
	    if (!buffers) return;

	    const size_t plane_stride = width_ * 2;
	    clear_template_size_ = scan_rows_ * planes_ * plane_stride;

	    // allocate DMA-capable memory
	    clear_template_ = (uint8_t*)heap_caps_malloc(clear_template_size_, MALLOC_CAP_DMA);
	    if (!clear_template_) return;

	    // Fill template by copying current back buffer then zeroing RGB bits
	    for (int row = 0; row < scan_rows_; ++row)
	    {
	        uint8_t* src = buffers[row].data;
	        uint8_t* dst = clear_template_ + row * planes_ * plane_stride;
	        memcpy(dst, src, planes_ * plane_stride);

	        constexpr uint16_t RGB_MASK = 0x003F;
	        for (int bit = 0; bit < planes_; ++bit)
	        {
	            uint16_t* buf = (uint16_t*)(dst + bit * plane_stride);
	            for (int x = 0; x < width_; ++x)
	                buf[x] &= ~RGB_MASK;
	        }
	    }
	}

	// ultra-fast clear
	void clear_fast()
	{
	    if (!driver_ || !clear_template_) return;
	    RowBitPlaneBuffer* buffers = driver_->get_back_buffer();
	    if (!buffers) return;

	    // If back buffer is contiguous starting at buffers[0].data:
	    uint8_t* back = (uint8_t*)buffers[0].data;
	    memcpy(back, clear_template_, clear_template_size_);

	    // If not contiguous, copy per-row:
	    // for (int row = 0; row < scan_rows_; ++row)
	    //   memcpy(buffers[row].data, clear_template_ + row*row_size, row_size);
	}


    // --------------------------------------------------
    // SAFE FAST PIXEL WRITE
    // --------------------------------------------------
    inline void draw_pixel(int x, int y,
                           uint8_t r,
                           uint8_t g,
                           uint8_t b)
    {
        if (!driver_) return;

        // ⭐ ALWAYS fetch current back buffer
        RowBitPlaneBuffer* buffers =
            driver_->get_back_buffer();

        if (!buffers) return;

        uint16_t row  = y & row_mask_;
        bool is_lower = (y & lower_mask_);

        const uint16_t rc = lut_[r];
        const uint16_t gc = lut_[g];
        const uint16_t bc = lut_[b];

        constexpr uint16_t upper_mask = 0x0007u;
        constexpr uint16_t lower_mask_rgb = 0x0038u;

        const size_t plane_stride = width_ * 2;

        uint8_t* base = buffers[row].data;

        if (!base) return;   // ← prevents panic

        for (int bit = 0; bit < planes_; bit++)
        {
            uint16_t* buf =
                (uint16_t*)(base + bit * plane_stride);

            uint16_t word = buf[x];

            if (is_lower)
            {
                word = (word & ~lower_mask_rgb)
                     | (((rc>>bit)&1)<<3)
                     | (((gc>>bit)&1)<<4)
                     | (((bc>>bit)&1)<<5);
            }
            else
            {
                word = (word & ~upper_mask)
                     | (((rc>>bit)&1)<<0)
                     | (((gc>>bit)&1)<<1)
                     | (((bc>>bit)&1)<<2);
            }

            buf[x] = word;
        }
    }

private:

    Hub75Driver*    driver_ = nullptr;
    const uint16_t* lut_    = nullptr;

    int planes_        = 0;
    int width_         = 0;
    int scan_rows_     = 0;
    int panel_height_  = 0;

    int row_mask_      = 0;
    int lower_mask_    = 0;
};

/*
void clear()
{
    if (!driver_) return;

    RowBitPlaneBuffer* buffers =
        driver_->get_back_buffer();

    if (!buffers) return;

    constexpr uint16_t RGB_MASK = 0x003F; // bits 0..5

    const size_t plane_stride = width_ * 2;

    for (int row = 0; row < scan_rows_; row++)
    {
        uint8_t* base = buffers[row].data;
        if (!base) continue;

        for (int bit = 0; bit < planes_; bit++)
        {
            uint16_t* buf =
                (uint16_t*)(base + bit * plane_stride);

            for (int x = 0; x < width_; x++)
            {
                buf[x] &= ~RGB_MASK;
            }
        }
    }
}
*/