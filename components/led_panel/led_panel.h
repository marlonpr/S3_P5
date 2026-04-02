#include "hub75.h"
#include "esp_heap_caps.h"

static Hub75Driver* matrix;

// DMA-friendly framebuffer (recommended)
static uint8_t* framebuffer;

#define WIDTH  64
#define HEIGHT 32