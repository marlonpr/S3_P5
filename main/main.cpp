#include "app.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_panel.h"
#include "frame_pacer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"

static const char* TAG = "STATS";

/*
void log_system_stats()
{
    static uint32_t last_time = 0;
    static int frames = 0;

    frames++;

    uint32_t now = esp_timer_get_time() / 1000; // ms

    if(now - last_time >= 1000)
    {
        TaskStatus_t tasks[10];
        uint32_t total_runtime;

        int n = uxTaskGetSystemState(
            tasks, 10, &total_runtime);

        uint32_t idle_runtime = 0;

        for(int i=0;i<n;i++)
            if(strstr(tasks[i].pcTaskName,"IDLE"))
                idle_runtime += tasks[i].ulRunTimeCounter;

        float idle_pct =
            100.0f * idle_runtime / total_runtime;

        ESP_LOGI(TAG,
            "FPS=%d  CPU=%.1f%%  IDLE=%.1f%%",
            frames,
            100.0f - idle_pct,
            idle_pct);

        frames = 0;
        last_time = now;
    }
}
*/

static Hub75Config make_config()
{
    Hub75Config config{};

    config.panel_width  = 64;
    config.panel_height = 32;
    config.scan_wiring  = Hub75ScanWiring::SCAN_1_8_32PX_FULL;
    config.shift_driver = Hub75ShiftDriver::FM6126A;
    config.double_buffer = true;

    // Pins
    config.pins.r1 = 4;
    config.pins.g1 = 5;
    config.pins.b1 = 6;
    config.pins.r2 = 7;
    config.pins.g2 = 15;
    config.pins.b2 = 16;

    config.pins.a = 11;
    config.pins.b = 12;
    config.pins.c = 13;
    config.pins.d = 14;
    config.pins.e = -1;

    config.pins.lat = 9;
    config.pins.oe  = 10;
    config.pins.clk = 8;

    return config;
}



static inline void fb_set_pixel(
    int x, int y,
    uint8_t r, uint8_t g, uint8_t b)
{
    int i = (y * WIDTH + x) * 3;

    framebuffer[i + 0] = r;
    framebuffer[i + 1] = g;
    framebuffer[i + 2] = b;
}

void render_frame(int frame)
{
    for(int y = 0; y < HEIGHT; y++)
    {
        for(int x = 0; x < WIDTH; x++)
        {
            uint8_t v = (x + frame) & 255;

            fb_set_pixel(
                x, y,
                v,
                0,
                255 - v
            );
        }
    }
}

void App::run()
{
	
	framebuffer = (uint8_t*)heap_caps_malloc(
	    WIDTH * HEIGHT * 3,
	    MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA
	);
	
	Hub75Config config = make_config();

	    matrix = new Hub75Driver(config);

	    if(!matrix->begin())
	    {
	        printf("HUB75 init failed\n");
	        return;
	    }

	    int frame = 0;

	    while(true)
	    {
	        // 1. render into RAM
	        render_frame(frame++);

	        // 2. ONE bulk DMA upload
	        matrix->draw_pixels(
	            0, 0,
	            WIDTH, HEIGHT,
	            framebuffer,
	            Hub75PixelFormat::RGB888
	        );

	        // 3. atomic swap
	        matrix->flip_buffer();
			
			//log_system_stats();   // ← ADD THIS

	        // ~60 FPS
	        vTaskDelay(pdMS_TO_TICKS(16));
	    }
}

extern "C" void app_main(void)
{
    static App app;
    app.run();
	
	// VERY IMPORTANT:
	vTaskDelete(nullptr);   // delete main_task cleanly    
}








/*
// Same apparent speed at any FPS:
// velocity = desired_px_per_second / fps

// 30 FPS — slower rendering, less CPU
frame_pacer_init(&pacer, 30.0f);
velocity = 0.44f;   // still 13 px/sec (0.44 × 30)

// 60 FPS — default, smooth subpixel blending
frame_pacer_init(&pacer, 60.0f);
velocity = 0.22f;   // 13 px/sec (0.22 × 60)

// 120 FPS — ultra smooth, but ESP32 DMA may struggle
frame_pacer_init(&pacer, 120.0f);
velocity = 0.11f;   // still 13 px/sec (0.11 × 120)
*/


