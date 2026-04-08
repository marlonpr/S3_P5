
// main.cpp
#include "hub75.h"
#include "hub75_fast_renderer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char* TAG = "MAIN";

static Hub75Config make_config()
{
    Hub75Config config{};

    config.panel_width  = 64;
    config.panel_height = 32;

    config.scan_wiring = Hub75ScanWiring::SCAN_1_8_32PX_FULL;
    config.shift_driver = Hub75ShiftDriver::FM6126A;
	
    config.double_buffer = true;

    // Upper RGB
    config.pins.r1 = 4;
    config.pins.g1 = 5;
    config.pins.b1 = 6;

    // Lower RGB
    config.pins.r2 = 7;
    config.pins.g2 = 15;
    config.pins.b2 = 16;

    // Address
    config.pins.a = 11;
    config.pins.b = 12;
    config.pins.c = 13;
    config.pins.d = 14;
    config.pins.e = -1;

    // Control
    config.pins.lat = 9;
    config.pins.oe  = 10;
    config.pins.clk = 8;

    return config;
}

void render_frame(Hub75FastRenderer& renderer, int frame)
{
    renderer.clear_fast();   // important for animation

    for (int y = 0; y < 64; y++)
    {
        for (int x = 0; x < 32; x++)
        {
            uint8_t v = (x + frame) & 255;

            renderer.draw_pixel(
                x,
                y,
                v,          // R
                0,          // G
                255 - v     // B
            );
        }
    }
}


extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Starting HUB75");

    //--------------------------------------------------
    // Create driver
    //--------------------------------------------------
    Hub75Config config = make_config();

    Hub75Driver driver(config);

    // IMPORTANT:
    // begin() already initializes AND starts DMA
    if (!driver.begin())
    {
        ESP_LOGE(TAG, "Driver start failed");
        return;
    }

    //--------------------------------------------------
    // Fast renderer
    //--------------------------------------------------
    Hub75FastRenderer renderer;
    renderer.begin(&driver);
	
	renderer.build_clear_template();

	int pos = 0;

	while (true)
	{
	    renderer.clear_fast();                // erase previous frame

	    renderer.draw_pixel(pos, 5, 255,255,255);
		
		renderer.draw_pixel(pos, 10, 255,255,255);
		
		renderer.draw_pixel(pos, 15, 255,255,255);
		
		renderer.draw_pixel(pos, 20, 255,255,255);
		
		renderer.draw_pixel(pos, 25, 255,255,255);

	    driver.flip_buffer();            // swap DMA buffers

	    pos = (pos + 1) % driver.get_width();

	    vTaskDelay(pdMS_TO_TICKS(160));   // ~60 FPS
	}
}


/*

// main.cpp
#include "hub75.h"
#include "hub75_fast_renderer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char* TAG = "MAIN";

static Hub75Config make_config()
{
    Hub75Config config{};

    config.panel_width  = 64;
    config.panel_height = 32;

    config.scan_wiring = Hub75ScanWiring::SCAN_1_8_32PX_FULL;
    config.shift_driver = Hub75ShiftDriver::FM6126A;
	
    config.double_buffer = true;

    // Upper RGB
    config.pins.r1 = 4;
    config.pins.g1 = 5;
    config.pins.b1 = 6;

    // Lower RGB
    config.pins.r2 = 7;
    config.pins.g2 = 15;
    config.pins.b2 = 16;

    // Address
    config.pins.a = 11;
    config.pins.b = 12;
    config.pins.c = 13;
    config.pins.d = 14;
    config.pins.e = -1;

    // Control
    config.pins.lat = 9;
    config.pins.oe  = 10;
    config.pins.clk = 8;

    return config;
}

void render_frame(Hub75FastRenderer& renderer, int frame)
{
    renderer.clear_fast();   // important for animation

    for (int y = 0; y < 64; y++)
    {
        for (int x = 0; x < 32; x++)
        {
            uint8_t v = (x + frame) & 255;

            renderer.draw_pixel(
                x,
                y,
                v,          // R
                0,          // G
                255 - v     // B
            );
        }
    }
}


extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Starting HUB75");

    //--------------------------------------------------
    // Create driver
    //--------------------------------------------------
    Hub75Config config = make_config();

    Hub75Driver driver(config);

    // IMPORTANT:
    // begin() already initializes AND starts DMA
    if (!driver.begin())
    {
        ESP_LOGE(TAG, "Driver start failed");
        return;
    }

    //--------------------------------------------------
    // Fast renderer
    //--------------------------------------------------
    Hub75FastRenderer renderer;
    renderer.begin(&driver);
	
	renderer.build_clear_template();

	int pos = 0;

	while (true)
	{
	    renderer.clear_fast();                // erase previous frame

	    renderer.draw_pixel(pos, 12, 255,255,255);

	    driver.flip_buffer();            // swap DMA buffers

	    pos = (pos + 1) % driver.get_width();

	    vTaskDelay(pdMS_TO_TICKS(16));   // ~60 FPS
	}
}


*/		