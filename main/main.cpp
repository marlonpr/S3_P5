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
	
	/*
	// Draw pixels - changes appear automatically!
	driver.set_pixel(10, 10, 255, 0, 0);  // Red
	driver.set_pixel(20, 20, 0, 255, 0);  // Green
	driver.set_pixel(30, 30, 0, 0, 255);  // Blue
	
	driver.flip_buffer();  // Atomic swap
	*/
	
	
	renderer.clear();
	renderer.set_pixel(10,10,255,0,0);
	renderer.set_pixel(20,20,0,255,0);
	renderer.set_pixel(30,31,0,0,255);

	renderer.render_planes();
	driver.flip_buffer();


	while (true)
	{
		vTaskDelay(pdMS_TO_TICKS(16));
	}
}

