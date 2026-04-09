// main.cpp
#include "hub75.h"
#include "hub75_fast_renderer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "esp_timer.h"

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



static const char* TAG2 = "STATS";

void log_system_stats()
{
    static uint64_t last_time    = 0;
    static int      frames       = 0;
    static uint32_t last_idle[2] = {0, 0};

    frames++;

    uint64_t now = esp_timer_get_time(); // µs
    if (now - last_time < 1000000ULL) return;

    TaskStatus_t tasks[16];
    uint32_t total_runtime;
    int n = uxTaskGetSystemState(tasks, 16, &total_runtime);

    uint32_t idle_runtime[2] = {0, 0};

    for (int i = 0; i < n; i++) {
        ESP_LOGI(TAG, "  %-16s runtime=%lu",
                 tasks[i].pcTaskName,
                 tasks[i].ulRunTimeCounter);

        if (strstr(tasks[i].pcTaskName, "IDLE0"))
            idle_runtime[0] += tasks[i].ulRunTimeCounter;
        else if (strstr(tasks[i].pcTaskName, "IDLE1"))
            idle_runtime[1] += tasks[i].ulRunTimeCounter;
    }

    uint32_t delta_idle[2] = {
        idle_runtime[0] - last_idle[0],
        idle_runtime[1] - last_idle[1],
    };

    last_idle[0] = idle_runtime[0];
    last_idle[1] = idle_runtime[1];

    if (last_time == 0) {
        last_time = now;
        frames    = 0;
        return;
    }

    uint64_t elapsed_us = now - last_time;

    float cpu0 = 100.0f - (100.0f * delta_idle[0] / elapsed_us);
    float cpu1 = 100.0f - (100.0f * delta_idle[1] / elapsed_us);
    float total_cpu = (cpu0 + cpu1) / 2.0f;

    ESP_LOGI(TAG2, "FPS=%d  CPU0=%.1f%%  CPU1=%.1f%%  TOTAL=%.1f%%",
             frames, cpu0, cpu1, total_cpu);

    frames    = 0;
    last_time = now;
}





void render_frame(Hub75FastRenderer& renderer, int frame)
{
    // renderer.clear(); // You might not even need this anymore if drawing full screen!
    renderer.draw_gradient(frame);
    renderer.render_planes();
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
	renderer.set_pixel(30,30,0,0,255);

	renderer.render_planes();
	driver.flip_buffer();

	int frame = 0;

	// Outside your loop:
	TickType_t xLastWakeTime = xTaskGetTickCount();
	const TickType_t xFrequency = pdMS_TO_TICKS(16); // 16ms for ~60 FPS

	while (true)
	{
	    render_frame(renderer, frame);
	    driver.flip_buffer();
	    frame = (frame + 1) & 255;
	    
	    log_system_stats();

	    // Replaces vTaskDelay. This guarantees exactly 16ms between loops!
	    vTaskDelayUntil(&xLastWakeTime, xFrequency); 
	}
}

/*
// Draw pixels - changes appear automatically!
driver.set_pixel(10, 10, 255, 0, 0);  // Red
driver.set_pixel(20, 20, 0, 255, 0);  // Green
driver.set_pixel(30, 30, 0, 0, 255);  // Blue

driver.flip_buffer();  // Atomic swap
*/
