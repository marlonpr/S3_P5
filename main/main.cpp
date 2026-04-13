// main.cpp
#include "hub75.h"
#include "hub75_fast_renderer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "esp_timer.h"

#include "esp_heap_caps.h"
#include "esp_system.h"
#include "esp_chip_info.h"
#include "freertos/atomic.h"

// Structure to pass multiple arguments to the task if needed
struct DisplayTaskConfig {
    Hub75FastRenderer* renderer;
    Hub75Driver* driver;
};

uint32_t g_frame_count = 0;

void display_update_task(void* pvParameters) {
    DisplayTaskConfig* config = (DisplayTaskConfig*)pvParameters;
    Hub75FastRenderer* renderer = config->renderer;
    Hub75Driver* driver = config->driver;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    //const TickType_t xFrequency = pdMS_TO_TICKS(16); // ~60 FPS
	const TickType_t xFrequency = pdMS_TO_TICKS(33); // ~30 FPS
    int frame = 0;

    while (true) {
        // 1. Logic & Drawing (Core 1)
        renderer->draw_gradient(frame); 
        
        // 2. Bit-packing (Core 1)
        renderer->render_planes();

        // 3. Buffer Swap (DMA is already running on hardware)
        driver->flip_buffer();

		// In display_update_task — increment:
		Atomic_Increment_u32(&g_frame_count);
		
		frame = (frame + 1) & 255;

        // 4. Timing
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}




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
    static uint32_t last_idle[2] = {0, 0};

	uint64_t now = esp_timer_get_time();
	if (now - last_time < 1000000ULL) return;  // ← interval check FIRST

	// Read and reset ONLY when we're actually going to log
	uint32_t fps = g_frame_count;
	g_frame_count = 0;                          // ← plain reset, no CompareAndSwap

    // -------------------------------------------------------
    // CPU usage (your existing logic)
    // -------------------------------------------------------
    TaskStatus_t tasks[16];
    uint32_t total_runtime;
    int n = uxTaskGetSystemState(tasks, 16, &total_runtime);

    uint32_t idle_runtime[2] = {0, 0};
    for (int i = 0; i < n; i++) {
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

    if (last_time == 0) { last_time = now; return; }

    uint64_t elapsed_us = now - last_time;
    float cpu0 = 100.0f - (100.0f * delta_idle[0] / elapsed_us);
    float cpu1 = 100.0f - (100.0f * delta_idle[1] / elapsed_us);

    // -------------------------------------------------------
    // Heap — most important for stability
    // Internal SRAM only (DMA buffers live here)
    // -------------------------------------------------------
    size_t heap_free        = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    size_t heap_min_ever    = heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL);
    size_t heap_largest     = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);

    // External PSRAM if present
    size_t psram_free       = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);

    // -------------------------------------------------------
    // Stack watermarks — catches stack overflows before crash
    // -------------------------------------------------------
    UBaseType_t display_stack_hwm = 0;
    UBaseType_t main_stack_hwm    = 0;
    for (int i = 0; i < n; i++) {
        if (strstr(tasks[i].pcTaskName, "DisplayTask"))
            display_stack_hwm = tasks[i].usStackHighWaterMark;
        else if (strstr(tasks[i].pcTaskName, "main"))
            main_stack_hwm = tasks[i].usStackHighWaterMark;
    }

    // -------------------------------------------------------
    // Uptime
    // -------------------------------------------------------
    uint32_t uptime_s = (uint32_t)(now / 1000000ULL);

    // -------------------------------------------------------
    // Log everything
    // -------------------------------------------------------
	ESP_LOGI(TAG2, "UP=%lus | FPS=%u | CPU0=%.1f%% CPU1=%.1f%%",
	         uptime_s, fps, cpu0, cpu1);

    ESP_LOGI(TAG2,
        "HEAP internal: free=%u  min_ever=%u  largest_block=%u",
        heap_free, heap_min_ever, heap_largest);

    if (psram_free > 0)
        ESP_LOGI(TAG2, "HEAP PSRAM: free=%u", psram_free);

    ESP_LOGI(TAG2,
        "STACK watermark: DisplayTask=%u words  main=%u words",
        display_stack_hwm, main_stack_hwm);

    // -------------------------------------------------------
    // Warnings — things that indicate impending problems
    // -------------------------------------------------------
    if (heap_free < 20000)
        ESP_LOGW(TAG2, "⚠ LOW HEAP: %u bytes free", heap_free);

    if (heap_largest < 8000)
        ESP_LOGW(TAG2, "⚠ HEAP FRAGMENTED: largest block only %u bytes", heap_largest);

    if (display_stack_hwm < 128)
        ESP_LOGW(TAG2, "⚠ DisplayTask STACK CLOSE TO OVERFLOW: %u words remaining", display_stack_hwm);

    if (main_stack_hwm < 128)
        ESP_LOGW(TAG2, "⚠ main STACK CLOSE TO OVERFLOW: %u words remaining", main_stack_hwm);

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
	
	// Prepare task configuration
	    static DisplayTaskConfig config2 = { &renderer, &driver };

	    // Create the task on Core 1 (APP_CPU)
	    xTaskCreatePinnedToCore(
	        display_update_task,   // Function to run
	        "DisplayTask",         // Name
	        8192,                  // Stack size (in bytes)
	        &config2,               // Parameters
	        10,                    // Priority (Higher = more important)
	        NULL,                  // Task handle
	        1                      // Core ID (1 = Core 1)
	    );

	    // Core 0 is now FREE! 
	    // You can start your Ethernet/W5500 or I2C tasks here.
	    while(true) {
	        log_system_stats(); // Log from Core 0 to see the new distribution
	        vTaskDelay(pdMS_TO_TICKS(1000));
	    }
}
