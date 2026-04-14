// main.cpp
#include "hub75.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"

uint32_t g_frame_count = 0;

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
	g_frame_count = 0;                          

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
	// Replace the three heap_caps calls with:
	size_t heap_free     = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
	size_t heap_min_ever = heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL);

    // External PSRAM if present
    size_t psram_free       = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);

    // -------------------------------------------------------
    // Stack watermarks — catches stack overflows before crash
    // -------------------------------------------------------
    UBaseType_t display_stack_hwm = 0;
    //UBaseType_t main_stack_hwm    = 0;
    for (int i = 0; i < n; i++) 
	{
        if (strstr(tasks[i].pcTaskName, "DisplayTask"))
            display_stack_hwm = tasks[i].usStackHighWaterMark;
        //else if (strstr(tasks[i].pcTaskName, "main"))
            //main_stack_hwm = tasks[i].usStackHighWaterMark;
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

	 ESP_LOGI(TAG2, "HEAP internal: free=%u  min_ever=%u",
	          heap_free, heap_min_ever);

    if (psram_free > 0)
        ESP_LOGI(TAG2, "HEAP PSRAM: free=%u", psram_free);

    ESP_LOGI(TAG2,
        "STACK water mark: DisplayTask=%u words",//  main=%u words",
        display_stack_hwm);//, main_stack_hwm);

    // -------------------------------------------------------
    // Warnings — things that indicate impending problems
    // -------------------------------------------------------
    if (heap_free < 20000)
        ESP_LOGW(TAG2, "⚠ LOW HEAP: %u bytes free", heap_free);


    if (display_stack_hwm < 128)
        ESP_LOGW(TAG2, "⚠ DisplayTask STACK CLOSE TO OVERFLOW: %u words remaining", display_stack_hwm);

    //if (main_stack_hwm < 128)
        //ESP_LOGW(TAG2, "⚠ main STACK CLOSE TO OVERFLOW: %u words remaining", main_stack_hwm);

    last_time = now;
}

// Structure to pass multiple arguments to the task if needed
struct DisplayTaskConfig {
    Hub75Driver* driver;
};

void display_update_task(void* pvParameters) {
    Hub75Driver* driver = (Hub75Driver*)pvParameters;
	
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(33); // ~30 FPS

    // 1. Create a dedicated counter for your animation
    uint32_t anim_frame = 0;

    while (true) 
    {
	    driver->clear();  // Clear back buffer
		
        // 2. Use the animation counter, keeping it constrained between 0 and 63
        uint32_t x_pos = anim_frame; 
		
		driver->set_pixel(x_pos, 10, 255, 255, 0);  // Red
		driver->set_pixel(x_pos, 20, 0, 255, 255);  // Green
		driver->set_pixel(x_pos, 30, 255, 0, 255);  // Blue
		
	    driver->flip_buffer();  // Atomic swap
				
        // 3. Increment both counters
        g_frame_count++; // Let the logger reset this one for FPS tracking
		// This says: Increment, then immediately wrap the result to 0-63
		anim_frame = (anim_frame + 1) % 64;    // Let this one grow forever for smooth animations
		
        // Timing
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// Better practice: create it statically so it lives forever
static Hub75Config config = make_config();
static Hub75Driver driver(config);


extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Starting HUB75");

    if (!driver.begin())
    {
        ESP_LOGE(TAG, "Driver start failed");
        return;
    }
	
	// Create the task on Core 1 (APP_CPU)
	xTaskCreatePinnedToCore(
	    display_update_task,   // Function to run
	    "DisplayTask",         // Name
	    8192,                  // Stack size (in bytes)
	    &driver,               // Parameters
	    1,                    // Priority 
	    NULL,                  // Task handle
	    1                      // Core ID (1 = Core 1)
	);
	
	while(true) {
	    log_system_stats(); // Log from Core 0 to see the new distribution
	    vTaskDelay(pdMS_TO_TICKS(1000));
	}

}



// 2×2 grid — 128×64 virtual display
//config.layout_rows  = 1;
//config.layout_cols  = 4;

//config.layout = Hub75PanelLayout::HORIZONTAL;

//config.output_clock_speed = Hub75ClockSpeed::HZ_16M;   // Clock speed