// led_panel.h
#include "hub75.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "draw_text.h"
#include "draw_bitmap.h"

#include "logo.h"



static Hub75Config make_config()
{
    Hub75Config config{};

    config.panel_width  = 64;
    config.panel_height = 32;	

    config.scan_wiring = Hub75ScanWiring::SCAN_1_8_32PX_FULL;
    config.shift_driver = Hub75ShiftDriver::FM6126A;
	
    config.double_buffer = true;	
	
	// N×M grid — virtual display
	config.layout_rows  = 1;
	config.layout_cols  = 1;

	config.layout = Hub75PanelLayout::TOP_LEFT_DOWN_ZIGZAG;
	

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



// Structure to pass multiple arguments to the task if needed
struct DisplayTaskConfig {
    Hub75Driver* driver;
};
