#include "app.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_panel.h"

static led_panel_t* g_panel;


static void render_task(void *arg)
{
	marquee_t mq = {
	    .text = "HELLO WORLD   ",
	    .scroll_x = 0.0f,
	    .velocity = 0.22f   // ⭐ SPEED CONTROL
	};
	
	led_panel_begin_frame(g_panel);

	led_panel_set_pixel(g_panel, 10, 10, 255, 0, 0); // FORCE RED PIXEL

	led_panel_end_frame(g_panel);

	while(true)
	{
	    led_panel_begin_frame(g_panel);

	    marquee_tick(g_panel,&mq);		

	    led_panel_end_frame(g_panel);

	    vTaskDelay(pdMS_TO_TICKS(32)); // 60 FPS
	}
}

void App::run()
{
	g_panel = led_panel_init();
	
    xTaskCreatePinnedToCore(
        render_task,
        "render_task",
        4096,
        nullptr,
        5,
        nullptr,
        1   // ✅ CPU1 (application core)
    );
}

extern "C" void app_main(void)
{
    static App app;
    app.run();
	
	// VERY IMPORTANT:
	vTaskDelete(nullptr);   // delete main_task cleanly    
}






