#include "app.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_panel.h"
#include "frame_pacer.h"

static led_panel_t* g_panel;


static void render_task(void *arg)
{
    marquee_t mq = {
        .text = "HELLO WORLD   ",
        .scroll_x = 0.0f,
        .velocity = 0.35f   // pixels per frame @60Hz
    };

    frame_pacer_t pacer;
    frame_pacer_init(&pacer, 60.0f);   // ⭐ HARD LOCK 60 FPS

    while(true)
    {
        //------------------------------------------------
        // VSYNC LOCK (precise frame boundary)
        //------------------------------------------------
        frame_pacer_wait(&pacer);

        //------------------------------------------------
        // BEGIN FRAME
        //------------------------------------------------
        led_panel_begin_frame(g_panel);

        marquee_tick(g_panel, &mq);

        //------------------------------------------------
        // PRESENT (acts like VSYNC swap)
        //------------------------------------------------
        led_panel_end_frame(g_panel);
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






