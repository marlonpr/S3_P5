#pragma once
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

typedef struct
{
    int64_t next_tick_us;
    int64_t period_us;
} frame_pacer_t;

static inline void frame_pacer_init(frame_pacer_t* p, float fps)
{
    p->period_us = (int64_t)(1000000.0f / fps);
    p->next_tick_us = esp_timer_get_time() + p->period_us;
}

static inline void frame_pacer_wait(frame_pacer_t* p)
{
    while (true)
    {
        int64_t now = esp_timer_get_time();
        int64_t diff = p->next_tick_us - now;

        if (diff <= 0)
            break;

        // sleep lightly (no busy wait)
        if (diff > 2000)
            vTaskDelay(1);
    }

    p->next_tick_us += p->period_us;
}