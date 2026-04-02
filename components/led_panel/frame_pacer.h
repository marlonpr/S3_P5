#pragma once
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

typedef struct
{
    int64_t next_tick_us;
    int64_t period_us;

    int64_t last_tick_us;   // ⭐ previous frame timestamp
    float   dt;             // ⭐ delta time (seconds)
} frame_pacer_t;

static inline void frame_pacer_init(frame_pacer_t* p, float fps)
{
    p->period_us = (int64_t)(1000000.0f / fps);

    int64_t now = esp_timer_get_time();

    p->next_tick_us = now + p->period_us;
    p->last_tick_us = now;
    p->dt = 1.0f / fps;
}

static inline void frame_pacer_wait(frame_pacer_t* p)
{
    while (true)
    {
        int64_t now = esp_timer_get_time();
        int64_t diff = p->next_tick_us - now;

        if (diff <= 0)
            break;

        if (diff > 2000)
            vTaskDelay(1);
    }

    int64_t now = esp_timer_get_time();

    // ----- measure actual delta -----
    int64_t frame_us = now - p->last_tick_us;
    p->last_tick_us  = now;

    p->dt = frame_us * 1e-6f;

    // ----- drift-safe scheduling -----
    p->next_tick_us += p->period_us;

    // If we ever fall behind badly, resync
    if (now > p->next_tick_us)
        p->next_tick_us = now + p->period_us;
}

static inline float frame_pacer_dt(frame_pacer_t* p)
{
    return p->dt;
}