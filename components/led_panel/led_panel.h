#pragma once

#include <stdint.h>   // ✅ REQUIRED (works for C and C++)
#include "string.h"

#include "hub75.h"     // ✅ REQUIRED (defines Hub75Driver)


#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t r,g,b;
} rgb_t;

typedef struct
{
    Hub75Driver* driver;

    int width;
    int height;
	
	rgb_t* fb;   // shadow framebuffer

} led_panel_t;

typedef struct
{
    const char* text;

    float scroll_x;     // fractional position
    float velocity;     // pixels per frame

} marquee_t;

/* Public API (C compatible) */
led_panel_t* led_panel_init(void);
void led_panel_draw_char(led_panel_t* panel,
                         char c,
                         int x,
                         int y,
                         uint8_t r,
                         uint8_t g,
                         uint8_t b);

void led_panel_draw_string(led_panel_t* panel,
                           const char* str,
                           int x,
                           int y,
                           uint8_t r,
                           uint8_t g,
                           uint8_t b);

void led_panel_flip(led_panel_t* panel);

void led_panel_clear(led_panel_t* panel);

void led_panel_begin_frame(led_panel_t* panel);

void led_panel_end_frame(led_panel_t* panel);

void marquee_tick(led_panel_t* panel,
                  marquee_t* mq);
				  
void led_panel_set_pixel(led_panel_t* panel,
                           int x,int y,
                           uint8_t r,uint8_t g,uint8_t b);

#ifdef __cplusplus
}
#endif