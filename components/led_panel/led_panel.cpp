#include "led_panel.h"
#include "hub75.h"
#include "font8x8_basic.h"
#include <cmath>


/* -------------------------------------------------- */
/* CONFIG                                             */
/* -------------------------------------------------- */

static Hub75Config make_config()
{
    Hub75Config config{};

    config.panel_width  = 64;
    config.panel_height = 32;
    config.scan_wiring  = Hub75ScanWiring::SCAN_1_8_32PX_FULL;
    config.shift_driver = Hub75ShiftDriver::FM6126A;
    config.double_buffer = true;

    // Pins
    config.pins.r1 = 4;
    config.pins.g1 = 5;
    config.pins.b1 = 6;
    config.pins.r2 = 7;
    config.pins.g2 = 15;
    config.pins.b2 = 16;

    config.pins.a = 11;
    config.pins.b = 12;
    config.pins.c = 13;
    config.pins.d = 14;
    config.pins.e = -1;

    config.pins.lat = 9;
    config.pins.oe  = 10;
    config.pins.clk = 8;

    return config;
}

/* -------------------------------------------------- */
/* INIT                                               */
/* -------------------------------------------------- */

led_panel_t* led_panel_init(void)
{
    led_panel_t* panel = new led_panel_t;

    Hub75Config cfg = make_config();
	
	panel->width  = cfg.panel_width;
	panel->height = cfg.panel_height;
	
	panel->fb = (rgb_t*)calloc(panel->width * panel->height,
	                           sizeof(rgb_t));

	panel->driver = new Hub75Driver(cfg);
	panel->driver->begin();   // ✅ initialize DMA engine

    return panel;
}


static inline uint8_t scale8(uint8_t v, float k)
{
    return (uint8_t)((float)v * k);
}



/* -------------------------------------------------- */
/* DRAW CHAR                                          */
/* -------------------------------------------------- */

void led_panel_draw_char(led_panel_t* panel,
                         char c,
                         int x0,
                         int y0,
                         uint8_t r,
                         uint8_t g,
                         uint8_t b)
{
    if (!panel) return;
    if (c < 32 || c > 126) return;

    const uint8_t* glyph = font8x8_basic[c - 32];

    for (int row = 0; row < 8; row++)
    {
        uint8_t bits = glyph[row];

        for (int col = 0; col < 8; col++)
        {
            if (bits & (1 << (7 - col)))
            {
                panel->driver->set_pixel(
                    x0 + col,
                    y0 + row,
                    r, g, b
                );
            }
        }
    }
}

/* -------------------------------------------------- */
/* DRAW STRING                                        */
/* -------------------------------------------------- */

void led_panel_draw_string(led_panel_t* panel,
                           const char* str,
                           int x,
                           int y,
                           uint8_t r,
                           uint8_t g,
                           uint8_t b)
{
    if (!panel || !str) return;

    while (*str)
    {
        led_panel_draw_char(panel, *str, x, y, r, g, b);
        x += 8;
        str++;
    }
}

/* -------------------------------------------------- */

void led_panel_flip(led_panel_t* panel)
{
    if (!panel) return;
    panel->driver->flip_buffer();
}
static inline uint8_t add_sat(uint8_t a, uint8_t b)
{
    int v = a + b;
    return (v > 255) ? 255 : v;
}

void led_panel_add_pixel(led_panel_t* panel,
                         int x,int y,
                         uint8_t r,uint8_t g,uint8_t b)
{
    if(x<0 || x>=panel->width ||
       y<0 || y>=panel->height)
        return;

    int i = y * panel->width + x;

    rgb_t& p = panel->fb[i];

    p.r = add_sat(p.r, r);
    p.g = add_sat(p.g, g);
    p.b = add_sat(p.b, b);
}


void led_panel_flush(led_panel_t* panel)
{
    for(int y=0;y<panel->height;y++)
    for(int x=0;x<panel->width;x++)
    {
        rgb_t& p = panel->fb[y*panel->width+x];

        panel->driver->set_pixel(x,y,p.r,p.g,p.b);
    }
}





void led_panel_clear(led_panel_t* panel)
{
    memset(panel->fb, 0,
           panel->width*panel->height*sizeof(rgb_t));
}

void led_panel_begin_frame(led_panel_t* panel)
{
    led_panel_clear(panel);   // clear FB only
}

void led_panel_end_frame(led_panel_t* panel)
{
    led_panel_flush(panel);   // upload once
    panel->driver->flip_buffer();
}



void led_panel_shift_left(led_panel_t* panel)
{
    for(int y=0; y<panel->height; y++)
    {
        for(int x=0; x<panel->width-1; x++)
        {
            rgb_t p = panel->fb[y*panel->width + x + 1];
            led_panel_set_pixel(panel,x,y,p.r,p.g,p.b);
        }

        led_panel_set_pixel(panel,
                            panel->width-1,
                            y,
                            0,0,0);
    }
}

void led_panel_draw_char_column(
    led_panel_t* panel,
    char c,
    int column,
    int screen_x,
    int y,
    uint8_t r,uint8_t g,uint8_t b)
{
    if(c < 32 || c > 126) return;

    const uint8_t* glyph = font8x8_basic[c-32];

    for(int row=0; row<8; row++)
    {
        if(glyph[row] & (1 << (7-column)))
            led_panel_set_pixel(panel, screen_x, y+row, r,g,b);
    }
}



void led_panel_draw_char_fractional(
    led_panel_t* panel, char c, float x, int y,
    uint8_t r, uint8_t g, uint8_t b)
{
    if(c < 32 || c > 126) return;

    const uint8_t* glyph = font8x8_basic[c-32];

    int ix = (int)floorf(x);   // ✅ correct for negative x
    float frac = x - (float)ix; // now always in [0.0, 1.0)

    for(int col=0; col<8; col++)
    {
        int screen_x = ix + col;

        float w0 = 1.0f - frac;
        float w1 = frac;

        for(int row=0; row<8; row++)
        {
            if(glyph[row] & (1 << (7-col)))
            {
                // LEFT PIXEL
                led_panel_add_pixel(
                    panel,
                    screen_x,
                    y+row,
                    scale8(r,w0),
                    scale8(g,w0),
                    scale8(b,w0));

                // RIGHT PIXEL
                led_panel_add_pixel(
                    panel,
                    screen_x+1,
                    y+row,
                    scale8(r,w1),
                    scale8(g,w1),
                    scale8(b,w1));
            }
        }
    }
}

void led_panel_draw_string_fractional(
    led_panel_t* panel,
    const char* str,
    float x,
    int y,
    uint8_t r,uint8_t g,uint8_t b)
{
    while(*str)
    {
        led_panel_draw_char_fractional(
            panel,
            *str,
            x,
            y,
            r,g,b);

        x += 8.0f;
        str++;
    }
}








#define CHAR_WIDTH 8

void marquee_tick(led_panel_t* panel, marquee_t* mq)
{
    if (!panel || !mq || !mq->text) return;

    int text_len = strlen(mq->text);
    int text_width = text_len * CHAR_WIDTH;

    /* ---------------------------- */
    /* clear back buffer            */
    /* ---------------------------- */
    led_panel_clear(panel);

    /* ---------------------------- */
    /* draw text                    */
    /* ---------------------------- */
	float draw_x = -mq->scroll_x;

	led_panel_draw_string_fractional(
	    panel,
	    mq->text,
	    draw_x,
	    0,
	    255,0,0);

    /* ---------------------------- */
    /* fractional motion update     */
    /* ---------------------------- */
    mq->scroll_x += mq->velocity;

    /* ---------------------------- */
    /* wrap-around                  */
    /* ---------------------------- */
	if (mq->scroll_x >= text_width)
	    mq->scroll_x -= text_width;  // ✅ preserve the fractional overshoot
}

