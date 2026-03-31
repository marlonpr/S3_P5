#include "led_panel.h"
#include "hub75.h"
#include "font8x8_basic.h"


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

void led_panel_clear(led_panel_t* panel)
{
    if (!panel) return;

    panel->driver->clear();   // <- fast memset inside DMA buffer
}

void led_panel_begin_frame(led_panel_t* panel)
{
    panel->driver->clear();
}

void led_panel_end_frame(led_panel_t* panel)
{
    panel->driver->flip_buffer();
}


void led_panel_set_pixel(led_panel_t* panel,
                         int x,int y,
                         uint8_t r,uint8_t g,uint8_t b)
{
    if(x<0 || x>=panel->width ||
       y<0 || y>=panel->height)
        return;

    int i = y * panel->width + x;

    panel->fb[i] = {r,g,b};

    panel->driver->set_pixel(x,y,r,g,b);
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
    int draw_x = -(int)(mq->scroll_x);

    led_panel_draw_string(panel,
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
        mq->scroll_x = 0.0f;
}