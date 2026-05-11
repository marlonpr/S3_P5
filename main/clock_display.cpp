#include "clock_display.h"

#include <stdio.h>

#include "led_panel.h"
#include "logo.h"

// =============================== DATE TEXT ===============================

static const char *dias_semana[] = {
    "DOMINGO",
    "LUNES",
    "MARTES",
    "MIERCOLES",
    "JUEVES",
    "VIERNES",
    "SABADO"
};

static const char *meses[] = {
    "ENERO",
    "FEBRERO",
    "MARZO",
    "ABRIL",
    "MAYO",
    "JUNIO",
    "JULIO",
    "AGOSTO",
    "SEPTIEMBRE",
    "OCTUBRE",
    "NOVIEMBRE",
    "DICIEMBRE"
};

static int get_weekday_index(const ds3231_time_t *time)
{
    if (!time) {
        return 0;
    }

    if (time->day_of_week < 1 || time->day_of_week > 7) {
        return 0;
    }

    return time->day_of_week - 1;
}

static int get_month_index(const ds3231_time_t *time)
{
    if (!time) {
        return 0;
    }

    if (time->month < 1 || time->month > 12) {
        return 0;
    }

    return time->month - 1;
}

void clock_display_make_date_scroll_text(const ds3231_time_t *time,
                                         char *buf,
                                         size_t size)
{
    if (!time || !buf || size == 0) {
        return;
    }

    int weekday_index = get_weekday_index(time);
    int month_index = get_month_index(time);

    snprintf(buf,
             size,
             "%s %d %s %04d",
             dias_semana[weekday_index],
             time->day,
             meses[month_index],
             time->year);
}

// =============================== TEXT HELPERS ===============================

static int text_width_5x7(const char *text)
{
    if (!text) {
        return 0;
    }

    int len = 0;

    while (*text++) {
        len++;
    }

    return len * 6; // 5 pixels glyph + 1 pixel spacing
}

int clock_display_center_x_5x7(const char *text)
{
    int width = text_width_5x7(text);

    if (width >= 64) {
        return 0;
    }

    return (64 - width) / 2;
}

// =============================== CLOCK / TEMP HELPERS ===============================

static int get_display_hour(const ds3231_time_t *time, hour_format_t format)
{
    int hour = (int)time->hour;

    if (hour < 0) {
        hour = 0;
    }

    if (hour > 23) {
        hour = 23;
    }

    if (format == FORMAT_24H) {
        return hour;
    }

    int hour12 = hour % 12;

    if (hour12 == 0) {
        hour12 = 12;
    }

    return hour12;
}

static int safe_minute(const ds3231_time_t *time)
{
    int minute = (int)time->minute;

    if (minute < 0) {
        minute = 0;
    }

    if (minute > 59) {
        minute = 59;
    }

    return minute;
}

static int safe_second(const ds3231_time_t *time)
{
    int second = (int)time->second;

    if (second < 0) {
        second = 0;
    }

    if (second > 59) {
        second = 59;
    }

    return second;
}

static void get_temp_color(float temp_c, uint8_t *r, uint8_t *g, uint8_t *b)
{
    if (!r || !g || !b) {
        return;
    }

    if (temp_c < 10.0f) {
        *r = 255;
        *g = 255;
        *b = 255;
    } else if (temp_c < 20.0f) {
        *r = 0;
        *g = 255;
        *b = 255;
    } else if (temp_c < 30.0f) {
        *r = 255;
        *g = 65;
        *b = 0;
    } else {
        *r = 255;
        *g = 0;
        *b = 0;
    }
}

static void make_temp_text(char *buf, size_t size, float temp_c, bool temp_valid)
{
    if (!buf || size == 0) {
        return;
    }

    if (temp_valid) {
        int temp_int = (int)temp_c;
        snprintf(buf, size, "%d*C", temp_int);
    } else {
        snprintf(buf, size, "T E");
    }
}

// =============================== SCREENS ===============================

void clock_display_draw_mode_1(Hub75Driver *driver,
                               const ds3231_time_t *time,
                               float temp_c,
                               bool temp_valid,
                               hour_format_t format)
{
    if (!driver || !time) {
        return;
    }

    char line1[32];
    char line2[16];

    static char date_scroll_text[64];

    if (!scroll_is_active()) {
        clock_display_make_date_scroll_text(time,
                                            date_scroll_text,
                                            sizeof(date_scroll_text));

        scroll_start(date_scroll_text, 12, 0, 255, 0, 10);
    }

    scroll_update(*driver);

    int hour = get_display_hour(time, format);
    int minute = safe_minute(time);
    int second = safe_second(time);

    if (format == FORMAT_12H && hour < 10) {
        snprintf(line1, sizeof(line1),
                 " %1d:%02d:%02d",
                 hour,
                 minute,
                 second);
    } else {
        snprintf(line1, sizeof(line1),
                 "%02d:%02d:%02d",
                 hour,
                 minute,
                 second);
    }

    draw_string(*driver, 1, 1, line1, 255, 255, 255);

    uint8_t r_temp = 255;
    uint8_t g_temp = 255;
    uint8_t b_temp = 255;

    get_temp_color(temp_c, &r_temp, &g_temp, &b_temp);
    make_temp_text(line2, sizeof(line2), temp_c, temp_valid);

    if (temp_valid) {
        draw_string(*driver, 20, 22, line2, r_temp, g_temp, b_temp);
    } else {
        draw_string(*driver, 20, 22, "T E", 255, 0, 0);
    }
}

void clock_display_draw_mode_2(Hub75Driver *driver,
                               const ds3231_time_t *time,
                               hour_format_t format)
{
    if (!driver || !time) {
        return;
    }

    char line1[32];
    char line2[32];
    char line3[32];

    int weekday_index = get_weekday_index(time);

    snprintf(line1, sizeof(line1), "%s", dias_semana[weekday_index]);

    snprintf(line2,
             sizeof(line2),
             "%02d-%02d-%02d",
             time->day,
             time->month,
             time->year - 2000);

    int hour = get_display_hour(time, format);
    int minute = safe_minute(time);
    int second = safe_second(time);

    bool colon_on = (second % 2) == 0;

    if (format == FORMAT_12H && hour < 10) {
        snprintf(line3,
                 sizeof(line3),
                 colon_on ? " %1d:%02d" : " %1d %02d",
                 hour,
                 minute);
    } else {
        snprintf(line3,
                 sizeof(line3),
                 colon_on ? "%02d:%02d" : "%02d %02d",
                 hour,
                 minute);
    }

    draw_string(*driver, clock_display_center_x_5x7(line1), 1, line1, 0, 255, 0);
    draw_string(*driver, clock_display_center_x_5x7(line2), 11, line2, 0, 0, 255);
    draw_string(*driver, clock_display_center_x_5x7(line3), 22, line3, 255, 255, 255);
}

void clock_display_draw_mode_3(Hub75Driver *driver,
                               float temp_c,
                               bool temp_valid)
{
    if (!driver) {
        return;
    }

    char line1[32];
    char line2[16];

    uint8_t r_temp = 255;
    uint8_t g_temp = 255;
    uint8_t b_temp = 255;

    get_temp_color(temp_c, &r_temp, &g_temp, &b_temp);

    snprintf(line1, sizeof(line1), "TEMP");

    if (temp_valid) {
        make_temp_text(line2, sizeof(line2), temp_c, temp_valid);
    } else {
        snprintf(line2, sizeof(line2), "T E");
    }

    draw_string(*driver, clock_display_center_x_5x7(line1), 5, line1, 255, 255, 255);

    if (temp_valid) {
        draw_string(*driver, clock_display_center_x_5x7(line2), 18, line2, r_temp, g_temp, b_temp);
    } else {
        draw_string(*driver, clock_display_center_x_5x7(line2), 18, line2, 255, 0, 0);
    }
}

void clock_display_draw_logo(Hub75Driver *driver)
{
    if (!driver) {
        return;
    }

    draw_bitmap_rgb32(*driver,
                      0,
                      0,
                      logo_bitmap,
                      LOGO_WIDTH,
                      LOGO_HEIGHT);
}

void clock_display_draw_startup(Hub75Driver *driver,
                                int display_mode,
                                int brightness_level,
                                hour_format_t format)
{
    if (!driver) {
        return;
    }

    char line1[16];
    char line2[16];
    char line3[16];

    snprintf(line1, sizeof(line1), "MODO:%d", display_mode);
    snprintf(line2, sizeof(line2), "BRILLO:%d", brightness_level);
    snprintf(line3, sizeof(line3), "%s",
             format == FORMAT_24H ? "24HRS:ON" : "24HRS:OFF");

    draw_string(*driver, clock_display_center_x_5x7(line1), 1,  line1, 255, 0, 0);
    draw_string(*driver, clock_display_center_x_5x7(line2), 11, line2, 0, 255, 0);
    draw_string(*driver, clock_display_center_x_5x7(line3), 22, line3, 0, 255, 255);
}



//==== mode 1 test===
/*
if (format == FORMAT_12H) {
    if (time->hour >= 12) {
        draw_string(*driver, 51, 22, "PM", 255, 255, 255);
    } else {
        draw_string(*driver, 51, 22, "AM", 255, 255, 255);
    }
} else {
    draw_string(*driver, 51, 22, "24", 255, 255, 255);
}
*/
