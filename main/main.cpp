#include <stdio.h>
#include "hub75.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "ds3231.h"
#include "led_panel.h"
#include "ds18b20.h"

#include "clock_settings.h"

#include "freertos/queue.h"
#include "driver/gpio.h"


#define DS18B20_GPIO GPIO_NUM_39
#define PIN_MENU GPIO_NUM_40
#define PIN_UP   GPIO_NUM_41
#define PIN_DOWN GPIO_NUM_42

#define BUTTON_HOLD_MS     1000
#define BUTTON_DEBOUNCE_MS 250

#define BTN_NONE ((button_t)-1)

#define MENU_TIMEOUT_US (10 * 1000000) // 10 seconds

// =============================== MENU STATE ===============================

static int64_t menu_last_action_us = 0;


typedef enum {
    MENU_IDLE = 0,
    MENU_BRIGHTNESS,
    MENU_HOUR,
    MENU_MINUTE,
    MENU_DAY,
    MENU_MONTH,
    MENU_YEAR,
} menu_state_t;

static bool menu_active = false;

static menu_state_t menu_state = MENU_IDLE;

static ds3231_time_t tmp_time = {
    .second = 0,
    .minute = 0,
    .hour = 0,
    .day_of_week = 1,
    .day = 1,
    .month = 1,
    .year = 2000,
};



// =============================== BUTTONS ===============================

typedef enum {
    BTN_MENU = 0,
    BTN_UP,
    BTN_DOWN
} button_t;

static QueueHandle_t button_queue = NULL;






static const char* TAG = "MAIN";

// =============================== SHARED DATA ===============================

static char g_message[32] = {0};
static bool g_message_active = false;
static int64_t g_message_until_us = 0;


static int brightness_level = 5;        // 1 to 10
static int temporal_brightness = 5;     // temporary menu value while editing

static uint8_t brightness_level_to_hub75(int level)
{
    if (level < 1) {
        level = 1;
    }

    if (level > 10) {
        level = 10;
    }

    return (uint8_t)((level * 255) / 10);
}





static ds3231_time_t g_now = {
    .second = 0,
    .minute = 0,
    .hour = 0,
    .day_of_week = 1,
    .day = 1,
    .month = 1,
    .year = 2000,
};
static float g_temp_c = 0.0f;
static bool g_rtc_valid = false;
static bool g_temp_valid = false;

static portMUX_TYPE g_data_mux = portMUX_INITIALIZER_UNLOCKED;



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


static void make_date_scroll_text(const ds3231_time_t *time,
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

// =============================== CLOCK / TEMP HELPERS ===============================

typedef enum {
    FORMAT_12H = 0,
    FORMAT_24H = 1,
} hour_format_t;

static hour_format_t clock_format = FORMAT_12H;

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
        *b = 255;      // white
    } else if (temp_c < 20.0f) {
        *r = 0;
        *g = 255;
        *b = 255;      // cyan
    } else if (temp_c < 30.0f) {
        *r = 255;
        *g = 65;
        *b = 0;        // orange
    } else {
        *r = 255;
        *g = 0;
        *b = 0;        // red
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


// =============================== DISPLAY MODES ===============================

typedef enum {
    MODE_1 = 1,
    MODE_2,
    MODE_3,
    MODE_ROTATION,
} display_mode_t;

static display_mode_t display_mode = MODE_1;


// =============================== ROTATION STATE ===============================

static display_mode_t rotation_screen = MODE_1;
static int64_t rotation_last_change_us = 0;
static const int64_t ROTATION_INTERVAL_US = 10 * 1000000; // 10 seconds




static void show_temp_message(const char *msg, uint32_t duration_ms)
{
    if (!msg) {
        return;
    }

    portENTER_CRITICAL(&g_data_mux);

    snprintf(g_message, sizeof(g_message), "%s", msg);
    g_message_active = true;
    g_message_until_us = esp_timer_get_time() + ((int64_t)duration_ms * 1000);

    portEXIT_CRITICAL(&g_data_mux);
}

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

static int center_x_5x7(const char *text)
{
    int width = text_width_5x7(text);

    if (width >= 64) {
        return 0;
    }

    return (64 - width) / 2;
}

static void draw_mode_1_screen(Hub75Driver *driver,
                               const ds3231_time_t *time,
                               float temp_c,
                               bool temp_valid,
                               hour_format_t format)
{
    char line1[32];
    char line2[16];

    static char date_scroll_text[64];

    if (!scroll_is_active()) {
        make_date_scroll_text(time,
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

    draw_string(*driver, 8, 2, line1, 0, 255, 255);

    uint8_t r_temp = 255;
    uint8_t g_temp = 255;
    uint8_t b_temp = 255;

    get_temp_color(temp_c, &r_temp, &g_temp, &b_temp);
    make_temp_text(line2, sizeof(line2), temp_c, temp_valid);

    if (temp_valid) {
        draw_string(*driver, 2, 22, line2, r_temp, g_temp, b_temp);
    } else {
        draw_string(*driver, 2, 22, "T E", 255, 0, 0);
    }

    if (format == FORMAT_12H) {
        if (time->hour >= 12) {
            draw_string(*driver, 51, 22, "PM", 255, 255, 255);
        } else {
            draw_string(*driver, 51, 22, "AM", 255, 255, 255);
        }
    } else {
        draw_string(*driver, 51, 22, "24", 255, 255, 255);
    }
}

static void draw_mode_2_screen(Hub75Driver *driver,
                               const ds3231_time_t *time,
                               hour_format_t format)
{
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

    draw_string(*driver, center_x_5x7(line1), 1, line1, 0, 255, 0);
    draw_string(*driver, center_x_5x7(line2), 11, line2, 0, 0, 255);
    draw_string(*driver, center_x_5x7(line3), 22, line3, 255, 255, 255);
}

static void draw_mode_3_screen(Hub75Driver *driver,
                               float temp_c,
                               bool temp_valid)
{
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

    draw_string(*driver, center_x_5x7(line1), 5, line1, 255, 255, 255);

    if (temp_valid) {
        draw_string(*driver, center_x_5x7(line2), 18, line2, r_temp, g_temp, b_temp);
    } else {
        draw_string(*driver, center_x_5x7(line2), 18, line2, 255, 0, 0);
    }
}

static display_mode_t get_rotation_screen(void)
{
    int64_t now_us = esp_timer_get_time();

    if (rotation_last_change_us == 0) {
        rotation_last_change_us = now_us;
        rotation_screen = MODE_1;
        return rotation_screen;
    }

    if ((now_us - rotation_last_change_us) >= ROTATION_INTERVAL_US) {
        rotation_last_change_us = now_us;

        if (rotation_screen == MODE_1) {
            rotation_screen = MODE_2;
        } else if (rotation_screen == MODE_2) {
            rotation_screen = MODE_3;
        } else {
            rotation_screen = MODE_1;
        }

        scroll_stop();
    }

    return rotation_screen;
}


static int calculate_weekday(int day, int month, int year)
{
    if (month < 3) {
        month += 12;
        year--;
    }

    int K = year % 100;
    int J = year / 100;

    int h = (day + 13 * (month + 1) / 5 + K + K / 4 + J / 4 + 5 * J) % 7;

    /*
     * Zeller:
     * 0 = Saturday
     * 1 = Sunday
     * ...
     * 6 = Friday
     *
     * DS3231:
     * 1 = Sunday
     * ...
     * 7 = Saturday
     */
    return ((h + 6) % 7) + 1;
}

static void update_tmp_weekday(void)
{
    tmp_time.day_of_week = calculate_weekday(
        tmp_time.day,
        tmp_time.month,
        tmp_time.year
    );
}



static void refresh_menu_timeout(void)
{
    menu_last_action_us = esp_timer_get_time();
}

static void check_menu_timeout(void)
{
    if (!menu_active) {
        return;
    }

    int64_t now_us = esp_timer_get_time();

    if ((now_us - menu_last_action_us) > MENU_TIMEOUT_US) {
        menu_active = false;
        menu_state = MENU_IDLE;

        scroll_stop();
        show_temp_message("SALIR", 1000);

        ESP_LOGI(TAG, "Menu timeout -> exit without saving");
    }
}




static void enter_menu(ds3231_dev_t *rtc)
{
    if (!rtc) {
        return;
    }

    if (ds3231_get_time(rtc, &tmp_time) != ESP_OK) {
        ESP_LOGE(TAG, "Cannot enter menu: failed to read RTC");
        return;
    }

    menu_active = true;
	temporal_brightness = brightness_level;
	menu_state = MENU_BRIGHTNESS;

    refresh_menu_timeout();

    scroll_stop();

    show_temp_message("MENU", 500);

    ESP_LOGI(TAG, "Menu entered");
}

static void exit_menu(void)
{
    menu_active = false;
    menu_state = MENU_IDLE;

    scroll_stop();

    ESP_LOGI(TAG, "Menu exited");
}


static void draw_menu_screen(Hub75Driver *driver)
{
    char buf[32];

    switch (menu_state)
    {
		case MENU_BRIGHTNESS:
		    snprintf(buf, sizeof(buf), "BRILLO:%d", temporal_brightness);
		    break;
			
        case MENU_HOUR:
            snprintf(buf, sizeof(buf), "HORA:%02d", tmp_time.hour);
            break;

        case MENU_MINUTE:
            snprintf(buf, sizeof(buf), "MIN:%02d", tmp_time.minute);
            break;

        case MENU_DAY:
            snprintf(buf, sizeof(buf), "DIA:%02d", tmp_time.day);
            break;

        case MENU_MONTH:
            snprintf(buf, sizeof(buf), "MES:%02d", tmp_time.month);
            break;

        case MENU_YEAR:
            snprintf(buf, sizeof(buf), "ANO:%02d", tmp_time.year - 2000);
            break;

        default:
            snprintf(buf, sizeof(buf), "MENU");
            break;
    }

    draw_string(*driver, center_x_5x7(buf), 8, buf, 255, 0, 0);
}








// =============================== DISPLAY TASK ===============================

void display_update_task(void* pvParameters)
{
    Hub75Driver* driver = (Hub75Driver*)pvParameters;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(33); // ~30 FPS
	
	display_mode_t mode_copy;
	char message_copy[32];
	bool message_active_copy;
	
	bool menu_active_copy;

    while (true) {
		check_menu_timeout();
		
		ds3231_time_t now_copy;
		float temp_copy;
		bool rtc_valid_copy;
		bool temp_valid_copy;
		hour_format_t format_copy;

		portENTER_CRITICAL(&g_data_mux);

		now_copy = g_now;
		temp_copy = g_temp_c;
		rtc_valid_copy = g_rtc_valid;
		temp_valid_copy = g_temp_valid;
		format_copy = clock_format;
		mode_copy = display_mode;
		
		menu_active_copy = menu_active;

		message_active_copy = g_message_active;
		snprintf(message_copy, sizeof(message_copy), "%s", g_message);

		if (g_message_active && esp_timer_get_time() > g_message_until_us) {
		    g_message_active = false;
		    message_active_copy = false;
		}

		portEXIT_CRITICAL(&g_data_mux);

		driver->clear();

		if (message_active_copy) {
		    draw_string(*driver, 8, 8, message_copy, 255, 0, 0);
		    driver->flip_buffer();

		    vTaskDelayUntil(&xLastWakeTime, xFrequency);
		    continue;
		}
		
		if (menu_active_copy) {
		    scroll_stop();
		    draw_menu_screen(driver);
		    driver->flip_buffer();

		    vTaskDelayUntil(&xLastWakeTime, xFrequency);
		    continue;
		}

		if (rtc_valid_copy) {
		    switch (mode_copy) {
		        case MODE_1:
		            draw_mode_1_screen(driver,
		                               &now_copy,
		                               temp_copy,
		                               temp_valid_copy,
		                               format_copy);
		            break;

		        case MODE_2:
		            scroll_stop();
		            draw_mode_2_screen(driver,
		                               &now_copy,
		                               format_copy);
		            break;

		        case MODE_3:
		            scroll_stop();
		            draw_mode_3_screen(driver,
		                               temp_copy,
		                               temp_valid_copy);
		            break;

					case MODE_ROTATION:
					{
					    display_mode_t active_rotation_screen = get_rotation_screen();

					    switch (active_rotation_screen) {
					        case MODE_1:
					            draw_mode_1_screen(driver,
					                               &now_copy,
					                               temp_copy,
					                               temp_valid_copy,
					                               format_copy);
					            break;

					        case MODE_2:
					            scroll_stop();
					            draw_mode_2_screen(driver,
					                               &now_copy,
					                               format_copy);
					            break;

					        case MODE_3:
					        default:
					            scroll_stop();
					            draw_mode_3_screen(driver,
					                               temp_copy,
					                               temp_valid_copy);
					            break;
					    }

					    break;
					}

					default:
					    draw_mode_1_screen(driver,
					                       &now_copy,
					                       temp_copy,
					                       temp_valid_copy,
					                       format_copy);
					    break;
		    }
		} else {
		    scroll_stop();
		    draw_string(*driver, 2, 2, "NO RTC", 255, 0, 0);
		}

		driver->flip_buffer();

		vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// =============================== RTC TASK ===============================

void rtc_task(void* pvParameters)
{
    ds3231_dev_t* rtc = (ds3231_dev_t*)pvParameters;

    while (true) {
        ds3231_time_t now;

        if (ds3231_get_time(rtc, &now) == ESP_OK) {
            portENTER_CRITICAL(&g_data_mux);
            g_now = now;
            g_rtc_valid = true;
            portEXIT_CRITICAL(&g_data_mux);
        } else {
            ESP_LOGE(TAG, "Failed to read DS3231");
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// =============================== DS18B20 TASK ===============================

void ds18b20_task(void* pvParameters)
{
    ds18b20_t* sensor = (ds18b20_t*)pvParameters;

    while (true) {
        float temp_c = 0.0f;

        esp_err_t ret = ds18b20_read_temperature(sensor, &temp_c);

        if (ret == ESP_OK) {
            portENTER_CRITICAL(&g_data_mux);
            g_temp_c = temp_c;
            g_temp_valid = true;
            portEXIT_CRITICAL(&g_data_mux);			
        } else if (ret == ESP_ERR_NOT_FOUND) {
            portENTER_CRITICAL(&g_data_mux);
            g_temp_valid = false;
            portEXIT_CRITICAL(&g_data_mux);

            ESP_LOGW(TAG, "DS18B20 not connected");
        } else {
            ESP_LOGE(TAG, "DS18B20 read failed: %s", esp_err_to_name(ret));
        }

        /*
         * ds18b20_read_temperature() already waits 750 ms.
         * 4250 ms + 750 ms = about 5000 ms total period.
         */
        vTaskDelay(pdMS_TO_TICKS(4250));
    }
}

// =============================== HUB75 GLOBAL OBJECTS ===============================

static Hub75Config config = make_config();
static Hub75Driver driver(config);





static void IRAM_ATTR button_isr_handler(void *arg)
{
    button_t btn = (button_t)(uintptr_t)arg;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (button_queue != NULL) {
        xQueueSendFromISR(button_queue, &btn, &xHigherPriorityTaskWoken);
    }

    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}


static void init_buttons(void)
{
    gpio_config_t io_conf = {};

    io_conf.pin_bit_mask =
        (1ULL << PIN_MENU) |
        (1ULL << PIN_UP)   |
        (1ULL << PIN_DOWN);

    io_conf.mode = GPIO_MODE_INPUT;

    /*
     * Use GPIO_PULLUP_DISABLE only if you have external pull-up resistors.
     * Button wiring:
     *
     * GPIO ---- button ---- GND
     * GPIO ---- 10k pull-up ---- 3.3V
     */
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;

    io_conf.intr_type = GPIO_INTR_NEGEDGE; // falling edge when button pressed

    ESP_ERROR_CHECK(gpio_config(&io_conf));

    button_queue = xQueueCreate(10, sizeof(button_t));

    if (button_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create button queue");
        return;
    }

    esp_err_t ret = gpio_install_isr_service(0);

    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "gpio_install_isr_service failed: %s", esp_err_to_name(ret));
        return;
    }

    ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_MENU,
                                         button_isr_handler,
                                         (void *)(uintptr_t)BTN_MENU));

    ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_UP,
                                         button_isr_handler,
                                         (void *)(uintptr_t)BTN_UP));

    ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_DOWN,
                                         button_isr_handler,
                                         (void *)(uintptr_t)BTN_DOWN));

    ESP_LOGI(TAG, "Buttons initialized: MENU=%d UP=%d DOWN=%d",
             PIN_MENU,
             PIN_UP,
             PIN_DOWN);
}








static void save_menu_values(ds3231_dev_t *rtc)
{
    if (!rtc) {
        return;
    }

    tmp_time.second = 0;
    update_tmp_weekday();

    esp_err_t ret = ds3231_set_time(rtc, &tmp_time);

	if (ret == ESP_OK) {
	    portENTER_CRITICAL(&g_data_mux);
	    g_now = tmp_time;
	    g_rtc_valid = true;
	    portEXIT_CRITICAL(&g_data_mux);

	    brightness_level = temporal_brightness;
	    driver.set_brightness(brightness_level_to_hub75(brightness_level));
	    clock_settings_save_brightness((uint8_t)brightness_level);

	    show_temp_message("GUARDADO", 1000);
	    ESP_LOGI(TAG, "RTC time and brightness saved");
	} else {
        show_temp_message("ERROR", 1000);
        ESP_LOGE(TAG, "Failed to save RTC time: %s", esp_err_to_name(ret));
    }

    exit_menu();
}





static void handle_menu_button(button_t btn, ds3231_dev_t *rtc)
{    
	refresh_menu_timeout();
	
	switch (menu_state)
    {
		case MENU_BRIGHTNESS:
		{
		    if (btn == BTN_UP && temporal_brightness < 10) {
		        temporal_brightness++;
		        driver.set_brightness(brightness_level_to_hub75(temporal_brightness));
		    }

		    if (btn == BTN_DOWN && temporal_brightness > 1) {
		        temporal_brightness--;
		        driver.set_brightness(brightness_level_to_hub75(temporal_brightness));
		    }

		    if (btn == BTN_MENU) {
		        menu_state = MENU_HOUR;
		    }

		    break;
		}
		
        case MENU_HOUR:
            if (btn == BTN_UP) {
                tmp_time.hour = (tmp_time.hour + 1) % 24;
            }

            if (btn == BTN_DOWN) {
                tmp_time.hour = (tmp_time.hour + 23) % 24;
            }

            if (btn == BTN_MENU) {
                menu_state = MENU_MINUTE;
            }

            break;

        case MENU_MINUTE:
            if (btn == BTN_UP) {
                tmp_time.minute = (tmp_time.minute + 1) % 60;
            }

            if (btn == BTN_DOWN) {
                tmp_time.minute = (tmp_time.minute + 59) % 60;
            }

            if (btn == BTN_MENU) {
                menu_state = MENU_DAY;
            }

            break;

        case MENU_DAY:
            if (btn == BTN_UP) {
                tmp_time.day = (tmp_time.day % 31) + 1;
            }

            if (btn == BTN_DOWN) {
                tmp_time.day = ((tmp_time.day + 29) % 31) + 1;
            }

            update_tmp_weekday();

            if (btn == BTN_MENU) {
                menu_state = MENU_MONTH;
            }

            break;

        case MENU_MONTH:
            if (btn == BTN_UP) {
                tmp_time.month = (tmp_time.month % 12) + 1;
            }

            if (btn == BTN_DOWN) {
                tmp_time.month = ((tmp_time.month + 10) % 12) + 1;
            }

            update_tmp_weekday();

            if (btn == BTN_MENU) {
                menu_state = MENU_YEAR;
            }

            break;

        case MENU_YEAR:
            if (btn == BTN_UP) {
                tmp_time.year = 2000 + ((tmp_time.year - 2000 + 1) % 100);
            }

            if (btn == BTN_DOWN) {
                tmp_time.year = 2000 + ((tmp_time.year - 2000 + 99) % 100);
            }

            update_tmp_weekday();

            if (btn == BTN_MENU) {
                save_menu_values(rtc);
            }

            break;

        default:
            break;
    }
}

static int button_pin(button_t btn)
{
    switch (btn)
    {
        case BTN_MENU:
            return PIN_MENU;

        case BTN_UP:
            return PIN_UP;

        case BTN_DOWN:
            return PIN_DOWN;

        default:
            return -1;
    }
}

static bool button_is_pressed(button_t btn)
{
    int pin = button_pin(btn);

    if (pin < 0) {
        return false;
    }

    /*
     * Your buttons use falling edge and external pull-ups:
     * released = HIGH
     * pressed  = LOW
     */
    return gpio_get_level((gpio_num_t)pin) == 0;
}

static bool all_buttons_released(void)
{
    return gpio_get_level(PIN_MENU) &&
           gpio_get_level(PIN_UP)   &&
           gpio_get_level(PIN_DOWN);
}

static void handle_normal_button(button_t btn, ds3231_dev_t *rtc)
{
    switch (btn)
    {
        case BTN_MENU:
            enter_menu(rtc);
            break;

        case BTN_UP:
        {
            display_mode_t new_mode;

            portENTER_CRITICAL(&g_data_mux);

            if (display_mode >= MODE_ROTATION) {
                display_mode = MODE_1;
            } else {
                display_mode = (display_mode_t)(display_mode + 1);
            }

            new_mode = display_mode;

            rotation_last_change_us = 0;
            rotation_screen = MODE_1;

            portEXIT_CRITICAL(&g_data_mux);

            clock_settings_save_mode((uint8_t)new_mode);

            scroll_stop();

            char msg[16];
            snprintf(msg, sizeof(msg), "MODO:%d", new_mode);
            show_temp_message(msg, 1000);

            ESP_LOGI(TAG, "Display mode changed to %d", new_mode);

            break;
        }

        case BTN_DOWN:
        {
            hour_format_t new_format;

            portENTER_CRITICAL(&g_data_mux);

            if (clock_format == FORMAT_12H) {
                clock_format = FORMAT_24H;
            } else {
                clock_format = FORMAT_12H;
            }

            new_format = clock_format;

            portEXIT_CRITICAL(&g_data_mux);

            clock_settings_save_format((uint8_t)new_format);

            show_temp_message(new_format == FORMAT_24H ? "24HRS:ON" : "24HRS:OFF",
                              1000);

            ESP_LOGI(TAG,
                     "Clock format changed to %s",
                     new_format == FORMAT_24H ? "24H" : "12H");

            break;
        }

        default:
            break;
    }
}




void button_task(void *arg)
{
    ds3231_dev_t *rtc = (ds3231_dev_t *)arg;

    TickType_t last_press_time[3] = {
        0,
        0,
        0
    };

    button_t pending_hold_btn = BTN_NONE;
    TickType_t pending_hold_start = 0;

    bool ignore_until_release = false;

    while (true)
    {
        button_t btn;
        TickType_t now = xTaskGetTickCount();

        /*
         * Use short timeout instead of portMAX_DELAY so the task can check
         * whether a pending button has been held long enough.
         */
		 if (xQueueReceive(button_queue, &btn, pdMS_TO_TICKS(10)))
		 {
		     if (btn < BTN_MENU || btn > BTN_DOWN) {
		         continue;
		     }

		     /*
		      * Important:
		      * After a hold action outside the menu, ignore any queued/bounce events
		      * until all buttons are released.
		      */
		     if (ignore_until_release) {
		         continue;
		     }

		     if ((now - last_press_time[btn]) < pdMS_TO_TICKS(BUTTON_DEBOUNCE_MS)) {
		         continue;
		     }

		     last_press_time[btn] = now;

		     /*
		      * Inside menu:
		      * Buttons act immediately.
		      */
		     if (menu_active)
		     {
		         handle_menu_button(btn, rtc);
		         continue;
		     }

		     /*
		      * Outside menu:
		      * Do not execute immediately.
		      * Start hold detection.
		      */
		     pending_hold_btn = btn;
		     pending_hold_start = now;

		     ESP_LOGI(TAG, "Button %d pressed, waiting for hold", btn);
		 }

        /*
         * Outside menu only:
         * Execute action after the button stays pressed for BUTTON_HOLD_MS.
         */
        if (!menu_active &&
            pending_hold_btn != BTN_NONE &&
            !ignore_until_release)
        {
            if (button_is_pressed(pending_hold_btn))
            {
                now = xTaskGetTickCount();

				if ((now - pending_hold_start) >= pdMS_TO_TICKS(BUTTON_HOLD_MS))
				{
				    ESP_LOGI(TAG, "Button %d hold accepted", pending_hold_btn);

				    handle_normal_button(pending_hold_btn, rtc);

				    /*
				     * Remove any queued bounce/repeat events generated during the hold.
				     */
				    xQueueReset(button_queue);

				    /*
				     * Prevent repeated triggers while the button remains held.
				     * Also prevents MENU from immediately advancing from BRILLO to HORA.
				     */
				    pending_hold_btn = BTN_NONE;
				    ignore_until_release = true;
				}
            }
            else
            {
                /*
                 * Button was released before hold time.
                 * Cancel action.
                 */
                ESP_LOGI(TAG, "Button hold cancelled");

                pending_hold_btn = BTN_NONE;
            }
        }

        /*
         * Re-arm buttons only after all are released.
         */
        if (all_buttons_released())
        {
            pending_hold_btn = BTN_NONE;
            ignore_until_release = false;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}








// =============================== APP MAIN ===============================

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Starting HUB75");

    if (!driver.begin()) {
        ESP_LOGE(TAG, "Driver start failed");
        return;
    }
	
	ESP_ERROR_CHECK(clock_settings_init());

	uint8_t saved_format = clock_settings_load_format((uint8_t)FORMAT_12H);
	if (saved_format > FORMAT_24H) {
	    saved_format = FORMAT_12H;
	}
	clock_format = (hour_format_t)saved_format;

	uint8_t saved_mode = clock_settings_load_mode((uint8_t)MODE_1);
	if (saved_mode < MODE_1 || saved_mode > MODE_ROTATION) {
	    saved_mode = MODE_1;
	}
	display_mode = (display_mode_t)saved_mode;

	brightness_level = clock_settings_load_brightness(5);

	if (brightness_level < 1) {
	    brightness_level = 1;
	}

	if (brightness_level > 10) {
	    brightness_level = 10;
	}

	temporal_brightness = brightness_level;

	driver.set_brightness(brightness_level_to_hub75(brightness_level));
	
	
	
	
	
	
	

    static ds18b20_t ambient_sensor;
    static ds3231_dev_t rtc;

    ESP_ERROR_CHECK(ds18b20_init(&ambient_sensor, DS18B20_GPIO));
    ESP_ERROR_CHECK(init_ds3231(&rtc));
	
	init_buttons();
	
	xTaskCreatePinnedToCore(
	    button_task,
	    "ButtonTask",
	    4096,
	    &rtc,
	    2,
	    NULL,
	    0
	);

    xTaskCreatePinnedToCore(
        display_update_task,
        "DisplayTask",
        8192,
        &driver,
        2,
        NULL,
        1
    );

    xTaskCreatePinnedToCore(
        rtc_task,
        "RtcTask",
        4096,
        &rtc,
        1,
        NULL,
        0
    );

    xTaskCreatePinnedToCore(
        ds18b20_task,
        "DS18B20Task",
        4096,
        &ambient_sensor,
        1,
        NULL,
        0
    );
}