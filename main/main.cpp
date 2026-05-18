#include <stdio.h>
#include "hub75.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "ds3231.h"
#include "led_panel.h"
#include "ds18b20.h"

#include "clock_display.h"
#include "clock_settings.h"
#include "clock_buttons.h"
#include "clock_menu.h"
#include "clock_ethernet.h"

#include "nvs.h"

#define DS18B20_GPIO GPIO_NUM_39
#define PIN_MENU GPIO_NUM_40
#define PIN_UP   GPIO_NUM_41
#define PIN_DOWN GPIO_NUM_42

#define BUTTON_HOLD_MS     1000
#define BUTTON_DEBOUNCE_MS 500
#define BUTTON_REPEAT_DELAY_MS 500
#define BUTTON_REPEAT_RATE_MS  500

static const char* TAG = "MAIN";

static const int64_t ROTATION_LOGO_INTERVAL_US = 3 * 1000000;   // 3 seconds
static const int64_t ROTATION_MODE_INTERVAL_US = 21 * 1000000;  // 10 seconds

static const int64_t FIXED_LOGO_INTERVAL_US = 3 * 1000000;   // 3 seconds
static const int64_t FIXED_MODE_INTERVAL_US = 21 * 1000000;  // 10 seconds


// =============================== HUB75 GLOBAL OBJECTS ===============================

static Hub75Config config = make_config();
static Hub75Driver driver(config);

//================================ GLOBALS =======================================
static bool g_startup_screen_active = true;
static int64_t g_startup_screen_until_us = 0;

static bool g_logo_screen_active = true;
static int64_t g_logo_screen_until_us = 0;

// =============================== SHARED DATA ===============================

static char g_message[32] = {0};
static bool g_message_active = false;
static int64_t g_message_until_us = 0;


static int brightness_level = 5;        // 1 to 10
static int temporal_brightness = 5;     // temporary menu value while editing

// =============================== ETHERNET DATA ===============================

static int g_eth_brightness_level = 5;
static bool g_eth_brightness_pending = false;

static bool g_eth_format_pending = false;
static hour_format_t g_eth_format = FORMAT_12H;

static bool g_eth_time_pending = false;
static ds3231_time_t g_eth_time = {};


static bool g_eth_alarms_dirty = false;
static int64_t g_eth_alarms_dirty_until_us = 0;




static bool g_clear_alarms_on_next_ca = false;


#define DEFAULT_BRIGHTNESS_LEVEL  5
#define DEFAULT_CLOCK_FORMAT      FORMAT_12H
#define DEFAULT_DISPLAY_MODE      MODE_ROTATION

static bool g_eth_factory_reset_pending = false;




// ================================= ALARM STORAGE =====================================================

typedef struct {
    bool configured;
    uint8_t alarm_id;
    uint8_t time_hh;
    uint8_t time_mm;
    uint8_t frequency;
    uint8_t duration_effect;
} ethernet_alarm_t;

#define MAX_ETH_ALARMS 60

static ethernet_alarm_t g_eth_alarms[MAX_ETH_ALARMS] = {};

static_assert(sizeof(ethernet_alarm_t) == 6,
              "ethernet_alarm_t size changed; NVS alarm blob compatibility affected");

// ===========================================================================================================================


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

// =============================== DEFAULT RTC TIME ===============================

static const ds3231_time_t default_time = {
    .second = 0,
    .minute = 52,
    .hour = 13,
    .day_of_week = 2,
    .day = 18,
    .month = 8,
    .year = 2025,
};



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

// =============================== CLOCK / TEMP HELPERS ===============================
static hour_format_t clock_format = FORMAT_12H;

// =============================== DISPLAY MODES ===============================

typedef enum {
    MODE_1 = 1,
    MODE_2,
    MODE_3,
    MODE_ROTATION,
} display_mode_t;

static display_mode_t display_mode = MODE_1;




typedef enum {
    ROT_ITEM_LOGO = 0,
    ROT_ITEM_MODE_1,
    ROT_ITEM_MODE_2,
    ROT_ITEM_MODE_3,
} rotation_item_t;

static rotation_item_t rotation_item = ROT_ITEM_LOGO;
static int64_t rotation_last_change_us = 0;

// =============================== FIXED MODE LOGO STATE ===============================

typedef enum {
    FIXED_ITEM_LOGO = 0,
    FIXED_ITEM_SCREEN,
} fixed_item_t;

static fixed_item_t fixed_item = FIXED_ITEM_LOGO;
static int64_t fixed_last_change_us = 0;

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


static rotation_item_t get_rotation_item(void)
{
    int64_t now_us = esp_timer_get_time();

    if (rotation_last_change_us == 0) {
        rotation_last_change_us = now_us;
        rotation_item = ROT_ITEM_LOGO;
        return rotation_item;
    }

    int64_t interval_us;

    if (rotation_item == ROT_ITEM_LOGO) {
        interval_us = ROTATION_LOGO_INTERVAL_US;
    } else {
        interval_us = ROTATION_MODE_INTERVAL_US;
    }

    if ((now_us - rotation_last_change_us) >= interval_us) {
        rotation_last_change_us = now_us;

        switch (rotation_item)
        {
            case ROT_ITEM_LOGO:
                rotation_item = ROT_ITEM_MODE_1;
                break;

            case ROT_ITEM_MODE_1:
                rotation_item = ROT_ITEM_MODE_2;
                break;

            case ROT_ITEM_MODE_2:
                rotation_item = ROT_ITEM_MODE_3;
                break;

            case ROT_ITEM_MODE_3:
            default:
                rotation_item = ROT_ITEM_LOGO;
                break;
        }

        scroll_stop();
    }

    return rotation_item;
}


static void start_logo_screen(uint32_t duration_ms)
{
    g_logo_screen_active = true;
    g_logo_screen_until_us = esp_timer_get_time() + ((int64_t)duration_ms * 1000);
}

static fixed_item_t get_fixed_item(void)
{
    int64_t now_us = esp_timer_get_time();

    if (fixed_last_change_us == 0) {
        fixed_last_change_us = now_us;
        fixed_item = FIXED_ITEM_LOGO;
        return fixed_item;
    }

    int64_t interval_us;

    if (fixed_item == FIXED_ITEM_LOGO) {
        interval_us = FIXED_LOGO_INTERVAL_US;
    } else {
        interval_us = FIXED_MODE_INTERVAL_US;
    }

    if ((now_us - fixed_last_change_us) >= interval_us) {
        fixed_last_change_us = now_us;

        if (fixed_item == FIXED_ITEM_LOGO) {
            fixed_item = FIXED_ITEM_SCREEN;
        } else {
            fixed_item = FIXED_ITEM_LOGO;
        }

        scroll_stop();
    }

    return fixed_item;
}

static void reset_mode_sequences(void)
{
    fixed_last_change_us = 0;
    fixed_item = FIXED_ITEM_LOGO;

    rotation_last_change_us = 0;
    rotation_item = ROT_ITEM_LOGO;

    scroll_stop();
}




static esp_err_t ethernet_alarms_save(void)
{
    ethernet_alarm_t alarms_copy[MAX_ETH_ALARMS];

    portENTER_CRITICAL(&g_data_mux);
    memcpy(alarms_copy, g_eth_alarms, sizeof(alarms_copy));
    portEXIT_CRITICAL(&g_data_mux);

    esp_err_t ret = clock_settings_save_ethernet_alarms(
        alarms_copy,
        sizeof(alarms_copy)
    );

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save alarms: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Ethernet alarms saved");

    return ESP_OK;
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
	
	bool startup_screen_active_copy;
	
	bool logo_screen_active_copy;

    while (true) {
		clock_menu_check_timeout();
		
		
		
		bool factory_reset_pending_copy = false;

		portENTER_CRITICAL(&g_data_mux);

		if (g_eth_factory_reset_pending) {
		    factory_reset_pending_copy = true;
		    g_eth_factory_reset_pending = false;
		}

		portEXIT_CRITICAL(&g_data_mux);

		if (factory_reset_pending_copy) {
		    ESP_LOGW(TAG, "Applying factory reset from Ethernet");

		    portENTER_CRITICAL(&g_data_mux);

		    memset(g_eth_alarms, 0, sizeof(g_eth_alarms));

		    brightness_level = DEFAULT_BRIGHTNESS_LEVEL;
		    temporal_brightness = DEFAULT_BRIGHTNESS_LEVEL;
		    clock_format = DEFAULT_CLOCK_FORMAT;
		    display_mode = DEFAULT_DISPLAY_MODE;

		    g_eth_alarms_dirty = false;
		    g_eth_alarms_dirty_until_us = 0;
		    g_clear_alarms_on_next_ca = false;

		    g_eth_brightness_pending = false;
		    g_eth_format_pending = false;
		    g_eth_time_pending = false;   // remove if not used

		    portEXIT_CRITICAL(&g_data_mux);

		    driver->set_brightness(
		        brightness_level_to_hub75(DEFAULT_BRIGHTNESS_LEVEL)
		    );

		    clock_settings_save_brightness(DEFAULT_BRIGHTNESS_LEVEL);
		    clock_settings_save_format((uint8_t)DEFAULT_CLOCK_FORMAT);
		    clock_settings_save_mode((uint8_t)DEFAULT_DISPLAY_MODE);

		    esp_err_t alarm_save_ret = ethernet_alarms_save();

		    if (alarm_save_ret != ESP_OK) {
		        ESP_LOGE(TAG,
		                 "Failed to save cleared alarms after factory reset: %s",
		                 esp_err_to_name(alarm_save_ret));
		    }

		    ESP_LOGW(TAG, "Factory reset from Ethernet applied");

		    show_temp_message("RESET", 1000);
		}
		

		bool save_alarms_now = false;

		portENTER_CRITICAL(&g_data_mux);

		if (g_eth_alarms_dirty &&
		    esp_timer_get_time() >= g_eth_alarms_dirty_until_us) {
		    g_eth_alarms_dirty = false;
		    save_alarms_now = true;
		}

		portEXIT_CRITICAL(&g_data_mux);

		if (save_alarms_now) {
		    esp_err_t ret = ethernet_alarms_save();

		    if (ret == ESP_OK) {
		        ESP_LOGI(TAG, "Ethernet alarms saved after batch update");
		    } else {
		        ESP_LOGE(TAG,
		                 "Failed to save Ethernet alarms after batch update: %s",
		                 esp_err_to_name(ret));
		    }
		}	
		
		bool eth_brightness_pending_copy = false;
		int eth_brightness_level_copy = 5;

		bool eth_format_pending_copy = false;
		hour_format_t eth_format_copy = FORMAT_12H;

		portENTER_CRITICAL(&g_data_mux);

		if (g_eth_brightness_pending) {
		    eth_brightness_pending_copy = true;
		    eth_brightness_level_copy = g_eth_brightness_level;
		    g_eth_brightness_pending = false;
		}

		if (g_eth_format_pending) {
		    eth_format_pending_copy = true;
		    eth_format_copy = g_eth_format;
		    g_eth_format_pending = false;
		}

		portEXIT_CRITICAL(&g_data_mux);

		if (eth_brightness_pending_copy) {
		    uint8_t hub75_brightness =
		        brightness_level_to_hub75(eth_brightness_level_copy);

		    driver->set_brightness(hub75_brightness);

		    portENTER_CRITICAL(&g_data_mux);
		    brightness_level = eth_brightness_level_copy;
		    temporal_brightness = eth_brightness_level_copy;
		    portEXIT_CRITICAL(&g_data_mux);

		    clock_settings_save_brightness(eth_brightness_level_copy);

		    ESP_LOGI(TAG,
		             "Brightness applied from Ethernet: level=%d hub75=%u",
		             eth_brightness_level_copy,
		             hub75_brightness);
		}

		if (eth_format_pending_copy) {
		    portENTER_CRITICAL(&g_data_mux);
		    clock_format = eth_format_copy;
		    portEXIT_CRITICAL(&g_data_mux);

		    clock_settings_save_format((uint8_t)eth_format_copy);

		    ESP_LOGI(TAG,
		             "Clock format applied from Ethernet: %s",
		             eth_format_copy == FORMAT_24H ? "24H" : "12H");
		}

		
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

		menu_active_copy = clock_menu_is_active();

		startup_screen_active_copy = g_startup_screen_active;

		if (g_startup_screen_active && esp_timer_get_time() > g_startup_screen_until_us) {
		    g_startup_screen_active = false;
		    startup_screen_active_copy = false;
		}

		message_active_copy = g_message_active;
		snprintf(message_copy, sizeof(message_copy), "%s", g_message);

		if (g_message_active && esp_timer_get_time() > g_message_until_us) {
		    g_message_active = false;
		    message_active_copy = false;
		}
		
		logo_screen_active_copy = g_logo_screen_active;

		if (g_logo_screen_active && esp_timer_get_time() > g_logo_screen_until_us) {
		    g_logo_screen_active = false;
		    logo_screen_active_copy = false;

		    /*
		     * Start the settings screen after logo finishes.
		     */
		    g_startup_screen_active = true;
		    g_startup_screen_until_us = esp_timer_get_time() + (3000 * 1000);
		}

		portEXIT_CRITICAL(&g_data_mux);

		driver->clear();
		
		if (logo_screen_active_copy) {
		    scroll_stop();

		    clock_display_draw_logo(driver);

		    driver->flip_buffer();

		    vTaskDelayUntil(&xLastWakeTime, xFrequency);
		    continue;
		}
		
		if (startup_screen_active_copy) {
		    scroll_stop();

			clock_display_draw_startup(driver,
			                           display_mode,
			                           brightness_level,
			                           clock_format);

		    driver->flip_buffer();

		    vTaskDelayUntil(&xLastWakeTime, xFrequency);
		    continue;
		}

		if (message_active_copy) {
		    draw_string(*driver, clock_display_center_x_6x9(message_copy), 8, message_copy, 255, 0, 0);
		    driver->flip_buffer();

		    vTaskDelayUntil(&xLastWakeTime, xFrequency);
		    continue;
		}
		
		if (menu_active_copy) {
		    scroll_stop();
		    clock_menu_draw(driver);
		    driver->flip_buffer();

		    vTaskDelayUntil(&xLastWakeTime, xFrequency);
		    continue;
		}

		if (rtc_valid_copy) {
		    switch (mode_copy) {
				case MODE_1:
				{
				    fixed_item_t active_fixed_item = get_fixed_item();

				    if (active_fixed_item == FIXED_ITEM_LOGO) {
				        scroll_stop();
				        clock_display_draw_logo(driver);
				    } else {
						clock_display_draw_mode_1(driver,
						                          &now_copy,
						                          temp_copy,
						                          temp_valid_copy,
						                          format_copy);
				    }

				    break;
				}

				case MODE_2:
				{
				    fixed_item_t active_fixed_item = get_fixed_item();

				    if (active_fixed_item == FIXED_ITEM_LOGO) {
				        scroll_stop();
				        clock_display_draw_logo(driver);
				    } else {
				        clock_display_draw_mode_2(driver,
				                                  &now_copy,
				                                  temp_copy,
				                                  temp_valid_copy,
				                                  format_copy);
				    }

				    break;
				}

				case MODE_3:
				{
				    fixed_item_t active_fixed_item = get_fixed_item();

				    if (active_fixed_item == FIXED_ITEM_LOGO) {
				        scroll_stop();
				        clock_display_draw_logo(driver);
				    } else {
				        scroll_stop();
						clock_display_draw_mode_3(driver,
						                          &now_copy,
						                          temp_copy,
						                          temp_valid_copy,
						                          format_copy);
				    }

				    break;
				}

					case MODE_ROTATION:
					{
					    rotation_item_t active_rotation_item = get_rotation_item();

					    switch (active_rotation_item)
					    {
					        case ROT_ITEM_LOGO:
					            scroll_stop();
					            clock_display_draw_logo(driver);
					            break;

					        case ROT_ITEM_MODE_1:
							clock_display_draw_mode_1(driver,
							                          &now_copy,
							                          temp_copy,
							                          temp_valid_copy,
							                          format_copy);
					            break;

							case ROT_ITEM_MODE_2:
							    clock_display_draw_mode_2(driver,
							                              &now_copy,
							                              temp_copy,
							                              temp_valid_copy,
							                              format_copy);
							    break;

					        case ROT_ITEM_MODE_3:
					        default:
					            scroll_stop();
								clock_display_draw_mode_3(driver,
								                          &now_copy,
								                          temp_copy,
								                          temp_valid_copy,
								                          format_copy);
					            break;
					    }

					    break;
					}

					default:
					clock_display_draw_mode_1(driver,
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
		
		
		bool eth_time_pending_copy = false;
		ds3231_time_t eth_time_copy = {};

		portENTER_CRITICAL(&g_data_mux);

		if (g_eth_time_pending) {
		    eth_time_pending_copy = true;
		    eth_time_copy = g_eth_time;
		    g_eth_time_pending = false;
		}

		portEXIT_CRITICAL(&g_data_mux);

		if (eth_time_pending_copy) {
		    esp_err_t set_ret = ds3231_set_time(rtc, &eth_time_copy);

		    if (set_ret == ESP_OK) {
		        portENTER_CRITICAL(&g_data_mux);
		        g_now = eth_time_copy;
		        g_rtc_valid = true;
		        portEXIT_CRITICAL(&g_data_mux);

		        ESP_LOGI(TAG,
		                 "RTC updated from Ethernet: %04d-%02d-%02d %02d:%02d:%02d DOW=%d",
		                 eth_time_copy.year,
		                 eth_time_copy.month,
		                 eth_time_copy.day,
		                 eth_time_copy.hour,
		                 eth_time_copy.minute,
		                 eth_time_copy.second,
		                 eth_time_copy.day_of_week);
		    } else {
		        ESP_LOGE(TAG,
		                 "Failed to update RTC from Ethernet: %s",
		                 esp_err_to_name(set_ret));
		    }
		}
		
		
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

static void handle_normal_button(button_t btn, ds3231_dev_t *rtc)
{
    switch (btn)
    {
        case BTN_MENU:
            clock_menu_enter();
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

			reset_mode_sequences();

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
	QueueHandle_t button_queue = clock_buttons_get_queue();


    TickType_t last_press_time[3] = {
        0,
        0,
        0
    };

    button_t pending_hold_btn = BTN_NONE;
    TickType_t pending_hold_start = 0;

    bool ignore_until_release = false;
	
	button_t menu_repeat_btn = BTN_NONE;
	TickType_t menu_repeat_start = 0;
	TickType_t menu_last_repeat = 0;

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
			  if (clock_menu_is_active()) 
			  {			  
				clock_menu_handle_button(btn);

			      /*
			       * Inside menu, only UP and DOWN repeat.
			       * MENU should not repeat, because it changes fields.
			       */
			      if (btn == BTN_UP || btn == BTN_DOWN) {
			          menu_repeat_btn = btn;
			          menu_repeat_start = now;
			          menu_last_repeat = now;
			      }

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
		  * Inside menu:
		  * Auto-repeat UP/DOWN while held.
		  */
		  if (clock_menu_is_active() &&
		      menu_repeat_btn != BTN_NONE)
		 {
		     if (clock_button_is_pressed(menu_repeat_btn))
		     {
		         now = xTaskGetTickCount();

		         if ((now - menu_repeat_start) >= pdMS_TO_TICKS(BUTTON_REPEAT_DELAY_MS))
		         {
		             if ((now - menu_last_repeat) >= pdMS_TO_TICKS(BUTTON_REPEAT_RATE_MS))
		             {
						clock_menu_handle_button(btn);
		                 menu_last_repeat = now;
		             }
		         }
		     }
		     else
		     {
		         menu_repeat_btn = BTN_NONE;
		     }
		 } 
		 
		 

        /*
         * Outside menu only:
         * Execute action after the button stays pressed for BUTTON_HOLD_MS.
         */
		 if (!clock_menu_is_active() &&
		     pending_hold_btn != BTN_NONE &&
		     !ignore_until_release)
        {
            if (clock_button_is_pressed(pending_hold_btn))
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
		 if (clock_buttons_all_released())
		 {
		     pending_hold_btn = BTN_NONE;
		     menu_repeat_btn = BTN_NONE;
		     ignore_until_release = false;
		 }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static bool rtc_time_is_valid(const ds3231_time_t *time)
{
    if (!time) {
        return false;
    }

    if (time->year < 2025 || time->year > 2099) {
        return false;
    }

    if (time->month < 1 || time->month > 12) {
        return false;
    }

    int max_day = clock_menu_days_in_month(time->month, time->year);

    if (time->day < 1 || time->day > max_day) {
        return false;
    }

    if (time->hour > 23) {
        return false;
    }

    if (time->minute > 59) {
        return false;
    }

    if (time->second > 59) {
        return false;
    }

    if (time->day_of_week < 1 || time->day_of_week > 7) {
        return false;
    }

    return true;
}



static void check_or_set_default_rtc(ds3231_dev_t *rtc)
{
    if (!rtc) {
        return;
    }

    ds3231_time_t now;

    esp_err_t ret = ds3231_get_time(rtc, &now);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read RTC during startup check: %s",
                 esp_err_to_name(ret));
        return;
    }

    if (!rtc_time_is_valid(&now)) {
        ESP_LOGW(TAG,
                 "RTC time invalid: %04d-%02d-%02d %02d:%02d:%02d DOW=%d",
                 now.year,
                 now.month,
                 now.day,
                 now.hour,
                 now.minute,
                 now.second,
                 now.day_of_week);

        ds3231_time_t fixed_time = default_time;

        fixed_time.day_of_week = clock_menu_calculate_weekday(
            fixed_time.day,
            fixed_time.month,
            fixed_time.year
        );

        ret = ds3231_set_time(rtc, &fixed_time);

        if (ret == ESP_OK) {
            ESP_LOGW(TAG,
                     "RTC set to default: %04d-%02d-%02d %02d:%02d:%02d DOW=%d",
                     fixed_time.year,
                     fixed_time.month,
                     fixed_time.day,
                     fixed_time.hour,
                     fixed_time.minute,
                     fixed_time.second,
                     fixed_time.day_of_week);

            portENTER_CRITICAL(&g_data_mux);
            g_now = fixed_time;
            g_rtc_valid = true;
            portEXIT_CRITICAL(&g_data_mux);
        } else {
            ESP_LOGE(TAG, "Failed to set default RTC time: %s",
                     esp_err_to_name(ret));
        }
    } else {
        ESP_LOGI(TAG,
                 "RTC startup time valid: %04d-%02d-%02d %02d:%02d:%02d DOW=%d",
                 now.year,
                 now.month,
                 now.day,
                 now.hour,
                 now.minute,
                 now.second,
                 now.day_of_week);
    }
}

static uint8_t bcd_to_dec(uint8_t bcd)
{
    return ((bcd >> 4) * 10) + (bcd & 0x0F);
}

static bool bcd_is_valid(uint8_t bcd)
{
    return ((bcd & 0x0F) <= 9) && (((bcd >> 4) & 0x0F) <= 9);
}



static int ethernet_intensity_to_level(uint8_t intensity)
{
    /*
     * Ethernet software sends intensity 0-255.
     * Local clock menu uses brightness level 1-10.
     */

    if (intensity < 1) {
        intensity = 1;
    }

    int level = ((int)intensity * 10 + 254) / 255;

    if (level < 1) {
        level = 1;
    }

    if (level > 10) {
        level = 10;
    }

    return level;
}



static uint8_t brightness_level_to_protocol_intensity(int level)
{
    if (level < 1) {
        level = 1;
    }

    if (level > 10) {
        level = 10;
    }

    return (uint8_t)((level * 255) / 10);
}



static esp_err_t ethernet_alarms_load(void)
{
    ethernet_alarm_t alarms_copy[MAX_ETH_ALARMS] = {};

    esp_err_t ret = clock_settings_load_ethernet_alarms(
        alarms_copy,
        sizeof(alarms_copy)
    );

    if (ret == ESP_ERR_NVS_NOT_FOUND ||
        ret == ESP_ERR_INVALID_SIZE) {
        ESP_LOGW(TAG, "No valid saved alarms, using defaults");
        return ESP_OK;
    }

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to load alarms: %s", esp_err_to_name(ret));
        return ret;
    }

    portENTER_CRITICAL(&g_data_mux);
    memcpy(g_eth_alarms, alarms_copy, sizeof(g_eth_alarms));
    portEXIT_CRITICAL(&g_data_mux);

    ESP_LOGI(TAG, "Ethernet alarms loaded");

    return ESP_OK;
}




static int ethernet_rx_callback(const uint8_t *p,
                                int len,
                                uint8_t *tx,
                                int tx_max)
{
    ESP_LOGI(TAG, "Command from Ethernet received: %d bytes", len);

    /*
     * Ignore keep-alive / poll.
     */
    if (len == 3 &&
        p[0] == 0x00 &&
        p[1] == 0x00 &&
        p[2] == 0x00) {
        ESP_LOGI(TAG, "Ethernet keep-alive / poll received");
        return -1;
    }

    /*
     * CT command:
     * /TA <ID> CT <Formato2412> <Intensidad_Luminosa> \
     *
	 * Formato2412 observed from Zeit software:
	 * 0 = 12H
	 * 1 = 24H
	 *
	 * Note:
	 * This is opposite to what the document appears to say,
	 * but this mapping matches the actual PC software behavior.
     *
     * Intensidad_Luminosa:
     * 0-255 from software.
     */
    if (len == 9 &&
        p[0] == '/' &&
        p[1] == 'T' &&
        p[2] == 'A' &&
        p[4] == 'C' &&
        p[5] == 'T' &&
        p[8] == '\\') {

        uint8_t board_id = p[3];
        uint8_t format_2412 = p[6];
        uint8_t intensity = p[7];

        ESP_LOGI(TAG,
                 "CT command: id=%u format=%u intensity=%u",
                 board_id,
                 format_2412,
                 intensity);

        /*
         * Accept only board ID 0 for now.
         */
        if (board_id != 0x00) {
            ESP_LOGW(TAG, "Ignoring CT command for different board ID: %u", board_id);
            return -1;
        }

		if (format_2412 > 1) {
		    ESP_LOGW(TAG, "Invalid CT format value: %u", format_2412);
		    return -1;
		}

		hour_format_t new_format =
		    (format_2412 == 0) ? FORMAT_12H : FORMAT_24H;

        int new_brightness_level =
            ethernet_intensity_to_level(intensity);

        ESP_LOGI(TAG,
                 "Ethernet intensity %u converted to brightness level %d",
                 intensity,
                 new_brightness_level);

        portENTER_CRITICAL(&g_data_mux);

        g_eth_brightness_level = new_brightness_level;
        g_eth_brightness_pending = true;

        g_eth_format = new_format;
        g_eth_format_pending = true;

        portEXIT_CRITICAL(&g_data_mux);

        return 0;
    }
	
	/*
	 * ES command:
	 * /TA <ID> ES \
	 *
	 * Estado del Sistema / Is Online.
	 * ACK is generated by clock_ethernet.cpp:
	 * /ta <ID> es \
	 */
	 if (len == 7 &&
	     p[0] == '/' &&
	     p[1] == 'T' &&
	     p[2] == 'A' &&
	     p[4] == 'E' &&
	     p[5] == 'S' &&
	     p[6] == '\\') {

	     uint8_t board_id = p[3];

	     ESP_LOGI(TAG, "ES online/status command received: id=%u", board_id);

	     if (board_id != 0x00) {
	         ESP_LOGW(TAG, "Ignoring ES command for different board ID: %u", board_id);
	         return -1;
	     }

	     portENTER_CRITICAL(&g_data_mux);
	     g_clear_alarms_on_next_ca = true;
	     portEXIT_CRITICAL(&g_data_mux);

	     ESP_LOGI(TAG, "ES received: next CA will start full alarm replacement");

	     return 0;
	 }
	
	/*
	 * UC command:
	 * /TA <ID> UC <ss><mm><hh><DoW><dd><MM><yy> \
	 *
	 * Time bytes are BCD according to the protocol.
	 */
	if (len == 14 &&
	    p[0] == '/' &&
	    p[1] == 'T' &&
	    p[2] == 'A' &&
	    p[4] == 'U' &&
	    p[5] == 'C' &&
	    p[13] == '\\') {

	    uint8_t board_id = p[3];

	    if (board_id != 0x00) {
	        ESP_LOGW(TAG, "Ignoring UC command for different board ID: %u", board_id);
	        return -1;
	    }

	    for (int i = 6; i <= 12; i++) {
	        if (!bcd_is_valid(p[i])) {
	            ESP_LOGW(TAG, "Invalid BCD byte in UC at index %d: 0x%02X", i, p[i]);
	            return -1;
	        }
	    }

	    ds3231_time_t new_time = {};

		new_time.second = bcd_to_dec(p[6]);
		new_time.minute = bcd_to_dec(p[7]);
		new_time.hour   = bcd_to_dec(p[8]);

		uint8_t received_dow = bcd_to_dec(p[9]);

		new_time.day   = bcd_to_dec(p[10]);
		new_time.month = bcd_to_dec(p[11]);
		new_time.year  = 2000 + bcd_to_dec(p[12]);

		/*
		 * Do not trust Ethernet DoW because the PC software may use
		 * a different convention:
		 *
		 * 0 = Sunday, 1 = Monday
		 * or
		 * 1 = Sunday, 2 = Monday
		 *
		 * Our clock already has a known-good weekday calculator.
		 */
		new_time.day_of_week = clock_menu_calculate_weekday(
		    new_time.day,
		    new_time.month,
		    new_time.year
		);

		ESP_LOGI(TAG,
		         "UC received DoW=%u, calculated DoW=%d",
		         received_dow,
		         new_time.day_of_week);

	    if (!rtc_time_is_valid(&new_time)) {
	        ESP_LOGW(TAG,
	                 "Invalid UC time: %04d-%02d-%02d %02d:%02d:%02d DOW=%d",
	                 new_time.year,
	                 new_time.month,
	                 new_time.day,
	                 new_time.hour,
	                 new_time.minute,
	                 new_time.second,
	                 new_time.day_of_week);
	        return -1;
	    }

	    portENTER_CRITICAL(&g_data_mux);
	    g_eth_time = new_time;
	    g_eth_time_pending = true;
	    portEXIT_CRITICAL(&g_data_mux);

	    ESP_LOGI(TAG,
	             "UC time received: %04d-%02d-%02d %02d:%02d:%02d DOW=%d",
	             new_time.year,
	             new_time.month,
	             new_time.day,
	             new_time.hour,
	             new_time.minute,
	             new_time.second,
	             new_time.day_of_week);

	    return 0;
	}
	
	/*
	 * RC command:
	 * /TA <ID> RC \
	 *
	 * Response:
	 * /ta <ID> rc <IMW><Formato2412><Intensidad><ss><mm><hh><DoW><dd><MM><yy> \
	 */
	if (len == 7 &&
	    p[0] == '/' &&
	    p[1] == 'T' &&
	    p[2] == 'A' &&
	    p[4] == 'R' &&
	    p[5] == 'C' &&
	    p[6] == '\\') {

	    uint8_t board_id = p[3];

	    ESP_LOGI(TAG, "RC read config command received: id=%u", board_id);

	    if (board_id != 0x00) {
	        ESP_LOGW(TAG, "Ignoring RC command for different board ID: %u", board_id);
	        return -1;
	    }

	    if (tx == NULL || tx_max < 17) {
	        ESP_LOGE(TAG, "TX buffer too small for RC response");
	        return -1;
	    }

	    ds3231_time_t now_copy;
	    hour_format_t format_copy;
	    int brightness_level_copy;
	    bool rtc_valid_copy;

	    portENTER_CRITICAL(&g_data_mux);

	    now_copy = g_now;
	    format_copy = clock_format;
	    brightness_level_copy = brightness_level;
	    rtc_valid_copy = g_rtc_valid;

	    portEXIT_CRITICAL(&g_data_mux);

	    if (!rtc_valid_copy) {
	        ESP_LOGW(TAG, "RC requested but RTC is not valid");
	        return -1;
	    }

	    uint8_t format_2412 = (format_copy == FORMAT_12H) ? 0 : 1;
	    uint8_t intensity = brightness_level_to_protocol_intensity(brightness_level_copy);

	    /*
	     * RC document says time fields are HEX, not BCD.
	     */
	    tx[0]  = '/';
	    tx[1]  = 't';
	    tx[2]  = 'a';
	    tx[3]  = board_id;
	    tx[4]  = 'r';
	    tx[5]  = 'c';

	    tx[6]  = 0x01; // IMW: IsMemoryWritten. Use 1 = valid/configured.
	    tx[7]  = format_2412;
	    tx[8]  = intensity;

	    tx[9]  = (uint8_t)now_copy.second;
	    tx[10] = (uint8_t)now_copy.minute;
	    tx[11] = (uint8_t)now_copy.hour;

	    /*
	     * Use your internal day_of_week value.
	     * If software shows wrong weekday later, we can map it here.
	     */
	    tx[12] = (uint8_t)now_copy.day_of_week;

	    tx[13] = (uint8_t)now_copy.day;
	    tx[14] = (uint8_t)now_copy.month;
	    tx[15] = (uint8_t)(now_copy.year % 100);

	    tx[16] = '\\';

	    ESP_LOGI(TAG,
	             "RC response: format=%u intensity=%u time=%04d-%02d-%02d %02d:%02d:%02d DOW=%d",
	             format_2412,
	             intensity,
	             now_copy.year,
	             now_copy.month,
	             now_copy.day,
	             now_copy.hour,
	             now_copy.minute,
	             now_copy.second,
	             now_copy.day_of_week);

	    return 17;
	}
	
	/*
	 * CA command:
	 * /TA <ID> CA <Alarma_ID><Hora_Alarma[2]><Frecuencia><Duracion&Efecto> \
	 *
	 * ACK:
	 * /ta <ID> ca <Alarma_ID> \
	 */
	if (len == 12 &&
	    p[0] == '/' &&
	    p[1] == 'T' &&
	    p[2] == 'A' &&
	    p[4] == 'C' &&
	    p[5] == 'A' &&
	    p[11] == '\\') {

	    uint8_t board_id = p[3];

	    if (board_id != 0x00) {
	        ESP_LOGW(TAG, "Ignoring CA command for different board ID: %u", board_id);
	        return -1;
	    }

	    uint8_t alarm_id = p[6];
	    uint8_t alarm_hh = p[7];
	    uint8_t alarm_mm = p[8];
	    uint8_t frequency = p[9];
	    uint8_t duration_effect = p[10];

	    if (alarm_id < 1 || alarm_id > MAX_ETH_ALARMS) {
	        ESP_LOGW(TAG, "Invalid CA alarm_id=%u", alarm_id);
	        return -1;
	    }

	    if (alarm_hh > 23 || alarm_mm > 59) {
	        ESP_LOGW(TAG, "Invalid CA alarm time: %02u:%02u", alarm_hh, alarm_mm);
	        return -1;
	    }

		bool alarm_enabled = (frequency & 0x80) != 0;
		bool cleared_alarm_table = false;

		portENTER_CRITICAL(&g_data_mux);

		if (g_clear_alarms_on_next_ca) {
		    memset(g_eth_alarms, 0, sizeof(g_eth_alarms));
		    g_clear_alarms_on_next_ca = false;
		    cleared_alarm_table = true;
		}

		g_eth_alarms[alarm_id - 1].configured = alarm_enabled;
		g_eth_alarms[alarm_id - 1].alarm_id = alarm_id;
		g_eth_alarms[alarm_id - 1].time_hh = alarm_hh;
		g_eth_alarms[alarm_id - 1].time_mm = alarm_mm;
		g_eth_alarms[alarm_id - 1].frequency = frequency;
		g_eth_alarms[alarm_id - 1].duration_effect = duration_effect;

		g_eth_alarms_dirty = true;
		g_eth_alarms_dirty_until_us = esp_timer_get_time() + 1000000;

		portEXIT_CRITICAL(&g_data_mux);

		if (cleared_alarm_table) {
		    ESP_LOGW(TAG, "First CA after ES: clearing all 60 alarms first");
		}

		ESP_LOGI(TAG,
		         "CA alarm stored: id=%u enabled=%d time=%02u:%02u freq=0x%02X dur_eff=0x%02X",
		         alarm_id,
		         alarm_enabled,
		         alarm_hh,
		         alarm_mm,
		         frequency,
		         duration_effect);


	    tx[0] = '/';
	    tx[1] = 't';
	    tx[2] = 'a';
	    tx[3] = board_id;
	    tx[4] = 'c';
	    tx[5] = 'a';
	    tx[6] = alarm_id;
	    tx[7] = '\\';

	    return 8;
	}
	
	/*
	 * LA command:
	 * /TA <ID> LA <Alarma_ID> \
	 *
	 * ACK:
	 * /ta <ID> la <Alarma_ID><HH><MM><Frecuencia><Duracion&Efecto> \
	 */
	if (len == 8 &&
	    p[0] == '/' &&
	    p[1] == 'T' &&
	    p[2] == 'A' &&
	    p[4] == 'L' &&
	    p[5] == 'A' &&
	    p[7] == '\\') {

	    uint8_t board_id = p[3];

	    if (board_id != 0x00) {
	        ESP_LOGW(TAG, "Ignoring LA command for different board ID: %u", board_id);
	        return -1;
	    }

	    uint8_t alarm_id = p[6];

	    if (alarm_id < 1 || alarm_id > MAX_ETH_ALARMS) {
	        ESP_LOGW(TAG, "Invalid LA alarm_id=%u", alarm_id);
	        return -1;
	    }

	    ethernet_alarm_t alarm_copy;

	    portENTER_CRITICAL(&g_data_mux);
	    alarm_copy = g_eth_alarms[alarm_id - 1];
	    portEXIT_CRITICAL(&g_data_mux);

	    if (!alarm_copy.configured) {
	        /*
	         * Default value for unconfigured alarm.
	         */
	        alarm_copy.configured = false;
	        alarm_copy.alarm_id = alarm_id;
	        alarm_copy.time_hh = 0;
	        alarm_copy.time_mm = 0;
	        alarm_copy.frequency = 0x00;
	        alarm_copy.duration_effect = 0x00;
	    }

	    ESP_LOGI(TAG,
	             "LA read alarm: id=%u configured=%d time=%02u:%02u freq=0x%02X dur_eff=0x%02X",
	             alarm_id,
	             alarm_copy.configured,
	             alarm_copy.time_hh,
	             alarm_copy.time_mm,
	             alarm_copy.frequency,
	             alarm_copy.duration_effect);

	    if (tx == NULL || tx_max < 12) {
	        return -1;
	    }

	    tx[0] = '/';
	    tx[1] = 't';
	    tx[2] = 'a';
	    tx[3] = board_id;
	    tx[4] = 'l';
	    tx[5] = 'a';

	    tx[6]  = alarm_id;
	    tx[7]  = alarm_copy.time_hh;
	    tx[8]  = alarm_copy.time_mm;
	    tx[9]  = alarm_copy.frequency;
	    tx[10] = alarm_copy.duration_effect;

	    tx[11] = '\\';

	    return 12;
	}
	
	
	
	/*
	 * RT command:
	 * /TA <ID> RT <Reset_ID> \
	 *
	 * reset_id = 0x00 -> reset device settings/defaults
	 *
	 * ACK:
	 * /ta <ID> rt \
	 */
	if (len == 8 &&
	    p[0] == '/' &&
	    p[1] == 'T' &&
	    p[2] == 'A' &&
	    p[4] == 'R' &&
	    p[5] == 'T' &&
	    p[7] == '\\') {

	    uint8_t board_id = p[3];
	    uint8_t reset_id = p[6];

	    ESP_LOGI(TAG,
	             "RT reset command received: board_id=%u reset_id=0x%02X",
	             board_id,
	             reset_id);

	    if (board_id != 0x00) {
	        ESP_LOGW(TAG, "Ignoring RT command for different board ID: %u", board_id);
	        return -1;
	    }

	    if (reset_id != 0x00) {
	        ESP_LOGW(TAG, "Unknown RT reset_id=0x%02X", reset_id);
	        return -1;
	    }

	    portENTER_CRITICAL(&g_data_mux);
	    g_eth_factory_reset_pending = true;
	    portEXIT_CRITICAL(&g_data_mux);

	    ESP_LOGW(TAG, "RT Reset All received, factory reset pending");

	    /*
	     * Generic ACK will be sent:
	     * /ta <ID> rt \
	     */
	    return 0;
	}
	

	ESP_LOGW(TAG, "Unknown Ethernet command");
	return -1;
}

// =============================== APP MAIN ===============================

extern "C" void app_main(void)
{  
	
	ESP_LOGI(TAG, "Starting Ethernet");
	ESP_ERROR_CHECK(clock_ethernet_init_static());
	ESP_ERROR_CHECK(clock_ethernet_start_tcp_server(ethernet_rx_callback));	
	
	ESP_LOGI(TAG, "Starting HUB75");

    if (!driver.begin()) {
        ESP_LOGE(TAG, "Driver start failed");
        return;
    }
	
	ESP_ERROR_CHECK(clock_settings_init());
	
	
	
	ESP_ERROR_CHECK(ethernet_alarms_load());
	
	

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
	
	
	start_logo_screen(3000);	
	
	

    static ds18b20_t ambient_sensor;
    static ds3231_dev_t rtc;

    ESP_ERROR_CHECK(ds18b20_init(&ambient_sensor, DS18B20_GPIO));
    ESP_ERROR_CHECK(init_ds3231(&rtc));
	
	check_or_set_default_rtc(&rtc);
	
	
	clock_menu_context_t menu_ctx = {
	    .driver = &driver,
	    .rtc = &rtc,

	    .brightness_level = &brightness_level,
	    .temporal_brightness = &temporal_brightness,

	    .data_mux = &g_data_mux,
	    .g_now = &g_now,
	    .g_rtc_valid = &g_rtc_valid,

	    .show_message = show_temp_message,
	};

	clock_menu_init(&menu_ctx);
	
	
	ESP_ERROR_CHECK(clock_buttons_init(PIN_MENU, PIN_UP, PIN_DOWN));
	
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