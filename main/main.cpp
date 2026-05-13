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





static const char* TAG = "MAIN";








#include "esp_event.h"
#include "esp_netif.h"
#include "esp_eth.h"

#include "driver/spi_master.h"

#include "esp_eth_mac.h"
#include "esp_eth_phy.h"

#include "esp_eth_mac_w5500.h"
#include "esp_eth_phy_w5500.h"




#define ETH_SPI_HOST       SPI2_HOST

#define ETH_MOSI_GPIO      GPIO_NUM_11
#define ETH_MISO_GPIO      GPIO_NUM_12
#define ETH_SCLK_GPIO      GPIO_NUM_13
#define ETH_CS_GPIO        GPIO_NUM_14
#define ETH_INT_GPIO       GPIO_NUM_10
#define ETH_RST_GPIO       GPIO_NUM_9

#define ETH_SPI_CLOCK_MHZ  20









static esp_eth_handle_t s_eth_handle = NULL;
static esp_netif_t *s_eth_netif = NULL;

static void eth_event_handler(void *arg,
                              esp_event_base_t event_base,
                              int32_t event_id,
                              void *event_data)
{
    switch (event_id) {
        case ETHERNET_EVENT_CONNECTED:
            ESP_LOGI(TAG, "Ethernet Link Up");
            break;

        case ETHERNET_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "Ethernet Link Down");
            break;

        case ETHERNET_EVENT_START:
            ESP_LOGI(TAG, "Ethernet Started");
            break;

        case ETHERNET_EVENT_STOP:
            ESP_LOGI(TAG, "Ethernet Stopped");
            break;

        default:
            break;
    }
}

static void got_ip_event_handler(void *arg,
                                 esp_event_base_t event_base,
                                 int32_t event_id,
                                 void *event_data)
{
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    const esp_netif_ip_info_t *ip_info = &event->ip_info;

    ESP_LOGI(TAG, "Ethernet Got IP Address");
    ESP_LOGI(TAG, "ETHIP: " IPSTR, IP2STR(&ip_info->ip));
    ESP_LOGI(TAG, "ETHMASK: " IPSTR, IP2STR(&ip_info->netmask));
    ESP_LOGI(TAG, "ETHGW: " IPSTR, IP2STR(&ip_info->gw));
}







static esp_err_t ethernet_w5500_init(void)
{
    ESP_LOGI(TAG, "Initializing W5500 Ethernet");

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_config_t netif_cfg = ESP_NETIF_DEFAULT_ETH();
    s_eth_netif = esp_netif_new(&netif_cfg);
    if (s_eth_netif == NULL) {
        ESP_LOGE(TAG, "Failed to create Ethernet netif");
        return ESP_FAIL;
    }

    spi_bus_config_t buscfg = {};
    buscfg.mosi_io_num = ETH_MOSI_GPIO;
    buscfg.miso_io_num = ETH_MISO_GPIO;
    buscfg.sclk_io_num = ETH_SCLK_GPIO;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;

    ESP_ERROR_CHECK(spi_bus_initialize(ETH_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t devcfg = {};
    devcfg.command_bits = 16;
    devcfg.address_bits = 8;
    devcfg.mode = 0;
    devcfg.clock_speed_hz = ETH_SPI_CLOCK_MHZ * 1000 * 1000;
    devcfg.spics_io_num = ETH_CS_GPIO;
    devcfg.queue_size = 20;

    eth_w5500_config_t w5500_config = ETH_W5500_DEFAULT_CONFIG(ETH_SPI_HOST, &devcfg);
    w5500_config.int_gpio_num = ETH_INT_GPIO;

    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();

    phy_config.phy_addr = 1;
    phy_config.reset_gpio_num = ETH_RST_GPIO;

    esp_eth_mac_t *mac = esp_eth_mac_new_w5500(&w5500_config, &mac_config);
    esp_eth_phy_t *phy = esp_eth_phy_new_w5500(&phy_config);

    esp_eth_config_t eth_config = ETH_DEFAULT_CONFIG(mac, phy);

    ESP_ERROR_CHECK(esp_eth_driver_install(&eth_config, &s_eth_handle));
    ESP_ERROR_CHECK(esp_netif_attach(s_eth_netif, esp_eth_new_netif_glue(s_eth_handle)));

    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT,
                                               ESP_EVENT_ANY_ID,
                                               &eth_event_handler,
                                               NULL));

    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT,
                                               IP_EVENT_ETH_GOT_IP,
                                               &got_ip_event_handler,
                                               NULL));

    ESP_ERROR_CHECK(esp_eth_start(s_eth_handle));

    ESP_LOGI(TAG, "W5500 Ethernet init done");

    return ESP_OK;
}


























#define DS18B20_GPIO GPIO_NUM_39
#define PIN_MENU GPIO_NUM_40
#define PIN_UP   GPIO_NUM_41
#define PIN_DOWN GPIO_NUM_42

#define BUTTON_HOLD_MS     1000
#define BUTTON_DEBOUNCE_MS 500
#define BUTTON_REPEAT_DELAY_MS 500
#define BUTTON_REPEAT_RATE_MS  500

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


// =============================== APP MAIN ===============================

extern "C" void app_main(void)
{
    
	
	
	
	
	ESP_LOGI(TAG, "Starting Ethernet test");

	ESP_ERROR_CHECK(ethernet_w5500_init());
	
	
	
	
	
	
	
	
	
	
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