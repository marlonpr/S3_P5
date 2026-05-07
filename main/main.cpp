#include <stdio.h>
#include "hub75.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "ds3231.h"
#include "led_panel.h"
#include "ds18b20.h"

#define DS18B20_GPIO GPIO_NUM_39

static const char* TAG = "MAIN";

// =============================== SHARED DATA ===============================

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

// =============================== DISPLAY TASK ===============================

void display_update_task(void* pvParameters)
{
    Hub75Driver* driver = (Hub75Driver*)pvParameters;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(33); // ~30 FPS

	char line1[32];
	char line2[32];		

    while (true) {
		ds3231_time_t now_copy;
		float temp_copy;
		bool rtc_valid_copy;
		bool temp_valid_copy;

		portENTER_CRITICAL(&g_data_mux);
		now_copy = g_now;
		temp_copy = g_temp_c;
		rtc_valid_copy = g_rtc_valid;
		temp_valid_copy = g_temp_valid;
		portEXIT_CRITICAL(&g_data_mux);
        driver->clear();
		
		if (!scroll_is_active()) {
		    scroll_start("JUEVES 7 MAYO 2026", 12, 0, 255, 0, 10);
		}
        scroll_update(*driver);	
		
		if (rtc_valid_copy) {
		    snprintf(line1, sizeof(line1),
		             "%02d:%02d:%02d",
		             now_copy.hour,
		             now_copy.minute,
		             now_copy.second);

		    draw_string(*driver, 8, 2, line1, 0, 255, 255);
		} else {
		    draw_string(*driver, 2, 2, "NO RTC", 255, 0, 0);
		}

		if (temp_valid_copy) {
		    snprintf(line2, sizeof(line2),
		             "%.1fC",
		             temp_copy);

		    draw_string(*driver, 17, 22, line2, 255, 128, 255);
		} else {
		    draw_string(*driver, 12, 22, "NO TEMP", 255, 0, 0);
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

// =============================== APP MAIN ===============================

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Starting HUB75");

    if (!driver.begin()) {
        ESP_LOGE(TAG, "Driver start failed");
        return;
    }

    static ds18b20_t ambient_sensor;
    static ds3231_dev_t rtc;

    ESP_ERROR_CHECK(ds18b20_init(&ambient_sensor, DS18B20_GPIO));
    ESP_ERROR_CHECK(init_ds3231(&rtc));

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