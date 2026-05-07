#include "clock_settings.h"

#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"

static const char *TAG = "CLOCK_SETTINGS";

#define NVS_NAMESPACE      "clock_cfg"
#define NVS_KEY_FORMAT     "format"
#define NVS_KEY_MODE       "mode"
#define NVS_KEY_BRIGHTNESS "brightness"

esp_err_t clock_settings_init(void)
{
    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "NVS initialized");
    } else {
        ESP_LOGE(TAG, "NVS init failed: %s", esp_err_to_name(ret));
    }

    return ret;
}

static void save_u8_value(const char *key, uint8_t value)
{
    nvs_handle_t handle;

    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS open failed for key '%s': %s",
                 key,
                 esp_err_to_name(ret));
        return;
    }

    ret = nvs_set_u8(handle, key, value);
    if (ret == ESP_OK) {
        ret = nvs_commit(handle);
    }

    nvs_close(handle);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Saved %s=%u", key, value);
    } else {
        ESP_LOGE(TAG, "Save failed for key '%s': %s",
                 key,
                 esp_err_to_name(ret));
    }
}

static uint8_t load_u8_value(const char *key, uint8_t default_value)
{
    nvs_handle_t handle;
    uint8_t value = default_value;

    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "No NVS namespace, using default %s=%u",
                 key,
                 default_value);
        return default_value;
    }

    ret = nvs_get_u8(handle, key, &value);
    nvs_close(handle);

    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Key '%s' not found, using default %u",
                 key,
                 default_value);
        return default_value;
    }

    ESP_LOGI(TAG, "Loaded %s=%u", key, value);

    return value;
}

void clock_settings_save_format(uint8_t format)
{
    save_u8_value(NVS_KEY_FORMAT, format);
}

uint8_t clock_settings_load_format(uint8_t default_format)
{
    return load_u8_value(NVS_KEY_FORMAT, default_format);
}

void clock_settings_save_mode(uint8_t mode)
{
    save_u8_value(NVS_KEY_MODE, mode);
}

uint8_t clock_settings_load_mode(uint8_t default_mode)
{
    return load_u8_value(NVS_KEY_MODE, default_mode);
}

void clock_settings_save_brightness(uint8_t brightness_level)
{
    if (brightness_level < 1) {
        brightness_level = 1;
    }

    if (brightness_level > 10) {
        brightness_level = 10;
    }

    save_u8_value(NVS_KEY_BRIGHTNESS, brightness_level);
}

uint8_t clock_settings_load_brightness(uint8_t default_brightness_level)
{
    uint8_t value = load_u8_value(NVS_KEY_BRIGHTNESS, default_brightness_level);

    if (value < 1 || value > 10) {
        value = default_brightness_level;
    }

    return value;
}