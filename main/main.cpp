#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "nvs_flash.h"

static const char* TAG = "APP";

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "BioSense ESP-IDF C++ start");

    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    ESP_ERROR_CHECK(ret);

    int counter = 0;

    while (true) {
        ESP_LOGI(TAG, "C++ app działa, counter=%d", counter++);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}