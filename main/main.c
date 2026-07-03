#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "biosense_secrets.h"

static const char *TAG = "BIOSENSE";

void app_main(void)
{
    ESP_LOGI(TAG, "Start projektu BioSense w ESP-IDF");

    int counter = 0;

    while (1) {
        ESP_LOGI(TAG, "Program dziala, counter=%d", counter++);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}