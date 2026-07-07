#include "wifi_sta.hpp"

#include <cstdio>
#include <cstring>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"

#include "biosense_secrets.hpp"

namespace biosense {

static const char* TAG = "WifiSta";

static constexpr int WIFI_MAX_RETRY = 10;
static constexpr EventBits_t WIFI_CONNECTED_BIT = BIT0;
static constexpr EventBits_t WIFI_FAIL_BIT = BIT1;

static EventGroupHandle_t wifi_event_group = nullptr;
static int retry_count = 0;

static void wifi_event_handler(
        void* arg,
        esp_event_base_t event_base,
        int32_t event_id,
        void* event_data
    ) {
        if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
        {
            esp_wifi_connect();
            return;
        }

        if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) 
        {
            if (retry_count < WIFI_MAX_RETRY)
            {
                retry_count++;
                ESP_LOGW(TAG, "Disconnected, retry %d/%d", retry_count, WIFI_MAX_RETRY);
                esp_wifi_connect();
            } else {
                xEventGroupSetBits(wifi_event_group, WIFI_FAIL_BIT);
            }
            return;
        }

        if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) 
        {
            auto* event = static_cast<ip_event_got_ip_t*>(event_data);

            ESP_LOGW(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));

            retry_count = 0;
            xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
            return;
        }
    }

esp_err_t WifiSta::connect() {
    wifi_event_group = xEventGroupCreate();
    if (wifi_event_group == nullptr)
    {
        return ESP_ERR_NO_MEM;
    }

    ESP_ERROR_CHECK(esp_netif_init());

    esp_err_t err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
    {
        return err;
    }

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_init_config));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT,
        ESP_EVENT_ANY_ID,
        wifi_event_handler,
        nullptr,
        &instance_any_id
    ));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT,
        IP_EVENT_STA_GOT_IP,
        wifi_event_handler,
        nullptr,
        &instance_got_ip
    ));

    wifi_config_t wifi_config = {};

    std::snprintf(
        reinterpret_cast<char*>(wifi_config.sta.ssid),
        sizeof(wifi_config.sta.ssid),
        "%s",
        WIFI_SSID
    );

    std::snprintf(
        reinterpret_cast<char*>(wifi_config.sta.password),
        sizeof(wifi_config.sta.password),
        "%s",
        WIFI_PASS
    );

    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Connecting to SSID: %s", WIFI_SSID);

    EventBits_t bits = xEventGroupWaitBits(
        wifi_event_group,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
        pdFALSE,
        pdFALSE,
        pdMS_TO_TICKS(20000)
    );

    if (bits & WIFI_CONNECTED_BIT)
    {
        ESP_LOGI(TAG, "Connected");
        return ESP_OK;
    }
    
    if (bits & WIFI_FAIL_BIT)
    {
        ESP_LOGE(TAG, "Connection failed");
        return ESP_FAIL;
    }
    

    ESP_LOGE(TAG, "Connection timeout");
    return ESP_ERR_TIMEOUT;
}

}   // namespace