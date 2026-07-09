// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"

// #include "esp_log.h"
// #include "nvs_flash.h"

// static const char* TAG = "APP";

// extern "C" void app_main(void)
// {
//     ESP_LOGI(TAG, "BioSense ESP-IDF C++ start");

//     esp_err_t ret = nvs_flash_init();

//     if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
//         ESP_ERROR_CHECK(nvs_flash_erase());
//         ret = nvs_flash_init();
//     }

//     ESP_ERROR_CHECK(ret);

//     int counter = 0;

//     while (true) {
//         ESP_LOGI(TAG, "C++ app działa, counter=%d", counter++);
//         vTaskDelay(pdMS_TO_TICKS(1000));
//     }
// }


#include <cstdio>
#include <cstdint>
#include <cstring>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"

#include "biosense_config.hpp"
#include "biosense_secrets.hpp"

#include "wifi_sta.hpp"
#include "udp_sender.hpp"

#include "adc_sensors.hpp"
#include "ecg_processor.hpp"
#include "telemetry.hpp"
#include "i2c_sensors.hpp"

#include "leds.hpp"

static const char* TAG = "APP";

static biosense::WifiSta wifi;
static biosense::UdpSender udp;

static biosense::adc_sensors adc;
static biosense::ecg_processor ecg;

static biosense::i2c_sensors i2c;

static biosense::leds leds;

static void init_nvs()
{
    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    ESP_ERROR_CHECK(ret);
}

static void sampling_task(void* arg)
{
    (void)arg;

    ESP_LOGI(TAG, "Sampling task started");

    uint32_t sample_index = 0;

    int gsr_adc = 0;
    float batt_v = 0.0f;

    char udp_buffer[UDP_BUFFER_SIZE];
    int udp_len = 0;
    int batch_count = 0;

    const int64_t sample_interval_us = 1000000LL / ECG_SAMPLE_RATE_HZ;
    int64_t next_sample_us = esp_timer_get_time();

    while (true) {
        int64_t now_us = esp_timer_get_time();

        if (now_us < next_sample_us) {
            int64_t wait_us = next_sample_us - now_us;

            if (wait_us > 1000) {
                vTaskDelay(pdMS_TO_TICKS(1));
            }

            continue;
        }

        next_sample_us += sample_interval_us;

        uint64_t now_ms = static_cast<uint64_t>(now_us / 1000);

        // I2C: na razie MLX. MAX zostawiamy wyzerowany.
        i2c.update(now_ms);
        const auto& i2c_data = i2c.readings();

        bool leads_off = adc.leads_off();

        int raw_ecg = 0;
        if (!leads_off) {
            raw_ecg = adc.read_ecg_raw();
        }

        biosense::ecg_result ecg_result = ecg.process(raw_ecg, leads_off, now_ms);

        // GSR nie musi być czytany 250 razy/s.
        if ((sample_index % 25) == 0) {
            gsr_adc = adc.read_gsr_raw();
        }

        // Bateria też nie musi być czytana 250 razy/s.
        if ((sample_index % 50) == 0) {
            batt_v = adc.read_batt_voltage();
        }

        if (sample_index % 250 == 0)
        {
            leds.set_battery_level(batt_v);
        }

        biosense::telemetry_frame frame = {};

        frame.sample = sample_index;

        frame.raw_ecg = ecg_result.raw;
        frame.filtered_ecg = ecg_result.filtered;
        frame.ecg_bpm = ecg_result.bpm;
        frame.rpeak = ecg_result.rpeak;
        frame.ecg_leads_off = ecg_result.leads_off;

        frame.gsr_adc = gsr_adc;
        frame.batt_v = batt_v;

        if (i2c_data.mlx_valid) {
            frame.temp_amb = i2c_data.temp_amb;
            frame.temp_obj = i2c_data.temp_obj;
        } else {
            frame.temp_amb = 0.0f;
            frame.temp_obj = 0.0f;
        }

        // MAX jeszcze niezaimplementowany — świadomie zerujemy.
        // MAX30102 / MAX30105
        if (i2c_data.max_ok) {
            frame.max_bpm = i2c_data.max_bpm;
            frame.spo2 = i2c_data.max_spo2;
            frame.spo2_valid = i2c_data.max_spo2_valid;
            frame.spo2_raw = static_cast<int>(i2c_data.max_spo2_raw);
            frame.max_finger = i2c_data.max_finger;
            frame.max_red = static_cast<int>(i2c_data.max_red);
            frame.max_ired = static_cast<int>(i2c_data.max_ired);
        } else {
            frame.max_bpm = 0;
            frame.spo2 = 0.0f;
            frame.spo2_valid = false;
            frame.spo2_raw = 0;
            frame.max_finger = false;
            frame.max_red = 0;
            frame.max_ired = 0;
        }

        char line[256];
        int written = biosense::format_telemetry_csv(line, sizeof(line), frame);

        if (written > 0 && written < static_cast<int>(sizeof(line))) {
            if (udp_len + written >= static_cast<int>(sizeof(udp_buffer))) {
                udp.send(udp_buffer, static_cast<size_t>(udp_len));
                udp_len = 0;
                batch_count = 0;
            }

            std::memcpy(udp_buffer + udp_len, line, static_cast<size_t>(written));
            udp_len += written;
            batch_count++;
        }

        if (batch_count >= UDP_BATCH_SIZE) {
            udp.send(udp_buffer, static_cast<size_t>(udp_len));
            udp_len = 0;
            batch_count = 0;
        }

        // Lekki log diagnostyczny raz na sekundę.
        // if ((sample_index % ECG_SAMPLE_RATE_HZ) == 0) {
        // //     ESP_LOGI(
        // //         TAG,
        // //         "ECG raw=%d filt=%d LO=%d GSR=%d BAT=%.2fV | MLX ok=%d valid=%d Tamb=%.2f Tobj=%.2f | MAX ok=%d finger=%d RED=%lu IR=%lu BPM=%d SpO2=%.1f valid=%d",
        // //         ecg_result.raw,
        // //         ecg_result.filtered,
        // //         ecg_result.leads_off ? 1 : 0,
        // //         gsr_adc,
        // //         batt_v,

        // //         i2c_data.mlx_ok ? 1 : 0,
        // //         i2c_data.mlx_valid ? 1 : 0,
        // //         i2c_data.temp_amb,
        // //         i2c_data.temp_obj,

        // //         i2c_data.max_ok ? 1 : 0,
        // //         i2c_data.max_finger ? 1 : 0,
        // //         static_cast<unsigned long>(i2c_data.max_red),
        // //         static_cast<unsigned long>(i2c_data.max_ir),
        // //         i2c_data.max_bpm,
        // //         i2c_data.max_spo2,
        // //         i2c_data.max_spo2_valid ? 1 : 0
        // //     );
        // }

        sample_index++;
    }
}

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "BioSense ESP-IDF C++ test start");

    init_nvs();

    ESP_ERROR_CHECK(adc.init());

    ESP_ERROR_CHECK(leds.init());
    ESP_ERROR_CHECK(leds.set_status(biosense::led_status::wifi_connecting));

    esp_err_t i2c_err = i2c.init();
    if (i2c_err != ESP_OK) {
        ESP_LOGW(TAG, "I2C init failed: %s", esp_err_to_name(i2c_err));
    }

    ESP_ERROR_CHECK(wifi.connect());
    ESP_ERROR_CHECK(udp.init(UDP_TARGET_IP, UDP_TARGET_PORT));

    ESP_ERROR_CHECK(leds.set_status(biosense::led_status::wifi_connected));
    vTaskDelay(pdMS_TO_TICKS(300));

    xTaskCreatePinnedToCore(
        sampling_task,
        "sampling_task",
        12288,
        nullptr,
        5,
        nullptr,
        1
    );
}