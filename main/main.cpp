#include <cstdint>
#include <cstring>
#include <cstddef>

#include "biosense_config.hpp"
#include "biosense_secrets.hpp"

#include "wifi_sta.hpp"
#include "udp_sender.hpp"
#include "mqtt_sender.hpp"

#include "adc_sensors.hpp"
#include "sensor_config.hpp"
#include "ecg_processor.hpp"
#include "i2c_sensors.hpp"
#include "telemetry.hpp"

#include "leds.hpp"
#include "buzzer.hpp"
#include "buttons.hpp"
#include "battery_soc.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"

static const char* TAG = "APP";

static biosense::WifiSta wifi;
static biosense::UdpSender udp;
static biosense::mqtt_sender mqtt;

static biosense::adc_sensors adc;
static biosense::ecg_processor ecg;
static biosense::i2c_sensors i2c;

static biosense::leds leds;
static biosense::buzzer buzzer;
static biosense::buttons buttons;

static bool i2c_ready = false;

static constexpr uint8_t UI_FLAG_SESSION_ACTIVE = 1U << 0;
static constexpr uint8_t UI_FLAG_BUZZER_MUTED = 1U << 1;

static void init_nvs()
{
    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
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

    biosense::gsr_reading gsr = {};
    float batt_v = 0.0f;
    float filtered_batt_v = 0.0f;
    bool batt_filter_init = false;

    static constexpr float BATT_FILTER_ALPHA = 0.05f;

    char udp_buffer[UDP_BUFFER_SIZE];
    int udp_len = 0;
    int batch_count = 0;

    const int64_t sample_interval_us =
        1000000LL / ECG_SAMPLE_RATE_HZ;

    int64_t next_sample_us = esp_timer_get_time();

    bool session_active = false;

    biosense::mqtt_ecg_batch mqtt_ecg_batch = {};
    biosense::mqtt_gsr_batch mqtt_gsr_batch = {};

    while (true)
    {
        int64_t now_us = esp_timer_get_time();

        if (now_us < next_sample_us)
        {
            const int64_t wait_us = next_sample_us - now_us;

            if (wait_us > 1000)
            {
                vTaskDelay(pdMS_TO_TICKS(1));
            }
            else
            {
                taskYIELD();
            }

            continue;
        }

        next_sample_us += sample_interval_us;

        const uint64_t now_ms = static_cast<uint64_t>(now_us / 1000);

        uint8_t ui_event_code = 0;

        biosense::ui_event event =
            biosense::ui_event::none;

        if (buttons.poll_event(event))
        {
            ui_event_code = static_cast<uint8_t>(event);

            ESP_LOGI(TAG, "UI event: %u", static_cast<unsigned>(ui_event_code));

            switch (event)
            {
                case biosense::ui_event::marker:
                {
                    buzzer.play(
                        biosense::buzzer_pattern::marker_set
                    );

                    break;
                }

                case biosense::ui_event::session_toggle:
                {
                    session_active = !session_active;

                    buzzer.play(
                        session_active
                            ? biosense::buzzer_pattern::session_start
                            : biosense::buzzer_pattern::session_stop
                    );

                    ESP_LOGI(
                        TAG,
                        "Session active: %d",
                        session_active ? 1 : 0
                    );

                    break;
                }

                case biosense::ui_event::mute_toggle:
                {
                    if (buzzer.muted())
                    {
                        buzzer.set_muted(false);

                        buzzer.play(
                            biosense::buzzer_pattern::mute_off
                        );

                        ESP_LOGI(TAG, "Buzzer unmuted");
                    }
                    else
                    {
                        buzzer.set_muted(true);

                        ESP_LOGI(TAG, "Buzzer muted");
                    }

                    break;
                }

                case biosense::ui_event::reset_measurements:
                {
                    ecg.reset();

                    if (i2c_ready)
                    {
                        i2c.reset_processing();
                    }

                    buzzer.play(
                        biosense::buzzer_pattern::reset_done
                    );

                    ESP_LOGI(TAG, "Measurement processing reset");

                    break;
                }

                case biosense::ui_event::diagnostics:
                {
                    /*
                     * A+B długo:
                     * na razie zdarzenie diagnostyczne.
                     *
                     * Reconnect Wi-Fi dodamy przez osobną metodę,
                     * aby nie inicjalizować ponownie całego stosu Wi-Fi.
                     */
                    buzzer.play(
                        biosense::buzzer_pattern::sensor_error
                    );

                    ESP_LOGI(TAG, "Diagnostic event requested");

                    break;
                }

                case biosense::ui_event::none:
                default:
                {
                    break;
                }
            }
        }

        if (i2c_ready)
        {
            i2c.update(now_ms);
        }

        const auto& i2c_data = i2c.readings();

        const bool leads_off = adc.leads_off();

        int raw_ecg = 0;

        if (!leads_off)
        {
            raw_ecg = adc.read_ecg_raw();
        }

        const biosense::ecg_result ecg_result =
            ecg.process(raw_ecg, leads_off, now_ms);

        if (mqtt_ecg_batch.count == 0)
        {
            mqtt_ecg_batch.sample_start =
                sample_index;
        }

        const std::size_t ecg_position =
            mqtt_ecg_batch.count;

        mqtt_ecg_batch.raw[ecg_position] =
            ecg_result.raw;

        mqtt_ecg_batch.filtered[ecg_position] =
            ecg_result.filtered;

        mqtt_ecg_batch.rpeak[ecg_position] =
            ecg_result.rpeak ? 1U : 0U;

        mqtt_ecg_batch.leads_off[ecg_position] =
            ecg_result.leads_off ? 1U : 0U;

        mqtt_ecg_batch.count++;

        if (
            mqtt_ecg_batch.count ==
            biosense::MQTT_ECG_BATCH_SIZE
        )
        {
            const esp_err_t error =
                mqtt.publish_ecg_batch(
                    mqtt_ecg_batch
                );

            if (error != ESP_OK)
            {
                ESP_LOGW(
                    TAG,
                    "MQTT ECG queueing failed: %s",
                    esp_err_to_name(error)
                );
            }

            mqtt_ecg_batch.count = 0;
        }

        /*
        * GSR is sampled at 10 Hz.
        * Two samples are sent in each MQTT batch,
        * resulting in five MQTT messages per second.
        */
        if (
            (
                sample_index
                % GSR_SAMPLE_INTERVAL_SAMPLES
            )
            == 0U
        )
        {
            gsr = adc.read_gsr();

            if (mqtt_gsr_batch.count == 0)
            {
                mqtt_gsr_batch.sample_start =
                    sample_index;

                mqtt_gsr_batch.calibration_raw =
                    biosense::GSR_CONFIG
                        .calibration_raw;
            }

            const std::size_t gsr_position =
                mqtt_gsr_batch.count;

            mqtt_gsr_batch.raw[gsr_position] =
                gsr.raw;

            mqtt_gsr_batch.resistance_ohm[
                gsr_position
            ] = gsr.resistance_ohm;

            mqtt_gsr_batch.conductance_us[
                gsr_position
            ] = gsr.conductance_us;

            mqtt_gsr_batch.valid[gsr_position] =
                gsr.valid ? 1U : 0U;

            mqtt_gsr_batch.count++;

            if (
                mqtt_gsr_batch.count
                == biosense::MQTT_GSR_BATCH_SIZE
            )
            {
                const esp_err_t error =
                    mqtt.publish_gsr_batch(
                        mqtt_gsr_batch
                    );

                if (error != ESP_OK)
                {
                    ESP_LOGW(
                        TAG,
                        "MQTT GSR queueing failed: %s",
                        esp_err_to_name(error)
                    );
                }

                mqtt_gsr_batch.count = 0;
            }
        }

        if ((sample_index % 50U) == 0U)
        {
            const float new_batt_v = adc.read_batt_voltage();

            if (!batt_filter_init)
            {
                filtered_batt_v = new_batt_v;
                batt_filter_init = true;
            }
            else
            {
                filtered_batt_v += BATT_FILTER_ALPHA * (new_batt_v - filtered_batt_v);
            }
            
            batt_v = filtered_batt_v;
        }

        const uint8_t batt_percent = biosense::battery_voltage_to_percent(batt_v);

        if ((sample_index % ECG_SAMPLE_RATE_HZ) == 0U)
        {
            leds.set_battery_level(batt_percent);
        }

        biosense::telemetry_frame frame = {};

        frame.sample = sample_index;

        frame.raw_ecg = ecg_result.raw;
        frame.filtered_ecg = ecg_result.filtered;
        frame.ecg_bpm = ecg_result.bpm;
        frame.rpeak = ecg_result.rpeak;
        frame.ecg_leads_off = ecg_result.leads_off;

        frame.gsr_adc = static_cast<int>(gsr.raw);
        frame.gsr_resistance_ohm = gsr.resistance_ohm;
        frame.gsr_conductance_us = gsr.conductance_us;
        frame.gsr_valid = gsr.valid;
        frame.gsr_calibration_raw = biosense::GSR_CONFIG.calibration_raw;

        frame.batt_v = batt_v;
        frame.batt_percent = batt_percent;

        if (i2c_data.mlx_valid)
        {
            frame.temp_amb = i2c_data.temp_amb;
            frame.temp_obj = i2c_data.temp_obj;
        }
        else
        {
            frame.temp_amb = 0.0f;
            frame.temp_obj = 0.0f;
        }

        /*
         * MAX30102 / MAX30105.
         */
        if (i2c_data.max_ok)
        {
            frame.max_bpm = i2c_data.max_bpm;
            frame.spo2 = i2c_data.max_spo2;
            frame.spo2_valid = i2c_data.max_spo2_valid;

            frame.spo2_raw =
                static_cast<int>(i2c_data.max_spo2_raw);

            frame.max_finger = i2c_data.max_finger;

            frame.max_red =
                static_cast<int>(i2c_data.max_red);

            frame.max_ired =
                static_cast<int>(i2c_data.max_ired);
        }
        else
        {
            frame.max_bpm = 0;
            frame.spo2 = 0.0f;
            frame.spo2_valid = false;
            frame.spo2_raw = 0;

            frame.max_finger = false;
            frame.max_red = 0;
            frame.max_ired = 0;
        }

        frame.ui_event = ui_event_code;
        frame.ui_flags = 0;

        if (session_active)
        {
            frame.ui_flags |= UI_FLAG_SESSION_ACTIVE;
        }

        if (buzzer.muted())
        {
            frame.ui_flags |= UI_FLAG_BUZZER_MUTED;
        }

        if ((sample_index % ECG_SAMPLE_RATE_HZ) == 0U)
        {
            const esp_err_t error =
                mqtt.publish_metrics(frame);

            if (error != ESP_OK)
            {
                ESP_LOGW(
                    TAG,
                    "MQTT metrics queueing failed: %s",
                    esp_err_to_name(error)
                );
            }
        }

        if (ui_event_code != 0U)
        {
            const esp_err_t error =
                mqtt.publish_event(
                    ui_event_code,
                    frame.ui_flags,
                    sample_index
                );

            if (error != ESP_OK)
            {
                ESP_LOGW(
                    TAG,
                    "MQTT event queueing failed: %s",
                    esp_err_to_name(error)
                );
            }
        }

        if (
            sample_index > 0U &&
            (sample_index % ECG_SAMPLE_RATE_HZ) == 0U
        )
        {
            const biosense::mqtt_sender_stats mqtt_stats =
                mqtt.stats();
            
            ESP_LOGI(
                TAG,
                "GSR: raw=%u, calibration=%u, "
                "resistance=%.1f ohm, "
                "conductance=%.3f uS, valid=%d",
                static_cast<unsigned>(gsr.raw),
                static_cast<unsigned>(
                    biosense::GSR_CONFIG
                        .calibration_raw
                ),
                gsr.resistance_ohm,
                gsr.conductance_us,
                gsr.valid ? 1 : 0
            );

            ESP_LOGI(
                TAG,
                "MQTT stats: sample=%lu, connected=%d, "
                "outbox=%d, "
                "ecg_q=%lu, ecg_sent=%lu, "
                "ecg_drop=%lu, ecg_depth=%lu, "
                "gsr_q=%lu, gsr_sent=%lu, "
                "gsr_drop=%lu, gsr_depth=%lu, " 
                "metrics_q=%lu, metrics_sent=%lu, "
                "metrics_overwrite=%lu, "
                "events_q=%lu, events_sent=%lu, "
                "events_drop=%lu, publish_errors=%lu",
                static_cast<unsigned long>(sample_index),
                mqtt.connected() ? 1 : 0,
                mqtt.outbox_size(),
                static_cast<unsigned long>(
                    mqtt_stats.ecg_queued
                ),
                static_cast<unsigned long>(
                    mqtt_stats.ecg_sent
                ),
                static_cast<unsigned long>(
                    mqtt_stats.ecg_dropped
                ),
                static_cast<unsigned long>(
                    mqtt_stats.ecg_queue_depth
                ),
                static_cast<unsigned long>(
                    mqtt_stats.gsr_queued
                ),
                static_cast<unsigned long>(
                    mqtt_stats.gsr_sent
                ),
                static_cast<unsigned long>(
                    mqtt_stats.gsr_dropped
                ),
                static_cast<unsigned long>(
                    mqtt_stats.gsr_queue_depth
                ),
                static_cast<unsigned long>(
                    mqtt_stats.metrics_queued
                ),
                static_cast<unsigned long>(
                    mqtt_stats.metrics_sent
                ),
                static_cast<unsigned long>(
                    mqtt_stats.metrics_overwritten
                ),
                static_cast<unsigned long>(
                    mqtt_stats.events_queued
                ),
                static_cast<unsigned long>(
                    mqtt_stats.events_sent
                ),
                static_cast<unsigned long>(
                    mqtt_stats.events_dropped
                ),
                static_cast<unsigned long>(
                    mqtt_stats.publish_errors
                )
            );
        }

        char line[256];

        const int written = biosense::format_telemetry_csv(
            line,
            sizeof(line),
            frame
        );

        if (written > 0 &&
            written < static_cast<int>(sizeof(line)))
        {
            if (udp_len + written >=
                static_cast<int>(sizeof(udp_buffer)))
            {
                if (udp_len > 0)
                {
                    udp.send(
                        udp_buffer,
                        static_cast<size_t>(udp_len)
                    );
                }

                udp_len = 0;
                batch_count = 0;
            }

            std::memcpy(
                udp_buffer + udp_len,
                line,
                static_cast<size_t>(written)
            );

            udp_len += written;
            batch_count++;
        }

        if (batch_count >= UDP_BATCH_SIZE)
        {
            if (udp_len > 0)
            {
                udp.send(
                    udp_buffer,
                    static_cast<size_t>(udp_len)
                );
            }

            udp_len = 0;
            batch_count = 0;
        }

        sample_index++;
    }
}

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "BioSense ESP-IDF C++ start");

    ESP_ERROR_CHECK(buzzer.init());

    buzzer.play(
        biosense::buzzer_pattern::startup
    );

    init_nvs();

    ESP_ERROR_CHECK(adc.init());
    ESP_ERROR_CHECK(buttons.init());
    ESP_ERROR_CHECK(leds.init());

    ESP_ERROR_CHECK(
        leds.set_status(
            biosense::led_status::wifi_connecting
        )
    );

    const esp_err_t i2c_err = i2c.init();

    if (i2c_err == ESP_OK)
    {
        i2c_ready = true;
    }
    else
    {
        i2c_ready = false;

        ESP_LOGW(
            TAG,
            "I2C init failed: %s",
            esp_err_to_name(i2c_err)
        );

        buzzer.play(
            biosense::buzzer_pattern::sensor_error
        );
    }

    ESP_ERROR_CHECK(wifi.connect());

    const esp_err_t mqtt_err = mqtt.init();

    if (mqtt_err != ESP_OK)
    {
        ESP_LOGW(TAG, "MQTT init failed: %s", esp_err_to_name(mqtt_err));
    }
    

    ESP_ERROR_CHECK(
        udp.init(
            UDP_TARGET_IP,
            UDP_TARGET_PORT
        )
    );

    buzzer.play(
        biosense::buzzer_pattern::wifi_connected
    );

    ESP_ERROR_CHECK(
        leds.set_status(
            biosense::led_status::wifi_connected
        )
    );

    vTaskDelay(pdMS_TO_TICKS(300));

    const BaseType_t task_result =
        xTaskCreatePinnedToCore(
            sampling_task,
            "sampling_task",
            12288,
            nullptr,
            5,
            nullptr,
            1
        );

    if (task_result != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create sampling task");
        ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
    }
}