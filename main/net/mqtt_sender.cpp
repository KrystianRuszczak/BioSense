#include "mqtt_sender.hpp"

#include <cstdio>
#include <cstring>
#include <type_traits>

#include "biosense_secrets.hpp"
#include "biosense_config.hpp"

#include "esp_crt_bundle.h"
#include "esp_log.h"
#include "mqtt_client.h"

namespace biosense
{
    
static const char* TAG = "mqtt_sender";

static constexpr char TOPIC_METRICS[] = "biosense/" MQTT_DEVICE_ID "/metrics";
static constexpr char TOPIC_EVENTS[] = "biosense/" MQTT_DEVICE_ID "/events";
static constexpr char TOPIC_STATUS[] = "biosense/" MQTT_DEVICE_ID "/status";

static constexpr char TOPIC_ECG[] = "biosense/" MQTT_DEVICE_ID "/ecg";
static constexpr char TOPIC_GSR[] = "biosense/" MQTT_DEVICE_ID "/gsr";

static constexpr char STATUS_ONLINE[] = "{\"online\":true}";
static constexpr char STATUS_OFFLINE[] = "{\"online\":false}";

static_assert(std::is_trivially_copyable_v<telemetry_frame>, "telemetry_frame must be trivially copyable");
static_assert(std::is_trivially_copyable_v<mqtt_ecg_batch>, "mqtt_ecg_batch must be trivially copyable");
static_assert(std::is_trivially_copyable_v<mqtt_gsr_batch>, "mqtt_gsr_batch must be trivially copyable");

esp_err_t mqtt_sender::init()
{
    if (client_handle_ != nullptr)
    {
        return ESP_ERR_INVALID_STATE;
    }
    
    esp_mqtt_client_config_t cfg = {};

    cfg.broker.address.uri = MQTT_BROKER_URI;
    cfg.broker.verification.crt_bundle_attach = esp_crt_bundle_attach;
    cfg.credentials.username = MQTT_USERNAME;
    cfg.credentials.authentication.password = MQTT_PASS;
    cfg.credentials.client_id = MQTT_DEVICE_ID;
    cfg.session.keepalive = 30;
    cfg.session.last_will.topic = TOPIC_STATUS;
    cfg.session.last_will.msg = STATUS_OFFLINE;

    cfg.session.last_will.qos = 1;
    cfg.session.last_will.retain = 1;

    cfg.network.reconnect_timeout_ms = 5000;

    cfg.buffer.size = 4096;
    cfg.buffer.out_size = 4096;

    cfg.outbox.limit = 16384;

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&cfg);

    if (client == nullptr)
    {
        ESP_LOGE(TAG, "esp_mqtt_client_init failed");
        return ESP_ERR_NO_MEM;
    }
    
    client_handle_ = client;

    esp_err_t err = esp_mqtt_client_register_event(client, MQTT_EVENT_ANY, mqtt_sender::event_handler, this);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Event registration failed: %s", esp_err_to_name(err));

        esp_mqtt_client_destroy(client);
        client_handle_ = nullptr;

        return err;
    }

    err = create_runtime();

    if (err != ESP_OK)
    {
        esp_mqtt_client_destroy(client);
        client_handle_ = nullptr;

        return err;
    }
    
    err = esp_mqtt_client_start(client);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "MQTT start failed: %s", esp_err_to_name(err));

        esp_mqtt_client_destroy(client);
        client_handle_ = nullptr;

        return err;
    }

    ESP_LOGI(TAG, "MQTT client started: %s", MQTT_DEVICE_ID);

    return ESP_OK;
}

esp_err_t mqtt_sender::create_runtime()
{
    ecg_queue_ = xQueueCreate(ECG_QUEUE_LEN, sizeof(mqtt_ecg_batch));
    gsr_queue_ = xQueueCreate(GSR_QUEUE_LEN, sizeof(mqtt_gsr_batch));
    metrics_queue_ = xQueueCreate(METRICS_QUEUE_LEN, sizeof(telemetry_frame));
    events_queue_ = xQueueCreate(EVENTS_QUEUE_LEN, sizeof(mqtt_event_message));

    if (ecg_queue_ == nullptr || gsr_queue_ == nullptr || metrics_queue_ == nullptr || events_queue_ == nullptr)
    {
        ESP_LOGE(TAG, "Failed to create MQTT queues");

        destroy_runtime();

        return ESP_ERR_NO_MEM;
    }
    
    const BaseType_t task_result = xTaskCreatePinnedToCore(
        mqtt_sender::publisher_task_entry,
        "mqtt_publish",
        PUBLISHER_TASK_STACK_SIZE,
        this,
        PUBLISHER_TASK_PRIOR,
        &publisher_task_handle_,
        PUBLISHER_TASK_CORE
    );

    if (task_result != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create MQTT publisher task");

        destroy_runtime();

        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

void mqtt_sender::destroy_runtime()
{
    if (publisher_task_handle_ != nullptr)
    {
        vTaskDelete(publisher_task_handle_);
        publisher_task_handle_ = nullptr;
    }

    if (events_queue_ != nullptr)
    {
        vQueueDelete(events_queue_);
        events_queue_ = nullptr;
    }
    
    if (metrics_queue_ != nullptr)
    {
        vQueueDelete(metrics_queue_);
        metrics_queue_ = nullptr;
    }

    if (gsr_queue_ != nullptr)
    {
        vQueueDelete(gsr_queue_);
        gsr_queue_ = nullptr;
    }
    

    if (ecg_queue_ != nullptr)
    {
        vQueueDelete(ecg_queue_);
        ecg_queue_ = nullptr;
    }
}

bool mqtt_sender::connected() const
{
    return connected_.load();
}

void mqtt_sender::event_handler(void* handler_arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    (void)event_base;

    auto* self = static_cast<mqtt_sender*>(handler_arg);

    if (self == nullptr)
    {
        return;
    }
    
    self->handle_event(event_id, event_data);
}

void mqtt_sender::handle_event(int32_t event_id, void* event_data)
{
    switch (static_cast<esp_mqtt_event_id_t>(event_id))
    {
        case MQTT_EVENT_CONNECTED:
        {
            connected_.store(true);

            ESP_LOGI(TAG, "MQTT connected");

            publish_status(true);
            break;
        }

        case MQTT_EVENT_DISCONNECTED:
        {
            connected_.store(false);

            ESP_LOGW(TAG, "MQTT disconnected");
            break;
        }

        case MQTT_EVENT_PUBLISHED:
        {
            break;
        }

        case MQTT_EVENT_ERROR:
        {
            connected_.store(false);

            auto event = static_cast<esp_mqtt_event_handle_t>(event_data);

            ESP_LOGE(TAG, "MQTT error");

            if (event != nullptr && event->error_handle != nullptr)
            {
                ESP_LOGE(TAG, "TLS error=0x%x, stack=0x%x, error=%d", event->error_handle->esp_tls_last_esp_err, event->error_handle->esp_tls_stack_err, event->error_handle->esp_transport_sock_errno);
            }
            
            break;
        }
            
        default:
            break;
    }
}

esp_err_t mqtt_sender::publish_status(bool online)
{
    if (client_handle_ == nullptr || !connected_.load())
    {
        return ESP_ERR_INVALID_STATE;
    }
    
    auto client = static_cast<esp_mqtt_client_handle_t>(client_handle_);
    
    const char* payload = online ? STATUS_ONLINE : STATUS_OFFLINE;

    const int message_id = esp_mqtt_client_enqueue(
        client,
        TOPIC_STATUS,
        payload,
        static_cast<int>(std::strlen(payload)),
        1,
        1,
        true
    );

    if (message_id == -2)
    {
        return ESP_ERR_NO_MEM;
    }

    if (message_id < 0)
    {
        return ESP_FAIL;
    }

    return ESP_OK;    
}

esp_err_t mqtt_sender::publish_metrics(const telemetry_frame& frame)
{
    if (metrics_queue_ == nullptr)
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (xQueueSend(metrics_queue_, &frame, 0) != pdPASS)
    {
        metrics_overwritten_.fetch_add(1);

        if (xQueueOverwrite(metrics_queue_, &frame) != pdPASS)
        {
            return ESP_FAIL;
        }
        
    }
    
    metrics_queued_.fetch_add(1);
    notify_publisher();

    return ESP_OK;
}

esp_err_t mqtt_sender::publish_event(uint8_t event_code, uint8_t ui_flags, uint32_t sample)
{
    if (events_queue_ == nullptr)
    {
        return ESP_ERR_INVALID_STATE;
    }

    mqtt_event_message event = {};

    event.event_code = event_code;
    event.ui_flags = ui_flags;
    event.sample = sample;

    if (xQueueSend(events_queue_, &event, 0) != pdPASS)
    {
        events_dropped_.fetch_add(1);

        ESP_LOGE(TAG, "MQTT event queue is full");
        
        return ESP_ERR_NO_MEM;
    }
    
    events_queued_.fetch_add(1);
    notify_publisher();

    return ESP_OK;
}

esp_err_t mqtt_sender::publish_ecg_batch(const mqtt_ecg_batch& batch)
{
    if (batch.count == 0 || batch.count > MQTT_ECG_BATCH_SIZE)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (ecg_queue_ == nullptr)
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (xQueueSend(ecg_queue_, &batch, 0) != pdPASS)
    {
        mqtt_ecg_batch discarded_batch = {};

        if (xQueueReceive(ecg_queue_, &discarded_batch, 0) == pdPASS)
        {
            ecg_dropped_.fetch_add(1);
        }
        
        if (xQueueSend(ecg_queue_, &batch, 0) != pdPASS)
        {
            ecg_dropped_.fetch_add(1);

            ESP_LOGE(TAG, "MQTT ECG queue if full");

            return ESP_ERR_NO_MEM;
        }
    }
    
    ecg_queued_.fetch_add(1);
    notify_publisher();

    return ESP_OK;
}

esp_err_t mqtt_sender::publish_gsr_batch(const mqtt_gsr_batch& batch)
{
    if (batch.count == 0 || batch.count > MQTT_GSR_BATCH_SIZE)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (gsr_queue_ == nullptr)
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (xQueueSend(gsr_queue_, &batch, 0) != pdPASS)
    {
        mqtt_gsr_batch discarded_batch = {};

        if (xQueueReceive(gsr_queue_, &discarded_batch, 0) != pdPASS)
        {
            return ESP_FAIL;
        }
        
        gsr_dropped_.fetch_add(1);

        if (xQueueSend(gsr_queue_, &batch, 0) != pdPASS)
        {
            gsr_dropped_.fetch_add(1);

            ESP_LOGE(TAG, "MQTT GSR queue if full");

            return ESP_ERR_NO_MEM;
        }
    }
    
    gsr_queued_.fetch_add(1);
    notify_publisher();

    return ESP_OK;
}

void mqtt_sender::notify_publisher()
{
    if (publisher_task_handle_ != nullptr)
    {
        xTaskNotifyGive(publisher_task_handle_);
    }
}

void mqtt_sender::publisher_task_entry(void* arg)
{
    auto* self = static_cast<mqtt_sender*>(arg);

    if (self == nullptr)
    {
        vTaskDelete(nullptr);
        return;
    }
    
    self->publisher_task();
}

void mqtt_sender::publisher_task()
{
    ESP_LOGI(TAG, "MQTT publisher task started");

    while (true)
    {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(250));

        if (!connected_.load())
        {
            continue;
        }
        
        bool retry_later = false;

        do
        {
            bool processed_message = false;

            mqtt_event_message event = {};

            if (xQueueReceive(events_queue_, &event, 0) == pdPASS)
            {
                processed_message = true;

                const esp_err_t err = publish_event_now(event);

                if (err == ESP_OK)
                {
                    events_sent_.fetch_add(1);
                }
                else
                {
                    publish_errors_.fetch_add(1);

                    ESP_LOGW(TAG, "MQTT event send failed: %s", esp_err_to_name(err));

                    if (should_retry_publish(err))
                    {
                        if (xQueueSendToFront(events_queue_, &event, 0) != pdPASS)
                        {
                            events_dropped_.fetch_add(1);
                        }
                        
                        retry_later = true;
                    }
                    else
                    {
                        events_dropped_.fetch_add(1);
                    }
                }
            }
            
            if (retry_later || !connected_.load())
            {
                break;
            }
            
            telemetry_frame metrics = {};

            if (xQueueReceive(metrics_queue_, &metrics, 0) == pdPASS)
            {
                processed_message = true;

                const esp_err_t err = publish_metrics_now(metrics);

                if (err == ESP_OK)
                {
                    metrics_sent_.fetch_add(1);
                }
                else
                {
                    publish_errors_.fetch_add(1);

                    ESP_LOGW(TAG, "MQTT metrics send failed: %s", esp_err_to_name(err));

                    if (should_retry_publish(err))
                    {
                        retry_later = true;
                    }
                }
            }

            if (retry_later || !connected_.load())
            {
                break;
            }
            
            mqtt_ecg_batch batch = {};

            if (xQueueReceive(ecg_queue_, &batch, 0) == pdPASS)
            {
                processed_message = true;

                const esp_err_t err = publish_ecg_batch_now(batch);

                if (err == ESP_OK)
                {
                    ecg_sent_.fetch_add(1);
                }
                else
                {
                    publish_errors_.fetch_add(1);

                    ESP_LOGW(TAG, "MQTT ECG send failed: %s", esp_err_to_name(err));

                    if (should_retry_publish(err))
                    {
                        if (xQueueSendToFront(ecg_queue_, &batch, 0) != pdPASS)
                        {
                            ecg_dropped_.fetch_add(1);
                        }
                        
                        retry_later = true;
                    }
                    else
                    {
                        ecg_dropped_.fetch_add(1);
                    }
                }
            }

            if (retry_later || !connected_.load())
            {
                break;
            }
            
            mqtt_gsr_batch gsr_batch = {};

            if (xQueueReceive(gsr_queue_, &gsr_batch, 0) == pdPASS)
            {
                processed_message = true;

                const esp_err_t err = publish_gsr_batch_now(gsr_batch);

                if (err == ESP_OK)
                {
                    gsr_sent_.fetch_add(1);
                }
                else
                {
                    publish_errors_.fetch_add(1);

                    ESP_LOGW(TAG, "MQTT GSR send failed: %s", esp_err_to_name(err));

                    if (should_retry_publish(err))
                    {
                        if (xQueueSendToFront(gsr_queue_, &gsr_batch, 0) != pdPASS)
                        {
                            gsr_dropped_.fetch_add(1);
                        }
                        
                        retry_later = true;
                    }
                    else
                    {
                        gsr_dropped_.fetch_add(1);
                    }
                }
            }
            
            if (!processed_message)
            {
                break;
            }

        } while (connected_.load() && !retry_later);
        
        if (retry_later)
        {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

esp_err_t mqtt_sender::publish_payload_now(const char* topic, const char* payload, int payload_length, int qos, bool retain)
{
    if (client_handle_ == nullptr || !connected_.load())
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (topic == nullptr || payload == nullptr || payload_length <= 0)
    {
        return ESP_ERR_INVALID_ARG;
    }
    
    auto client = static_cast<esp_mqtt_client_handle_t>(client_handle_);

    const int message_id = esp_mqtt_client_publish(client, topic, payload, payload_length, qos, retain ? 1:0);

    if (message_id == -2)
    {
        return ESP_ERR_NO_MEM;
    }
    
    if (message_id < 0)
    {
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

esp_err_t mqtt_sender::publish_metrics_now(const telemetry_frame& frame)
{
    char payload[768];

    const int written = std::snprintf(
        payload,
        sizeof(payload),

        "{"
        "\"v\":1,"
        "\"device\":\"%s\","
        "\"sample\":%lu,"
        "\"ecg_bpm\":%u,"
        "\"leads_off\":%d,"
        "\"gsr\":%d,"
        "\"gsr_resistance_ohm\":%.3f,"
        "\"gsr_conductance_us\":%.6f,"
        "\"gsr_valid\":%d,"
        "\"gsr_calibration_raw\":%u,"
        "\"batt_v\":%.3f,"
        "\"batt_percent\":%u,"
        "\"temp_amb\":%.3f,"
        "\"temp_obj\":%.3f,"
        "\"max_bpm\":%d,"
        "\"spo2\":%.1f,"
        "\"spo2_valid\":%d,"
        "\"max_finger\":%d,"
        "\"ui_flags\":%u"
        "}",

        MQTT_DEVICE_ID,

        static_cast<unsigned long>(frame.sample),

        static_cast<unsigned>(frame.ecg_bpm),

        frame.ecg_leads_off ? 1:0,
        frame.gsr_adc,
        frame.gsr_resistance_ohm,
        frame.gsr_conductance_us,
        frame.gsr_valid ? 1:0,
        static_cast<unsigned>(frame.gsr_calibration_raw),
        frame.batt_v,
        static_cast<unsigned>(frame.batt_percent),
        frame.temp_amb,
        frame.temp_obj,
        frame.max_bpm,
        frame.spo2,
        frame.spo2_valid ? 1:0,
        frame.max_finger ? 1:0,

        static_cast<unsigned>(frame.ui_flags)
    );

    if (written <= 0 || written >= static_cast<int>(sizeof(payload)))
    {
        ESP_LOGE(TAG, "Metrics JSON overflow");

        return ESP_ERR_INVALID_SIZE;
    }
    
    return publish_payload_now(TOPIC_METRICS, payload, written, 0, false);
}

esp_err_t mqtt_sender::publish_event_now(const mqtt_event_message& event)
{
    char payload[256];

    const int written = std::snprintf(
        payload,
        sizeof(payload),

        "{"
        "\"v\":1,"
        "\"device\":\"%s\","
        "\"sample\":%lu,"
        "\"event\":%u,"
        "\"ui_flags\":%u"
        "}",

        MQTT_DEVICE_ID,
        static_cast<unsigned long>(event.sample),
        static_cast<unsigned>(event.event_code),
        static_cast<unsigned>(event.ui_flags)
    );

    if (written <= 0 || written >= static_cast<int>(sizeof(payload)))
    {
        ESP_LOGE(TAG, "Event JSON overflow");

        return ESP_ERR_INVALID_SIZE;
    }
    
    return publish_payload_now(TOPIC_EVENTS, payload, written, 1, false);
}

esp_err_t mqtt_sender::publish_ecg_batch_now(const mqtt_ecg_batch& batch)
{
    if (batch.count == 0 || batch.count > MQTT_ECG_BATCH_SIZE)
    {
        return ESP_ERR_INVALID_ARG;
    }
    
    char payload[2048];
    std::size_t used = 0;

    auto append = [&](const char* format, auto... args) -> bool {
        const std::size_t remaining = sizeof(payload) - used;

        const int written = std::snprintf(payload + used, remaining, format, args...);

        if (written < 0 || static_cast<std::size_t>(written) >= remaining)
        {
            return false;
        }
        
        used += static_cast<std::size_t>(written);

        return true;
    };

    if (!append(
        "{"
        "\"v\":1,"
        "\"device\":\"%s\","
        "\"sample_start\":%lu,"
        "\"sample_rate\":%u,"
        "\"count\":%u,"
        "\"raw\":[",

        MQTT_DEVICE_ID,
        static_cast<unsigned long>(batch.sample_start),
        static_cast<unsigned>(ECG_SAMPLE_RATE_HZ),
        static_cast<unsigned>(batch.count)
    ))
    {
        return ESP_ERR_INVALID_SIZE;
    }

    for (std::size_t i = 0; i < batch.count; i++)
    {
        if (!append("%s%d", i == 0 ? "":",", batch.raw[i]))
        {
            return ESP_ERR_INVALID_SIZE;
        }
    }
    
    if (!append("],\"filtered\":["))
    {
        return ESP_ERR_INVALID_SIZE;
    }
    
    for (std::size_t i = 0; i < batch.count; i++)
    {
        if (!append("%s%d", i == 0 ? "":",", batch.filtered[i]))
        {
            return ESP_ERR_INVALID_SIZE;
        }
        
    }
    
    if (!append("],\"rpeak\":["))
    {
        return ESP_ERR_INVALID_SIZE;
    }
    
    for (std::size_t i = 0; i < batch.count; i++)
    {
        if (!append("%s%u", i == 0 ? "":",", static_cast<unsigned>(batch.rpeak[i])))
        {
            return ESP_ERR_INVALID_SIZE;
        }
    }

    if (!append("],\"leads_off\":["))
    {
        return ESP_ERR_INVALID_SIZE;
    }
    
    for (std::size_t i = 0; i < batch.count; i++)
    {
        if (!append("%s%u", i == 0 ? "":",", static_cast<unsigned>(batch.leads_off[i])))
        {
            return ESP_ERR_INVALID_SIZE;
        }
    }

    if (!append("]}"))
    {
        return ESP_ERR_INVALID_SIZE;
    }
    
    return publish_payload_now(TOPIC_ECG, payload, static_cast<int>(used), 0, false);
}

esp_err_t mqtt_sender::publish_gsr_batch_now(const mqtt_gsr_batch& batch)
{
    if (batch.count == 0 || batch.count > MQTT_GSR_BATCH_SIZE)
    {
        return ESP_ERR_INVALID_ARG;
    }
    
    char payload[1024];
    size_t used = 0;

    auto append = [&](const char* format, auto... args) -> bool {
        const size_t remaining = sizeof(payload) - used;

        const int written = snprintf(payload + used, remaining, format, args...);

        if (written < 0 || static_cast<size_t>(written) >= remaining)
        {
            return false;
        }
        
        used += static_cast<size_t>(written);

        return true;
    };

    if (!append(
        "{"
        "\"v\":1,"
        "\"device\":\"%s\","
        "\"sample_start\":%lu,"
        "\"sample_step\":%u,"
        "\"sample_rate\":%u,"
        "\"count\":%u,"
        "\"calibration_raw\":%u,"
        "\"raw\":[",

        MQTT_DEVICE_ID,
        static_cast<unsigned long>(batch.sample_start),
        static_cast<unsigned>(GSR_SAMPLE_INTERVAL_SAMPLES),
        static_cast<unsigned>(GSR_SAMPLE_RATE_HZ),
        static_cast<unsigned>(batch.count),
        static_cast<unsigned>(batch.calibration_raw)
    ))
    {
        return ESP_ERR_INVALID_SIZE;
    }

    for (size_t i = 0; i < batch.count; i++)
    {
        if (!append("%s%u", i == 0 ? "":",", static_cast<unsigned>(batch.raw[i])))
        {
            return ESP_ERR_INVALID_SIZE;
        }
    }
    
    if (!append("],\"resistance_ohm\":["))
    {
        return ESP_ERR_INVALID_SIZE;
    }
    
    for (size_t i = 0; i < batch.count; i++)
    {
        if (!append("%s%.3f", i == 0 ? "":",", batch.resistance_ohm[i]))
        {
            return ESP_ERR_INVALID_SIZE;
        }
        
    }
    
    if (!append("],\"conductance_us\":["))
    {
        return ESP_ERR_INVALID_SIZE;
    }
    
    for (size_t i = 0; i < batch.count; i++)
    {
        if (!append("%s%.6f", i == 0 ? "":",", batch.conductance_us[i]))
        {
            return ESP_ERR_INVALID_SIZE;
        }
    }

    if (!append("],\"valid\":["))
    {
        return ESP_ERR_INVALID_SIZE;
    }
    
    for (std::size_t i = 0; i < batch.count; i++)
    {
        if (!append("%s%u", i == 0 ? "":",", static_cast<unsigned>(batch.valid[i])))
        {
            return ESP_ERR_INVALID_SIZE;
        }
    }

    if (!append("]}"))
    {
        return ESP_ERR_INVALID_SIZE;
    }
    
    return publish_payload_now(TOPIC_GSR, payload, static_cast<int>(used), 0, false);
}

bool mqtt_sender::should_retry_publish(esp_err_t error)
{
    return (
        error == ESP_FAIL ||
        error == ESP_ERR_INVALID_STATE ||
        error == ESP_ERR_NO_MEM ||
        error == ESP_ERR_TIMEOUT
    );
}

mqtt_sender_stats mqtt_sender::stats() const
{
    mqtt_sender_stats result = {};

    result.ecg_queued = ecg_queued_.load();
    result.ecg_sent = ecg_sent_.load();
    result.ecg_dropped = ecg_dropped_.load();

    if (ecg_queue_ != nullptr)
    {
        result.ecg_queue_depth = static_cast<uint32_t>(uxQueueMessagesWaiting(ecg_queue_));
    }

    result.gsr_queued = gsr_queued_.load();
    result.gsr_sent = gsr_sent_.load();
    result.gsr_dropped = gsr_dropped_.load();

    if (gsr_queue_ != nullptr)
    {
        result.gsr_queue_depth = static_cast<uint32_t>(uxQueueMessagesWaiting(gsr_queue_));
    }
    
    
    result.metrics_queued = metrics_queued_.load();
    result.metrics_sent = metrics_sent_.load();
    result.metrics_overwritten = metrics_overwritten_.load();

    if (metrics_queue_ != nullptr)
    {
        result.metrics_queue_depth = static_cast<uint32_t>(uxQueueMessagesWaiting(metrics_queue_));
    }
    
    result.events_queued = events_queued_.load();
    result.events_sent = events_sent_.load();
    result.events_dropped = events_dropped_.load();

    if (events_queue_ != nullptr)
    {
        result.events_queue_depth = static_cast<uint32_t>(uxQueueMessagesWaiting(events_queue_));
    }
    
    result.publish_errors = publish_errors_.load();

    return result;
}

int mqtt_sender::outbox_size() const
{
    if (client_handle_ == nullptr)
    {
        return 0;
    }

    auto client =
        static_cast<esp_mqtt_client_handle_t>(
            client_handle_
        );

    return esp_mqtt_client_get_outbox_size(client);
}


} // namespace biosense
