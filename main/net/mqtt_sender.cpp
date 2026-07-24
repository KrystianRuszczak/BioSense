#include "mqtt_sender.hpp"

#include <cstdio>
#include <cstring>

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

static constexpr char STATUS_ONLINE[] = "{\"online\":true}";
static constexpr char STATUS_OFFLINE[] = "{\"online\":false}";

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

esp_err_t mqtt_sender::enqueue(const char* topic, const char* payload, int qos, bool retain, bool store)
{
    if (client_handle_ == nullptr || !connected_.load())
    {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (topic == nullptr || payload == nullptr)
    {
        return ESP_ERR_INVALID_ARG;
    }
    
    auto client = static_cast<esp_mqtt_client_handle_t>(client_handle_);
    
    const int message_id = esp_mqtt_client_enqueue(
        client,
        topic,
        payload,
        static_cast<int>(std::strlen(payload)),
        qos,
        retain ? 1: 0,
        store
    );

    if (message_id < 0)
    {
        ESP_LOGW(TAG, "MQTT enqueue failed: topic=%s result=%d", topic, message_id);
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t mqtt_sender::publish_status(bool online)
{
    return enqueue(TOPIC_STATUS, online ? STATUS_ONLINE : STATUS_OFFLINE, 1, true, true);
}

esp_err_t mqtt_sender::publish_metrics(const telemetry_frame& frame)
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

    return enqueue(TOPIC_METRICS, payload, 0, false, true);
}

esp_err_t mqtt_sender::publish_event(uint8_t event_code, uint8_t ui_flags, uint32_t sample)
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
        static_cast<unsigned long>(sample),
        static_cast<unsigned>(event_code),
        static_cast<unsigned>(ui_flags)
    );

    if (written <= 0 || written >= static_cast<int>(sizeof(payload)))
    {
        return ESP_ERR_INVALID_SIZE;
    }

    return enqueue(TOPIC_EVENTS, payload, 1, false, true);
}

esp_err_t mqtt_sender::publish_ecg_batch(const mqtt_ecg_batch& batch)
{
    if (batch.count == 0 || batch.count > MQTT_ECG_BATCH_SIZE)
    {
        return ESP_ERR_INVALID_ARG;
    }
    
    char payload[2048];
    std::size_t used = 0;

    auto append = [&](const char* format, auto... args) -> bool
    {
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
        if (!append("%s%d", i == 0 ? "" : ",", batch.raw[i]))
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
        if (!append("%s%d", i == 0 ? "" : ",", batch.filtered[i]))
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
        if (!append("%s%u", i == 0 ? "" : ",", static_cast<unsigned>(batch.rpeak[i])))
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
        if (!append("%s%u", i == 0 ? "" : ",", static_cast<unsigned>(batch.leads_off[i])))
        {
            return ESP_ERR_INVALID_SIZE;
        }
    }

    if (!append("]}"))
    {
        return ESP_ERR_INVALID_SIZE;
    }

    return enqueue(TOPIC_ECG, payload, 0, false, true);
}

} // namespace biosense
