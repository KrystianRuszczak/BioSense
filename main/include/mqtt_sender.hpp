#pragma once

#include <atomic>
#include <cstdint>
#include <cstddef>

#include "esp_err.h"
#include "esp_event.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "telemetry.hpp"

namespace biosense
{

static constexpr std::size_t MQTT_ECG_BATCH_SIZE = 25;
static constexpr size_t MQTT_GSR_BATCH_SIZE = 2;

struct mqtt_ecg_batch
{
    uint32_t sample_start = 0;
    uint16_t count = 0;

    int raw[MQTT_ECG_BATCH_SIZE] = {};
    int filtered[MQTT_ECG_BATCH_SIZE] = {};

    uint8_t rpeak[MQTT_ECG_BATCH_SIZE] = {};
    uint8_t leads_off[MQTT_ECG_BATCH_SIZE] = {};
};

struct mqtt_gsr_batch
{
    uint32_t sample_start = 0;
    uint16_t count = 0;
    uint16_t calibration_raw = 0;

    uint16_t raw[MQTT_GSR_BATCH_SIZE] = {};
    
    float resistance_ohm[MQTT_GSR_BATCH_SIZE] = {};
    float conductance_us[MQTT_GSR_BATCH_SIZE] = {};

    uint8_t valid[MQTT_GSR_BATCH_SIZE] = {};
};

struct mqtt_sender_stats
{
    uint32_t ecg_queued = 0;
    uint32_t ecg_sent = 0;
    uint32_t ecg_dropped = 0;
    uint32_t ecg_queue_depth = 0;

    uint32_t gsr_queued = 0;
    uint32_t gsr_sent = 0;
    uint32_t gsr_dropped = 0;
    uint32_t gsr_queue_depth = 0;

    uint32_t metrics_queued = 0;
    uint32_t metrics_sent = 0;
    uint32_t metrics_overwritten = 0;
    uint32_t metrics_queue_depth = 0;

    uint32_t events_queued = 0;
    uint32_t events_sent = 0;
    uint32_t events_dropped = 0;
    uint32_t events_queue_depth = 0;

    uint32_t publish_errors = 0;
};

    
class mqtt_sender
{
private:
    struct mqtt_event_message
    {
        uint8_t event_code = 0;
        uint8_t ui_flags = 0;
        uint32_t sample = 0;
    };
    
    static constexpr UBaseType_t ECG_QUEUE_LEN = 64;
    static constexpr UBaseType_t GSR_QUEUE_LEN = 64;
    static constexpr UBaseType_t METRICS_QUEUE_LEN = 1;
    static constexpr UBaseType_t EVENTS_QUEUE_LEN = 8;

    static constexpr uint32_t PUBLISHER_TASK_STACK_SIZE = 10240;
    static constexpr UBaseType_t PUBLISHER_TASK_PRIOR = 4;
    static constexpr BaseType_t PUBLISHER_TASK_CORE = 0;

    static void event_handler(void* handler_arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
    
    static void publisher_task_entry(void* arg);

    void handle_event(int32_t event_id, void* event_data);

    esp_err_t create_runtime();
    void destroy_runtime();

    void publisher_task();
    void notify_publisher();

    esp_err_t publish_status(bool online);

    esp_err_t publish_payload_now(const char* topic, const char* payload, int payload_length, int qos, bool retain);

    esp_err_t publish_metrics_now(const telemetry_frame& frame);

    esp_err_t publish_event_now(const mqtt_event_message& event);

    esp_err_t publish_ecg_batch_now(const mqtt_ecg_batch& batch);

    esp_err_t publish_gsr_batch_now(const mqtt_gsr_batch& batch);

    static bool should_retry_publish(esp_err_t error);

    void* client_handle_ = nullptr;

    QueueHandle_t ecg_queue_ = nullptr;
    QueueHandle_t gsr_queue_ = nullptr;
    QueueHandle_t metrics_queue_ = nullptr;
    QueueHandle_t events_queue_ = nullptr;

    TaskHandle_t publisher_task_handle_ = nullptr;

    std::atomic_bool connected_{false};

    std::atomic<uint32_t> ecg_queued_{0};
    std::atomic<uint32_t> ecg_sent_{0};
    std::atomic<uint32_t> ecg_dropped_{0};

    std::atomic<uint32_t> gsr_queued_{0};
    std::atomic<uint32_t> gsr_sent_{0};
    std::atomic<uint32_t> gsr_dropped_{0};

    std::atomic<uint32_t> metrics_queued_{0};
    std::atomic<uint32_t> metrics_sent_{0};
    std::atomic<uint32_t> metrics_overwritten_{0};

    std::atomic<uint32_t> events_queued_{0};
    std::atomic<uint32_t> events_sent_{0};
    std::atomic<uint32_t> events_dropped_{0};

    std::atomic<uint32_t> publish_errors_{0};
public:
    esp_err_t init();

    bool connected() const;

    esp_err_t publish_metrics(const telemetry_frame& frame);
    esp_err_t publish_event (uint8_t event_code, uint8_t ui_flags, uint32_t sample);

    esp_err_t publish_ecg_batch(const mqtt_ecg_batch& batch);

    esp_err_t publish_gsr_batch(const mqtt_gsr_batch& batch);

    mqtt_sender_stats stats() const;

    int outbox_size() const;
};

} // namespace biosense
