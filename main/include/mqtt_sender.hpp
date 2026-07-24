#pragma once

#include <atomic>
#include <cstdint>
#include <cstddef>

#include "esp_err.h"
#include "esp_event.h"

#include "telemetry.hpp"

namespace biosense
{

static constexpr std::size_t MQTT_ECG_BATCH_SIZE = 25;

struct mqtt_ecg_batch
{
    uint32_t sample_start = 0;
    uint16_t count = 0;

    int raw[MQTT_ECG_BATCH_SIZE] = {};
    int filtered[MQTT_ECG_BATCH_SIZE] = {};

    uint8_t rpeak[MQTT_ECG_BATCH_SIZE] = {};
    uint8_t leads_off[MQTT_ECG_BATCH_SIZE] = {};
};

    
class mqtt_sender
{
private:
    static void event_handler(void* handler_arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
    
    void handle_event(int32_t event_id, void* event_data);

    esp_err_t publish_status(bool online);

    esp_err_t enqueue(const char* topic, const char* payload, int qos, bool retain, bool store);

    void* client_handle_ = nullptr;

    std::atomic_bool connected_{false};
public:
    esp_err_t init();

    bool connected() const;

    esp_err_t publish_metrics(const telemetry_frame& frame);
    esp_err_t publish_event (uint8_t event_code, uint8_t ui_flags, uint32_t sample);

    esp_err_t publish_ecg_batch(const mqtt_ecg_batch& batch);
};

} // namespace biosense
