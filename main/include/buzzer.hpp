#pragma once

#include <cstdint>

#include "esp_err.h"

namespace biosense
{
    
enum class buzzer_pattern
{
    startup,
    // button_click,
    wifi_connected,
    session_start,
    session_stop,
    sensor_error,
    low_battery,
    critical_battery,
    reset_done,
    mute_on,
    mute_off,
    marker_set
};

class buzzer
{
public:
    esp_err_t init();

    esp_err_t play(buzzer_pattern pattern);
    
    void set_muted(bool muted);
    bool muted() const;

    esp_err_t stop();

private:
    struct buzzer_command
    {
        buzzer_pattern pattern;
    };
    
    static void task_entry(void* arg);
    void task_loop();

    void run_pattern(buzzer_pattern pattern);

    esp_err_t start_tone(uint32_t freeq_hz);
    esp_err_t stop_tone();

    void tone(uint32_t freq_hz, uint32_t duration_ms);
    void pause(uint32_t duration_ms);

    void* queue_handle_ = nullptr;
    void* task_handle_ = nullptr;

    bool initialized_ = false;
    bool muted_ = false;
    
};

} // namespace biosense
