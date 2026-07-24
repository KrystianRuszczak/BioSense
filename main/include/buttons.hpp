#pragma once

#include <cstdint>

#include "esp_err.h"

namespace biosense
{

enum class ui_event : uint8_t
{
    none = 0,

    marker = 1,
    session_toggle = 2,
    mute_toggle = 3,
    reset_measurements = 4,
    diagnostics = 5
};

class buttons
{
public:
    esp_err_t init();
    
    bool poll_event(ui_event& event);

private:
    struct button_state
    {
        bool raw_pressed = false;
        bool stable_pressed = false;
        bool supress_release = false;
        bool long_reported = false;

        uint64_t raw_change_ms = 0;
        uint64_t press_start_ms = 0;
    };

    enum class transition : uint8_t
    {
        none,
        pressed,
        released
    };

    static void task_entry(void* arg);
    void task_loop();

    transition update_button(button_state& state, bool raw_pressed, uint64_t now_ms);

    void handle_release(button_state& state, ui_event short_event, uint64_t now_ms);

    void post_event(ui_event event);

    void maybe_report_long(button_state& state, ui_event long_event, uint64_t now_ms);

    void* queue_handle_ = nullptr;
    void* task_handle_ = nullptr;

    button_state button_a_;
    button_state button_b_;

    uint64_t combo_start_ms_ = 0;
    bool combo_reported_ = false;
};

} // namespace biosense
