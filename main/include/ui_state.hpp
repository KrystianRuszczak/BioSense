#pragma once

#include <cstdint>

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

struct ui_state
{
    bool session_active = false;
    bool buzzer_muted = false;

    uint8_t alarm_flags = 0;
    ui_event event = ui_event::none;
};

extern ui_state g_ui_state;

void post_ui_event(ui_event event);
ui_event consume_ui_event();
} // namespace biosense
