#include "ui_state.hpp"

namespace biosense
{

ui_state g_ui_state;

void post_ui_event(ui_event event)
{
    g_ui_state.event = event;
}

ui_event consume_ui_event()
{
    ui_event e = g_ui_state.event;
    g_ui_state.event = ui_event::none;
    return e;
}

} // namespace biosense
