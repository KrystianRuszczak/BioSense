#include "telemetry.hpp"

#include <cstdio>

namespace biosense
{
   
int format_telemetry_csv(char* buffer, size_t buffer_size, const telemetry_frame& frame)
{
    return std::snprintf(
        buffer,
        buffer_size,
        "%lu,%d,%d,%u,%d,%d,%d,%.3f,%u,%.3f,%.3f,%d,%.1f,%d,%d,%d,%d,%d,%u,%u\n",
        static_cast<unsigned long>(frame.sample),
        frame.raw_ecg,
        frame.filtered_ecg,
        frame.ecg_bpm,
        frame.rpeak ? 1 : 0,
        frame.ecg_leads_off ? 1 : 0,
        frame.gsr_adc,
        frame.batt_v,
        static_cast<unsigned>(frame.batt_percent),
        frame.temp_amb,
        frame.temp_obj,
        frame.max_bpm,
        frame.spo2,
        frame.spo2_valid ? 1 : 0,
        frame.spo2_raw,
        frame.max_finger ? 1 : 0,
        frame.max_red,
        frame.max_ired,
        static_cast<unsigned>(frame.ui_event),
        static_cast<unsigned>(frame.ui_flags)
    );
}

} // namespace biosense