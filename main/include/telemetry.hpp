#pragma once

#include <cstddef>
#include <cstdint>

namespace biosense
{

struct telemetry_frame
{
    uint32_t sample = 0;

    int raw_ecg = 0;
    int filtered_ecg = 2048;
    unsigned ecg_bpm = 0;
    bool rpeak = false;
    bool ecg_leads_off = true;

    int gsr_adc = 0;
    float batt_v = 0.0f;
    uint8_t batt_percent;

    float temp_amb = 0.0f;
    float temp_obj = 0.0f;

    int max_bpm = 0;
    float spo2 = 0.0f;
    bool spo2_valid = false;
    int spo2_raw = 0;
    bool max_finger = false;
    int max_red = 0;
    int max_ired = 0;

    uint8_t ui_event = 0;
    uint8_t ui_flags = 0;
};

int format_telemetry_csv(char* buffer, size_t buffer_size, const telemetry_frame& frame);


} // namespace biosense
