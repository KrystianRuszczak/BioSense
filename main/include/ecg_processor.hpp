#pragma once

#include <cstdint>

namespace biosense
{

struct ecg_result
{
    int raw = 0;
    int filtered = 2048;
    unsigned bpm = 0;
    bool rpeak = false;
    bool leads_off = true;
};

class ecg_processor
{
private:
    float filtered_ = 2048.0f;
    float signal_min_ = 4095.0f;
    float signal_max_ = 0.0f;
    uint64_t last_peak_ms_ = 0;
    unsigned bpm_ = 0;
public:
    ecg_result process(int raw_adc, bool leads_off, uint64_t now_ms);

    void reset();
};

} // namespace biosense
