#include "ecg_processor.hpp"

#include "biosense_config.hpp"

namespace biosense
{
    
ecg_result ecg_processor::process(int raw_adc, bool leads_off, uint64_t now_ms)
{
    ecg_result result = {};
    result.raw = raw_adc;
    result.leads_off = leads_off;

    if (leads_off)
    {
        filtered_ = 2048.0f;
        bpm_ = 0;

        result.filtered = static_cast<int>(filtered_);
        result.bpm = bpm_;
        result.rpeak = false;

        return result;
    }
    
    filtered_ = ECG_LPF_ALPHA * static_cast<float>(raw_adc) + (1.0f - ECG_LPF_ALPHA) * filtered_;

    signal_min_ = 0.999f * signal_min_ + 0.001f * filtered_;
    signal_max_ = 0.999f * signal_max_ + 0.001f * filtered_;

    if (filtered_ < signal_min_)
    {
        signal_min_ = filtered_;
    }
    
    if (filtered_ > signal_max_)
    {
        signal_max_ = filtered_;
    }
    
    float amplitude = signal_max_ - signal_min_;
    float threshold = signal_min_ + amplitude * ECG_RPEAK_THRESHOLD_RATIO;

    bool is_rpeak = amplitude > 150.0f && filtered_ > threshold && (now_ms - last_peak_ms_ > ECG_RPEAK_REFRACTORY_MS);

    if (is_rpeak)
    {
        uint64_t rr = now_ms - last_peak_ms_;

        if (rr > 300 && rr < 2000)
        {
            bpm_ = static_cast<unsigned>(60000ULL / rr);
        }

        last_peak_ms_ = now_ms;
    }
    
    result.filtered = static_cast<int>(filtered_);
    result.bpm = bpm_;
    result.rpeak = is_rpeak;

    return result;
}

} // namespace biosense

