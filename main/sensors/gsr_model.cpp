#include "gsr_model.hpp"

#include <cmath>

namespace biosense {

namespace {

constexpr float RESIST_SCALE_OHM = 10000.0f;

gsr_reading invalid_reading(uint16_t raw)
{
    return {
        .raw = raw,
        .resistance_ohm = 0.0f,
        .conductance_us = 0.0f,
        .valid = false,
    };
}

gsr_reading convert_grove_v12(uint16_t raw, const gsr_sensor_config& config)
{
    if (raw == 0 || raw > config.adc_max_raw || config.calibration_raw > config.adc_max_raw || raw >= config.calibration_raw)
    {
        return invalid_reading(raw);
    }

    const float adc_range = static_cast<float>(config.adc_max_raw) + 1.0f;
    const float raw_value = static_cast<float>(raw);
    const float calibration = static_cast<float>(config.calibration_raw);
    const float resistance_ohm = ((adc_range + 2.0f * raw_value) * RESIST_SCALE_OHM) / (calibration - raw_value);

    if (!std::isfinite(resistance_ohm) || resistance_ohm <= 0.0f)
    {
        return invalid_reading(raw);
    }
    
    const float conductance_us = 1000000.0f / resistance_ohm;

    return {
        .raw = raw,
        .resistance_ohm = resistance_ohm,
        .conductance_us = conductance_us,
        .valid = true
    };
}

} // namespace

gsr_reading convert_gsr_raw(uint16_t raw, const gsr_sensor_config& config)
{
    switch (config.model)
    {
    case gsr_sensor_model::GROVE_V12:
        return convert_grove_v12(raw, config);
        break;
    
    default:
        break;
    }

    return invalid_reading(raw);
}

}   //namespace biosense