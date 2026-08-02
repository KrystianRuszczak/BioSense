#pragma once

#include <cstdint>

namespace biosense{

enum class gsr_sensor_model {
    GROVE_V12
};

struct gsr_sensor_config
{
    gsr_sensor_model model;

    uint16_t adc_max_raw;
    uint16_t calibration_raw;
};

struct gsr_reading
{
    uint16_t raw;

    float resistance_ohm;
    float conductance_us;

    bool valid;
};

gsr_reading convert_gsr_raw(
    uint16_t raw,
    const gsr_sensor_config& config
);

}   // namespace biosense