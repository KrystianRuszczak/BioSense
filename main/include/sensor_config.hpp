#pragma once

#include "gsr_model.hpp"

namespace biosense {

inline constexpr gsr_sensor_config GSR_CONFIG = {
    .model = gsr_sensor_model::GROVE_V12,

    .adc_max_raw = 4095,
    .calibration_raw = 2096,
};

}   //namespace biosense