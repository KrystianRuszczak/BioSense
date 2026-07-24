#pragma once

#include "esp_err.h"

namespace biosense
{

struct adc_readings
{
    int ecg_raw = 0;
    int gsr_raw = 0;
    int batt_raw = 0;
    float batt_v = 0.0f;
    bool leads_off = true;
};

class adc_sensors
{
private:
    int read_channel(int channel);
    float batt_voltage_from_raw(int raw) const;
    void* adc1_handle_ = nullptr;

    void* batt_cali_handle_ = nullptr;
    bool batt_cali_ready_ = false;

public:
    esp_err_t init();
    
    int read_ecg_raw();
    int read_gsr_raw();
    int read_batt_raw();
    float read_batt_voltage();
    bool leads_off() const;
};

} // namespace biosense
