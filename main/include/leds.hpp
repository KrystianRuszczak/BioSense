#pragma once

#include <cstdint>

#include "esp_err.h"

namespace biosense
{
    
enum class led_status
{
    battery,
    wifi_connecting,
    wifi_connected,
    sensor_error,
    critical_battery
};

class leds
{
public:
    esp_err_t init();

    esp_err_t clear();

    esp_err_t set_battery_level(float batt_v);

    esp_err_t set_status(led_status status);

private:
    esp_err_t set_pixel(uint8_t index, uint8_t r, uint8_t g, uint8_t b);
    esp_err_t refresh();

    void voltage_to_led_count_and_color(float batt_v, uint8_t& count, uint8_t& r, uint8_t& g, uint8_t& b);
    void* strip_handle_ = nullptr;
};

} // namespace biosense
