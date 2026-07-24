#include "leds.hpp"

#include "biosense_config.hpp"

#include "esp_log.h"
#include "led_strip.h"
#include "led_strip_rmt.h"

static const char* TAG = "leds";

namespace biosense
{

esp_err_t leds::init()
{
    led_strip_config_t strip_config = {};
    strip_config.strip_gpio_num = PIN_LEDS;
    strip_config.max_leds = NUM_LEDS;
    strip_config.led_model = LED_MODEL_WS2812;
    strip_config.color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB;
    strip_config.flags.invert_out = false;

    led_strip_rmt_config_t rmt_config = {};
    rmt_config.clk_src = RMT_CLK_SRC_DEFAULT;
    rmt_config.resolution_hz = 10 * 1000 * 1000;
    rmt_config.mem_block_symbols = 64;
    rmt_config.flags.with_dma = false;

    led_strip_handle_t handle = nullptr;

    esp_err_t err = led_strip_new_rmt_device(&strip_config, &rmt_config, &handle);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "led_strip_new_rmt_device failed: %s", esp_err_to_name(err));
        return err;
    }

    strip_handle_ = handle;

    clear();

    ESP_LOGI(TAG, "LED strip initialized: GPIO=%d, leds=%d", PIN_LEDS, NUM_LEDS);

    return ESP_OK;
}

esp_err_t leds::clear()
{
    if (strip_handle_ == nullptr)
    {
        return ESP_ERR_INVALID_STATE;
    }

    auto handle = static_cast<led_strip_handle_t>(strip_handle_);

    return led_strip_clear(handle);
}

esp_err_t leds::set_pixel(uint8_t index, uint8_t r, uint8_t g, uint8_t b)
{
    if (strip_handle_ == nullptr)
    {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (index >= NUM_LEDS)
    {
        return ESP_ERR_INVALID_ARG;
    }

    auto handle = static_cast<led_strip_handle_t>(strip_handle_);
    return led_strip_set_pixel(handle, index, r, g, b);
}

esp_err_t leds::refresh()
{
    if (strip_handle_ == nullptr)
    {
        return ESP_ERR_INVALID_STATE;
    }
    
    auto handle = static_cast<led_strip_handle_t>(strip_handle_);
    return led_strip_refresh(handle);
}

void leds::percent_to_led_count_and_color(uint8_t percent, uint8_t& count, uint8_t& r, uint8_t& g, uint8_t& b)
{
    if (percent >= 75)
    {
        count = 4;
        r = 0;
        g = 20;             // Low brightness 20/255
        b = 0;
    }
    else if (percent >= 50)
    {
        count = 3;
        r = 4;
        g = 20;
        b = 0;
    }
    else if (percent >= 25)
    {
        count = 2;
        r = 20;
        g = 12;
        b = 0;
    }
    else if (percent >= 10)
    {
        count = 1;
        r = 20;
        g = 4;
        b = 0;
    }
    else
    {
        count = 1;
        r = 20;
        g = 0;
        b = 0;
    }
}

esp_err_t leds::set_battery_level(uint8_t percent)
{
    uint8_t count = 0;
    uint8_t r = 0;
    uint8_t g = 0;
    uint8_t b = 0;

    percent_to_led_count_and_color(percent, count, r, g, b);

    for (uint8_t i = 0; i < NUM_LEDS; i++)
    {
        if (i < count)
        {
            ESP_ERROR_CHECK_WITHOUT_ABORT(set_pixel(i, r, g, b));
        }
        else
        {
            ESP_ERROR_CHECK_WITHOUT_ABORT(set_pixel(i, 0 , 0, 0));
        }
    }

    return refresh();
}

esp_err_t leds::set_status(led_status status)
{
    switch (status)
    {
    case led_status::wifi_connecting:
        for (uint8_t i = 0; i < NUM_LEDS; i++)
        {
            ESP_ERROR_CHECK_WITHOUT_ABORT(set_pixel(i, 0,0,15));
        }
        break;
    
    case led_status::wifi_connected:
        for (uint8_t i = 0; i < NUM_LEDS; i++)
        {
            ESP_ERROR_CHECK_WITHOUT_ABORT(set_pixel(i, 0,15,0));
        }
        break;

    case led_status::sensor_error:
        for (uint8_t i = 0; i < NUM_LEDS; i++)
        {
            ESP_ERROR_CHECK_WITHOUT_ABORT(set_pixel(i, 20,0,0));
        }
        break;

    case led_status::critical_battery:
        for (uint8_t i = 0; i < NUM_LEDS; i++)
        {
            ESP_ERROR_CHECK_WITHOUT_ABORT(set_pixel(i, 20,0,0));
        }
        break;
    
    case led_status::battery:
    default:
        return ESP_OK;
    }

    return refresh();
}

} // namespace biosense
