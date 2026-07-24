#include "adc_sensors.hpp"

#include "biosense_config.hpp"

#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

namespace biosense
{
    
static const char* TAG = "ADC_sensors";

static constexpr adc_channel_t ADC_CH_GSR = ADC_CHANNEL_0;
static constexpr adc_channel_t ADC_CH_ECG = ADC_CHANNEL_3;
static constexpr adc_channel_t ADC_CH_BATT = ADC_CHANNEL_5;

static constexpr float BATT_DIVIDER_RATIO = 1.834f;  // For measured - batt voltage: 4.09 V; GPIO33: 2.23 V
static constexpr float ADC_APPROX_FULL_SCALE_V = 3.1f;
static constexpr int BATT_SAMPLE_COUNT = 32;

static constexpr float BATT_ADC_GAIN = 3.96f / 4.42f;

esp_err_t adc_sensors::init()
{
    gpio_config_t lo_cfg = {};
    lo_cfg.pin_bit_mask = (1ULL << PIN_LO_PLUS) | (1ULL << PIN_LO_MINUS);
    lo_cfg.mode = GPIO_MODE_INPUT;
    lo_cfg.pull_up_en = GPIO_PULLUP_DISABLE;
    lo_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    lo_cfg.intr_type = GPIO_INTR_DISABLE;

    ESP_ERROR_CHECK(gpio_config(&lo_cfg));

    adc_oneshot_unit_init_cfg_t uint_cfg = {};
    uint_cfg.unit_id = ADC_UNIT_1;

    adc_oneshot_unit_handle_t handle = nullptr;
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&uint_cfg, &handle));
    adc1_handle_ = handle;

    adc_oneshot_chan_cfg_t chan_cfg = {};
    chan_cfg.bitwidth = ADC_BITWIDTH_12;
    chan_cfg.atten = ADC_ATTEN_DB_12;

    ESP_ERROR_CHECK(adc_oneshot_config_channel(handle, ADC_CH_GSR, &chan_cfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(handle, ADC_CH_ECG, &chan_cfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(handle, ADC_CH_BATT, &chan_cfg));

    adc_cali_line_fitting_config_t cali_cfg = {};
    cali_cfg.unit_id = ADC_UNIT_1;
    cali_cfg.atten = ADC_ATTEN_DB_12;
    cali_cfg.bitwidth = ADC_BITWIDTH_12;
    cali_cfg.default_vref = 0;

    adc_cali_handle_t cali_handle = nullptr;

    esp_err_t cali_err = adc_cali_create_scheme_line_fitting(&cali_cfg, &cali_handle);

    if (cali_err == ESP_OK)
    {
        batt_cali_handle_ = cali_handle;
        batt_cali_ready_ = true;

        ESP_LOGI(TAG, "ADC battery calibration initialized");
    }
    else
    {
        ESP_LOGW(TAG, "ADC battery calibration unavailable: %s", esp_err_to_name(cali_err));
    }
    

    ESP_LOGI(TAG, "ADC initialized");

    return ESP_OK;
}

int adc_sensors::read_channel(int channel)
{
    auto handle = static_cast<adc_oneshot_unit_handle_t>(adc1_handle_);

    int raw = 0;
    esp_err_t err = adc_oneshot_read(handle, static_cast<adc_channel_t>(channel), &raw);

    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "ADC read failed: %s", esp_err_to_name(err));
        return 0;
    }

    return raw;
}

int adc_sensors::read_gsr_raw()
{
    return read_channel(ADC_CH_GSR);
}

int adc_sensors::read_ecg_raw()
{
    int a = read_channel(ADC_CH_ECG);
    int b = read_channel(ADC_CH_ECG);
    return (a+b) >> 1;
}

bool adc_sensors::leads_off() const
{
    return gpio_get_level(static_cast<gpio_num_t>(PIN_LO_PLUS)) || gpio_get_level(static_cast<gpio_num_t>(PIN_LO_MINUS));
}

int adc_sensors::read_batt_raw()
{
    int32_t sum = 0;

    for (int i = 0; i < BATT_SAMPLE_COUNT; i++)
    {
        sum += read_channel(ADC_CH_BATT);
    }
    
    return static_cast<int>(sum / BATT_SAMPLE_COUNT);
}

float adc_sensors::read_batt_voltage()
{
    const int raw = read_batt_raw();

    if (batt_cali_ready_)
    {
        auto cali_handle = static_cast<adc_cali_handle_t>(batt_cali_handle_);

        int pin_voltage_mv = 0;

        esp_err_t err = adc_cali_raw_to_voltage(cali_handle, raw, &pin_voltage_mv);

        if (err == ESP_OK)
        {
            const float pin_voltage = static_cast<float>(pin_voltage_mv)/ 1000.0f;

            return pin_voltage * BATT_DIVIDER_RATIO * BATT_ADC_GAIN;
        }

        ESP_LOGW(TAG, "ADC calibration conversion failed: %s", esp_err_to_name(err));
        
    }

    return batt_voltage_from_raw(raw);
}

float adc_sensors::batt_voltage_from_raw(int raw) const
{
    const float v_pin = (static_cast<float>(raw) / 4095.0f) * ADC_APPROX_FULL_SCALE_V;

    const float batt_v_uncalibrated = v_pin * BATT_DIVIDER_RATIO;
    
    return batt_v_uncalibrated * BATT_ADC_GAIN;
}

} // namespace biosense
