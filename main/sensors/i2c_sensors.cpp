#include "i2c_sensors.hpp"

#include "biosense_config.hpp"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/i2c_master.h"
#include "driver/gpio.h"

namespace biosense
{

static const char* TAG = "I2C_sensors";

static constexpr uint8_t MLX_ADDR = 0x5A;
static constexpr uint8_t MAX_ADDR = 0x57;

static constexpr uint64_t MLX_INTERVAL_MS = 500;
static constexpr uint64_t MAX_INTERVAL_MS = 20;

static constexpr uint8_t MLX_REG_TA = 0x06;
static constexpr uint8_t MLX_REG_TOBJ = 0x07;

static constexpr uint8_t MAX_REG_INTR_STAT_1 = 0x00;
static constexpr uint8_t MAX_REG_INTR_STAT_2 = 0x01;
static constexpr uint8_t MAX_REG_INTR_EN_1 = 0x02;
static constexpr uint8_t MAX_REG_INTR_EN_2 = 0x03;

static constexpr uint8_t MAX_REG_FIFO_WR_PTR = 0x04;
static constexpr uint8_t MAX_REG_OVF_COUNTER = 0x05;
static constexpr uint8_t MAX_REG_FIFO_RD_PTR = 0x06;
static constexpr uint8_t MAX_REG_FIFO_DATA = 0x07;

static constexpr uint8_t MAX_REG_FIFO_CONFIG = 0x08;
static constexpr uint8_t MAX_REG_MODE_CONFIG = 0x09;
static constexpr uint8_t MAX_REG_SPO2_CONFIG = 0x0A;

static constexpr uint8_t MAX_REG_LED1_PA = 0x0C;
static constexpr uint8_t MAX_REG_LED2_PA = 0x0D;

esp_err_t i2c_sensors::init()
{
    ESP_LOGI(TAG, "Initializing I2C sensors");

    esp_err_t err = init_bus();

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C bus init failed");
        return err;
    }

    esp_err_t mlx_err = init_mlx();
    readings_.mlx_ok = (mlx_err == ESP_OK);

    esp_err_t max_err = init_max();
    readings_.max_ok = (max_err == ESP_OK);

    ESP_LOGI(TAG, "MLX90614: %s", readings_.mlx_ok ? "OK":"FAILED");
    ESP_LOGI(TAG, "MAX30102: %s", readings_.max_ok ? "OK":"FAILED");

    return ESP_OK;
}

esp_err_t i2c_sensors::update(uint64_t now_ms)
{
    if (readings_.mlx_ok && now_ms - last_mlx_ms_ >= MLX_INTERVAL_MS)
    {
        last_mlx_ms_ = now_ms;
        update_mlx(now_ms);
    }

    if (readings_.max_ok && now_ms - last_max_ms_ >= MAX_INTERVAL_MS)
    {
        last_max_ms_ = now_ms;
        update_max(now_ms);
    }

    return ESP_OK;
}

const i2c_readings& i2c_sensors::readings() const
{
    return readings_;
}

esp_err_t i2c_sensors::init_bus()
{
    i2c_master_bus_config_t bus_config = {};
    bus_config.i2c_port = I2C_NUM_0;
    bus_config.sda_io_num = static_cast<gpio_num_t>(I2C_SDA);
    bus_config.scl_io_num = static_cast<gpio_num_t>(I2C_SCL);
    bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_config.glitch_ignore_cnt = 7;
    bus_config.flags.enable_internal_pullup = false;

    esp_err_t err = i2c_new_master_bus(&bus_config, &bus_handle_);

    if(err != ESP_OK)
    {
        ESP_LOGE(TAG, "i2c_new_master_bus failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGE(TAG, "I2C bus initialized: SDA=%d, SCL=%d, freq=%d HZ", 
        I2C_SDA, I2C_SCL, I2C_MASTER_FREQ_HZ);

    return ESP_OK;
}

esp_err_t i2c_sensors::init_mlx()
{
    i2c_device_config_t dev_config = {};
    dev_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_config.device_address = MLX_ADDR;
    dev_config.scl_speed_hz = I2C_MASTER_FREQ_HZ;

    esp_err_t err = i2c_master_bus_add_device(bus_handle_, &dev_config, &mlx_handle_);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to add MLX90614 device: %s", esp_err_to_name(err));
        return err;
    }
    
    err = i2c_master_probe(bus_handle_, MLX_ADDR, 100);

    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "MLX90614 not detected at 0x%02X: %s", MLX_ADDR, esp_err_to_name(err));
        return err;
    }
    
    ESP_LOGI(TAG, "MLX90614 detected at 0x%02X", MLX_ADDR);
    return ESP_OK;
}

esp_err_t i2c_sensors::read_mlx_word(uint8_t reg, uint16_t* value)
{
    if (value == nullptr)
    {
        return ESP_ERR_INVALID_ARG;
    }
    
    uint8_t data[3] = {0,0,0};

    esp_err_t err = read_regs(mlx_handle_, reg, data, sizeof(data));

    if (err != ESP_OK)
    {
        return err;
    }
    
    *value = static_cast<uint16_t>(data[0]) | (static_cast<uint16_t>(data[1]) << 8);

    return ESP_OK;
}

esp_err_t i2c_sensors::update_mlx(uint64_t now_ms)
{
    (void)now_ms;

    uint16_t raw_amb = 0;
    uint16_t raw_obj = 0;
    
    esp_err_t err_amb = read_mlx_word(MLX_REG_TA, &raw_amb);
    esp_err_t err_obj = read_mlx_word(MLX_REG_TOBJ, &raw_obj);

    if (err_amb != ESP_OK || err_obj != ESP_OK)
    {
        readings_.mlx_valid = false;

        ESP_LOGW(TAG, "MLX90614 read failed: amb=%s obj=%s", esp_err_to_name(err_amb), esp_err_to_name(err_obj));
        
        return ESP_FAIL;
    }

    readings_.mlx_raw_amb = raw_amb;
    readings_.mlx_raw_obj = raw_obj;

    readings_.temp_amb = static_cast<float>(raw_amb) * 0.02f - 273.15f;
    readings_.temp_obj = static_cast<float>(raw_obj) * 0.02f - 273.15f;

    readings_.mlx_valid = true;

    return ESP_OK;
}

esp_err_t i2c_sensors::init_max()
{
    i2c_device_config_t dev_config = {};
    dev_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_config.device_address = MAX_ADDR;
    dev_config.scl_speed_hz = I2C_MASTER_FREQ_HZ;

    esp_err_t err = i2c_master_bus_add_device(bus_handle_, &dev_config, &max_handle_);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to add MAX30102 device: %s", esp_err_to_name(err));
        return err;
    }
    
    err = i2c_master_probe(bus_handle_, MAX_ADDR, 100);

    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "MAX30102 not detected at 0x%02X: %s", MAX_ADDR, esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "MAX30102 detected at 0x%02X", MAX_ADDR);

    ESP_RETURN_ON_ERROR(write_reg(max_handle_, MAX_REG_MODE_CONFIG, 0x40), TAG, "MAX30102 reset failed");

    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_RETURN_ON_ERROR(write_reg(max_handle_, MAX_REG_FIFO_WR_PTR, 0x00), TAG, "MAX30102 FIFO_WR_PTR clear failed");
    ESP_RETURN_ON_ERROR(write_reg(max_handle_, MAX_REG_OVF_COUNTER, 0x00), TAG, "MAX30102 REG_OVF_COUNTER clear failed");
    ESP_RETURN_ON_ERROR(write_reg(max_handle_, MAX_REG_FIFO_RD_PTR, 0x00), TAG, "MAX30102 FIFO_RD_PTR clear failed");
    
    ESP_RETURN_ON_ERROR(write_reg(max_handle_, MAX_REG_FIFO_CONFIG, 0x5F), TAG, "MAX30102 FIFO config failed");
    ESP_RETURN_ON_ERROR(write_reg(max_handle_, MAX_REG_MODE_CONFIG, 0x03), TAG, "MAX30102 mode config failed");
    ESP_RETURN_ON_ERROR(write_reg(max_handle_, MAX_REG_SPO2_CONFIG, 0x47), TAG, "MAX30102 SpO2 config failed");

    ESP_RETURN_ON_ERROR(write_reg(max_handle_, MAX_REG_LED1_PA, 0x24), TAG, "MAX30102 RED LED current failed");
    ESP_RETURN_ON_ERROR(write_reg(max_handle_, MAX_REG_LED2_PA, 0x24), TAG, "MAX30102 IR LED current failed");
    
    uint8_t dummy = 0;
    read_reg(max_handle_, MAX_REG_INTR_STAT_1, &dummy);
    read_reg(max_handle_, MAX_REG_INTR_STAT_2, &dummy);

    ESP_LOGI(TAG, "MAX30102 configured");
    return ESP_OK;
}

esp_err_t i2c_sensors::read_max_fifo_sample(uint32_t* red, uint32_t* ired)
{
    if (red == nullptr || ired == nullptr)
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t data[6] = {0};

    esp_err_t err = read_regs(max_handle_, MAX_REG_FIFO_DATA, data, sizeof(data));

    if (err != ESP_OK)
    {
        return err;
    }

    uint32_t raw_red = (static_cast<uint32_t>(data[0]) << 16) | (static_cast<uint32_t>(data[1]) << 8) | (static_cast<uint32_t>(data[2]));
    uint32_t raw_ired = (static_cast<uint32_t>(data[3]) << 16) | (static_cast<uint32_t>(data[4]) << 8) | (static_cast<uint32_t>(data[5]));
    
    raw_red &= 0x3FFFF;
    raw_ired &= 0x3FFFF;

    *red = raw_red;
    *ired = raw_ired;

    return ESP_OK;
}

void i2c_sensors::process_max_sample(uint32_t red, uint32_t ired, uint64_t now_ms)
{
    readings_.max_red = red;
    readings_.max_ired = ired;

    bool finger_present = ired > 20000 &&
        red > 20000 &&
        ired < 250000 &&
        red < 250000;

    readings_.max_finger = finger_present;

    if (!finger_present)
    {
        readings_.max_bpm = 0;
        readings_.max_spo2 = 0.0f;
        readings_.max_spo2_valid = false;
        readings_.max_spo2_raw = 0;

        max_ired_min_ = 0xFFFFFFFF;
        max_ired_max_ = 0;
        max_signal_above_threshold_ = false;
        max_last_beat_ms_ = 0;

        return;
    }
    
    if (ired < max_ired_min_)
    { max_ired_min_ = ired; }
    if (ired > max_ired_max_)
    { max_ired_max_ = ired; }

    max_ired_min_ = static_cast<uint32_t>(0.999f * max_ired_min_ + 0.001f * ired);
    max_ired_max_ = static_cast<uint32_t>(0.999f * max_ired_max_ + 0.001f * ired);

    uint32_t amplitude = max_ired_max_ - max_ired_min_;

    if (amplitude < 1000)
    {
        return;
    }

    uint32_t threshold = max_ired_min_ + static_cast<uint32_t>(amplitude * 0.60f);

    bool above = ired > threshold;

    if (above && !max_signal_above_threshold_)
    {
        uint64_t delta_ms = now_ms - max_last_beat_ms_;

        if (max_last_beat_ms_ != 0 && delta_ms > 300 && delta_ms < 2000)
        {
            readings_.max_bpm = static_cast<int>(60000ULL / delta_ms);
        }
        
        max_last_beat_ms_ = now_ms;
    }
    
    max_signal_above_threshold_ = above;

    readings_.max_spo2_valid = false;
    readings_.max_spo2 = 0.0f;
    readings_.max_spo2_raw = 0;
}

esp_err_t i2c_sensors::update_max(uint64_t now_ms)
{
    if (!readings_.max_ok || max_handle_ == nullptr)
    {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t wr_ptr = 0;
    uint8_t rd_ptr = 0;

    esp_err_t err_wr = read_reg(max_handle_, MAX_REG_FIFO_WR_PTR, &wr_ptr);
    esp_err_t err_rd = read_reg(max_handle_, MAX_REG_FIFO_RD_PTR, &rd_ptr);

    if (err_wr != ESP_OK || err_rd != ESP_OK)
    {
        ESP_LOGW(TAG, "MAX FIFO ptr read failed: wr=%s rd=%s", esp_err_to_name(err_wr), esp_err_to_name(err_rd));

        readings_.max_finger = false;
        return ESP_FAIL;
    }
    
    wr_ptr &= 0x1F;
    rd_ptr &= 0x1F;

    uint8_t samples_available = 0;
    
    if (wr_ptr >= rd_ptr)
    {
        samples_available = wr_ptr - rd_ptr;
    }
    else
    {
        samples_available = 32 + wr_ptr - rd_ptr;
    }
    
    if (samples_available == 0)
    {
        return ESP_OK;
    }
    
    if (samples_available > 4)
    {
        samples_available = 4;
    }

    for (uint8_t i = 0; i < samples_available; i++)
    {
        uint32_t red = 0;
        uint32_t ired = 0;

        esp_err_t err = read_max_fifo_sample(&red, &ired);

        if (err != ESP_OK)
        {
            ESP_LOGW(TAG, "MAX FIFO sample read failed: %s", esp_err_to_name(err));
            return err;
        }
        
        process_max_sample(red, ired, now_ms);
    }

    return ESP_OK;
}

esp_err_t i2c_sensors::write_reg(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t value)
{
    uint8_t data[2] = {reg, value};

    return i2c_master_transmit(dev, data, sizeof(data), 100);
}

esp_err_t i2c_sensors::read_reg(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t* value)
{
    if (value == nullptr){ return ESP_ERR_INVALID_ARG; }
    
    return i2c_master_transmit_receive(dev, &reg, 1, value, 1, 100);
}

esp_err_t i2c_sensors::read_regs(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t* data, size_t len)
{
    if (data == nullptr || len == 0)
    {
        return ESP_ERR_INVALID_ARG;
    }
    
    return i2c_master_transmit_receive(dev, &reg, 1, data, len, 100);
}

} // namespace biosense
