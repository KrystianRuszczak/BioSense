#pragma once

#include <cstdint>
#include <cstddef>

#include "esp_err.h"
#include "driver/i2c_master.h"
namespace biosense
{

struct i2c_readings
{

    uint16_t mlx_raw_obj = 0;
    uint16_t mlx_raw_amb = 0;

    float temp_obj = 0;
    float temp_amb = 0;

    bool mlx_ok = false;
    bool mlx_valid  = false;

    int max_bpm = 0;
    
    float max_spo2 = 0.0f;
    bool max_spo2_valid = false;
    int max_spo2_raw = 0.0f;
    
    bool max_finger = false;
    bool max_ok = false;
    
    uint32_t max_red = 0;
    uint32_t max_ired = 0;
};

class i2c_sensors
{
public:
    esp_err_t init();

    esp_err_t update(uint64_t now_ms);

    const i2c_readings& readings() const;

    void reset_processing();
private:

    uint64_t max_last_beat_ms_ = 0;
    bool max_finger_state_ = false;
    
    static constexpr uint8_t MAX_BPM_AVG_SIZE = 4;

    int max_bpm_history_[MAX_BPM_AVG_SIZE] = {0, 0, 0, 0};
    uint8_t max_bpm_history_pos_ = 0;
    uint8_t max_bpm_history_count_ = 0;

    static constexpr uint8_t SPO2_BUFFER_SIZE = 100;
    static constexpr uint8_t SPO2_STEP_SIZE = 25;
    static constexpr uint8_t SPO2_DECIMATION = 4;

    uint32_t spo2_ired_buffer_[SPO2_BUFFER_SIZE] = {0};
    uint32_t spo2_red_buffer_[SPO2_BUFFER_SIZE] = {0};

    uint8_t spo2_sample_count_ = 0;
    uint8_t spo2_decimation_count_ = 0;

    void process_spo2_sample(uint32_t red, uint32_t ired);
    void reset_spo2_state();
    
    esp_err_t init_bus();

    esp_err_t init_mlx();
    esp_err_t update_mlx(uint64_t now_ms);
    esp_err_t read_mlx_word(uint8_t reg, uint16_t* value);

    esp_err_t init_max();
    esp_err_t update_max(uint64_t now_ms);

    esp_err_t write_reg(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t value);
    esp_err_t read_reg(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t* value);
    esp_err_t read_regs(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t* data, size_t len);

    esp_err_t read_max_fifo_sample(uint32_t* red, uint32_t* ired);
    void process_max_sample(uint32_t red, uint32_t ired, uint64_t now_ms);

    i2c_master_bus_handle_t bus_handle_ = nullptr;
    i2c_master_dev_handle_t mlx_handle_ = nullptr;
    i2c_master_dev_handle_t max_handle_ = nullptr;

    i2c_readings readings_ {};

    uint64_t last_mlx_ms_ = 0;
    uint64_t last_max_ms_ = 0;
};


} // namespace biosense
