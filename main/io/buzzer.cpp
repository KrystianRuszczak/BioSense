#include "buzzer.hpp"

#include "biosense_config.hpp"

#include "esp_err.h"
#include "esp_log.h"

#include "driver/gpio.h"
#include "driver/ledc.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#ifndef BUZZER_ACTIVE_LOW
#define BUZZER_ACTIVE_LOW 0
#endif

namespace biosense
{

static const char* TAG = "buzzer";

static constexpr ledc_mode_t BUZZER_LEDC_MODE = LEDC_LOW_SPEED_MODE;
static constexpr ledc_timer_t BUZZER_LEDC_TIMER = LEDC_TIMER_0;
static constexpr ledc_channel_t BUZZER_LEDC_CHANNEL = LEDC_CHANNEL_0;
static constexpr ledc_timer_bit_t BUZZER_LEDC_RESOLUTION = LEDC_TIMER_10_BIT;

static constexpr uint32_t BUZZER_DEFAULT_FREQ_HZ = 2000;
static constexpr uint32_t BUZZER_DUTY = 256;        // 25% FOR 10-BIT

static constexpr uint32_t BUZZER_TASK_STACK = 3072;
static constexpr UBaseType_t BUZZER_TASK_PRIORITY = 3;
static constexpr uint32_t BUZZER_QUEUE_LEN = 8;

static constexpr int BUZZER_IDLE_LEVEL = BUZZER_ACTIVE_LOW ? 1:0;

esp_err_t buzzer::init()
{
    gpio_config_t gpio_cfg = {};
    gpio_cfg.pin_bit_mask = (1ULL << PIN_BUZZER);
    gpio_cfg.mode = GPIO_MODE_OUTPUT;
    gpio_cfg.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_cfg.intr_type = GPIO_INTR_DISABLE;

    ESP_ERROR_CHECK(gpio_config(&gpio_cfg));
    gpio_set_level(static_cast<gpio_num_t>(PIN_BUZZER), BUZZER_IDLE_LEVEL);

    ledc_timer_config_t timer_cfg = {};
    timer_cfg.speed_mode = BUZZER_LEDC_MODE;
    timer_cfg.timer_num = BUZZER_LEDC_TIMER;
    timer_cfg.duty_resolution = BUZZER_LEDC_RESOLUTION;
    timer_cfg.freq_hz = BUZZER_DEFAULT_FREQ_HZ;
    timer_cfg.clk_cfg = LEDC_AUTO_CLK;

    esp_err_t err = ledc_timer_config(&timer_cfg);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "ledc_timer_config failed: %s", esp_err_to_name(err));
        return err;
    }
    
    ledc_channel_config_t channel_cfg = {};
    channel_cfg.gpio_num = PIN_BUZZER;
    channel_cfg.speed_mode = BUZZER_LEDC_MODE;
    channel_cfg.channel = BUZZER_LEDC_CHANNEL;
    channel_cfg.intr_type = LEDC_INTR_DISABLE;
    channel_cfg.timer_sel = BUZZER_LEDC_TIMER;
    channel_cfg.duty = 0;
    channel_cfg.hpoint = 0;
    channel_cfg.flags.output_invert = BUZZER_ACTIVE_LOW ? 1:0;

    err = ledc_channel_config(&channel_cfg);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "ledc_channel_config failed: %s", esp_err_to_name(err));
        return err;
    }

    stop_tone();

    QueueHandle_t queue = xQueueCreate(BUZZER_QUEUE_LEN, sizeof(buzzer_command));
    if (queue == nullptr)
    {
        ESP_LOGE(TAG, "xQueueCreate failed");
        return ESP_FAIL;
    }

    queue_handle_ = queue;
    
    TaskHandle_t task = nullptr;
    BaseType_t task_ok = xTaskCreatePinnedToCore(
        buzzer::task_entry,
        "buzzer_task",
        BUZZER_TASK_STACK,
        this,
        BUZZER_TASK_PRIORITY,
        &task,
        1
    );

    if (task_ok != pdPASS)
    {
        ESP_LOGE(TAG, "xTaskCreatePinedToCore failed");
        return ESP_FAIL;
    }

    task_handle_ = task;
    initialized_ = true;

    ESP_LOGI(TAG, "Buzzer initialized: GPIO=%d", PIN_BUZZER);
    
    return ESP_OK;
}

esp_err_t buzzer::play(buzzer_pattern pattern)
{
    if (!initialized_ || queue_handle_ == nullptr)
    {
        return ESP_ERR_INVALID_STATE;
    }
    
    buzzer_command cmd = {};
    cmd.pattern = pattern;

    QueueHandle_t queue = static_cast<QueueHandle_t>(queue_handle_);

    BaseType_t ok = xQueueSend(queue, &cmd, 0);
    if (ok != pdPASS)
    {
        return ESP_ERR_NO_MEM;
    }
    
    return ESP_OK;
}

void buzzer::set_muted(bool muted)
{
    muted_ = muted;

    if (muted_)
    {
        stop_tone();
    }
    
}

bool buzzer::muted() const
{
    return muted_;
}

esp_err_t buzzer::stop()
{
    return stop_tone();
}

void buzzer::task_entry(void* arg)
{
    auto* self = static_cast<buzzer*>(arg);
    self->task_loop();
}

void buzzer::task_loop()
{
    QueueHandle_t queue = static_cast<QueueHandle_t>(queue_handle_);

    buzzer_command cmd = {};

    while (true)
    {
        if (xQueueReceive(queue, &cmd, portMAX_DELAY) == pdPASS)
        {
            run_pattern(cmd.pattern);
        }
    }
}

esp_err_t buzzer::start_tone(uint32_t freq_hz)
{
    if (muted_)
    {
        return ESP_OK;
    }

    esp_err_t err = ledc_set_freq(BUZZER_LEDC_MODE, BUZZER_LEDC_TIMER, freq_hz);

    if (err != ESP_OK)
    {
        return err;
    }

    err = ledc_set_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL, BUZZER_DUTY);

    if (err != ESP_OK)
    {
        return err;
    }
    
    return ledc_update_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL);
}

esp_err_t buzzer::stop_tone()
{
    esp_err_t err = ledc_set_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL, 0);

    if (err != ESP_OK)
    {
        return err;
    }
    
    err = ledc_update_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL);

    gpio_set_level(static_cast<gpio_num_t>(PIN_BUZZER), BUZZER_IDLE_LEVEL);

    return err;
}

void buzzer::tone(uint32_t freq_hz, uint32_t duration_ms)
{
    if (!muted_)
    {
        start_tone(freq_hz);
    }
    
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
    stop_tone();
}

void buzzer::pause(uint32_t duration_ms)
{
    stop_tone();
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
}

void buzzer::run_pattern(buzzer_pattern pattern)
{
    switch (pattern)
    {
    case buzzer_pattern::startup:
        tone(1600, 70);
        break;

    case buzzer_pattern::wifi_connected:
        tone(1800, 70);
        pause(80);
        tone(2200, 70);
        break;

    case buzzer_pattern::session_start:
        tone(1800, 80);
        pause(80);
        tone(2400, 80);
        break;

    case buzzer_pattern::session_stop:
        tone(2400, 80);
        pause(80);
        tone(1800, 80);
        break;

    case buzzer_pattern::sensor_error:
        tone(500, 120);
        pause(80);
        tone(500, 120);
        pause(80);
        tone(500, 120);
        break;

    case buzzer_pattern::low_battery:
        tone(700, 150);
        pause(120);
        tone(700, 150);
        break;

    case buzzer_pattern::critical_battery:
        tone(500, 100);
        pause(70);
        tone(500, 100);
        pause(70);
        tone(500, 100);
        break;
        
    case buzzer_pattern::reset_done:
        tone(180, 60);
        pause(60);
        tone(1800, 60);
        pause(60);
        tone(1800, 60);
        break;

    case buzzer_pattern::mute_on:
        tone(700, 70);
        break;

    case buzzer_pattern::mute_off:
        tone(1400, 70);
        pause(70);
        tone(1800, 70);
        break;

    case buzzer_pattern::marker_set:
        tone(1500, 60);
        pause(40);
        tone(1000, 50);
        break;

    default:
        stop_tone();
        break;
    }
    
    stop_tone();
}

}