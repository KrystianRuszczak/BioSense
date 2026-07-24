#include "buttons.hpp"
#include "biosense_config.hpp"

#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

namespace biosense
{

static const char* TAG = "Buttons";

static constexpr uint32_t POLL_MS = 10;
static constexpr uint32_t DEBOUNCE_MS = 30;
static constexpr uint32_t LONG_PRESS_MS = 1500;
static constexpr uint32_t COMBO_LONG_MS = 2500;

static constexpr uint32_t TASK_STACK = 3072;
static constexpr UBaseType_t TASK_PRIOR = 2;
static constexpr uint32_t EVENT_QUEUE_LEN = 8;

static constexpr int BUTTON_PRESSED_LEVEL = 0;

static bool read_button_pressed(int pin)
{
    return gpio_get_level(static_cast<gpio_num_t>(pin)) == BUTTON_PRESSED_LEVEL;
}

esp_err_t buttons::init()
{
    gpio_config_t cfg = {};

    cfg.pin_bit_mask = (1ULL << PIN_BUTTON_A) | (1ULL << PIN_BUTTON_B);
    cfg.mode = GPIO_MODE_INPUT;
    cfg.pull_up_en = GPIO_PULLUP_ENABLE;
    cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    cfg.intr_type = GPIO_INTR_DISABLE;

    esp_err_t err = gpio_config(&cfg);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "gpio_config failed: %s", esp_err_to_name(err));
        return err;
    }
    
    QueueHandle_t queue = xQueueCreate(EVENT_QUEUE_LEN, sizeof(ui_event));

    if (queue == nullptr)
    {
        ESP_LOGE(TAG, "xQueueCreate failed");
        return ESP_FAIL;
    }
    
    queue_handle_ = queue;

    const uint64_t now_ms = static_cast<uint64_t>(esp_timer_get_time() / 1000);

    button_a_.raw_pressed = read_button_pressed(PIN_BUTTON_A);
    button_a_.stable_pressed = button_a_.raw_pressed;
    button_a_.raw_change_ms = now_ms;

    if (button_a_.stable_pressed)
    {
        button_a_.press_start_ms = now_ms;
    }

    button_b_.raw_pressed = read_button_pressed(PIN_BUTTON_B);
    button_b_.stable_pressed = button_b_.raw_pressed;
    button_b_.raw_change_ms = now_ms;

    if (button_b_.stable_pressed)
    {
        button_b_.press_start_ms = now_ms;
    }

    TaskHandle_t task = nullptr;

    BaseType_t task_result = xTaskCreate(buttons::task_entry, "buttons_task", TASK_STACK, this, TASK_PRIOR, &task);

    if (task_result != pdPASS)
    {
        ESP_LOGE(TAG, "xTaskCreate failed");
        
        vQueueDelete(queue);
        queue_handle_ = nullptr;

        return ESP_FAIL;
    }
    
    task_handle_ = task;

    ESP_LOGI(TAG, "Buttons initialized");

    return ESP_OK;
}

bool buttons::poll_event(ui_event& event)
{
    if (queue_handle_ == nullptr)
    {
        event = ui_event::none;
        return false;
    }
    
    QueueHandle_t queue = static_cast<QueueHandle_t>(queue_handle_);

    return xQueueReceive(queue, &event, 0) == pdPASS;
}

void buttons::post_event(ui_event event)
{
    if (queue_handle_ == nullptr)
    {
        return;
    }

    QueueHandle_t queue = static_cast<QueueHandle_t>(queue_handle_);

    if (xQueueSend(queue, &event, 0) != pdPASS)
    {
        ESP_LOGW(TAG, "Button event queue full");
    }
}

buttons::transition buttons::update_button(button_state& state, bool raw_pressed, uint64_t now_ms)
{
    if (raw_pressed != state.raw_pressed)
    {
        state.raw_pressed = raw_pressed;
        state.raw_change_ms = now_ms;
    }

    if (state.stable_pressed == state.raw_pressed)
    {
        return transition::none;
    }
    
    if (now_ms - state.raw_change_ms < DEBOUNCE_MS)
    {
        return transition::none;
    }
    
    state.stable_pressed = state.raw_pressed;

    if (state.stable_pressed)
    {
        state.press_start_ms = now_ms;
        state.supress_release = false;
        state.long_reported = false;

        return transition::pressed;
    }

    return transition::released;
}

void buttons::maybe_report_long(button_state& state, ui_event long_event, uint64_t now_ms)
{
    if (!state.stable_pressed)
    {
        return;
    }
    
    if (state.long_reported || state.supress_release)
    {
        return;
    }
    
    if (state.press_start_ms == 0)
    {
        return;
    }
    
    if (now_ms - state.press_start_ms < LONG_PRESS_MS)
    {
        return;
    }
    
    state.long_reported = true;
    state.supress_release = true;

    post_event(long_event);
}

void buttons::handle_release(button_state& state, ui_event short_event, uint64_t now_ms)
{
    if (state.supress_release || state.long_reported)
    {
        state.supress_release = false;
        state.long_reported = false;
        state.press_start_ms = 0;
        return;
    }
    
    if (state.press_start_ms == 0)
    {
        return;
    }
    
    const uint64_t press_duration_ms = now_ms - state.press_start_ms;

    state.press_start_ms = 0;

    if (press_duration_ms < LONG_PRESS_MS)
    {
        post_event(short_event);
    }
}

void buttons::task_entry(void* arg)
{
    auto* self = static_cast<buttons*>(arg);
    self->task_loop();
}

void buttons::task_loop()
{
    while (true)
    {
        const uint64_t now_ms = static_cast<uint64_t>(esp_timer_get_time() / 1000);

        const bool raw_a = read_button_pressed(PIN_BUTTON_A);
        const bool raw_b = read_button_pressed(PIN_BUTTON_B);

        const transition transition_a = update_button(button_a_, raw_a, now_ms);
        const transition transition_b = update_button(button_b_, raw_b, now_ms);

        const bool both_pressed = button_a_.stable_pressed && button_b_.stable_pressed;

        if (both_pressed)
        {
            button_a_.supress_release = true;
            button_b_.supress_release = true;

            if (combo_start_ms_ == 0)
            {
                combo_start_ms_ = now_ms;
            }
            
            if (!combo_reported_ && now_ms - combo_start_ms_ >= COMBO_LONG_MS)
            {
                combo_reported_ = true;

                button_a_.supress_release = true;
                button_b_.supress_release = true;

                post_event(ui_event::diagnostics);
            }
        }
        else
        {
            combo_start_ms_ = 0;

            maybe_report_long(button_a_, ui_event::session_toggle, now_ms);
            maybe_report_long(button_b_, ui_event::reset_measurements, now_ms);

            if (!button_a_.stable_pressed && !button_b_.stable_pressed)
            {
                combo_reported_ = false;
            }
        }

        if (transition_a == transition::released)
        {
            handle_release(button_a_, ui_event::marker, now_ms);
        }
        
        if (transition_b == transition::released)
        {
            handle_release(button_b_, ui_event::mute_toggle, now_ms);
        }
        
        vTaskDelay(pdMS_TO_TICKS(POLL_MS));
    }    
}

} // namespace biosense
