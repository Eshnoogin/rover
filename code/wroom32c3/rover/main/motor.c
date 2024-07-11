#include "driver/ledc.h"
#include "math.h"

#include "esp_log.h"

#include "motor.h"

const char *TAG = "MVMT";


esp_err_t motor_init(
    motor_handle_t *handle,
    int gpio_in_1, int gpio_in_2,
    ledc_channel_t channel_in_1,
    ledc_channel_t channel_in_2,
    ledc_timer_config_t *timer_config)
{
    // configure pwm for in1
    ledc_channel_config_t m1_ledc_channel = {
        .gpio_num = gpio_in_1,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = channel_in_1,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = timer_config->timer_num,
        .duty = 0,
        .hpoint = 0, // start point of the pwm signal 
    };
    ESP_ERROR_CHECK(ledc_channel_config(&m1_ledc_channel));

    // configure pwm for in2
    ledc_channel_config_t m2_ledc_channel = {
        .gpio_num = gpio_in_2,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = channel_in_2,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = timer_config->timer_num,
        .duty = 0,
        .hpoint = 0, // start point of the pwm signal 
    };
    ESP_ERROR_CHECK(ledc_channel_config(&m2_ledc_channel));

    handle->in_1 = channel_in_1;
    handle->in_2 = channel_in_2;
    handle->max_duty = pow(2, (int)timer_config->duty_resolution) - 1;

    ESP_LOGI(TAG, "Motor created at channels %i and %i", channel_in_1, channel_in_2);

    return ESP_OK;
}

esp_err_t motor_stop(motor_handle_t *handle)
{
    if (handle->is_moving)
    {
        ESP_LOGI(TAG, "Stopping motor at channels %i and %i", handle->in_1, handle->in_2);

        // set in1 and in2 to 0 to stop motor
        ESP_ERROR_CHECK(ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, handle->in_1, 0, 0));
        ESP_ERROR_CHECK(ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, handle->in_2, 0, 0));

        // update state
        handle->is_moving = false;

        return ESP_OK;
    }

    return ESP_OK;
}

esp_err_t motor_move_at(motor_handle_t *handle, float duty_percent)
{
    if (duty_percent == 0)
    {
        motor_stop(handle);
        return ESP_OK;
    }

    int calcuated_duty = fabs((duty_percent)*handle->max_duty);
    ESP_LOGI(TAG, "Moving motor at channels %i and %i with a duty cycle of %i", handle->in_1, handle->in_2, calcuated_duty);

    if (duty_percent <= 0)
    {
        // in1 goes to duty cycle and in2 goes low
        ESP_ERROR_CHECK(ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, handle->in_1, calcuated_duty, 0)); // thread-safe version, just in case
        ESP_ERROR_CHECK(ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, handle->in_2, 0, 0));
    }
    else
    {
        // in2 goes to duty cycle and in1 goes low
        ESP_ERROR_CHECK(ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, handle->in_2, calcuated_duty, 0));
        ESP_ERROR_CHECK(ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, handle->in_1, 0, 0));
    }

    // update state
    handle->is_moving = true;

    return ESP_OK;
}