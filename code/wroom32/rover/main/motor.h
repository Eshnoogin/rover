#ifndef MOTOR_H
#define MOTOR_H

#include "driver/ledc.h"
#include "math.h"
#include "esp_log.h"

typedef struct
{
    ledc_channel_t in_1;
    ledc_channel_t in_2;
    int max_duty;
    bool is_moving;
} motor_handle_t;

esp_err_t motor_init(
    motor_handle_t *handle,
    int gpio_in_1, int gpio_in_2,
    ledc_channel_t channel_in_1,
    ledc_channel_t channel_in_2,
    ledc_timer_config_t *timer_config);

esp_err_t motor_stop(motor_handle_t *handle);

esp_err_t motor_move_at(motor_handle_t *handle, float duty_percent);

#endif