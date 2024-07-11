#include <stdio.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "driver/mcpwm_timer.h"
#include "driver/mcpwm_oper.h"
#include "driver/mcpwm_gen.h"
#include "driver/mcpwm_cmpr.h"
#include "esp_timer.h"

#include "driver/gpio.h"

#include "imu.h"
#include "common.h"
#include "motor.h"
#include "comm.h"

#define TAG "RVR"

#define M1_IN1 26
#define M1_IN2 27
#define M2_IN1 16
#define M2_IN2 17
#define M3_IN1 32
#define M3_IN2 25

i2c_master_bus_handle_t i2c_mst_bus_handle;

imu_handle_t imu;

motor_handle_t motor1;
motor_handle_t motor2;
motor_handle_t motor3;

float measured_z_rotation;
SemaphoreHandle_t z_rot_mutex;

void handle_imu()
{
    while (1)
    {
        imu_update_gyro(&imu);

        CRITICAL_SECTION(imu.imu_mutex, 15, {
            CRITICAL_SECTION(z_rot_mutex, 15, {
                if (fabs(imu.gyro_z) > 10){ // some noise will be present in the raw sensor data
                    measured_z_rotation += imu.gyro_z;
                }
            });
        });

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void handle_movement()
{
    while (1)
    {
        float v1 = 0;
        float v2 = 0;
        float v3 = 0;

        // read gyroscope data
        CRITICAL_SECTION(imu.imu_mutex, 15, {
            bool do_move = true;
            CRITICAL_SECTION(z_rot_mutex, 15, {
                if(fabs(measured_z_rotation) < 300){
                    do_move = false;
                }
                if(do_move){
                    if(fabs(measured_z_rotation) < 450){ //helps the robot face the center in less steps
                        v1 += (measured_z_rotation < 0) ? -0.25 : 0.25;
                        v2 += (measured_z_rotation < 0) ? -0.25 : 0.25;
                        v3 += (measured_z_rotation < 0) ? -0.25 : 0.25;     
                    }else{  
                        v1 += (measured_z_rotation < 0) ? -0.5 : 0.5;
                        v2 += (measured_z_rotation < 0) ? -0.5 : 0.5;
                        v3 += (measured_z_rotation < 0) ? -0.5 : 0.5;        
                    }
                }
            });
        });

        // read espnow data
        CRITICAL_SECTION(last_command.cmd_mutex, 15, {
            if(last_command.time + 20000 > esp_timer_get_time()){
                v1 += cos(last_command.direction);
                v2 += -.5 * cos(last_command.direction) + ((sqrt(3) / 2) * sin(last_command.direction));
                v3 += -.5 * cos(last_command.direction) - ((sqrt(3) / 2) * sin(last_command.direction));

                // normalize values
                float v_max = fmax(fabs(v1), fmax(fabs(v2), fabs(v3)));
                if (v_max != 0)
                {
                    v1 /= v_max;
                    v2 /= v_max;
                    v3 /= v_max;
                }
            }
        });

        ESP_LOGI(TAG, "M1: %f M2: %f M3: %f", v1, v2, v3);

        // move motors
        motor_move_at(&motor1, v1);
        motor_move_at(&motor2, v2);
        motor_move_at(&motor3, v3);

        vTaskDelay(pdMS_TO_TICKS(25));
    }
}

void app_main(void)
{
    // setup i2c for gyroscope and camera
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = -1,
        .scl_io_num = 22,
        .sda_io_num = 21,
        .glitch_ignore_cnt = 7,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &i2c_mst_bus_handle));

    // init imu
    z_rot_mutex = xSemaphoreCreateMutex();
    TaskHandle_t handle_imu_handler;
    imu_init(&imu);
    xTaskCreate(handle_imu, "handle_imu", 2048, NULL, 1, &handle_imu_handler);

    // configure timer, shared across all pwms
    ledc_fade_func_install(0);
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .freq_hz = 25,
        .clk_cfg = LEDC_AUTO_CLK};
    ledc_timer_config(&ledc_timer);

    // init motors
    motor_init(&motor1, M1_IN1, M1_IN2, LEDC_CHANNEL_0, LEDC_CHANNEL_1, &ledc_timer);
    motor_init(&motor2, M2_IN1, M2_IN2, LEDC_CHANNEL_2, LEDC_CHANNEL_3, &ledc_timer);
    motor_init(&motor3, M3_IN1, M3_IN2, LEDC_CHANNEL_4, LEDC_CHANNEL_5, &ledc_timer);

    // setup esp-now
    espnow_init();

    // init movement
    TaskHandle_t handle_movement_handler;
    xTaskCreate(handle_movement, "handle_movement", 2048, NULL, 1, &handle_movement_handler);
}