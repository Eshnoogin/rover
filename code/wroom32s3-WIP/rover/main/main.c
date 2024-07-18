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

#define M1_IN1 17
#define M1_IN2 18
#define M1_FAULT 21

#define M2_IN1 45
#define M2_IN2 46
#define M2_FAULT 47

#define M3_IN1 40
#define M3_IN2 41
#define M3_FAULT 42


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
    // Setup GPIO 17 as output
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;   // Disable interrupt
    io_conf.mode = GPIO_MODE_OUTPUT;         // Set as output mode
    io_conf.pin_bit_mask = (1ULL << M1_IN1) | (1ULL << M1_IN2) | (1ULL << M2_IN1) | (1ULL << M2_IN2) | (1ULL << M3_IN1) | (1ULL << M3_IN2);     // Bit mask of the pins to set
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;                // Disable pull-down mode
    io_conf.pull_up_en = 0;                  // Disable pull-up mode
    gpio_config(&io_conf);

    gpio_set_level(M1_IN1, 1);
}