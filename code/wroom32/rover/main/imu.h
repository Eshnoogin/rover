#ifndef IMU_H
#define IMU_H

typedef struct
{
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float acc_x;
    float acc_y;
    float acc_z;

    i2c_master_dev_handle_t imu_i2c_handle;
    SemaphoreHandle_t imu_mutex;
    bool is_init;
} imu_handle_t;

void imu_init(imu_handle_t *imu);
void imu_update_gyro(imu_handle_t *imu);
#endif