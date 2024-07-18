#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "driver/i2c_master.h"
#include "esp_log.h"

#include "common.h"
#include "imu.h"

const uint8_t ADDR = 0x68;
const uint8_t GYRO_XOUT_H =  0x43;
const uint8_t PWR_MGMT_1 =  0x6B;

#define TAG "IMU"

void imu_init(imu_handle_t *imu){
    //make sure imu is connected
    ESP_ERROR_CHECK(i2c_master_probe(i2c_mst_bus_handle, ADDR, 1000));

    //register it with i2c bus
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ADDR,
        .scl_speed_hz = 100000,
    }; 
    i2c_master_dev_handle_t dev_handle;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_mst_bus_handle, &dev_cfg, &dev_handle));
    imu->imu_i2c_handle = dev_handle;

    //enable imu
    uint8_t enable_code[2];

    enable_code[0] = (uint8_t) PWR_MGMT_1; //register in imu to write to
    enable_code[1] = (uint8_t) 0b00001000; //what to write to register 6b; puts imu out of sleep mode
    ESP_ERROR_CHECK(i2c_master_transmit(imu->imu_i2c_handle, enable_code, 2, -1));

    //instantiate mutex
    imu->imu_mutex = xSemaphoreCreateMutex();
    imu->is_init = true;

    ESP_LOGI(TAG, "IMU Registered");
}

void imu_update_gyro(imu_handle_t *imu){
    if(!imu->is_init){
        ESP_LOGE(TAG, "Trying to read from an IMU that is not initialized");
        ESP_ERROR_CHECK(ESP_ERR_INVALID_STATE);
    }

    uint8_t reg = GYRO_XOUT_H; //register to read from
    uint8_t data[6]; // bytes to read; 2 for each measurement
    i2c_master_transmit_receive(imu->imu_i2c_handle, &reg, 1, data, 6, -1);
    
    if(xSemaphoreTake(imu->imu_mutex, 0) == pdTRUE){
        imu->gyro_x = ((int16_t) (data[0] << 8) | data[1]) / 131.0;
        imu->gyro_y = ((int16_t) (data[2] << 8) | data[3]) / 131.0;
        imu->gyro_z = ((int16_t) (data[4] << 8) | data[5]) / 131.0;

        xSemaphoreGive(imu->imu_mutex);
    }else{
        ESP_LOGW(TAG, "IMU packet lost");
    }
}