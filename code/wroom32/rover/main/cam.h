#ifndef CAM_H
#define CAM_H
#include "driver/i2c_master.h"
#include "driver/spi_master.h"


typedef struct {
    i2c_master_dev_handle_t cam_i2c_handle;
    spi_device_handle_t cam_spi_handle;
} cam_handle_t;

typedef struct {
    uint16_t reg;
	uint16_t val;
} sensor_data;       

void cam_init(cam_handle_t *cam_handle);

#endif