#ifndef COMMON_H
#define COMMON_H

#include "driver/i2c_master.h"
#include "imu.h"

#define CRITICAL_SECTION(mutex, ms, code)                       \
    do                                                          \
    {                                                           \
        if (xSemaphoreTake(mutex, pdMS_TO_TICKS(ms)) == pdTRUE) \
        {                                                       \
            code                                                \
                xSemaphoreGive(mutex);                          \
        }                                                       \
    } while (0)

extern i2c_master_bus_handle_t i2c_mst_bus_handle;

#endif