#ifndef COMM_H
#define COMM_H

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_now.h"

typedef struct
{
    float speed;
    float direction; // angle that the rover will travel
    float rotation;  // angle that the rover will face

    int64_t time;
    SemaphoreHandle_t cmd_mutex;
} rover_mvmt_cmd_t;

void espnow_init();

extern rover_mvmt_cmd_t last_command;

#endif