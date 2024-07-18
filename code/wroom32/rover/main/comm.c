#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_log.h"
#include "esp_timer.h"

#include <string.h>

#include "comm.h"

rover_mvmt_cmd_t last_command;
#define TAG "COMM"

void on_data_recv(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int data_len)
{
    ESP_LOGI(TAG, "rdata");
    if (xSemaphoreTake(last_command.cmd_mutex, 15))
    {
        memcpy(&last_command.speed, data, sizeof(float));
        memcpy(&last_command.direction, data + sizeof(float), sizeof(float));
        memcpy(&last_command.rotation, data + 2 * sizeof(float), sizeof(float));
        last_command.time = esp_timer_get_time();
        xSemaphoreGive(last_command.cmd_mutex);
    }
}

void espnow_init()
{
    // init nvs
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // init wifi
    wifi_init_config_t wifi_config = WIFI_INIT_CONFIG_DEFAULT();
    uint8_t macAddr[6] = {0x1A, 0x2A, 0x3A, 0x4A, 0x5A, 0x6A};
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_config));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_mac(WIFI_IF_STA, macAddr));
    ESP_ERROR_CHECK(esp_wifi_start());

    last_command.cmd_mutex = xSemaphoreCreateMutex();

    // init esp-now
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(on_data_recv));
}