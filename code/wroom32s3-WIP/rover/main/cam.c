#include "common.h"
#include "esp_log.h"

#define TAG "CAM"

void init_cam(){
    for(int i=0; i<255; i++){
        esp_err_t dev_found = i2c_master_probe(i2c_mst_bus_handle, i, -1);
        if(dev_found == ESP_OK){
            ESP_LOGI(TAG, "Found addr %x", i);
        }
    }
}