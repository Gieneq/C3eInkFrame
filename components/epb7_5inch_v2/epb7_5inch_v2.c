#include "epb7_5inch_v2.h"

#include <stdio.h>
#include <esp_log.h>

#include "EPD_7in5_V2.h"
#include "DEV_Config.h"

static const char* TAG = "EPB";

esp_err_t epb7_5inch_v2_create() {
    ESP_LOGI(TAG, "epb7_5inch_v2_create");
    DEV_Module_Setup();
    
    EPD_7IN5_V2_Init();
    EPD_7IN5_V2_ClearBlack();
    vTaskDelay(5000);
    ESP_LOGI(TAG, "epb7_5inch_v2_create --> done");
    return ESP_OK;
}

esp_err_t epb7_5inch_v2_start() {

    return ESP_OK;
}