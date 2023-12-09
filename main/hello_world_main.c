/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <esp_err.h>
#include <esp_log.h>

#include "shtc3_sensor.h"
#include "gindicator.h"

static const char* TAG = "Main";

float mapFloat(float value, float inMin, float inMax, float outMin, float outMax) {
    // Check for division by zero to avoid potential issues
    if (inMax - inMin == 0) {
        ESP_LOGW(TAG, "Error: Division by zero in mapFloat function.");
        return 0.0f;
    }

    // Perform the linear interpolation
    return (value - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

void app_main(void) {

    esp_err_t ret = shtc3_init();
    ESP_ERROR_CHECK(ret);

    shtc3_soft_reset();

    uint16_t sesns_id = shtc3_read_id();

    ESP_LOGI(TAG, "Got ID: 0x%04X", sesns_id);

    gindicator_init();

    while(1) {
        shtc3_refresh();

        const float temperature = shtc3_get_temperature();
        const float value = mapFloat(temperature, 10.0f, 40.0f, 0, 30);
        const uint8_t red = (uint8_t)((uint32_t)value);
        const uint8_t blue = 20 - red;

        gindicator_set_rgb(red, 0, blue);

        // ESP_LOGI(TAG, "Temperature is %.2f*C", temperature);

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
