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
#include "gcaptive.h"

#include <esp_wifi.h>
#include <esp_system.h>
#include <nvs_flash.h>


static const char* TAG = "Main";

/* Setup */

static void setup_sensors() {
    esp_err_t ret = shtc3_init();
    ESP_ERROR_CHECK(ret);

    shtc3_soft_reset();

    uint16_t sesns_id = shtc3_read_id();

    ESP_LOGI(TAG, "Got ID: 0x%04X", sesns_id);

}

static void setup_indicators() {
    gindicator_init();
    gindicator_set_rgb(0, 0, 0);
}

static void setup_gemeral() {
        // ESP_ERROR_CHECK(nvs_flash_init());
        // ESP_ERROR_CHECK(esp_netif_init());
        // ESP_ERROR_CHECK(esp_event_loop_create_default());
}

static void setup_connectivity() {
    gcaptive_create();
    gcaptive_start();
}

/* Refresh */

static void refresh_sensors() {
    shtc3_refresh();

    const float temperature = shtc3_get_temperature();
    const float humidity = shtc3_get_humidity();

    ESP_LOGI(TAG, "Temperature is %.2f*C, humidity is: %.2f%%", temperature, humidity);
}

void app_main(void) {
    setup_gemeral();
    setup_indicators();
    setup_sensors();
    setup_connectivity();

    while(1) {
        // refresh_sensors();
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
