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

#include <esp_wifi.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <driver/gpio.h>

#include "shtc3_sensor.h"
#include "gindicator.h"
#include "spi_epd_7in5v2.h"
#include "gframe.h"
#include "gstorage.h"

#define TOUCH_SENS_PIN GPIO_NUM_20

#define STHC3_SENSOR_REFRESH_INTERVAL       5000
#define MAIN_TASK_LOOPING_DELAY_MS          10
#define LED_TOUCH_VISUALIZATION_DELAY_MS    500
static const char* TAG = "Main";

static volatile int ts_counter = 0;

void IRAM_ATTR gpio_isr_handler(void* arg) {
    /* This function will be called 
    * on the rising edge interrupt 
    */ 
    ++ts_counter;
}


/* Setup */
static void setup_epb() {
    ESP_ERROR_CHECK(epd7in5v2_create());
    ESP_ERROR_CHECK(epd7in5v2_start());
    epd7in5v2_set_rotation(2);
}

static void setup_ts() {
    gpio_config_t gpio_ts_config = {
        .pin_bit_mask = 1 << TOUCH_SENS_PIN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE
    };

    gpio_config(&gpio_ts_config);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(TOUCH_SENS_PIN, gpio_isr_handler, (void*) TOUCH_SENS_PIN);
}

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

static void setup_general() {
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(gstorage_mount(GSTORAGE_BASE_PATH));
}

static void setup_gframe() {
        ESP_ERROR_CHECK(gframe_create());
        ESP_ERROR_CHECK(gframe_start());
}

/* Refresh */

static void refresh_sensors() {
    shtc3_refresh();

    const float temperature = shtc3_get_temperature();
    const float humidity = shtc3_get_humidity();

    ESP_LOGI(TAG, "Temperature is %.2f*C, humidity is: %.2f%%", temperature, humidity);
}

void app_main(void) {
    setup_general();
    setup_ts();
    setup_epb();
    setup_gframe();
    setup_indicators();
    setup_sensors();

    refresh_sensors();
    int last_ts_counter = ts_counter;

    int sensor_refresh_counter = 0;

    ESP_LOGI(TAG, "Start loop!");
    while(1) {
        if(last_ts_counter != ts_counter) {
            last_ts_counter = ts_counter;
            gindicator_set_rgb(0, 45, 140);
            printf("GPIO Interrupt Triggered: %d!\n", ts_counter);
            gframe_enque_shortclick();
            vTaskDelay(pdMS_TO_TICKS(LED_TOUCH_VISUALIZATION_DELAY_MS));
            gindicator_set_rgb(0, 0, 0);
        }

        vTaskDelay(pdMS_TO_TICKS(MAIN_TASK_LOOPING_DELAY_MS));
        sensor_refresh_counter += MAIN_TASK_LOOPING_DELAY_MS;
        if(sensor_refresh_counter > STHC3_SENSOR_REFRESH_INTERVAL) {
            sensor_refresh_counter -= STHC3_SENSOR_REFRESH_INTERVAL;
            refresh_sensors();
        }
    }
}
