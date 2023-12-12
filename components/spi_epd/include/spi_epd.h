#pragma once

#include <stdlib.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <esp_err.h>
#include <driver/gpio.h>

esp_err_t spi_epd_create();

esp_err_t spi_epd_sleep();

esp_err_t spi_epd_clear_white();