#pragma once

#include <stdlib.h>
#include <esp_err.h>

esp_err_t shtc3_init();

uint16_t shtc3_read_id();

void shtc3_soft_reset();

void shtc3_refresh();

float shtc3_get_temperature();

float shtc3_get_humidity();
