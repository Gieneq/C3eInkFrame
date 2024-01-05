#pragma once

#include <stdlib.h>
#include <stdbool.h>
#include <esp_err.h>

#define GCAPTIVE_MAC_SIZE (32)
typedef struct connected_device_data_t {
    char mac_str[GCAPTIVE_MAC_SIZE];
    uint8_t aid;
} connected_device_data_t;

esp_err_t gcaptive_create();

esp_err_t gcaptive_start();

esp_err_t gcaptive_stop();

bool gcaptive_is_started();

const char* gcaptive_get_ssid();

const char* gcaptive_get_psswd();

const char* gcaptive_get_ipv4_str();

bool gcaptive_has_connected_devices_changed();

bool gcaptive_was_image_uploaded();

void gcaptive_get_connected_devices(uint8_t* devices_count);

bool gcaptive_is_running();