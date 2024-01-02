#pragma once
#include <esp_err.h>

#define GSTORAGE_BASE_PATH "/data"

esp_err_t gstorage_mount(const char* base_path);