#pragma once

#include <stdint.h>
#include <esp_err.h>
#include <stdbool.h>

#define BMP_FILE_READER_CHUNK_SIZE (64)
typedef struct bmp_file_reader_t {
    struct {
        const char* file_start;
        uint32_t file_size;
        uint32_t width;
        uint32_t aligned_width;
        uint32_t height;
        uint32_t data_offset;
        uint32_t data_size;
    } image;
    uint32_t recent_offset;
    uint8_t chunk[BMP_FILE_READER_CHUNK_SIZE];
    uint8_t chunk_size;
    bool has_finished;
} bmp_file_reader_t;

esp_err_t bmp_loader_init(const char* file_start, const char* file_end, bmp_file_reader_t* reader);

bool bmp_loader_read_if_any(bmp_file_reader_t* reader);