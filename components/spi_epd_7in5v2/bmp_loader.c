#include "bmp_loader.h"

#include <stdio.h>
#include <string.h>

#include <esp_log.h>

#define ALIGN_TO_32(_value) (32 * ((_value / 32) + ((_value % 32) > 0 ? 1 : 0)))

#define BMP_FILESIZE_OFFSET   (0x02)
#define BMP_DATASTART_OFFSET  (0x0A)
#define BMP_WIDTH_OFFSET      (0x12)
#define BMP_HEIGHT_OFFSET     (0x16)
#define BMP_BPP_OFFSET        (0x1A)
#define BMP_DATASIZE_OFFSET   (0x22)

#define BMP_MIN_BYTES_COUNT   (BMP_BPP_OFFSET)

static const char* TAG = "BMPLoader";

esp_err_t bmp_loader_init(const char* file_start, const char* file_end, bmp_file_reader_t* reader) {
    const uint32_t bytes_count = ((uint32_t)file_end) - ((uint32_t)file_start);

    if ((file_start == NULL) || (file_end == NULL) || (bytes_count < BMP_MIN_BYTES_COUNT)) {
        ESP_LOGE(TAG, "File seems corrupted");
        return ESP_FAIL;
    }

    if (reader == NULL) {
        ESP_LOGE(TAG, "Returning variables are NULL");
        return ESP_FAIL;
    }
    memset(reader, 0, sizeof(bmp_file_reader_t));
    
    /* Start fillingup reader */
    reader->image.file_start = file_start;

    const uint32_t filesize = *((uint32_t*)(file_start + BMP_FILESIZE_OFFSET));
    if (filesize != bytes_count) {
        ESP_LOGE(TAG, "Corrupted filesize! Argument passed=%lu != read from file %lu", bytes_count, filesize);
        return ESP_FAIL;
    }
    reader->image.file_size = bytes_count;
    
    const uint16_t bitsperpixel = *((uint16_t*)(file_start + BMP_BPP_OFFSET));
    if (bitsperpixel != 0x1) {
        ESP_LOGE(TAG, "Image not monochrome");
        return ESP_FAIL;
    }

    reader->image.data_offset   = *((uint32_t*)(file_start + BMP_DATASTART_OFFSET));
    reader->image.width         = *((uint32_t*)(file_start + BMP_WIDTH_OFFSET));
    reader->image.height        = *((uint32_t*)(file_start + BMP_HEIGHT_OFFSET));

    reader->image.aligned_width = ALIGN_TO_32(reader->image.width);

    const uint16_t datasize = *((uint16_t*)(file_start + BMP_DATASIZE_OFFSET));
    const uint32_t datasize_calculated = bytes_count - reader->image.data_offset;
    if (((uint32_t)datasize) != datasize_calculated) {
        ESP_LOGE(TAG, "Datasize not match: %u != %lu", datasize, datasize_calculated);
        return ESP_FAIL;
    }
    reader->image.data_size = datasize_calculated;

    /* Final check */
    if (((reader->image.aligned_width / 8) * reader->image.height) != reader->image.data_size) {
        ESP_LOGE(TAG, "Image not aligned width=%lu, aligned_width=%lu, height=%lu, data_size=%lu",
            reader->image.width,
            reader->image.aligned_width,
            reader->image.height,
            reader->image.data_size
        );
        return ESP_FAIL;
    }

    /* Just to make more readible */
    reader->has_finished = false;
    
    /* Used to iterate */
    reader->recent_offset = 0;

    /* With 'chunk' buffer used to return chunk of data */
    reader->chunk_size = 0;

    return ESP_OK;
}

bool bmp_loader_read_if_any(bmp_file_reader_t* reader) {
    if (reader->has_finished == true) {
        return false;
    }

    const uint32_t bytes_left = reader->image.data_size - reader->recent_offset;
    const uint32_t recent_chunk_size = (bytes_left >= BMP_FILE_READER_CHUNK_SIZE) ? BMP_FILE_READER_CHUNK_SIZE : bytes_left;
    if (recent_chunk_size == 0) {
        /* No data to read left */
        reader->has_finished = true;
        return false;
    }

    /* Has some data to read */
    const char* read_from = reader->image.file_start + reader->image.data_offset + reader->recent_offset;

    /* Finally */
    memcpy(reader->chunk, read_from, recent_chunk_size * sizeof(uint8_t));
    reader->chunk_size = (uint8_t)recent_chunk_size;
    reader->recent_offset += recent_chunk_size;
    // ESP_LOGW(TAG, "  data_size=%lu, bytes_left=%lu, recent_chunk_size=%lu\n", reader->image.data_size, bytes_left, recent_chunk_size);
    return true;
}