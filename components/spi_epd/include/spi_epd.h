#pragma once

#include <stdlib.h>
#include <esp_err.h>
#include <esp_types.h>

#include <freertos/FreeRTOS.h>

typedef enum spi_epd_color_t {
    SPI_EPD_COLOR_WHITE = 0,
    SPI_EPD_COLOR_LIGHTGRAY,
    SPI_EPD_COLOR_DARKGRAY,
    SPI_EPD_COLOR_BLACK,
    SPI_EPD_COLOR_RED
} spi_epd_color_t;


/* Control */
esp_err_t spi_epd_create();

esp_err_t spi_epd_destroy();

esp_err_t spi_epd_start();

esp_err_t spi_epd_stop();


/* Drawing environment - wrap all drawing tools */
bool spi_epd_start_draw(TickType_t timeout);

void spi_epd_end_draw();


/* Drawing tools */
void spi_epd_fill_color(const spi_epd_color_t color);

// esp_err_t spi_epd_set_pixel(const uint32_t x, const uint32_t y, const uint8_t gray, const uint8_t red);

// esp_err_t spi_epd_set_pixel(const uint32_t x, const uint32_t y, const uint8_t color_ggr);

bool spi_epd_is_refreshed();

/* Transfer drawn stuff onto display */
bool spi_epd_attempt_refresh(TickType_t bus_access_timeout);

