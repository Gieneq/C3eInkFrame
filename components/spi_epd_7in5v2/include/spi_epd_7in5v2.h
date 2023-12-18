#pragma once

#include <stdlib.h>
#include <esp_err.h>
#include <esp_types.h>

#include <freertos/FreeRTOS.h>

#include <driver/gpio.h>
#include <driver/spi_master.h>

#define EPD7IN5V2_SPI         SPI2_HOST

#define EPD7IN5V2_RST_PIN     GPIO_NUM_5 /* 0 = reseting */
#define EPD7IN5V2_PWR_PIN     GPIO_NUM_0 /* 0 = off, 1 = on */
#define EPD7IN5V2_DC_PIN      GPIO_NUM_6 /* 0 = cmd, 1 = data*/
#define EPD7IN5V2_BUSY_PIN    GPIO_NUM_7 /* 0 = busy */

#define EPD7IN5V2_CS_PIN      GPIO_NUM_3 /* 0 = active */
#define EPD7IN5V2_DIN_PIN     GPIO_NUM_1
#define EPD7IN5V2_CLK_PIN     GPIO_NUM_4

#define EPD7IN5V2_WIDTH       800
#define EPD7IN5V2_HEIGHT      480

#if ((EPD7IN5V2_WIDTH * EPD7IN5V2_HEIGHT) % 8) != 0
#error (EPD7IN5V2_WIDTH * EPD7IN5V2_HEIGHT) should be divisible by 8
#endif


/* Control */
esp_err_t epd7in5v2_create();

esp_err_t epd7in5v2_destroy();

esp_err_t epd7in5v2_start();

esp_err_t epd7in5v2_stop();


/* Drawing environment - wrap all drawing tools */
bool epd7in5v2_start_draw(TickType_t timeout);

void epd7in5v2_stop_draw();


/* Drawing tools */
void epd7in5v2_fill_color(const bool white);

// esp_err_t epd7in5v2_set_pixel(const uint32_t x, const uint32_t y, const bool white);

bool epd7in5v2_is_refreshed();

/* Transfer drawn stuff onto display */
bool epd7in5v2_attempt_refresh(TickType_t bus_access_timeout);

