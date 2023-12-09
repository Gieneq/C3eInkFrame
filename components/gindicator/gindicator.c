#include "gindicator.h"
#include <stdio.h>
#include "led_strip.h"
#include <esp_log.h>
#include <driver/gpio.h>

static const char *TAG = "GIndicator";

#define GINDICATOR_RGB_PIN    GPIO_NUM_2
#define GINDICATOR_RED_PIN    GPIO_NUM_7

static led_strip_handle_t led_strip;


/**
 * @brief Simple helper function, converting HSV color space to RGB color space
 *
 * Wiki: https://en.wikipedia.org/wiki/HSL_and_HSV
 *
 */
static void hsv2rgb(uint32_t h, uint32_t s, uint32_t v, uint32_t *r, uint32_t *g, uint32_t *b)
{
    h %= 360; // h -> [0,360]
    uint32_t rgb_max = v * 2.55f;
    uint32_t rgb_min = rgb_max * (100 - s) / 100.0f;

    uint32_t i = h / 60;
    uint32_t diff = h % 60;

    // RGB adjustment amount by hue
    uint32_t rgb_adj = (rgb_max - rgb_min) * diff / 60;

    switch (i) {
    case 0:
        *r = rgb_max;
        *g = rgb_min + rgb_adj;
        *b = rgb_min;
        break;
    case 1:
        *r = rgb_max - rgb_adj;
        *g = rgb_max;
        *b = rgb_min;
        break;
    case 2:
        *r = rgb_min;
        *g = rgb_max;
        *b = rgb_min + rgb_adj;
        break;
    case 3:
        *r = rgb_min;
        *g = rgb_max - rgb_adj;
        *b = rgb_max;
        break;
    case 4:
        *r = rgb_min + rgb_adj;
        *g = rgb_min;
        *b = rgb_max;
        break;
    default:
        *r = rgb_max;
        *g = rgb_min;
        *b = rgb_max - rgb_adj;
        break;
    }
}

void gindicator_init() {
    ESP_LOGI(TAG, "GIndicator initializing...");
    
    led_strip_config_t strip_config = {
        .strip_gpio_num = GINDICATOR_RGB_PIN,
        .max_leds = 1, 
    };

    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));

    led_strip_clear(led_strip);

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GINDICATOR_RED_PIN),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io_conf);


    ESP_LOGI(TAG, "GIndicator done!");

    gindicator_set_hsv(60, 255, 255);
}


void gindicator_set_hsv(uint16_t h, uint8_t s, uint8_t v) {
    /* h[0,260], s[0,255], v[0,255] */
    uint32_t r, g, b;

    hsv2rgb(
        (uint32_t)h, (uint32_t)s, (uint32_t)v,
        &r, &g, &b
    );

    led_strip_set_pixel(led_strip, 0, (uint8_t)r, (uint8_t)g, (uint8_t)b);
    led_strip_refresh(led_strip);
}

void gindicator_set_rgb(uint8_t r, uint8_t g, uint8_t b) {
    led_strip_set_pixel(led_strip, 0, r, g, b);
    led_strip_refresh(led_strip);
}