#include "spi_epd_7in5v2.h"

#include <stdio.h>
#include <string.h>

#include <freertos/task.h>

#include <freertos/event_groups.h>
#include <freertos/semphr.h>

#include <esp_log.h>
#include <rom/ets_sys.h>

#define EPD_FRAMEBUFFER_SIZE   ((EPD7IN5V2_WIDTH * EPD7IN5V2_HEIGHT) / 8)

#define EPD_FRAMEBUFFER_CHUNK_SIZE (50)

#define EPD_CHUNKS_COUNT (EPD_FRAMEBUFFER_SIZE / EPD_FRAMEBUFFER_CHUNK_SIZE)

static uint8_t epd_framebuffer[EPD_FRAMEBUFFER_SIZE];

static bool need_clear = false;
static bool clear_to_white = true;

#define COMMANDS_DATA_SIZE 42
typedef struct command_data_size_t {
    uint8_t cmd;
    uint8_t data[COMMANDS_DATA_SIZE];
    uint8_t data_length;
} command_data_size_t;

// static const uint8_t Voltage_Frame_7IN5_V2[]={
// 	0x6, 0x3F, 0x3F, 0x11, 0x24, 0x7, 0x17,
// };

/* Commands and commands sets */
static const command_data_size_t power_settings_data[] = {
    {0x01, {0x17, 0x17, 0x3F, 0x3F, 0x11}, 5},    /* Power settings:
                                                   * initial power, VGH&VGL, VSH, VSL, VSHR
                                                   */
    {0x82, {0x24}, 1},    /* VCOM DC Setting */

    {0x06, {0x27, 0x27, 0x2F, 0x17}, 4},          /* Booster settings */


    {0x30, {0x06}, 1},                          /* OSC Setting: 2-0=100: N=4  ; 5-3=111: M=7  ;  3C=50Hz     3A=100HZ*/

    {0x04, {}, 0},                          /* Power on */
};

static const command_data_size_t panel_settings_data[] = {
    {0x00, {0x3F}, 1},  /* PANEL_SETTING : KW-3f   KWR-2F	BWROTP 0f	BWOTP 1f */

    {0x61, {0x03, 0x20, 0x01, 0xE0}, 4},  /* /tres, //source 800, //gate 480 */

    {0x15, {0x00}, 1},  /* ?? */

    {0x50, {0x10, 0x00}, 2},  /* VCOM AND DATA INTERVAL SETTING */

    {0x60, {0x22}, 1},  /* TCON SETTING */

    {0x65, {0x00, 0x00, 0x00, 0x00}, 4},  /* Resolution setting */


    // {0x50, {0x37}, 1},  /* VCOM_AND_DATA_INTERVAL_SETTING from datasheet must be 0x17, 
    //                      * something about border */
    // {0x30, {0x39}, 1},  /* PLL_CONTROL, 0x2A in datasheet */
    // {0x61, {0xC8, 0x00, 0xC8}, 3},  /* TCON_RESOLUTION set x and y */
    // {0x82, {0x0E}, 1},  /* VCM_DC_SETTING_REGISTER */
};

static const command_data_size_t lut_data[] = {
    {0x20, 
    {0x0,	0xF,	0xF,	0x0,	0x0,	0x1,	
	0x0,	0xF,	0x1,	0xF,	0x1,	0x2,	
	0x0,	0xF,	0xF,	0x0,	0x0,	0x1,	
	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	
	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	
	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	
	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,}, 42}, /* vcom0 */

    {0x21, 
    {0x10,	0xF,	0xF,	0x0,	0x0,	0x1,	
	0x84,	0xF,	0x1,	0xF,	0x1,	0x2,	
	0x20,	0xF,	0xF,	0x0,	0x0,	0x1,	
	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	
	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	
	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	
	0x0,	0x0,	0x0,	0x0,	0x0,	0x0}, 42}, /* ww    */

    {0x22, 
    {0x10,	0xF,	0xF,	0x0,	0x0,	0x1,	
	0x84,	0xF,	0x1,	0xF,	0x1,	0x2,	
	0x20,	0xF,	0xF,	0x0,	0x0,	0x1,	
	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	
	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	
	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	
	0x0,	0x0,	0x0,	0x0,	0x0,	0x0}, 42}, /* bb    */

    {0x23, 
    {0x80,	0xF,	0xF,	0x0,	0x0,	0x1,	
	0x84,	0xF,	0x1,	0xF,	0x1,	0x2,	
	0x40,	0xF,	0xF,	0x0,	0x0,	0x1,	
	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	
	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	
	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	
	0x0,	0x0,	0x0,	0x0,	0x0,	0x0}, 42}, /* wb    */

    {0x24, 
    {0x80,	0xF,	0xF,	0x0,	0x0,	0x1,	
	0x84,	0xF,	0x1,	0xF,	0x1,	0x2,	
	0x40,	0xF,	0xF,	0x0,	0x0,	0x1,	
	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	
	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	
	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	
	0x0,	0x0,	0x0,	0x0,	0x0,	0x0}, 42}, /* bb    */
};





// static const command_data_size_t sleep_initialization_data[] = {
//     {0x50, {0x17}, 1}, /* VCOM_AND_DATA_INTERVAL_SETTING */
//     {0x82, {0x00}, 1}, /* VCM_DC_SETTING_REGISTER */
//     {0x01, {0x02, 0x00, 0x00, 0x00}, 4}, /* POWER_SETTING */
// };
// static const uint8_t sleep_power_off_cmd = 0x02;

static const char* TAG = "EPD7IN5V2";

static spi_device_handle_t epd_spi_device_handle;

#define EPD_EVENTS_NEED_REFRESH                      (1<<0)
#define EPD_EVENTS_ALREADY_REFRESHED                 (1<<1)
static EventGroupHandle_t epd_events;

static SemaphoreHandle_t epd_framebuffer_mutex;

static TaskHandle_t epd_task_handle;

static bool epd_is_sleeping = false;

/* Static prototypes */

static esp_err_t epd_wake_up();

static esp_err_t epd_sleep();

static esp_err_t epd_send_refresh();

static esp_err_t epd_reset();

static esp_err_t epd_write_settings();

/* Static functions */

static void epd_task(void* params) {
    esp_err_t ret = ESP_OK;
    ret = epd_reset();
    if(ret == ESP_OK) {
        ret = epd_write_settings();
    }

    epd_is_sleeping = false;

    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "Starting epd task failed!");
        vTaskDelete(NULL);
        return;
    }

    xEventGroupSetBits(epd_events, EPD_EVENTS_ALREADY_REFRESHED);

    while(1) {
        xEventGroupWaitBits(epd_events, EPD_EVENTS_NEED_REFRESH, pdTRUE, pdFALSE, portMAX_DELAY);
        ESP_LOGI(TAG, "Refreshing started...");
        /* Wake up */
        ret = epd_wake_up();

        /* Send data, refresh and wait until done */
        if(ret == ESP_OK) {
            ret = epd_send_refresh();
        }

        /* Go to sleep */
        if(ret == ESP_OK) {
            ret = epd_sleep();
        }

        /* Error check */
        if(ret != ESP_OK) {
            ESP_LOGE(TAG, "Something bad happen during refresh!");
        }

        ESP_LOGI(TAG, "Refreshing done!");
        xEventGroupSetBits(epd_events, EPD_EVENTS_ALREADY_REFRESHED);
    }
    
    vTaskDelete(NULL);
}

static esp_err_t epd_send_cmd(const uint8_t cmd) {
    esp_err_t ret = ESP_OK;

    spi_transaction_t transaction;
    memset(&transaction, 0, sizeof(spi_transaction_t));
    transaction.length = 1 * 8; // in bits
    transaction.tx_buffer = (void*)&cmd;
    transaction.user = (void*)0;

    ret = spi_device_polling_transmit(epd_spi_device_handle, &transaction);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Sending cmd 0x%02X failed", cmd);
        fflush(stdout);
        return ret;
    }
    return ESP_OK;
}

static esp_err_t epd_send_data(const uint8_t* data, const size_t data_length, bool keep_cs_active) {
    esp_err_t ret = ESP_OK;
    if(data_length == 0) {
        return ESP_OK;
    }

    spi_transaction_t transaction;
    memset(&transaction, 0, sizeof(spi_transaction_t));
    transaction.length = data_length * 8; // in bits
    transaction.tx_buffer = (void*)data;
    transaction.user = (void*)1;
    if (keep_cs_active) {
        transaction.flags = SPI_TRANS_CS_KEEP_ACTIVE;   //Keep CS active after data transfer
    }
    
    ret = spi_device_polling_transmit(epd_spi_device_handle, &transaction);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Sending data of length=%u failed. Data is null?%d", data_length, data==NULL);
        fflush(stdout);
        return ret;
    }
    return ESP_OK;
}

static void epd_spi_pre_transfer_callback(spi_transaction_t *t) {
    const int dc_level = (int)t->user;
    gpio_set_level(EPD7IN5V2_DC_PIN, dc_level);
}

static void epd_wait_until_not_busy() {
    /* 1 is not busy, 0 is busy */
    // do {
    //     // if(epd_send_cmd(0x71) != ESP_OK) {
    //     //     ESP_LOGE(TAG, "Sending cmd 0x71 failed during \'epd_wait_until_not_busy\'");
    //     // }
    //     vTaskDelay(1);
    // } while(gpio_get_level(EPD7IN5V2_BUSY_PIN) == 0);
    
    while(gpio_get_level(EPD7IN5V2_BUSY_PIN) == 0) {
        vTaskDelay(1);
    }

    ESP_LOGD(TAG, "Waiting done!");
}

static esp_err_t epd_setup_peripherals() {
    esp_err_t ret = ESP_OK;
    ESP_LOGI(TAG, "Peripherals epd initializing...");

    /* Control outputs */
    const gpio_config_t gpio_outputs_config = {
        .pin_bit_mask = (1 << EPD7IN5V2_RST_PIN) | (1 << EPD7IN5V2_DC_PIN) | (1 << EPD7IN5V2_PWR_PIN),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };

    ret = gpio_config(&gpio_outputs_config);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = gpio_set_level(EPD7IN5V2_RST_PIN, 0);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = gpio_set_level(EPD7IN5V2_DC_PIN, 0);
    if (ret != ESP_OK) {
        return ret;
    }

    /* 1 sets to power ON */
    ret = gpio_set_level(EPD7IN5V2_PWR_PIN, 1);
    if (ret != ESP_OK) {
        return ret;
    }

    /* Control outputs */
    const gpio_config_t gpio_inputs_config = {
        .pin_bit_mask = (1 << EPD7IN5V2_BUSY_PIN),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };

    ret = gpio_config(&gpio_inputs_config);
    if (ret != ESP_OK) {
        return ret;
    }

    /* Configure SPI bus */
    const spi_bus_config_t bus_config = {
        .miso_io_num = -1, // Set to -1 if not used
        .mosi_io_num = EPD7IN5V2_DIN_PIN, // MOSI pin
        .sclk_io_num = EPD7IN5V2_CLK_PIN, // SCLK pin
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0, // in bytes
    };

    /* Initialize the SPI bus */ 
    ret = spi_bus_initialize(EPD7IN5V2_SPI, &bus_config, 0);
    if (ret != ESP_OK) {
        return ret;
    }

    /* Configure the SPI device */ 
    const spi_device_interface_config_t dev_config = {
        .clock_speed_hz = 500 * 1000,  // Clock speed
        .mode = 0,                           // SPI mode 0
        .spics_io_num = EPD7IN5V2_CS_PIN,    // CS pin
        .queue_size = 1,                     // Maximum number of transactions in the SPI hardware queue
        .pre_cb = epd_spi_pre_transfer_callback
    };

    ret = spi_bus_add_device(EPD7IN5V2_SPI, &dev_config, &epd_spi_device_handle);
    if (ret != ESP_OK) {
        return ret;
    }

    ESP_LOGI(TAG, "Peripherals epd done!");
    return ESP_OK;
}

static esp_err_t epd_reset() {
    esp_err_t ret = ESP_OK;

    ret = gpio_set_level(EPD7IN5V2_RST_PIN, 1);
    if (ret != ESP_OK) {
        return ret;
    }
    vTaskDelay(200 / portTICK_PERIOD_MS);

    ret = gpio_set_level(EPD7IN5V2_RST_PIN, 0);
    if (ret != ESP_OK) {
        return ret;
    }
    vTaskDelay(20 / portTICK_PERIOD_MS);

    ret = gpio_set_level(EPD7IN5V2_RST_PIN, 1);
    if (ret != ESP_OK) {
        return ret;
    }
    vTaskDelay(200 / portTICK_PERIOD_MS);
    
    return ESP_OK;
}

static esp_err_t epd_apply_command_data_size(const command_data_size_t* cds, const size_t cds_count) {
    esp_err_t ret = ESP_OK;

    if(cds_count > COMMANDS_DATA_SIZE) {
        ESP_LOGE(TAG, "CDS count=%d overflow %d", cds_count, COMMANDS_DATA_SIZE);
        fflush(stdout);
        return ESP_FAIL;
    }

    for(size_t cds_idx = 0; cds_idx < cds_count; ++cds_idx) {
        const command_data_size_t* selected_cds = &cds[cds_idx];
        ret = epd_send_cmd(selected_cds->cmd);
        if (ret != ESP_OK) {
            fflush(stdout);
            return ret;
        }
        ets_delay_us(10);

        if(cds_count > 0) {
            /* cds_count is small compared to max transer size */
            ret = epd_send_data(selected_cds->data, selected_cds->data_length, false);
            if (ret != ESP_OK) {
                fflush(stdout);
                return ret;
            }
        }
        ets_delay_us(100);
    }

    return ESP_OK;
}

static esp_err_t epd_write_settings() {
    /* Initial commands */
    esp_err_t ret = ESP_OK;
    
    ESP_LOGI(TAG, "Initializing...");

    ret = spi_device_acquire_bus(epd_spi_device_handle, portMAX_DELAY);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Send Data: Aquiring bus failed");
        fflush(stdout);
        return ret;
    }

    /* Power settings related commands */
    ret = epd_apply_command_data_size(power_settings_data, sizeof(power_settings_data) / sizeof(power_settings_data[0]));
    if(ret != ESP_OK) {
        return ret;
    }
    epd_wait_until_not_busy();

    /* Panel settings related commands */
    ret = epd_apply_command_data_size(panel_settings_data, sizeof(panel_settings_data) / sizeof(panel_settings_data[0]));
    if(ret != ESP_OK) {
        return ret;
    }
    
    /* LUT */
    ret = epd_apply_command_data_size(lut_data, sizeof(lut_data) / sizeof(lut_data[0]));
    if(ret != ESP_OK) {
        return ret;
    }
    
    spi_device_release_bus(epd_spi_device_handle);

    epd_wait_until_not_busy();

    ESP_LOGI(TAG, "Initializing done!");
    return ESP_OK;
}

static esp_err_t epd_wake_up() {
    if(epd_is_sleeping == false) {
        /* Already wakend up */
        return ESP_OK;
    }
    ESP_LOGI(TAG, "Wake up started...");
    ESP_LOGI(TAG, "Wake up done!");
    epd_is_sleeping = false;
    return ESP_OK;
}

static esp_err_t epd_sleep() {
    if(epd_is_sleeping == true) {
        /* Already sleeping */
        return ESP_OK;
    }
    esp_err_t ret = ESP_OK;
    ESP_LOGI(TAG, "Going sleep...");

    ret = spi_device_acquire_bus(epd_spi_device_handle, portMAX_DELAY);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Send Data: Aquiring bus failed");
        fflush(stdout);
        return ret;
    }
    
    // ret = spi_epd_apply_command_data_size(sleep_initialization_data, sizeof(sleep_initialization_data) / sizeof(sleep_initialization_data[0]));
    // if(ret != ESP_OK) {
    //     return ret;
    // }

    // spi_epd_wait_until_not_busy();

    // vTaskDelay(pdMS_TO_TICKS(1000));

    // ret = spi_epd_send_cmd(sleep_power_off_cmd);
    // if(ret != ESP_OK) {
    //     return ret;
    // }

    spi_device_release_bus(epd_spi_device_handle);

    ESP_LOGI(TAG, "Going sleep done!");
    epd_is_sleeping = true;
    return ESP_OK;
}

static esp_err_t epd_send_refresh() {
    ESP_LOGI(TAG, "Refreshing started...");
    esp_err_t ret = ESP_OK;

    ret = spi_device_acquire_bus(epd_spi_device_handle, portMAX_DELAY);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Send Data: Aquiring bus failed");
        fflush(stdout);
        return ret;
    }
    
    /* Black & White framebuffer data */
    if (need_clear == true) {
        if (clear_to_white == true) {
            ret = epd_send_cmd(0x10);
            if(ret != ESP_OK) {
                return ret;
            }

            for(size_t chunk_idx = 0; chunk_idx < EPD_CHUNKS_COUNT; ++chunk_idx) {
                const bool has_next_chunk = (chunk_idx < (EPD_CHUNKS_COUNT - 1)) ? true : false;
                const bool extend_cs = has_next_chunk;
                ret = epd_send_data(
                    epd_framebuffer + (chunk_idx * EPD_FRAMEBUFFER_CHUNK_SIZE),
                    EPD_FRAMEBUFFER_CHUNK_SIZE,
                    extend_cs
                );
                if(ret != ESP_OK) {
                    return ret;
                }
            }
            vTaskDelay(1);
        }
    }

    ret = epd_send_cmd(0x13);
    if(ret != ESP_OK) {
        return ret;
    }

    for(size_t chunk_idx = 0; chunk_idx < EPD_CHUNKS_COUNT; ++chunk_idx) {
        const bool has_next_chunk = (chunk_idx < (EPD_CHUNKS_COUNT - 1)) ? true : false;
        const bool extend_cs = has_next_chunk;
        ret = epd_send_data(
            epd_framebuffer + (chunk_idx * EPD_FRAMEBUFFER_CHUNK_SIZE),
            EPD_FRAMEBUFFER_CHUNK_SIZE,
            extend_cs
        );
        if(ret != ESP_OK) {
            return ret;
        }
    }
    vTaskDelay(1);

    ESP_LOGI(TAG, "SPI sending done - bus released. Waiting till refreshed...");
    ret = epd_send_cmd(0x12);
    if(ret != ESP_OK) {
        return ret;
    }

    vTaskDelay(10);

    epd_wait_until_not_busy();

    return ESP_OK;
}

/* Public */

esp_err_t epd7in5v2_create() {
    esp_err_t ret = ESP_OK;
    
    ret = epd_setup_peripherals();
    if(ret != ESP_OK) {
        return ret;
    }

    epd_events = xEventGroupCreate();
    if(epd_events == NULL) {
        ESP_LOGE(TAG, "Creating epd event group failed");
        return ESP_FAIL;
    }

    epd_framebuffer_mutex = xSemaphoreCreateMutex();
    if (epd_framebuffer_mutex == NULL) {
        ESP_LOGE(TAG, "Creating epd mutex failed");
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t epd7in5v2_destroy() {
    // esp_err_t ret = ESP_OK;
    ESP_LOGE(TAG, "Not implemented");
    return ESP_FAIL; //todo not implemented
}

esp_err_t epd7in5v2_start() {
    // esp_err_t ret = ESbP_OK;

    if (epd_task_handle != NULL) {
        ESP_LOGE(TAG, "epd task already created");
        return ESP_FAIL;
    }

    xTaskCreate(
        epd_task,
        TAG,
        2048,
        NULL,
        20,
        &epd_task_handle
    );

    if(epd_task_handle == NULL) {
        ESP_LOGE(TAG, "epd task creating failed");
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t epd7in5v2_stop() {
    // esp_err_t ret = ESP_OK;
    ESP_LOGE(TAG, "Not implemented");
    return ESP_FAIL; //todo not implemented
}

void epd7in5v2_fill_color(const bool white) {
    /* Black & white:
     * 0x0 -> WHITE
     * 0x1 -> BLACK
    */

   need_clear = true;
   clear_to_white = white;

    uint8_t color_8_pixels = (white == true) ? 0x00 : 0xFF;
    /* Fill framebuffers */
    memset(epd_framebuffer, color_8_pixels, sizeof(epd_framebuffer));
    ESP_LOGI(TAG, "Filling framebuffer done!");
}

void epd7in5v2_set_pixel(const int32_t x, const int32_t y, const bool white) {
    if((x < 0) || (x >= EPD7IN5V2_WIDTH) || (y < 0) || (y >= EPD7IN5V2_HEIGHT)) {
        return;
    }

    const uint32_t px_idx = x + y * EPD7IN5V2_WIDTH;
    const uint32_t byte_idx = px_idx / 8;
    const uint8_t bit_offset = 8 - (px_idx % 8) - 1;

    // epd_framebuffer[byte_idx] = (white == true) ? 0x00 : 0xFF;

    epd_framebuffer[byte_idx] = (epd_framebuffer[byte_idx] & ~(1 << bit_offset)) | (((white == true) ? 0x0 : 0x1) << bit_offset);
}

void epd7in5v2_fill_rect(const int32_t x, const int32_t y, const uint32_t width, const uint32_t height, const bool white) {
    for (uint32_t iy=0; iy<height; ++iy) {
        for (uint32_t ix=0; ix<width; ++ix) {
            epd7in5v2_set_pixel(x + ix, y + iy, white);
        }
    }
}

bool epd7in5v2_attempt_refresh(TickType_t bus_access_timeout) {
    xEventGroupWaitBits(epd_events, EPD_EVENTS_ALREADY_REFRESHED, pdTRUE, pdFALSE, bus_access_timeout);
    xEventGroupSetBits(epd_events, EPD_EVENTS_NEED_REFRESH);
    return true;
}

bool epd7in5v2_start_draw(TickType_t timeout) {
    if (xSemaphoreTake(epd_framebuffer_mutex, timeout) != pdTRUE) {
        return false;
    }
    return true;
}

void epd7in5v2_stop_draw() {
    xSemaphoreGive(epd_framebuffer_mutex);
}

bool epd7in5v2_is_refreshed() {
    const EventBits_t bits = xEventGroupGetBits(epd_events);
    return ((bits & EPD_EVENTS_ALREADY_REFRESHED) > 0) ? true : false;
}