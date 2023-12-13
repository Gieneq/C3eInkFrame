#include "spi_epd.h"

#include <stdio.h>
#include <string.h>

#include <esp_log.h>
#include <driver/spi_master.h>

#define EPD1IN54B_SPI         SPI2_HOST

#define EPD1IN54B_RST_PIN     GPIO_NUM_7
#define EPD1IN54B_DC_PIN      GPIO_NUM_6
#define EPD1IN54B_BUSY_PIN    GPIO_NUM_5

#define EPD1IN54B_CS_PIN      GPIO_NUM_3
#define EPD1IN54B_DIN_PIN     GPIO_NUM_1
#define EPD1IN54B_CLK_PIN     GPIO_NUM_4

#define EPD1IN54B_WIDTH       200
#define EPD1IN54B_HEIGHT      200

#define EPD1IN54B_LINES_COUNT (EPD1IN54B_HEIGHT)
#define EPD1IN54B_BW_LINE_BYTES_COUNT (EPD1IN54B_WIDTH / (8 / 2))
#define EPD1IN54B_R_LINE_BYTES_COUNT (EPD1IN54B_WIDTH / 8)
static uint8_t bw_line_bytes_buffer[EPD1IN54B_BW_LINE_BYTES_COUNT];
static uint8_t r_line_bytes_buffer[EPD1IN54B_R_LINE_BYTES_COUNT];

#define EPD1IN54B_MAX_TRANSFER_SIZE  (700)

static spi_device_handle_t spi_handle;

// #define EPD1IN54B_BUFFER_SIZE (EPD1IN54B_MAX_TRANSFER_SIZE)
// static uint8_t spi_epd_buffer[EPD1IN54B_BUFFER_SIZE];

/* LUT */
// static const unsigned char EPD1IN54B_LUT_vcom0[] = {0x0E, 0x14, 0x01, 0x0A, 0x06, 0x04, 0x0A, 0x0A, 0x0F, 0x03, 0x03, 0x0C, 0x06, 0x0A, 0x00};
// static const unsigned char EPD1IN54B_LUT_w[]     = {0x0E, 0x14, 0x01, 0x0A, 0x46, 0x04, 0x8A, 0x4A, 0x0F, 0x83, 0x43, 0x0C, 0x86, 0x0A, 0x04};
// static const unsigned char EPD1IN54B_LUT_b[]     = {0x0E, 0x14, 0x01, 0x8A, 0x06, 0x04, 0x8A, 0x4A, 0x0F, 0x83, 0x43, 0x0C, 0x06, 0x4A, 0x04};
// static const unsigned char EPD1IN54B_LUT_g1[]    = {0x8E, 0x94, 0x01, 0x8A, 0x06, 0x04, 0x8A, 0x4A, 0x0F, 0x83, 0x43, 0x0C, 0x06, 0x0A, 0x04};
// static const unsigned char EPD1IN54B_LUT_g2[]    = {0x8E, 0x94, 0x01, 0x8A, 0x06, 0x04, 0x8A, 0x4A, 0x0F, 0x83, 0x43, 0x0C, 0x06, 0x0A, 0x04};

// static const unsigned char EPD1IN54B_LUT_vcom1[] = {0x03, 0x1D, 0x01, 0x01, 0x08, 0x23, 0x37, 0x37, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
// static const unsigned char EPD1IN54B_LUT_red0[]  = {0x83, 0x5D, 0x01, 0x81, 0x48, 0x23, 0x77, 0x77, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
// static const unsigned char EPD1IN54B_LUT_red1[]  = {0x03, 0x1D, 0x01, 0x01, 0x08, 0x23, 0x37, 0x37, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

#define COMMANDS_DATA_SIZE 16
typedef struct command_data_size_t {
    uint8_t cmd;
    uint8_t data[COMMANDS_DATA_SIZE];
    uint8_t data_length;
} command_data_size_t;

static const command_data_size_t power_settings_data[] = {
    {0x01, {0x07, 0x00, 0x0F, 0x0F}, 4},    /* Power settings */
    {0x06, {0x07, 0x07, 0x07}, 3},          /* Booster settings */
    {0x04, {}, 0},                          /* Power on */
};

static const command_data_size_t panel_settings_data[] = {
    {0x00, {0xCF}, 1},  /* PANEL_SETTING */
    {0x50, {0x37}, 1},  /* VCOM_AND_DATA_INTERVAL_SETTING from datasheet must be 0x17, 
                         * something about border */
    {0x30, {0x39}, 1},  /* PLL_CONTROL, 0x2A in datasheet */
    {0x61, {0xC8, 0x00, 0xC8}, 3},  /* TCON_RESOLUTION set x and y */
    {0x82, {0x0E}, 1},  /* VCM_DC_SETTING_REGISTER */
};

static const command_data_size_t lut_bw_data[] = {
    {0x20, {0x0E, 0x14, 0x01, 0x0A, 0x06, 0x04, 0x0A, 0x0A, 0x0F, 0x03, 0x03, 0x0C, 0x06, 0x0A, 0x00}, 15}, /* vcom0 */
    {0x21, {0x0E, 0x14, 0x01, 0x0A, 0x46, 0x04, 0x8A, 0x4A, 0x0F, 0x83, 0x43, 0x0C, 0x86, 0x0A, 0x04}, 15}, /* w     */
    {0x22, {0x0E, 0x14, 0x01, 0x8A, 0x06, 0x04, 0x8A, 0x4A, 0x0F, 0x83, 0x43, 0x0C, 0x06, 0x4A, 0x04}, 15}, /* b     */
    {0x23, {0x8E, 0x94, 0x01, 0x8A, 0x06, 0x04, 0x8A, 0x4A, 0x0F, 0x83, 0x43, 0x0C, 0x06, 0x0A, 0x04}, 15}, /* g1    */
    {0x24, {0x8E, 0x94, 0x01, 0x8A, 0x06, 0x04, 0x8A, 0x4A, 0x0F, 0x83, 0x43, 0x0C, 0x06, 0x0A, 0x04}, 15}, /* g2    */
};

static const command_data_size_t lut_red_data[] = {
    {0x25, {0x03, 0x1D, 0x01, 0x01, 0x08, 0x23, 0x37, 0x37, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 15}, /* vcom1 */
    {0x26, {0x83, 0x5D, 0x01, 0x81, 0x48, 0x23, 0x77, 0x77, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 15}, /* red0  */
    {0x27, {0x03, 0x1D, 0x01, 0x01, 0x08, 0x23, 0x37, 0x37, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 15}, /* red1  */
};

static const command_data_size_t sleep_initialization_data[] = {
    {0x50, {0x17}, 1}, /* VCOM_AND_DATA_INTERVAL_SETTING */
    {0x82, {0x00}, 1}, /* VCM_DC_SETTING_REGISTER */
    {0x01, {0x02, 0x00, 0x00, 0x00}, 4}, /* POWER_SETTING */
};
static const uint8_t sleep_power_off_cmd = 0x02;

static const char* TAG = "SPI_EPD";

static esp_err_t spi_epd_send_cmd(const uint8_t cmd) {
    esp_err_t ret = ESP_OK;
    // ret = spi_device_acquire_bus(spi_handle, portMAX_DELAY);
    // if (ret != ESP_OK) {
    //     ESP_LOGE(TAG, "Send CMD: Aquiring bus failed");
    //     fflush(stdout);
    //     return ret;
    // }

    spi_transaction_t transaction;
    memset(&transaction, 0, sizeof(spi_transaction_t));
    transaction.length = 1 * 8; // in bits
    transaction.tx_buffer = (void*)&cmd;
    transaction.user = (void*)0;

    ret = spi_device_polling_transmit(spi_handle, &transaction);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Sending cmd 0x%02X failed", cmd);
        fflush(stdout);
        return ret;
    }
    
    // spi_device_release_bus(spi_handle);

    return ESP_OK;
}

static esp_err_t spi_epd_send_data(const uint8_t* data, const size_t data_length, bool keep_cs_active) {
    esp_err_t ret = ESP_OK;
    if(data_length == 0) {
        return ESP_OK;
    }

    // ret = spi_device_acquire_bus(spi_handle, portMAX_DELAY);
    // if (ret != ESP_OK) {
    //     ESP_LOGE(TAG, "Send Data: Aquiring bus failed");
    //     fflush(stdout);
    //     return ret;
    // }

    spi_transaction_t transaction;
    memset(&transaction, 0, sizeof(spi_transaction_t));
    transaction.length = data_length * 8; // in bits
    transaction.tx_buffer = (void*)data;
    transaction.user = (void*)1;
    if (keep_cs_active) {
        transaction.flags = SPI_TRANS_CS_KEEP_ACTIVE;   //Keep CS active after data transfer
    }
    
    ret = spi_device_polling_transmit(spi_handle, &transaction);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Sending data of length=%u failed. Data is null?%d", data_length, data==NULL);
        fflush(stdout);
        return ret;
    }
    
    // spi_device_release_bus(spi_handle);

    return ESP_OK;
}

static void spi_epd_pre_transfer_callback(spi_transaction_t *t) {
    const int dc_level = (int)t->user;
    gpio_set_level(EPD1IN54B_DC_PIN, dc_level);
}

static void spi_epd_wait_until_not_busy() {
    while(gpio_get_level(EPD1IN54B_BUSY_PIN) == 0) {
        vTaskDelay(1);
    }
    ESP_LOGI(TAG, "Waiting done!");
}

static esp_err_t spi_epd_setup_peripherals() {
    esp_err_t ret = ESP_OK;
    ESP_LOGI(TAG, "Peripherals initializing...");

    /* Control outputs */
    const gpio_config_t gpio_outputs_config = {
        .pin_bit_mask = (1 << EPD1IN54B_RST_PIN) | (1 << EPD1IN54B_DC_PIN),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };

    ret = gpio_config(&gpio_outputs_config);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = gpio_set_level(EPD1IN54B_RST_PIN, 0);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = gpio_set_level(EPD1IN54B_DC_PIN, 0);
    if (ret != ESP_OK) {
        return ret;
    }

    /* Control outputs */
    const gpio_config_t gpio_inputs_config = {
        .pin_bit_mask = (1 << EPD1IN54B_BUSY_PIN),
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
    spi_bus_config_t bus_config = {
        .miso_io_num = -1, // Set to -1 if not used
        .mosi_io_num = EPD1IN54B_DIN_PIN, // MOSI pin
        .sclk_io_num = EPD1IN54B_CLK_PIN, // SCLK pin
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0, // in bytes
    };

    /* Initialize the SPI bus */ 
    ret = spi_bus_initialize(EPD1IN54B_SPI, &bus_config, 0);
    if (ret != ESP_OK) {
        return ret;
    }

    /* Configure the SPI device */ 
    spi_device_interface_config_t dev_config = {
        .clock_speed_hz = 10 * 1000 * 1000,  // Clock speed
        .mode = 0,                           // SPI mode 0
        .spics_io_num = EPD1IN54B_CS_PIN,    // CS pin
        .queue_size = 1,                     // Maximum number of transactions in the SPI hardware queue
        .pre_cb = spi_epd_pre_transfer_callback
    };

    ret = spi_bus_add_device(EPD1IN54B_SPI, &dev_config, &spi_handle);
    if (ret != ESP_OK) {
        return ret;
    }

    ESP_LOGI(TAG, "Peripherals done!");
    return ESP_OK;
}

static esp_err_t spi_epd_reset() {
    esp_err_t ret = ESP_OK;

    ret = gpio_set_level(EPD1IN54B_RST_PIN, 0);
    if (ret != ESP_OK) {
        return ret;
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);

    ret = gpio_set_level(EPD1IN54B_RST_PIN, 1);
    if (ret != ESP_OK) {
        return ret;
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
    
    return ESP_OK;
}

static esp_err_t spi_epd_apply_command_data_size(const command_data_size_t* cds, const size_t cds_count) {
    esp_err_t ret = ESP_OK;

    if(cds_count > COMMANDS_DATA_SIZE) {
        ESP_LOGE(TAG, "CDS count=%d overflow %d", cds_count, COMMANDS_DATA_SIZE);
        fflush(stdout);
        return ESP_FAIL;
    }

    for(size_t cds_idx = 0; cds_idx < cds_count; ++cds_idx) {
        const command_data_size_t* selected_cds = &cds[cds_idx];
        ret = spi_epd_send_cmd(selected_cds->cmd);
        if (ret != ESP_OK) {
            fflush(stdout);
            return ret;
        }
        vTaskDelay(1);

        if(cds_count > 0) {
            /* cds_count is small compared to max transer size */
            ret = spi_epd_send_data(selected_cds->data, selected_cds->data_length, false);
            if (ret != ESP_OK) {
                fflush(stdout);
                return ret;
            }
        }
        vTaskDelay(1);
    }

    return ESP_OK;
}

static esp_err_t spi_epd_init() {
    esp_err_t ret = ESP_OK;
    ESP_LOGI(TAG, "Initializing...");
    

    /* Initial commands */
    ret = spi_epd_apply_command_data_size(power_settings_data, sizeof(power_settings_data) / sizeof(power_settings_data[0]));
    if(ret != ESP_OK) {
        return ret;
    }
    spi_epd_wait_until_not_busy();
    ESP_LOGI(TAG, "HERE1!");

    ret = spi_epd_apply_command_data_size(panel_settings_data, sizeof(panel_settings_data) / sizeof(panel_settings_data[0]));
    if(ret != ESP_OK) {
        return ret;
    }
    ESP_LOGI(TAG, "HERE2!");
    
    ret = spi_epd_apply_command_data_size(lut_bw_data, sizeof(lut_bw_data) / sizeof(lut_bw_data[0]));
    if(ret != ESP_OK) {
        return ret;
    }
    ESP_LOGI(TAG, "HERE3!");
    
    ret = spi_epd_apply_command_data_size(lut_red_data, sizeof(lut_red_data) / sizeof(lut_red_data[0]));
    if(ret != ESP_OK) {
        return ret;
    }
    ESP_LOGI(TAG, "HERE4!");

    spi_epd_wait_until_not_busy();

    ESP_LOGI(TAG, "Initializing done!");
    return ESP_OK;
}

esp_err_t spi_epd_create() {
    esp_err_t ret = ESP_OK;
    
    ret = spi_epd_setup_peripherals();
    if(ret != ESP_OK) {
        return ret;
    }

    ret = spi_epd_reset();
    if(ret != ESP_OK) {
        return ret;
    }

    ret = spi_epd_init();
    if(ret != ESP_OK) {
        return ret;
    }

    return ESP_OK;
}

esp_err_t spi_epd_sleep() {
    esp_err_t ret = ESP_OK;
    ESP_LOGI(TAG, "Going sleep...");
    
    ret = spi_epd_apply_command_data_size(sleep_initialization_data, sizeof(sleep_initialization_data) / sizeof(sleep_initialization_data[0]));
    if(ret != ESP_OK) {
        return ret;
    }

    spi_epd_wait_until_not_busy();

    vTaskDelay(pdMS_TO_TICKS(1000));

    ret = spi_epd_send_cmd(sleep_power_off_cmd);
    if(ret != ESP_OK) {
        return ret;
    }

    ESP_LOGI(TAG, "Going sleep done!");
    return ESP_OK;
}

esp_err_t spi_epd_clear_white() {
    ESP_LOGI(TAG, "Clearing white...");
    esp_err_t ret = ESP_OK;

    /* Fill chunk buffer with black data */
    /* Grayscale:
     * 0x3 = b11 -> white
     * 0x2 = b10 -> lightgray
     * 0x1 = b01 -> darkgray
     * 0x0 - b00 -> black
     * 
     * 0xff -> 4x white
     * 0xAA -> 4x lightgray
     * 0x55 -> 4x darkgray
     * 0x00 -> 4x black
    */
    for(size_t line_byte_idx = 0; line_byte_idx < EPD1IN54B_BW_LINE_BYTES_COUNT; ++line_byte_idx) {
        bw_line_bytes_buffer[line_byte_idx] = 0xFF; // full white
        // bw_line_bytes_buffer[line_byte_idx] = 0xAA; // full lightgray
        // bw_line_bytes_buffer[line_byte_idx] = 0x55; // full darkgray
        // bw_line_bytes_buffer[line_byte_idx] = 0x00; // full black
        // bw_line_bytes_buffer[line_byte_idx] = 0xE4; // gradients left(white)->right(black)
    }
    
    /* Fill chunk buffer with black data */
    for(size_t line_byte_idx = 0; line_byte_idx < EPD1IN54B_R_LINE_BYTES_COUNT; ++line_byte_idx) {
        r_line_bytes_buffer[line_byte_idx] = 0xFF; // full white
        // r_line_bytes_buffer[line_byte_idx] = 0x00; // full red
    }

    /* Black */
    ret = spi_epd_send_cmd(0x10);
    if(ret != ESP_OK) {
        return ret;
    }

    for(size_t line_idx = 0; line_idx < EPD1IN54B_LINES_COUNT; ++line_idx) {
        const bool has_next_line = (line_idx < (EPD1IN54B_LINES_COUNT - 1)) ? true : false;
        ret = spi_epd_send_data(bw_line_bytes_buffer, EPD1IN54B_BW_LINE_BYTES_COUNT, false);
        if(ret != ESP_OK) {
            return ret;
        }
    }
    vTaskDelay(1);

    /* Red */
    ret = spi_epd_send_cmd(0x13);
    if(ret != ESP_OK) {
        return ret;
    }

    for(size_t line_idx = 0; line_idx < EPD1IN54B_LINES_COUNT; ++line_idx) {
        const bool has_next_line = (line_idx < (EPD1IN54B_LINES_COUNT - 1)) ? true : false;
        ret = spi_epd_send_data(r_line_bytes_buffer, EPD1IN54B_R_LINE_BYTES_COUNT, false);
        if(ret != ESP_OK) {
            return ret;
        }
    }
    vTaskDelay(1);

    ESP_LOGI(TAG, "Sending done, waiting...");
    ret = spi_epd_send_cmd(0x12);
    if(ret != ESP_OK) {
        return ret;
    }
    spi_epd_wait_until_not_busy();
    
    ESP_LOGI(TAG, "Clearing white done!");
    return ESP_OK;
}

esp_err_t spi_epd_draw_sth() {
    ESP_LOGI(TAG, "Drawing sth...");

    esp_err_t ret = ESP_OK;

    for(size_t line_byte_idx = 0; line_byte_idx < EPD1IN54B_BW_LINE_BYTES_COUNT; ++line_byte_idx) {
        bw_line_bytes_buffer[line_byte_idx] = 0xFF; // full white
    }
    
    /* Fill chunk buffer with black data */
    for(size_t line_byte_idx = 0; line_byte_idx < EPD1IN54B_R_LINE_BYTES_COUNT; ++line_byte_idx) {
        r_line_bytes_buffer[line_byte_idx] = 0x00; // full red
    }

    /* Black */
    ret = spi_epd_send_cmd(0x10);
    if(ret != ESP_OK) {
        return ret;
    }

    for(size_t line_idx = 0; line_idx < 32; ++line_idx) {
        const bool has_next_line = (line_idx < (EPD1IN54B_LINES_COUNT - 1)) ? true : false;
        ret = spi_epd_send_data(bw_line_bytes_buffer, EPD1IN54B_BW_LINE_BYTES_COUNT, false);
        if(ret != ESP_OK) {
            return ret;
        }
    }
    
    for(size_t line_byte_idx = 0; line_byte_idx < EPD1IN54B_BW_LINE_BYTES_COUNT; ++line_byte_idx) {
        bw_line_bytes_buffer[line_byte_idx] = 0x00; // full black
    }

    for(size_t line_idx = 0; line_idx < EPD1IN54B_LINES_COUNT - 32; ++line_idx) {
        const bool has_next_line = (line_idx < (EPD1IN54B_LINES_COUNT - 1)) ? true : false;
        ret = spi_epd_send_data(bw_line_bytes_buffer, EPD1IN54B_BW_LINE_BYTES_COUNT, false);
        if(ret != ESP_OK) {
            return ret;
        }
    }
    vTaskDelay(1);

    /* Red */
    ret = spi_epd_send_cmd(0x13);
    if(ret != ESP_OK) {
        return ret;
    }

    for(size_t line_idx = 0; line_idx < 24; ++line_idx) {
        const bool has_next_line = (line_idx < (EPD1IN54B_LINES_COUNT - 1)) ? true : false;
        ret = spi_epd_send_data(r_line_bytes_buffer, EPD1IN54B_R_LINE_BYTES_COUNT, false);
        if(ret != ESP_OK) {
            return ret;
        }
    }
    vTaskDelay(1);

    ESP_LOGI(TAG, "Sth to draw sending done, waiting...");
    ret = spi_epd_send_cmd(0x12);
    if(ret != ESP_OK) {
        return ret;
    }
    spi_epd_wait_until_not_busy();
    
    ESP_LOGI(TAG, "Drawing sth done!");
    return ESP_OK;
}
