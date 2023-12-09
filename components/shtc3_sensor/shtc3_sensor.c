#include "shtc3_sensor.h"

#include <stdio.h>
#include <esp_log.h>
#include <driver/i2c.h>

#define SHTC3_DEBUG_LOG                              (0)

#define SHTC3_SDA_PIN                                 10
#define SHTC3_SCL_PIN                                  8
#define SHTC3_I2C_FREQ                 ((uint32_t)100000)
#define SHTC3_ADDRESS                                0x70
#define SHTC3_I2C_PORT_NUM       ((i2c_port_t )I2C_NUM_0)

#define I2C_MASTER_TX_BUF_DISABLE                       0 
#define I2C_MASTER_RX_BUF_DISABLE                       0

#define SHTC3_READ_ID_CMD        0xEFC8
#define SHTC3_MEASURE_CMD        0x7CA2
#define SHTC3_SOFT_RESET_CMD     0x805D
#define SHTC3_WAKEUP_CMD         0x3517
#define SHTC3_SLEEP_CMD          0xB098

static const char* TAG = "SHTC3";

static float temperature = 0.0f;
static float humidity = 0.0f;

esp_err_t shtc3_init() {
#ifdef SHTC3_DEBUG_LOG
    esp_log_level_set(TAG, ESP_LOG_DEBUG);
#else
    esp_log_level_set(TAG, ESP_LOG_NONE);
#endif

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SHTC3_SDA_PIN,
        .scl_io_num = SHTC3_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = SHTC3_I2C_FREQ,
    };
    
    esp_err_t ret = i2c_param_config(SHTC3_I2C_PORT_NUM, &conf);
    if (ret == ESP_OK) {
        ret = i2c_driver_install(
            SHTC3_I2C_PORT_NUM, 
            conf.mode, 
            I2C_MASTER_RX_BUF_DISABLE, 
            I2C_MASTER_TX_BUF_DISABLE, 
            0
        );
    }

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "SHTC I2C master created successfully!");
    }

    return ret;
}

void shtc3_soft_reset() {
    ESP_LOGI(TAG, "Soft Reset attempt...");
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SHTC3_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, (SHTC3_SOFT_RESET_CMD >> 8) & 0xFF, true);
    i2c_master_write_byte(cmd, SHTC3_SOFT_RESET_CMD & 0xFF, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(SHTC3_I2C_PORT_NUM, cmd, portMAX_DELAY);
    i2c_cmd_link_delete(cmd);
    vTaskDelay(pdMS_TO_TICKS(15));
    ESP_LOGI(TAG, "Soft Reset done.");
}

uint16_t shtc3_read_id() {
    uint8_t data[2] = {0};
    ESP_LOGI(TAG, "Read ID attempt...");
    {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (SHTC3_ADDRESS << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, (SHTC3_READ_ID_CMD >> 8) & 0xFF, true);
        i2c_master_write_byte(cmd, SHTC3_READ_ID_CMD & 0xFF, true);
        i2c_master_stop(cmd);
        i2c_master_cmd_begin(SHTC3_I2C_PORT_NUM, cmd, portMAX_DELAY);
        i2c_cmd_link_delete(cmd);
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (SHTC3_ADDRESS << 1) | I2C_MASTER_READ, true);
        i2c_master_read(cmd, data, sizeof(data), I2C_MASTER_ACK);
        i2c_master_stop(cmd);
        i2c_master_cmd_begin(SHTC3_I2C_PORT_NUM, cmd, portMAX_DELAY);
        i2c_cmd_link_delete(cmd);
    }
    vTaskDelay(pdMS_TO_TICKS(15));

    return ((uint16_t)data[1]) + (((uint16_t)data[0]) << 8);
}

void shtc3_refresh() {
    uint8_t data[6];

    ESP_LOGI(TAG, "Write attempt...");
    {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        /* Begin transmission */
        i2c_master_write_byte(cmd, (SHTC3_ADDRESS << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, (SHTC3_WAKEUP_CMD >> 8) & 0xFF, true);
        i2c_master_write_byte(cmd, SHTC3_WAKEUP_CMD & 0xFF, true);
        /* End transmission */
        i2c_master_stop(cmd);
        i2c_master_cmd_begin(SHTC3_I2C_PORT_NUM, cmd, portMAX_DELAY);
        i2c_cmd_link_delete(cmd);
    }
    vTaskDelay(pdMS_TO_TICKS(15));
    
    {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        /* Begin transmission */
        i2c_master_write_byte(cmd, (SHTC3_ADDRESS << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, (SHTC3_MEASURE_CMD >> 8) & 0xFF, true);
        i2c_master_write_byte(cmd, SHTC3_MEASURE_CMD & 0xFF, true);
        /* End transmission */
        i2c_master_stop(cmd);
        i2c_master_cmd_begin(SHTC3_I2C_PORT_NUM, cmd, portMAX_DELAY);
        i2c_cmd_link_delete(cmd);
    }
    vTaskDelay(pdMS_TO_TICKS(20));
    
    {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        /* Begin transmission */
        i2c_master_write_byte(cmd, (SHTC3_ADDRESS << 1) | I2C_MASTER_READ, true);
        i2c_master_read(cmd, data, sizeof(data), I2C_MASTER_ACK); //ACK for each byte read
        /* End transmission */
        i2c_master_stop(cmd);
        i2c_master_cmd_begin(SHTC3_I2C_PORT_NUM, cmd, portMAX_DELAY);
        i2c_cmd_link_delete(cmd);
    }
    vTaskDelay(pdMS_TO_TICKS(15));

    {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        /* Begin transmission */
        i2c_master_write_byte(cmd, (SHTC3_ADDRESS << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, (SHTC3_SLEEP_CMD >> 8) & 0xFF, true);
        i2c_master_write_byte(cmd, SHTC3_SLEEP_CMD & 0xFF, true);
        /* End transmission */
        i2c_master_stop(cmd);
        i2c_master_cmd_begin(SHTC3_I2C_PORT_NUM, cmd, portMAX_DELAY);
        i2c_cmd_link_delete(cmd);
    }
    vTaskDelay(pdMS_TO_TICKS(20));

    ESP_LOGI(TAG, "Read done! Raw: %02X %02X %02X %02X %02X %02X.", data[0], data[1], data[2], data[3], data[4], data[5]);

    int16_t raw_humidity = (data[3] << 8) | data[4];
    humidity = 100.0f * ((float)raw_humidity / 65535.0f);

    int16_t raw_temperature = (data[0] << 8) | data[1];
    temperature = -45.0f + 175.0f * ((float)raw_temperature / 65535.0f);
}

float shtc3_get_temperature() {
    return temperature;
}

float shtc3_get_humidity() {
    return humidity;
}
