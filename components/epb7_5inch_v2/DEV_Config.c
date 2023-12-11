/*****************************************************************************
* | File      	:   DEV_Config.c
* | Author      :   Waveshare team
* | Function    :   Hardware underlying interface
* | Info        :
*                Used to shield the underlying layers of each master
*                and enhance portability
*----------------
* |	This version:   V2.0
* | Date        :   2018-10-30
* | Info        :
# ******************************************************************************
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documnetation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to  whom the Software is
# furished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS OR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#
******************************************************************************/
#include "DEV_Config.h"
#include <driver/spi_master.h>
#include <string.h>

spi_device_handle_t spi_handle;

#define DEV_SPI SPI2_HOST

int DEV_Module_Setup(void) {
    /* Control GPIOs */
    gpio_reset_pin(EPD_RST_PIN);
    gpio_set_direction(EPD_RST_PIN, GPIO_MODE_OUTPUT);
    gpio_reset_pin(EPD_DC_PIN);
    gpio_set_direction(EPD_DC_PIN, GPIO_MODE_OUTPUT);
    gpio_reset_pin(EPD_PWR_PIN);
    gpio_set_direction(EPD_PWR_PIN, GPIO_MODE_OUTPUT);

    gpio_reset_pin(EPD_CS_PIN);
    gpio_set_direction(EPD_CS_PIN, GPIO_MODE_OUTPUT);

    gpio_reset_pin(EPD_BUSY_PIN);
    gpio_set_direction(EPD_BUSY_PIN, GPIO_MODE_INPUT);
    
    /* SPI */
    esp_err_t ret;

    // Configure SPI bus
    spi_bus_config_t bus_config = {
        .miso_io_num = -1, // Set to -1 if not used
        .mosi_io_num = EPD_DIN_PIN, // MOSI pin
        .sclk_io_num = EPD_CLK_PIN, // SCLK pin
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0, // 0 means default, otherwise set the maximum transaction size
    };

    // Initialize the SPI bus
    ret = spi_bus_initialize(DEV_SPI, &bus_config, 0);
    ESP_ERROR_CHECK(ret);

    // Configure the SPI device
    spi_device_interface_config_t dev_config = {
        .clock_speed_hz = 10 * 1000 * 1000,  // Clock speed
        .mode = 0,                           // SPI mode 0
        .spics_io_num = -1,          // CS pin
        .queue_size = 1,                     // Maximum number of transactions in the SPI hardware queue
    };

    ret = spi_bus_add_device(DEV_SPI, &dev_config, &spi_handle);
    ESP_ERROR_CHECK(ret);
    return 0;
}

void DEV_SPI_WriteByte(UBYTE value)
{
    // HAL_SPI_Transmit(&spi_handle, &value, 1, 1000);
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length = 8;                   //Command is 8 bits
    t.tx_buffer = &value;             //The data is the cmd itself
    // t.user = (void*)0;              //D/C needs to be set to 0
    // if (keep_cs_active) {
    //     t.flags = SPI_TRANS_CS_KEEP_ACTIVE;   //Keep CS active after data transfer
    // }
    ret = spi_device_polling_transmit(spi_handle, &t); //Transmit!
    assert(ret == ESP_OK);          //Should have had no issues.
}

// int DEV_Module_Init(void)
// {
//     DEV_Digital_Write(EPD_DC_PIN, 0);
//     spi_device_acquire_bus(spi_handle, portMAX_DELAY);
//     // DEV_Digital_Write(EPD_CS_PIN, 0);
// 	DEV_Digital_Write(EPD_PWR_PIN, 1);
//     DEV_Digital_Write(EPD_RST_PIN, 1);
// 		return 0;
// }

// void DEV_Module_Exit(void)
// {
//     DEV_Digital_Write(EPD_DC_PIN, 0);
//     spi_device_release_bus(spi_handle);
//     // DEV_Digital_Write(EPD_CS_PIN, 0);

//     //close 5V
// 	DEV_Digital_Write(EPD_PWR_PIN, 0);
//     DEV_Digital_Write(EPD_RST_PIN, 0);
// }
