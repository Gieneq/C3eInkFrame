#include "gframe.h"

#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <freertos/queue.h>

#include <esp_log.h>

#include "gcaptive.h"
#include "spi_epd_7in5v2.h"
#include "bmp_loader.h"
#include "gstorage.h"
#include <sys/param.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include <dirent.h>

extern const char bmp_arrowdown_start[] asm("_binary_arrowdown_bmp_start");
extern const char bmp_arrowdown_end[] asm("_binary_arrowdown_bmp_end");

extern const char bmp_intro_start[] asm("_binary_intro_bmp_start");
extern const char bmp_intro_end[] asm("_binary_intro_bmp_end");

extern const char bmp_oko_start[] asm("_binary_oko_bmp_start");
extern const char bmp_oko_end[] asm("_binary_oko_bmp_end");

#define CHUNK_BUFFER_SIZE  (1200) //800 / 8 * 12
static char chunk_buff[CHUNK_BUFFER_SIZE];

#define GFRAME_TASK_STACK_SIZE  (2048 * 2)
#define GFRAME_TASK_PRIORITY      8
TaskHandle_t gframe_task_handle;
static const char* TAG = "GFrame";

#define CLICK_EVENTS_QUEUE_SIZE (10)
QueueHandle_t click_event_queue;

static gframe_sm_state_t gframe_sm_state = GFRAME_SM_STATE_PREIDLE;

static const char* gframe_sm_state_str(const gframe_sm_state_t state) {
    switch (state) {
        case GFRAME_SM_STATE_PREIDLE:
            return "PREIDLE";
        case GFRAME_SM_STATE_IDLE:
            return "IDLE";
        case GFRAME_SM_STATE_CAPTIVE_PORTAL_OPENING:
            return "CAPTIVE_PORTAL_OPENING";
        case GFRAME_SM_STATE_CAPTIVE_PORTAL_OPENED:
            return "CAPTIVE_PORTAL_OPENED";
        case GFRAME_SM_STATE_CAPTIVE_PORTAL_CLOSING:
            return "CAPTIVE_PORTAL_CLOSING";
        default:
            return "UNKNOWN";
    }
}

#define ALIGN_TO_32(_value) (32 * ((_value / 32) + ((_value % 32) > 0 ? 1 : 0)))
static void gframe_process_statemachine() {
    gframe_sm_state_t next_sm_state = gframe_sm_state;

    //todo maybe add some forced state change or feedback notificatgions

    if (gframe_sm_state == GFRAME_SM_STATE_PREIDLE) {
        if (epd7in5v2_is_refreshed() == true) {
            if (epd7in5v2_start_draw(portMAX_DELAY) == true) {
                epd7in5v2_fill_color(true);
                // epd7in5v2_draw_text(300, 240, 24, "Epapierowa ramka: witam!");

                const char* filename = "image.bmp";
                const char* filepath = "/data/image.bmp";
                FILE *fd = NULL;
                struct stat file_stat;
                
                const bool has_stored_image_bmp = (stat(filepath, &file_stat) == -1) ? false : true;
                if (has_stored_image_bmp == false) {
                    epd7in5v2_draw_image(0, 0, bmp_oko_start, bmp_oko_end);
                } 
                else {
                    fd = fopen(filepath, "r");
                    size_t chunk_size;
                    if (fd) {
                        //file_stat.st_size
                        chunk_size = fread(chunk_buff, 1, 244, fd); //drop
                        for(int i = 0; i< 62; ++i) {
                            printf("0x%02X, ", chunk_buff[i]);
                            if((i == 15) || (i == 15+16) || (i == 15+32) || (i == 15+32*3) || (i == 15+32*4) || (i == 15+32*5)) {
                                printf("\n");
                            }
                        }
                        printf("\n");
                        /* 
                         * Chunking 800 x 480
                         * 800 * 10 -> Chunk
                        */

                        int chunk_idx = 0;
                        uint32_t pixel_index = 0;
                        uint32_t iy = 0;          /* From 0 to bmp_reader.image.height */
                        uint32_t ix = 0;          /* From 0 to bmp_reader.image.width  */

                        const uint32_t width = 800;
                        const uint32_t aligned_width = ALIGN_TO_32(width);
                        const uint32_t height = 480;

                        do {
                            /* Read file in chunks into the scratch buffer */
                            chunk_size = fread(chunk_buff, 1, CHUNK_BUFFER_SIZE, fd);
                            if (chunk_size > 0) {
                                /* Has something in reader chunk */
                                for (uint32_t chunk_byte_idx=0; chunk_byte_idx<chunk_size; ++chunk_byte_idx) {
                                    const uint32_t chunk_byte = chunk_buff[chunk_byte_idx];
                                    for (int32_t byte_idx=7; byte_idx>=0; --byte_idx) {

                                        iy = height - (pixel_index / aligned_width) - 1;
                                        ix = pixel_index % aligned_width;

                                        if (ix < width) {
                                            /* Here valid image data */
                                            const bool is_white = (chunk_byte & (1<<byte_idx)) > 0 ? true : false;
                                            epd7in5v2_set_pixel(0 + ix, 0 + iy, is_white);
                                        }

                                        ++pixel_index;
                                    }
                                }
                            }

                            /* Keep looping till the whole file is sent */
                        } while (chunk_size != 0);
                    }
                }

                epd7in5v2_stop_draw();
            }
            epd7in5v2_attempt_refresh(portMAX_DELAY);

            vTaskDelay(pdMS_TO_TICKS(500));
            while (epd7in5v2_is_refreshed() == false) {
                vTaskDelay(pdMS_TO_TICKS(100));
            }

            next_sm_state = GFRAME_SM_STATE_IDLE;
        }
    }

    else if (gframe_sm_state == GFRAME_SM_STATE_IDLE) {
        click_event_t received_click_event;
        if (xQueueReceive(click_event_queue, &received_click_event, portMAX_DELAY) == pdTRUE) {
            next_sm_state = GFRAME_SM_STATE_CAPTIVE_PORTAL_OPENING;
        }
    }

    else if (gframe_sm_state == GFRAME_SM_STATE_CAPTIVE_PORTAL_OPENING) {
        if (true) {
            gcaptive_create();
            gcaptive_start();

            vTaskDelay(pdMS_TO_TICKS(500));
            while (gcaptive_is_started() == false) {
                vTaskDelay(pdMS_TO_TICKS(100));
            }

            if (epd7in5v2_start_draw(portMAX_DELAY) == true) {
                epd7in5v2_fill_color(true);
                epd7in5v2_draw_text(120, 120, 24, "Settings: portal");
                epd7in5v2_draw_text(140, 120 + 30     , 16, "IP:");    epd7in5v2_draw_text(230, 120 + 30     , 16, gcaptive_get_ipv4_str());
                epd7in5v2_draw_text(140, 120 + 30 + 20, 16, "SSID:");  epd7in5v2_draw_text(230, 120 + 30 + 20, 16, gcaptive_get_ssid());
                epd7in5v2_draw_text(140, 120 + 30 + 40, 16, "PSSWD:"); epd7in5v2_draw_text(230, 120 + 30 + 40, 16, gcaptive_get_psswd());
                epd7in5v2_stop_draw();
            }
            epd7in5v2_attempt_refresh(portMAX_DELAY);

            vTaskDelay(pdMS_TO_TICKS(500));
            while (epd7in5v2_is_refreshed() == false) {
                vTaskDelay(pdMS_TO_TICKS(100));
            }

            next_sm_state = GFRAME_SM_STATE_CAPTIVE_PORTAL_OPENED;
        }
    }

    else if (gframe_sm_state == GFRAME_SM_STATE_CAPTIVE_PORTAL_OPENED) {
        click_event_t received_click_event;

        if (gcaptive_has_connected_devices_changed() == true) {
            uint8_t devices_count = 0;
            gcaptive_get_connected_devices(&devices_count);

            /* Refresh display */
            if (epd7in5v2_start_draw(portMAX_DELAY) == true) {
                epd7in5v2_fill_color(true);
                epd7in5v2_draw_text(120, 120, 24, "Settings: portal");
                epd7in5v2_draw_text(140, 120 + 30     , 16, "IP:");    epd7in5v2_draw_text(330, 120 + 30     , 16, gcaptive_get_ipv4_str());
                epd7in5v2_draw_text(140, 120 + 30 + 20, 16, "SSID:");  epd7in5v2_draw_text(330, 120 + 30 + 20, 16, gcaptive_get_ssid());
                epd7in5v2_draw_text(140, 120 + 30 + 40, 16, "PSSWD:"); epd7in5v2_draw_text(330, 120 + 30 + 40, 16, gcaptive_get_psswd());

                char buff[16] = {0};
                snprintf(buff, 16, "%d", devices_count);
                epd7in5v2_draw_text(140, 120 + 30 + 60, 16, "Devices:"); epd7in5v2_draw_text(330, 120 + 30 + 60, 16, buff);
                epd7in5v2_stop_draw();
            }
            epd7in5v2_attempt_refresh(portMAX_DELAY);

            vTaskDelay(pdMS_TO_TICKS(500));
            while (epd7in5v2_is_refreshed() == false) {
                vTaskDelay(pdMS_TO_TICKS(100));
            }
        }

        if (gcaptive_is_running() == false) {
            /* For example image was uploaded - restart from PREIDLE */
            next_sm_state = GFRAME_SM_STATE_PREIDLE;
        }

        /* Poll for button event */
        if (xQueueReceive(click_event_queue, &received_click_event, 0) == pdTRUE) {
            // next_sm_state = GFRAME_SM_STATE_CAPTIVE_PORTAL_CLOSING;
            ESP_LOGW(TAG, "Not implemented!");
        }
    }

    else if (gframe_sm_state == GFRAME_SM_STATE_CAPTIVE_PORTAL_CLOSING) {
        if (true) {
            // gcaptive_stop();
            next_sm_state = GFRAME_SM_STATE_IDLE;
        }
    }

    /* Transition */
    if (next_sm_state != gframe_sm_state) {
        ESP_LOGI(TAG, "Transition from \'%s\' to \'%s\'", 
            gframe_sm_state_str(gframe_sm_state), 
            gframe_sm_state_str(next_sm_state)
        );
        gframe_sm_state = next_sm_state;
    }
} 

static void gframe_task(void* params) {
    ESP_LOGI(TAG, "GFrame task running!");

    while(1) {
        gframe_process_statemachine();
        vTaskDelay(1);
    }
}

esp_err_t gframe_create() {
    click_event_queue = xQueueCreate(CLICK_EVENTS_QUEUE_SIZE, sizeof(click_event_t));
    if (click_event_queue == NULL) {
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t gframe_start() {
    xTaskCreate(gframe_task, TAG, GFRAME_TASK_STACK_SIZE, NULL, GFRAME_TASK_PRIORITY, &gframe_task_handle);
    if (gframe_task_handle == NULL) {
        return ESP_FAIL;
    }

    return ESP_OK;
}

void gframe_enque_shortclick() {
    const click_event_t event = {.type = CLICK_EVENT_SHORT};
    xQueueSend(click_event_queue, &event, portMAX_DELAY);
}