#include "gcaptive.h"

#include <stdio.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <sys/param.h>
#include <esp_log.h>
#include <esp_event.h>
#include <esp_mac.h>
#include <esp_netif.h>
#include <lwip/inet.h>
#include <esp_http_server.h>
#include <esp_wifi.h>
#include <sdkconfig.h>

// #include "dns_server.h"
#include <sys/param.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"

#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "lwip/inet.h"

#include "esp_http_server.h"
// #include "dns_server.h"
#include "gstorage.h"

#define EXAMPLE_ESP_WIFI_SSID "GCaptiveSSID"
#define EXAMPLE_ESP_WIFI_PASS "raspberry"
#define EXAMPLE_MAX_STA_CONN 2
#define EXAMPLE_MAX_SOCKETS  3

static httpd_handle_t server = NULL;
static bool server_is_running = false;

extern const char esp32c3_png_start[] asm("_binary_esp32c3_png_start");
extern const char esp32c3_png_end[] asm("_binary_esp32c3_png_end");

extern const char favicon_ico_start[] asm("_binary_favicon_ico_start");
extern const char favicon_ico_end[] asm("_binary_favicon_ico_end");

extern const char gframe_icon_png_start[] asm("_binary_gframe_icon_png_start");
extern const char gframe_icon_png_end[] asm("_binary_gframe_icon_png_end");

extern const char index_html_start[] asm("_binary_index_html_start");
extern const char index_html_end[] asm("_binary_index_html_end");

extern const char scripts_js_start[] asm("_binary_scripts_js_start");
extern const char scripts_js_end[] asm("_binary_scripts_js_end");

extern const char styles_css_start[] asm("_binary_styles_css_start");
extern const char styles_css_end[] asm("_binary_styles_css_end");

#define UPLOAD_BUFFER_SIZE  (1024 * 8)
static char upload_bmp_buff[UPLOAD_BUFFER_SIZE];

#define GCAPTIVE_SSID_SIZE (64)
#define GCAPTIVE_PSSWD_SIZE (64)
#define GCAPTIVE_IPV4_SIZE (16)
static struct {
    char ssid[GCAPTIVE_SSID_SIZE];
    char psswd[GCAPTIVE_PSSWD_SIZE];
    char ipv4[GCAPTIVE_IPV4_SIZE];
    bool is_started;
} gcaptive_data;

static connected_device_data_t connected_devices_data[EXAMPLE_MAX_STA_CONN];
static uint8_t connected_devices_count = 0;

static const char *TAG = "example";

static bool connected_devices_has_changed = false;

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)event_data;
        ESP_LOGI(TAG, "station " MACSTR " join, AID=%d",
                 MAC2STR(event->mac), event->aid);

        if (connected_devices_count < EXAMPLE_MAX_STA_CONN) {
            connected_devices_count += 1;
        }
        connected_devices_has_changed = true;
    } 
    
    else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *)event_data;
        ESP_LOGI(TAG, "station " MACSTR " leave, AID=%d",
                 MAC2STR(event->mac), event->aid);
        if (connected_devices_count > 0) {
            connected_devices_count -= 1;
        }
        connected_devices_has_changed = true;
    }
}

static void wifi_init_softap(void)
{
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .ssid_len = strlen(EXAMPLE_ESP_WIFI_SSID),
            .password = EXAMPLE_ESP_WIFI_PASS,
            .max_connection = EXAMPLE_MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };
    if (strlen(EXAMPLE_ESP_WIFI_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    esp_netif_ip_info_t ip_info;
    esp_netif_get_ip_info(esp_netif_get_handle_from_ifkey("WIFI_AP_DEF"), &ip_info);

    char ip_addr[16];
    inet_ntoa_r(ip_info.ip.addr, ip_addr, 16);
    ESP_LOGI(TAG, "Set up softAP with IP: %s", ip_addr);

    ESP_LOGI(TAG, "wifi_init_softap finished. SSID:'%s' password:'%s'",
             EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);

    strcpy(gcaptive_data.ipv4, ip_addr);
    strcpy(gcaptive_data.ssid, EXAMPLE_ESP_WIFI_SSID);
    strcpy(gcaptive_data.psswd, EXAMPLE_ESP_WIFI_PASS);
    gcaptive_data.is_started = true;
}


/****************************************
 *                                      *
 *              ENDPOINTS               *
 *                                      *
 * **************************************/

esp_err_t http_404_error_handler(httpd_req_t *req, httpd_err_code_t err) {
    // Set status
    httpd_resp_set_status(req, "302 Temporary Redirect");
    // Redirect to the "/" root directory
    httpd_resp_set_hdr(req, "Location", "/");
    // iOS requires content in the response to detect a captive portal, simply redirecting is not sufficient.
    httpd_resp_send(req, "Redirect to the captive portal", HTTPD_RESP_USE_STRLEN);

    ESP_LOGI(TAG, "Redirecting to root");
    return ESP_OK;
}

/* GET esp32c3.png */
static esp_err_t esp32c3_png_get_handler(httpd_req_t *req) {
    const uint32_t esp32c3_png_len = esp32c3_png_end - esp32c3_png_start;
    httpd_resp_set_type(req, "image/png");
    httpd_resp_send(req, esp32c3_png_start, esp32c3_png_len);
    ESP_LOGI(TAG, "Serve esp32c3.png");
    return ESP_OK;
}

static const httpd_uri_t esp32c3_png = {
    .uri = "/esp32c3.png",
    .method = HTTP_GET,
    .handler = esp32c3_png_get_handler
};

/* GET favicon.ico */
static esp_err_t favicon_ico_get_handler(httpd_req_t *req) {
    const uint32_t favicon_ico_len = favicon_ico_end - favicon_ico_start;
    httpd_resp_set_type(req, "image/x-icon");
    httpd_resp_send(req, favicon_ico_start, favicon_ico_len);
    ESP_LOGI(TAG, "Serve favicon.ico");
    return ESP_OK;
}

static const httpd_uri_t favicon_ico = {
    .uri = "/favicon.ico",
    .method = HTTP_GET,
    .handler = favicon_ico_get_handler
};

/* GET gframe_icon.png */
static esp_err_t gframe_icon_png_get_handler(httpd_req_t *req) {
    const uint32_t gframe_icon_png_len = gframe_icon_png_end - gframe_icon_png_start;
    httpd_resp_set_type(req, "image/png");
    httpd_resp_send(req, gframe_icon_png_start, gframe_icon_png_len);
    ESP_LOGI(TAG, "Serve gframe_icon.png");
    return ESP_OK;
}

static const httpd_uri_t gframe_icon_png = {
    .uri = "/gframe_icon.png",
    .method = HTTP_GET,
    .handler = gframe_icon_png_get_handler
};

/* GET index.html */
static esp_err_t index_html_get_handler(httpd_req_t *req) {
    const uint32_t index_html_len = index_html_end - index_html_start;
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, index_html_start, index_html_len);
    ESP_LOGI(TAG, "Serve index.html");
    return ESP_OK;
}

static const httpd_uri_t index_html = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = index_html_get_handler
};

/* GET scripts.js */
static esp_err_t scripts_js_get_handler(httpd_req_t *req) {
    const uint32_t scripts_js_len = scripts_js_end - scripts_js_start;
    httpd_resp_set_type(req, "text/javascript");
    httpd_resp_send(req, scripts_js_start, scripts_js_len);
    ESP_LOGI(TAG, "Serve scripts.js");
    return ESP_OK;
}

static const httpd_uri_t scripts_js = {
    .uri = "/scripts.js",
    .method = HTTP_GET,
    .handler = scripts_js_get_handler
};

/* GET styles.css */
static esp_err_t styles_css_get_handler(httpd_req_t *req) {
    const uint32_t styles_css_len = styles_css_end - styles_css_start;
    httpd_resp_set_type(req, "text/css");
    httpd_resp_send(req, styles_css_start, styles_css_len);
    ESP_LOGI(TAG, "Serve styles.css");
    return ESP_OK;
}

static const httpd_uri_t styles_css = {
    .uri = "/styles.css",
    .method = HTTP_GET,
    .handler = styles_css_get_handler
};



/* POST upload_bmp */

static esp_err_t upload_bmp_post_handler(httpd_req_t *req)
{    
    int ret, remaining = req->content_len;
    ESP_LOGI(TAG, "Receiving %d B of BMP image", req->content_len);

    while (remaining > 0) {
        /* Read the data for the request */
        if ((ret = httpd_req_recv(req, upload_bmp_buff,
                        MIN(remaining, sizeof(upload_bmp_buff)))) <= 0) {
            if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
                /* Retry receiving if timeout occurred */
                continue;
            }
            return ESP_FAIL;
        }

        /* Send back the same data */
        // httpd_resp_send_chunk(req, upload_bmp_buff, ret);
        remaining -= ret;
    }
    ESP_LOGI(TAG, "Receiving DONE!");

    /* Respond because needed in promise */
    const char RESPONSE_OK[] = "{\"status\":200}\n";
    httpd_resp_send(req, RESPONSE_OK, sizeof(RESPONSE_OK));
    server_is_running = false;
    return ESP_OK;
}

static const httpd_uri_t upload_bmp = {
    .uri       = "/upload_bmp",
    .method    = HTTP_POST,
    .handler   = upload_bmp_post_handler,
    .user_ctx  = NULL
};


static httpd_handle_t start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_open_sockets = EXAMPLE_MAX_SOCKETS;
    config.lru_purge_enable = true;

    // Start the httpd server
    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        // Set URI handlers
        ESP_LOGI(TAG, "Registering URI handlers");
        /* Find in endpoints.h */
        httpd_register_uri_handler(server, &upload_bmp);
        httpd_register_uri_handler(server, &esp32c3_png);
        httpd_register_uri_handler(server, &favicon_ico);
        httpd_register_uri_handler(server, &gframe_icon_png);
        httpd_register_uri_handler(server, &index_html);
        httpd_register_uri_handler(server, &scripts_js);
        httpd_register_uri_handler(server, &styles_css);

        httpd_register_err_handler(server, HTTPD_404_NOT_FOUND, http_404_error_handler);
    }
    return server;
}

static void gcaptive_task(void* params) {
    // esp_netif_create_default_wifi_ap();
    // wifi_init_softap();     
    // start_webserver();

    // dns_server_config_t config = DNS_SERVER_CONFIG_SINGLE("*" /* all A queries */, "WIFI_AP_DEF" /* softAP netif ID */);
    // start_dns_server(&config);

        /*
        Turn of warnings from HTTP server as redirecting traffic will yield
        lots of invalid requests
    */
    esp_log_level_set("httpd_uri", ESP_LOG_ERROR);
    esp_log_level_set("httpd_txrx", ESP_LOG_ERROR);
    esp_log_level_set("httpd_parse", ESP_LOG_ERROR);


    // Initialize networking stack
    // ESP_ERROR_CHECK(esp_netif_init());

    // Create default event loop needed by the  main app
    // ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Initialize NVS needed by Wi-Fi
    // ESP_ERROR_CHECK(nvs_flash_init());

    // Initialize Wi-Fi including netif with default config
    esp_netif_create_default_wifi_ap();

    // Initialise ESP32 in SoftAP mode
    wifi_init_softap();

    // Start the server for the first time
    start_webserver();

    // Start the DNS server that will redirect all queries to the softAP IP
    // dns_server_config_t config = DNS_SERVER_CONFIG_SINGLE("*" /* all A queries */, "WIFI_AP_DEF" /* softAP netif ID */);
    // start_dns_server(&config);

    while(1) {
        vTaskDelay(10);
        if(server_is_running == false) {
            vTaskDelay(pdMS_TO_TICKS(500));
            break;
        }
    }

    /* Stop server */
    ESP_LOGI(TAG, "Stopping the server...");
    httpd_stop(server);
    vTaskDelete(NULL);
}


esp_err_t gcaptive_create() {
    memset(gcaptive_data.ssid, '\0', GCAPTIVE_SSID_SIZE);
    memset(gcaptive_data.psswd, '\0', GCAPTIVE_PSSWD_SIZE);
    memset(gcaptive_data.ipv4, '\0', GCAPTIVE_IPV4_SIZE);
    gcaptive_data.is_started = false;

    connected_devices_has_changed = false;
    memset(connected_devices_data, 0, sizeof(connected_device_data_t) * EXAMPLE_MAX_STA_CONN);
    connected_devices_count = 0;

    return ESP_OK;
}

esp_err_t gcaptive_start() {
    xTaskCreate(gcaptive_task, TAG, 2 * 2048, NULL, 6, NULL);
    server_is_running = true;
    // if(gcaptive.handle == NULL) {
    //     return ESP_FAIL;
    // }
    return ESP_OK;
}

esp_err_t gcaptive_stop() {
    server_is_running = false;
    return ESP_OK;
}

bool gcaptive_is_started() {
    return gcaptive_data.is_started;
}

const char* gcaptive_get_ssid() {
    return gcaptive_data.ssid;
}

const char* gcaptive_get_psswd() {
    return gcaptive_data.psswd;
}

const char* gcaptive_get_ipv4_str() {
    return gcaptive_data.ipv4;
}

bool gcaptive_has_connected_devices_changed() {
    if (connected_devices_has_changed == true) {
        connected_devices_has_changed = false; // mark
        return true;
    }
    return false;
}

void gcaptive_get_connected_devices(uint8_t* devices_count) {
    *devices_count = connected_devices_count;
}

bool gcaptive_is_running() {
    return server_is_running;
}