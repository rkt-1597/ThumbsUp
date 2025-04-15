#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"
#include "driver/ledc.h"
#include "esp_err.h"


static const char *TAG = "ESP32_AP";

// Wi-Fi Access Point configuration
#define WIFI_SSID "ESP32_AP"
#define WIFI_PASS "password123"
#define MAX_CONN 1               // Allow only 1 client
#define UDP_PORT 5005            // Port to listen for UDP packets

void wifi_init_ap(void) {
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = WIFI_SSID,
            .ssid_len = strlen(WIFI_SSID),
            .password = WIFI_PASS,
            .max_connection = MAX_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK,
        },
    };

    // If no password is set, use open authentication
    if (strlen(WIFI_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Wi-Fi Access Point started. SSID: %s, Password: %s", WIFI_SSID, WIFI_PASS);
}

// UDP Server Task to Receive Angles
void udp_server_task(void *pvParameters) {
    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY); // Listen on all interfaces
    server_addr.sin_port = htons(UDP_PORT);

    // Create a socket
    int server_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (server_sock < 0) {
        ESP_LOGE(TAG, "Socket creation failed!");
        vTaskDelete(NULL);
        return;
    }

    // Bind the socket
    if (bind(server_sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        ESP_LOGE(TAG, "Socket bind failed!");
        close(server_sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "UDP server listening on port %d", UDP_PORT);

    char rx_buffer[128];
    struct sockaddr_in source_addr;
    socklen_t source_addr_len = sizeof(source_addr);

    while (1) {
        // Receive UDP packet (blocking call)
        int len = recvfrom(server_sock, rx_buffer, sizeof(rx_buffer) - 1, 0,
                           (struct sockaddr *)&source_addr, &source_addr_len);
        if (len < 0) {
            ESP_LOGE(TAG, "UDP receive failed!");
            continue;
        }

        rx_buffer[len] = 0; // Null-terminate received string
        ESP_LOGI(TAG, "Received message: %s", rx_buffer);
    }

    // Close the socket
    close(server_sock);
    vTaskDelete(NULL);
}

// Main application entry point
void app_main(void) {
    ESP_LOGI(TAG, "Starting ESP32 Access Point with UDP receiver");

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize Wi-Fi in Access Point mode
    wifi_init_ap();

    // Start the UDP server task
    xTaskCreate(udp_server_task, "udp_server", 4096, NULL, 5, NULL);
}