#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "esp_system.h"
#include "esp_log.h"
#include "lwip/sockets.h"

#include "parameter.h"

extern QueueHandle_t xQueueTrans;
extern int conditionValue;

static const char *TAG = "SEND";

// UDP Send Task
void udp_trans(void *pvParameters) {
    ESP_LOGI(TAG, "Starting UDP Transmitter Task");

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(5005);  // UDP server port
    addr.sin_addr.s_addr = htonl(INADDR_BROADCAST); // Broadcast IP address

    // Create the UDP socket
    int fd = lwip_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (fd < 0) {
        ESP_LOGE(TAG, "Socket creation failed!");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "UDP socket created successfully");

    while (1) {
        // Wait to receive an integer value from the queue
        if (xQueueReceive(xQueueTrans, &conditionValue, portMAX_DELAY)) {
            ESP_LOGI(TAG, "Received integer from queue: %d", conditionValue);

            // Transmit the integer via UDP
            int ret = lwip_sendto(fd, &conditionValue, sizeof(conditionValue), 0,
                                  (struct sockaddr *)&addr, sizeof(addr));
            if (ret == sizeof(conditionValue)) {
                ESP_LOGI(TAG, "Integer %d sent successfully via UDP", conditionValue);
            } else {
                ESP_LOGE(TAG, "UDP send failed. Sent=%d bytes, Expected=%d bytes", ret, (int)sizeof(conditionValue));
            }
        } else {
            ESP_LOGE(TAG, "Failed to receive data from queue");
        }
    }

    // Close the UDP socket (though this should not be reached in a normal flow)
    if (lwip_close(fd) == 0) {
        ESP_LOGI(TAG, "UDP socket closed successfully");
    } else {
        ESP_LOGE(TAG, "Failed to close UDP socket");
    }

    vTaskDelete(NULL);
}
