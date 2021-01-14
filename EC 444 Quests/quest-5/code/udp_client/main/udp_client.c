//Sam Krasnoff, Abd El Aziz Hussein, Yanni Peng
#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/uart.h"
#include "esp_vfs_dev.h"
#include "driver/i2c.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

int flag = 1;
char check[10];
char addr_str[128];
char rx_buffer[128];
char buff[512];
static char *payload = "";
static const char *TAG = "example";

static void send_task(void *pvParameters, char *ip)
{
    while (1)
    {
    char rx_buffer[128];
    char addr_str[128];
    int addr_family = (int)pvParameters;
    int ip_protocol = 0;
    struct sockaddr_in6 dest_addr;

    struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
    dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr_ip4->sin_family = AF_INET;
    dest_addr_ip4->sin_port = htons(3030);
    ip_protocol = IPPROTO_IP;

    int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
    if (sock < 0)
    {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        return;
    }
    ESP_LOGI(TAG, "Socket created");

    int err = bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err < 0)
    {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
    }
    ESP_LOGI(TAG, "Socket bound, port %d", 3030);

    struct sockaddr_in source_addr; // Large enough for both IPv4 or IPv6
    socklen_t socklen = sizeof(source_addr);
    int len = 1;
    // Error occurred during receiving
    if (len < 0)
    {
        ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
        return;
    }
    // Data received
    else
    {
        source_addr.sin_addr.s_addr = inet_addr(ip);
        source_addr.sin_family = AF_INET;
        source_addr.sin_port = htons(3030);

        rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string...
        gets(check);
        sprintf(buff, "%d", flag);
        flag ^= 1;
        payload = buff;
        printf("sending %c to %s\n", buff[0], ip);

        int err = sendto(sock, payload, strlen(payload), 0, (struct sockaddr *)&source_addr, sizeof(source_addr));
        if (err < 0)
        {
            ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
            break;
        }
    }

    if (sock != -1)
    {
        ESP_LOGE(TAG, "Shutting down socket and restarting...");
        shutdown(sock, 0);
        close(sock);
    }
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

void app_main(){
        ESP_ERROR_CHECK( uart_driver_install(UART_NUM_0,256, 0, 0, NULL, 0) );
    esp_vfs_dev_uart_use_driver(UART_NUM_0);
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(example_connect());
    while (1)
    {
        send_task((void *)AF_INET,"192.168.43.159");
    }
    
    
}