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
#include "driver/timer.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define esp1ip "192.168.43.159"
#define esp2ip "192.168.43.217"
#define esp3ip "192.168.43.147"
#define TIMER_DIVIDER         16    //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // to seconds
#define TIMER_INTERVAL_SEC   (1)    // Sample test interval for the first timer
#define TEST_WITH_RELOAD      1     // Testing will be done with auto reload
#define MYID 2
#define esp3id 3
#define esp2id  1
static char *payload = "";
char buff[512];
int data_out[4];
int min_list[2];
int minval, status, start, newminval;
int check1, check2;
int counter = 0;
char addr_str[128];
char rx_buffer[128];

static const char *TAG = "example";

typedef struct {
    int flag;     // flag for enabling stuff in main code
} timer_event_t;
xQueueHandle timer_queue;

void IRAM_ATTR timer_group0_isr(void *para) {

    // Prepare basic event data, aka set flag
    timer_event_t evt;
    evt.flag = 1;

    // Clear the interrupt, Timer 0 in group 0
    TIMERG0.int_clr_timers.t0 = 1;

    // After the alarm triggers, we need to re-enable it to trigger it next time
    TIMERG0.hw_timer[TIMER_0].config.alarm_en = TIMER_ALARM_EN;

    // Send the event data back to the main program task
    xQueueSendFromISR(timer_queue, &evt, NULL);
}

static void send_task(void *pvParameters, char *ip)
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

    for (int j = 0; j < 1; j++)
    {
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

            sprintf(buff, "%d%d%d%d", start, MYID, minval, status);
            payload = buff;
            printf("sending %c%c%c%c to %s\n", buff[0], buff[1], buff[2], buff[3], ip);

            int err = sendto(sock, payload, strlen(payload), 0, (struct sockaddr *)&source_addr, sizeof(source_addr));
            if (err < 0)
            {
                ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                break;
            }
        }
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }

    if (sock != -1)
    {
        ESP_LOGE(TAG, "Shutting down socket and restarting...");
        shutdown(sock, 0);
        close(sock);
    }
}
static void receive_task(void *pvParameters)
{

    char buffer[128];

    int addr_family = (int)pvParameters;
    int ip_protocol = 0;
    struct sockaddr_in6 dest_addr;

    while (1)
    {

        struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
        dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
        dest_addr_ip4->sin_family = AF_INET;
        dest_addr_ip4->sin_port = htons(3030);
        ip_protocol = IPPROTO_IP;

        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0)
        {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created");

        int err = bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err < 0)
        {
            ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        }
        ESP_LOGI(TAG, "Socket bound, port %d", 3030);

        while (1)
        {

            ESP_LOGI(TAG, "Waiting for data");
            struct sockaddr_in source_addr; // Large enough for both IPv4 or IPv6
            socklen_t socklen = sizeof(source_addr);
            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);
            // Error occurred during receiving
            if (len < 0)
            {
                ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
                break;
            }
            // Data received
            else
            {
                // // Get the sender's ip address as string
                if (source_addr.sin_family == PF_INET)
                {
                    inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr.s_addr, addr_str, sizeof(addr_str) - 1);
                }

                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string...
                ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);
                printf("receiving %c%c%c%c\n", rx_buffer[0], rx_buffer[1], rx_buffer[2], rx_buffer[3]);
            }
        }

        if (sock != -1)
        {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}
void init()
{
    minval = MYID;
    status = 0;
    start = 1;
}
static void timer_evt_task(void *arg) {
    while (1) {
        // Create dummy structure to store structure from queue
        vTaskDelay(1000 / portTICK_RATE_MS);
        counter++;
        printf("counter is: %d\n",counter);
    }
}
void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());
    init();

    xTaskCreate(receive_task, "receive_task", 4096, (void *)AF_INET, 5, NULL);
    xTaskCreate(timer_evt_task, "timer_evt_task", 2048, NULL, 5, NULL);
    int both = 0;
    //send_task();
    for (int i = 0; i < 3; i++)
    {
        if (!strcmp(addr_str, esp2ip))
        {
            both++;
            break;
        }
        else if (!strcmp(addr_str, esp3ip))
        {
            both++;
            break;
        }
        vTaskDelay(3000 / portTICK_PERIOD_MS);
    }

    if (both < 1)
    {
         printf("I will start election\n");
        
            
            //for(int k =0; k <3 ; k++){
            send_task((void *)AF_INET, esp2ip);
            send_task((void *)AF_INET, esp3ip);

        while (1)
        {
            // }
            printf("waiting to recieve \n");
            if (!strcmp(addr_str, esp2ip))
            {
                printf("recieved from esp1 \n");
                check1 = 1;
                min_list[0] = rx_buffer[2]-48;
            }
            else if (!strcmp(addr_str, esp3ip))
            {
                printf("recieved from esp3 \n");
                check2 = 1;
                min_list[1] = rx_buffer[2]-48;
            }
            if (check1 & check2)
            {
                printf("recieved from both \n");
                break;
            }
        }

        if (min_list[0] < min_list[1])
        {
            if (min_list[0] < minval)
            {
                minval = min_list[0];
                status =0;
                send_task((void *)AF_INET, esp2ip);
                send_task((void *)AF_INET, esp3ip);
                 printf("oop, esp1 is the leader lol\n");
                 return;
            }
            else
            {
                status =1;
                for(int i = 0; i<2 ; i++){
                send_task((void *)AF_INET, esp2ip);
                send_task((void *)AF_INET, esp2ip);
                send_task((void *)AF_INET, esp3ip);
                send_task((void *)AF_INET, esp3ip);
                }
                 printf(" 2 I am the leader exiting now byeee\n");
                return;
            }
        }
        else
        {
            if (min_list[1] < minval)
            {
                minval = min_list[1];
                status =0;
                send_task((void *)AF_INET, esp2ip);
                send_task((void *)AF_INET, esp3ip);
                printf("oop, esp3 is the leader lol\n");
                return;


            }
            else
            {
                status =1;
                for(int i = 0; i<2 ; i++){
                send_task((void *)AF_INET, esp2ip);
                send_task((void *)AF_INET, esp2ip);
                send_task((void *)AF_INET, esp3ip);
                send_task((void *)AF_INET, esp3ip);
                }
                 printf(" 1 I am the leader exiting now byeee\n");
                return;
            }
        }
    }
    else
    {
        
        printf("election has already started\n");
        int recv = 0;
        int rx;
        for (int l = 0; l < 2; l++)
        {
            send_task((void *)AF_INET, addr_str);
        }
        addr_str[0] = "c";
        while (1)
        {
            printf("still here\n");
            if (!strcmp(addr_str, esp3ip))
            {
                printf("recieced from esp3\n");
                recv = 1;
                min_list[0] = rx_buffer[2];
                break;
            }
            else if (!strcmp(addr_str, esp2ip))
            {
                printf("recieced from esp2\n");
                recv = 1;
                 printf("minval  %d\n",minval);
                    printf("rx  %d\n",(rx_buffer[2]));
                rx = rx_buffer[2]-48;
                if (rx < minval)
                {
                    status = 0;
                   
                    minval = rx;
                    for (int l = 0; l < 2; l++)
                    {
                        send_task((void *)AF_INET, addr_str);
                    }
                    while (1)
                    {
                        
                        printf("waiting to recieve\n");
                        if (!strcmp(addr_str, esp3ip))
                        {
                            rx = rx_buffer[3]-48;
                            if (rx == 1)
                            {
                                printf("esp3 is the leader\n");
                                return;
                            }
                            else if ((rx_buffer[2]-48)!= MYID)
                            {
                                printf("esp2 is the leader\n");
                            }
                        }
                        else if (!strcmp(addr_str, esp2ip))
                        {
                            rx = rx_buffer[3]-48;
                            if (rx == 1)
                            {
                                printf("esp2 is the leader\n");
                                return;
                            }
                            else if ((rx_buffer[2]-48)!= MYID){
                                printf("esp3 is the leader\n");
                            }
                        }
                    }
                }

                else
                {
                    status = 1;
                    printf("I am the mf leader min val is %d\n", minval);
                    for (int l = 0; l < 2; l++)
                    {
                        send_task((void *)AF_INET, addr_str);
                    }
                    return;
                }
                break;
            }
        }
    }

}
