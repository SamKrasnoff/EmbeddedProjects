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
#include <netdb.h> //hostent
#include <arpa/inet.h>

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/rmt.h"
#include "soc/rmt_reg.h"
#include "driver/uart.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"

#define esp1ip "192.168.1.67"
#define esp2ip "192.168.1.72"
#define esp3ip "192.168.1.73"
#define piaddress "pi.yanni.dev"
#define MYID 2
#define esp1id 2
#define esp2id 1
static char *payload = "";
char buff[512];
int min_list[2];
int minval, status, start, newminval;
int check1, check2;
int vote = 2;
char addr_str[128];
char rx_buffer[128];
char ServerIP[100];

char * myip = esp3ip;
int lenout1 = 2;
static const char *TAG = "example";
static const char *TAG_SYSTEM = "system"; // For debug logs

int voterin;
int votein;
char *leaderip;

SemaphoreHandle_t mux = NULL;
static xQueueHandle gpio_evt_queue = NULL;
static xQueueHandle timer_queue;

// RMT definitions
#define RMT_TX_CHANNEL 1                                 // RMT channel for transmitter
#define RMT_TX_GPIO_NUM 25                               // GPIO number for transmitter signal -- A1
#define RMT_CLK_DIV 100                                  // RMT counter clock divider
#define RMT_TICK_10_US (80000000 / RMT_CLK_DIV / 100000) // RMT counter value for 10 us.(Source clock is APB clock)
#define rmt_item32_tIMEOUT_US 9500                       // RMT receiver timeout value(us)

// UART definitions
#define UART_TX_GPIO_NUM 26 // A0
#define UART_RX_GPIO_NUM 34 // A2
#define BUF_SIZE (1024)

// Hardware interrupt definitions
#define GPIO_INPUT_IO_1 4
#define ESP_INTR_FLAG_DEFAULT 0
#define GPIO_INPUT_PIN_SEL 1ULL << GPIO_INPUT_IO_1

// LED Output pins definitions
#define GREENPIN 32
#define YELLOWLED 15
#define REDPIN 14
#define ONBOARD 13

#define TIMER_DIVIDER 16                             //  Hardware timer clock divider
#define TIMER_SCALE (TIMER_BASE_CLK / TIMER_DIVIDER) // to seconds
#define TIMER_INTERVAL_2_SEC (2)
#define TIMER_INTERVAL_10_SEC (10)
#define TEST_WITH_RELOAD 1 // Testing will be done with auto reload

// Default ID/color
#define ID 3
#define COLOR 'R'

// Variables for my ID, minVal and status plus string fragments
//char start = 0x1B;
char myID = (char)ID;
char myColor = (char)COLOR;
int len_out = 4;

int hostname_to_ip(char *hostname, char *ip)
{
    struct hostent *he;
    struct in_addr **addr_list;
    int i;

    if ((he = gethostbyname(hostname)) == NULL)
    {
        // get the host info
        perror("gethostbyname");
        return 1;
    }

    addr_list = (struct in_addr **)he->h_addr_list;

    for (i = 0; addr_list[i] != NULL; i++)
    {
        //Return the first one;
        strcpy(ip, inet_ntoa(*addr_list[i]));
        return 0;
    }

    return 1;
}

static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t)arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}
static void rmt_tx_init()
{
    rmt_config_t rmt_tx;
    rmt_tx.channel = RMT_TX_CHANNEL;
    rmt_tx.gpio_num = RMT_TX_GPIO_NUM;
    rmt_tx.mem_block_num = 1;
    rmt_tx.clk_div = RMT_CLK_DIV;
    rmt_tx.tx_config.loop_en = false;
    rmt_tx.tx_config.carrier_duty_percent = 50;
    // Carrier Frequency of the IR receiver
    rmt_tx.tx_config.carrier_freq_hz = 38000;
    rmt_tx.tx_config.carrier_level = 1;
    rmt_tx.tx_config.carrier_en = 1;
    // Never idle -> aka ontinuous TX of 38kHz pulses
    rmt_tx.tx_config.idle_level = 1;
    rmt_tx.tx_config.idle_output_en = true;
    rmt_tx.rmt_mode = 0;
    rmt_config(&rmt_tx);
    rmt_driver_install(rmt_tx.channel, 0, 0);
}

// Configure UART
static void uart_init()
{
    // Basic configs
    uart_config_t uart_config = {
        .baud_rate = 1200, // Slow BAUD rate
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
    uart_param_config(UART_NUM_1, &uart_config);

    // Set UART pins using UART0 default pins
    uart_set_pin(UART_NUM_1, UART_TX_GPIO_NUM, UART_RX_GPIO_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // Reverse receive logic line
    uart_set_line_inverse(UART_NUM_1, UART_SIGNAL_RXD_INV);

    // Install UART driver
    uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0);
}

static void button_init()
{
    gpio_config_t io_conf;
    //interrupt of rising edge
    io_conf.intr_type = GPIO_PIN_INTR_POSEDGE;
    //bit mask of the pins, use GPIO4 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
    gpio_intr_enable(GPIO_INPUT_IO_1);
    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL3);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_1, gpio_isr_handler, (void *)GPIO_INPUT_IO_1);
    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    //start gpio task
}

void send_task1()
{
    for (int i = 0; i < 10; i++)
    {
        int check;
        char *data_out = (char *)malloc(lenout1);
        xSemaphoreTake(mux, portMAX_DELAY);
         data_out[0] = (char)MYID;
        
         data_out[1] = (vote);
        //sprintf(data_out,"%d%d",MYID,vote);

        printf("vote is %d\n",vote);
        printf("data_out[1] is %c\n",data_out[1]);
        check = uart_write_bytes(UART_NUM_1, data_out, lenout1);
        // printf("Sent payload, %d\n", check);
        xSemaphoreGive(mux);

        vTaskDelay(5 / portTICK_PERIOD_MS);
    }
}

// Button task -- rotate through myIDs
void button_task()
{
    uint32_t io_num;
    while (1)
    {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY))
        {
            xSemaphoreTake(mux, portMAX_DELAY);
            if (myID == 3)
            {
                myID = 1;
            }
            else
            {
                myID++;
            }

            xSemaphoreGive(mux);
            send_task1();
            printf("Button pressed.\n");
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
// Receives task -- looks for Start byte then stores received values

static void send_server(void *pvParameters, char *ip)
{
    char rx_buffer[128];
    char addr_str[128];
    int addr_family = (int)pvParameters;
    int ip_protocol = 0;
    struct sockaddr_in6 dest_addr;

    struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
    dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr_ip4->sin_family = AF_INET;
    dest_addr_ip4->sin_port = htons(3333);
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
    ESP_LOGI(TAG, "Socket bound, port %d", 3333);

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
            source_addr.sin_port = htons(3333);

            rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string...

            // sprintf(buff, "%d%d%d%d", voterin, votein);
            // sprintf(buff,"\”votedFor\”: %d,\”votedFor\”: %d,\”votedFor\”: %d,\”status\”: %d\”fobID\”: %d, \”votedFor\”: %d}", voterin, votein);
            sprintf(buff, "{\"fobID\":%d,\"votedFor\":%d}", voterin, votein);
            payload = buff;

            printf("sending %c%c to %s\n", buff[0], buff[1], ip);
            printf("sending: %d%d\n", voterin, votein);

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
    dest_addr_ip4->sin_port = htons(3333);
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
    ESP_LOGI(TAG, "Socket bound, port %d", 3333);

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
            source_addr.sin_port = htons(3333);

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
void recv_task()
{
    // Buffer for input data
    uint8_t *data_in = (uint8_t *)malloc(BUF_SIZE);
    while (1)
    {
        int len_in = uart_read_bytes(UART_NUM_1, data_in, BUF_SIZE, 20 / portTICK_RATE_MS);
        if (len_in > 0)
        {
            if (data_in[0] == start)
            {

                ESP_LOG_BUFFER_HEXDUMP(TAG_SYSTEM, data_in, lenout1, ESP_LOG_INFO);

                 voterin = 3;
               // votein = data_in[1];
                votein = 1;
                if (strcmp(leaderip, myip))
                {
                    send_server((void *)AF_INET, ServerIP);
                }
                else
                {
                    send_task((void *)AF_INET, leaderip);
                }

                printf("Received!!\n");
            }
        }
        else
        {
            // printf("Nothing received.\n");
        }
        vTaskDelay(5 / portTICK_PERIOD_MS);
    }
    free(data_in);
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
        dest_addr_ip4->sin_port = htons(3333);
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
        ESP_LOGI(TAG, "Socket bound, port %d", 3333);

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

void app_main(void)
{

  

    rmt_tx_init();
    uart_init();
    button_init();

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());
    init();
 char *hostname = piaddress;
   hostname_to_ip(hostname, ServerIP);

    xTaskCreate(receive_task, "receive_task", 4096, (void *)AF_INET, 5, NULL);
    xTaskCreate(button_task, "button_task", 1024 * 2, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(recv_task, "uart_rx_task", 1024 * 4, NULL, configMAX_PRIORITIES, NULL);
    mux = xSemaphoreCreateMutex();
    int both = 0;
    //send_task();
    for (int i = 0; i < 3; i++)
    {
        if (!strcmp(addr_str, esp2ip))
        {
            both++;
            break;
        }
        else if (!strcmp(addr_str, esp1ip))
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
        send_task((void *)AF_INET, esp1ip);
        send_task((void *)AF_INET, esp2ip);

        while (1)
        {
            // }
            printf("waiting to recieve \n");
            if (!strcmp(addr_str, esp1ip))
            {
                printf("recieved from esp1 \n");
                check1 = 1;
                min_list[0] = rx_buffer[2] - 48;
            }
            else if (!strcmp(addr_str, esp2ip))
            {
                printf("recieved from esp2 \n");
                check2 = 1;
                min_list[1] = rx_buffer[2] - 48;
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
                status = 0;
                send_task((void *)AF_INET, esp1ip);
                send_task((void *)AF_INET, esp2ip);
                printf("oop, esp1 is the leader lol\n");
                leaderip = esp1ip;
                return;
            }
            else
            {
                status = 1;
                for (int i = 0; i < 2; i++)
                {
                    send_task((void *)AF_INET, esp1ip);
                    send_task((void *)AF_INET, esp1ip);
                    send_task((void *)AF_INET, esp2ip);
                    send_task((void *)AF_INET, esp2ip);
                }
                printf(" 2 I am the leader exiting now byeee\n");
                leaderip = esp3ip;
                while (1)
                {
                    addr_str[0] = 'c';
                    while (1)
                    {
                        if (!strcmp(addr_str, esp1ip))
                        {
                            printf("recieved from esp1 \n");
                            send_server((void *)AF_INET, ServerIP);
                            printf("sending to pi");
                            break;
                        }
                        else if (!strcmp(addr_str, esp2ip))
                        {
                            printf("recieved from esp2 \n");
                            send_server((void *)AF_INET, ServerIP);
                            printf("sending to pi");
                            break;
                        }
                    }
                }
                //
            }
        }
        else
        {
            if (min_list[1] < minval)
            {
                minval = min_list[1];
                status = 0;
                send_task((void *)AF_INET, esp1ip);
                send_task((void *)AF_INET, esp2ip);
                printf("oop, esp2 is the leader lol\n");
                leaderip = esp2ip;

                return;
            }
            else
            {
                status = 1;
                for (int i = 0; i < 2; i++)
                {
                    send_task((void *)AF_INET, esp1ip);
                    send_task((void *)AF_INET, esp1ip);
                    send_task((void *)AF_INET, esp2ip);
                    send_task((void *)AF_INET, esp2ip);
                }
                printf(" 1 I am the leader exiting now byeee\n");
                leaderip = esp3ip;
                while (1)
                {
                    addr_str[0] = 'c';
                    while (1)
                    {
                        if (!strcmp(addr_str, esp1ip))
                        {
                            printf("recieved from esp1 \n");
                            send_server((void *)AF_INET, ServerIP);
                            printf("sending to pi");
                            break;
                        }
                        else if (!strcmp(addr_str, esp2ip))
                        {
                            printf("recieved from esp2 \n");
                            send_server((void *)AF_INET, ServerIP);
                            printf("sending to pi");
                            break;
                        }
                    }
                }
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
            if (!strcmp(addr_str, esp1ip))
            {
                printf("recieced from esp1\n");
                recv = 1;
                min_list[0] = rx_buffer[2];
                break;
            }
            else if (!strcmp(addr_str, esp2ip))
            {
                printf("recieced from esp2\n");
                recv = 1;
                printf("minval  %d\n", minval);
                printf("rx  %d\n", (rx_buffer[2]));
                rx = rx_buffer[2] - 48;
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
                        if (!strcmp(addr_str, esp1ip))
                        {
                            rx = rx_buffer[3] - 48;
                            if (rx == 1)
                            {
                                printf("esp1 is the leader\n");
                                leaderip = esp1ip;

                                return;
                            }
                            else if ((rx_buffer[2] - 48) != MYID)
                            {
                                printf("esp2 is the leader\n");
                                leaderip = esp2ip;
                            }
                        }
                        else if (!strcmp(addr_str, esp2ip))
                        {
                            rx = rx_buffer[3] - 48;
                            if (rx == 1)
                            {
                                printf("esp2 is the leader\n");
                                leaderip = esp2ip;

                                return;
                            }
                            else if ((rx_buffer[2] - 48) != MYID)
                            {
                                printf("i AM THE LEADERr\n");
                                leaderip = esp3ip;
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

                    while (1)
                {
                    addr_str[0] = "c";
                    while (1)
                    {
                        if (!strcmp(addr_str, esp1ip))
                        {
                            printf("recieved from esp1 \n");
                            send_server((void *)AF_INET, ServerIP);
                            printf("sending to pi");
                            break;
                        }
                        else if (!strcmp(addr_str, esp2ip))
                        {
                            printf("recieved from esp2 \n");
                            send_server((void *)AF_INET, ServerIP);
                            printf("sending to pi");
                            break;
                        }
                    }
                }
                }
                break;
            }
        }
    }
}
