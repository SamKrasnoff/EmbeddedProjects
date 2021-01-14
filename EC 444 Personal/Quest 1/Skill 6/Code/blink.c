/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_vfs_dev.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

/* Can use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define BLINK_GPIO CONFIG_BLINK_GPIO

void app_main(void)
{
    ESP_ERROR_CHECK( uart_driver_install(UART_NUM_0,256, 0, 0, NULL, 0) );
    esp_vfs_dev_uart_use_driver(UART_NUM_0);

    int highlow = 0;
    int toggler = 0;
    char convert[5];
    char buf[20];
    gpio_pad_select_gpio(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
    while(1) {
        if (toggler == 0)
        {
            char c = getchar();
            if (highlow == 0 && c == 't')
            {
                    printf("Read: t\n");
                    gpio_set_level(BLINK_GPIO, 1);
                    highlow = 1;
                    vTaskDelay(100 / portTICK_PERIOD_MS);
            }
            else if (highlow == 1 && c == 't') {
                    printf("Read: t\n");
                    gpio_set_level(BLINK_GPIO, 0);
                    highlow = 0;
                    vTaskDelay(100 / portTICK_PERIOD_MS);
            }
            else if (c=='s'){
                toggler++;
                printf("\nEcho Mode");
            }
        }
        else if (toggler == 1) {
            fflush(stdin);
            gets(buf);
            if (buf[0]=='s' && buf[1] == '\0'){
                toggler++;
                printf("\nHex Mode:");
                continue;
            }
            printf("echo: %s\n",buf);
            //vTaskDelay(100 / portTICK_PERIOD_MS);
        }
        else if (toggler == 2){
            scanf("%s", convert);
            if (convert[0]=='s' && convert[1] == '\0'){
                toggler = 0;
                printf("Toggle Mode:");
                continue;
            }
            int i = atoi(convert);
            printf("hex: %X\n", i);
            //vTaskDelay(100 / portTICK_PERIOD_MS);
        }
    }
}
