/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

/* Can use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define BLINK_GPIO 26
#define BLINK_GPIO1 25
#define BLINK_GPIO2 4
#define BLINK_GPIO3 21
#define BUTTON 14
#define REDGPIO 13
void app_main(void)
{
    /* Configure the IOMUX register for pad BLINK_GPIO (some pads are
       muxed to GPIO on reset already, but some default to other
       functions and need to be switched to GPIO. Consult the
       Technical Reference for a list of pads and their default
       functions.)
    */
    int count=0;
    int temp;
    gpio_pad_select_gpio(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
    gpio_pad_select_gpio(BLINK_GPIO1);
    gpio_set_direction(BLINK_GPIO1, GPIO_MODE_OUTPUT);
    gpio_pad_select_gpio(BLINK_GPIO2);
    gpio_set_direction(BLINK_GPIO2, GPIO_MODE_OUTPUT);
    gpio_pad_select_gpio(BLINK_GPIO3);
    gpio_set_direction(BLINK_GPIO3, GPIO_MODE_OUTPUT);
    while(1) {
        temp = count;
        if (temp/8 == 1){
            gpio_set_level(BLINK_GPIO, 1);
            temp -= 8;
        }
        else{
            gpio_set_level(BLINK_GPIO, 0);
        }
        if (temp/4 == 1){
            gpio_set_level(BLINK_GPIO1, 1);
            temp -= 4;
        }
        else{
            gpio_set_level(BLINK_GPIO1, 0);
        }
        if (temp/2 == 1){
            gpio_set_level(BLINK_GPIO2, 1);
            temp -= 2;
        }
        else{
            gpio_set_level(BLINK_GPIO2, 0);
        }
        if (temp/1 == 1){
            gpio_set_level(BLINK_GPIO3, 1);
            temp -= 1;
        }
        else{
            gpio_set_level(BLINK_GPIO3, 0);
        }
        count++;
        if(count == 16){
            count = 0;
        }
        /* Blink off (output low) */
        // gpio_set_level(BLINK_GPIO, 0);
        // gpio_set_level(BLINK_GPIO1, 0);
        // gpio_set_level(BLINK_GPIO2, 0);
        // gpio_set_level(BLINK_GPIO3, 0);
        // vTaskDelay(1000 / portTICK_PERIOD_MS);
        // /* Blink on (output high) */
        // gpio_set_level(BLINK_GPIO1, 1);
        // gpio_set_level(BLINK_GPIO2, 1);
        // gpio_set_level(BLINK_GPIO3, 1);
        // gpio_set_level(BLINK_GPIO, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
