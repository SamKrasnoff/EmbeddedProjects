#include <stdio.h>
#include "driver/i2c.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_intr_alloc.h"

#define BLINK_GPIO 26
#define BLINK_GPIO1 25
#define BLINK_GPIO2 4
#define BLINK_GPIO3 21
#define BUTTON 14
#define REDGPIO 13

#define SLAVE_ADDR                         0x70 // alphanumeric address
#define OSC                                0x21 // oscillator cmd
#define HT16K33_BLINK_DISPLAYON            0x01 // Display on cmd
#define HT16K33_BLINK_OFF                  0    // Blink off cmd
#define HT16K33_BLINK_CMD                  0x80 // Blink cmd
#define HT16K33_CMD_BRIGHTNESS             0xE0 // Brightness cmd
#define ESP_INTR_FLAG_LEVEL3		(1<<3)

// Master I2C
#define I2C_EXAMPLE_MASTER_SCL_IO          22   // gpio number for i2c clk
#define I2C_EXAMPLE_MASTER_SDA_IO          23   // gpio number for i2c data
#define I2C_EXAMPLE_MASTER_NUM             I2C_NUM_0  // i2c port
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE  0    // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE  0    // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_FREQ_HZ         100000     // i2c master clock freq
#define WRITE_BIT                          I2C_MASTER_WRITE // i2c master write
#define READ_BIT                           I2C_MASTER_READ  // i2c master read
#define ACK_CHECK_EN                       true // i2c master will check ack
#define ACK_CHECK_DIS                      false// i2c master will not check ack
#define ACK_VAL                            0x00 // i2c ack value
#define NACK_VAL                           0xFF // i2c nack value
int count=0;
int temp;
int flag = 0;
uint16_t displaybuffer[8];
int ret;
int alpha_oscillator() {
  int ret;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, OSC, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  vTaskDelay(200 / portTICK_RATE_MS);
  return ret;
}

// Set blink rate to off
int no_blink() {
  int ret;
  i2c_cmd_handle_t cmd2 = i2c_cmd_link_create();
  i2c_master_start(cmd2);
  i2c_master_write_byte(cmd2, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd2, HT16K33_BLINK_CMD | HT16K33_BLINK_DISPLAYON | (HT16K33_BLINK_OFF << 1), ACK_CHECK_EN);
  i2c_master_stop(cmd2);
  ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd2, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd2);
  vTaskDelay(200 / portTICK_RATE_MS);
  return ret;
}

// Set Brightness
int set_brightness_max(uint8_t val) {
  int ret;
  i2c_cmd_handle_t cmd3 = i2c_cmd_link_create();
  i2c_master_start(cmd3);
  i2c_master_write_byte(cmd3, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd3, HT16K33_CMD_BRIGHTNESS | val, ACK_CHECK_EN);
  i2c_master_stop(cmd3);
  ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd3, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd3);
  vTaskDelay(200 / portTICK_RATE_MS);
  return ret;
}

static void IRAM_ATTR gpio_isr_handler(void* arg){  // Interrupt handler for your GPIO
    flag ^= 1;// Toggle state of flag
}
void init() {
    gpio_pad_select_gpio(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
    gpio_pad_select_gpio(BLINK_GPIO1);
    gpio_set_direction(BLINK_GPIO1, GPIO_MODE_OUTPUT);
    gpio_pad_select_gpio(BLINK_GPIO2);
    gpio_set_direction(BLINK_GPIO2, GPIO_MODE_OUTPUT);
    gpio_pad_select_gpio(BLINK_GPIO3);
    gpio_set_direction(BLINK_GPIO3, GPIO_MODE_OUTPUT);
    gpio_pad_select_gpio(BUTTON);
    gpio_set_direction(BUTTON, GPIO_MODE_INPUT);
    gpio_pad_select_gpio(REDGPIO);
    gpio_set_direction(REDGPIO, GPIO_MODE_OUTPUT);
    int err;

    // Port configuration
    int i2c_master_port = I2C_EXAMPLE_MASTER_NUM;

    /// Define I2C configurations
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;                              // Master mode
    conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;              // Default SDA pin
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;                  // Internal pullup
    conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;              // Default SCL pin
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;                  // Internal pullup
    conf.master.clk_speed = I2C_EXAMPLE_MASTER_FREQ_HZ;       // CLK frequency
    err = i2c_param_config(i2c_master_port, &conf);           // Configure
    if (err == ESP_OK) {printf("- parameters: ok\n");}
    // Install I2C driver
    err = i2c_driver_install(i2c_master_port, conf.mode,
                       I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
                       I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0);
    // i2c_set_data_mode(i2c_master_port,I2C_DATA_MODE_LSB_FIRST,I2C_DATA_MODE_LSB_FIRST);
    if (err == ESP_OK) {printf("- initialized: yes\n\n");}
    // Dat in MSB mode
    i2c_set_data_mode(i2c_master_port, I2C_DATA_MODE_MSB_FIRST, I2C_DATA_MODE_MSB_FIRST);

    printf(">> Test Alphanumeric Display: \n");

    // Set up routines
    // Turn on alpha oscillator
    ret = alpha_oscillator();
    if(ret == ESP_OK) {printf("- oscillator: ok \n");}
    // Set display blink off
    ret = no_blink();
    if(ret == ESP_OK) {printf("- blink: off \n");}
    ret = set_brightness_max(0xF);
    if(ret == ESP_OK) {printf("- brightness: max \n");}
}

static void task_1(){
    while(1){
        if(flag == 1){
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
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}
static void task_2(){
    while(1) {
        if(flag == 0){  //!gpio_get_level(14)
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
            count--;
            if(count == 0){
                count = 15;
            }
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}
static void task_3(){
    while (1)
    {
        if (flag == 1){
            displaybuffer[0] = 0b0000000000111110;
            displaybuffer[1] = 0b0000000011110011;
            displaybuffer[2] = 0b0000000000000000;
            displaybuffer[3] = 0b0000000000000000;
        }
        if (flag == 0){
            displaybuffer[0] = 0b0001001000001111;
            displaybuffer[1] = 0b0000000000111111;
            displaybuffer[2] = 0b0010100000110110;
            displaybuffer[3] = 0b0010000100110110;
        }
        i2c_cmd_handle_t cmd4 = i2c_cmd_link_create();
        i2c_master_start(cmd4);
        i2c_master_write_byte(cmd4, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
        i2c_master_write_byte(cmd4, (uint8_t)0x00, ACK_CHECK_EN);
        for (uint8_t i=0; i<8; i++) {
            i2c_master_write_byte(cmd4, displaybuffer[i] & 0xFF, ACK_CHECK_EN);
            i2c_master_write_byte(cmd4, displaybuffer[i] >> 8, ACK_CHECK_EN);
        }
        i2c_master_stop(cmd4);
        ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd4, 1000 / portTICK_RATE_MS);
        i2c_cmd_link_delete(cmd4);
        vTaskDelay(250 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);  
}
static void task_4(){
    gpio_intr_enable(BUTTON);
    gpio_set_intr_type(BUTTON, 1);
    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL3);
    gpio_isr_handler_add(BUTTON, gpio_isr_handler, (void *)BUTTON);
    while (1)
    {
        if (flag == 1){
            gpio_set_level(REDGPIO, 1);
            vTaskDelay(50);
        }
        if (flag == 0){
            gpio_set_level(REDGPIO, 0);
            vTaskDelay(50);
        }
    }
    
}
void app_main(void)
{
    init();
    xTaskCreate(task_1, "task_1", 2048, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(task_2, "task_2", 2048, NULL, configMAX_PRIORITIES-1, NULL);
    xTaskCreate(task_3, "task_3", 2048, NULL, configMAX_PRIORITIES-2, NULL);
    xTaskCreate(task_4, "task_4", 2048, NULL, configMAX_PRIORITIES-3, NULL);

}
