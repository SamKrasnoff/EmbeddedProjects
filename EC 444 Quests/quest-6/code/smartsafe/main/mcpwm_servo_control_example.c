/* brushed dc motor control example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/*
 * This example will show you how to use MCPWM module to control brushed dc motor.
 * This code is tested with L298 motor driver.
 * User may need to make changes according to the motor driver they use.
*/

#include <stdio.h>
#include <math.h>
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/periph_ctrl.h"

#include "esp_attr.h"
#include "esp_log.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "driver/pcnt.h"

#include <string.h>
#include <sys/param.h>

#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"

#include "nvs_flash.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"

#include "driver/adc.h"
#include "driver/uart.h"
#include "esp_vfs_dev.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include <stdlib.h>

#define PCNT_TEST_UNIT PCNT_UNIT_0
#define PCNT_H_LIM_VAL 10000
#define PCNT_L_LIM_VAL -10
#define PCNT_THRESH1_VAL 5
#define PCNT_THRESH0_VAL -5
#define PCNT_INPUT_SIG_IO 34 // Pulse Input GPIO
//A2 34 A3 39
#define PCNT_INPUT_CTRL_IO 5 // Control GPIO HIGH=count up, LOW=count down
#define LEDC_OUTPUT_IO 18    // Output GPIO of a sample 1 Hz pulse generator

#define I2C_EXAMPLE_MASTER_SCL_IO 22        // gpio number for i2c clk
#define I2C_EXAMPLE_MASTER_SDA_IO 23        // gpio number for i2c data
#define I2C_EXAMPLE_MASTER_NUM I2C_NUM_0    // i2c port
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE 0 // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE 0 // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_FREQ_HZ 10000    // i2c master clock freq
#define WRITE_BIT I2C_MASTER_WRITE          // i2c master write
#define READ_BIT I2C_MASTER_READ            // i2c master read
#define ACK_CHECK_EN true                   // i2c master will check ack
#define ACK_CHECK_DIS false                 // i2c master will not check ack
#define ACK_VAL 0x00                        // i2c ack value
#define NACK_VAL 0xFF

#define SLAVE_ADDR 0x62 // Lidar slave address

#define GPIO_PWM0A_OUT 15 //Set GPIO 15 as PWM0A
#define GPIO_PWM0B_OUT 33 //Set GPIO 16 as PWM0B

#define GPIO_INPUT_IO_0 23
#define GPIO_INPUT_IO_1 22
#define GPIO_INPUT_PIN_SEL ((1ULL << GPIO_INPUT_IO_0) | (1ULL << GPIO_INPUT_IO_1))
#define ESP_INTR_FLAG_DEFAULT 0
#define GPIO_A_IN0 32                     //Set GPIO 15 as PWM0A
#define GPIO_A_IN1 14                     //Set GPIO 15 as PWM0A
#define GPIO_B_IN0 12                     //Set GPIO 15 as PWM0A
#define GPIO_B_IN1 27                     //Set GPIO 15 as PWM0A
xQueueHandle pcnt_evt_queue;              // A queue to handle pulse counter events
pcnt_isr_handle_t user_isr_handle = NULL; //user's ISR service handle

static void i2c_master_init()
{
    // Debug
    printf("\n>> i2c Config\n");
    int err;

    // Port configuration
    int i2c_master_port = I2C_EXAMPLE_MASTER_NUM;

    /// Define I2C configurations
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;                        // Master mode
    conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;        // Default SDA pin
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;            // Internal pullup
    conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;        // Default SCL pin
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;            // Internal pullup
    conf.master.clk_speed = I2C_EXAMPLE_MASTER_FREQ_HZ; // CLK frequency
    err = i2c_param_config(i2c_master_port, &conf);     // Configure
    if (err == ESP_OK)
    {
        printf("- parameters: ok\n");
    }

    // Install I2C driver
    err = i2c_driver_install(i2c_master_port, conf.mode,
                             I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
                             I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0);
    if (err == ESP_OK)
    {
        printf("- initialized: yes\n");
    }

    // Data in MSB mode
    i2c_set_data_mode(i2c_master_port, I2C_DATA_MODE_MSB_FIRST, I2C_DATA_MODE_MSB_FIRST);
}

// Utility  Functions //////////////////////////////////////////////////////////

// Utility function to test for I2C device address -- not used in deploy
int testConnection(uint8_t devAddr, int32_t timeout)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (devAddr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    int err = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return err;
}

// Utility function to scan for i2c device
static void i2c_scanner()
{
    int32_t scanTimeout = 1000;
    printf("\n>> I2C scanning ..."
           "\n");
    uint8_t count = 0;
    for (uint8_t i = 1; i < 127; i++)
    {
        // printf("0x%X%s", i, "\n");
        if (testConnection(i, scanTimeout) == ESP_OK)
        {
            printf("- Device found at address: 0x%X%s", i, "\n");
            count++;
        }
    }
    if (count == 0)
    {
        printf("- No I2C devices found!"
               "\n");
    }
}

////////////////////////////////////////////////////////////////////////////////

// ADXL343 Functions ///////////////////////////////////////////////////////////

// Get Device ID
int getDeviceID(uint8_t *data)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SLAVE_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, data, ACK_CHECK_DIS);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

// Write one byte to register
void writeRegister(uint8_t reg, uint8_t data)
{
    // int ret;
    // printf("--Writing %d to reg %d!--\n", data, reg);
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    //start command
    i2c_master_start(cmd);
    //slave address followed by write bit
    i2c_master_write_byte(cmd, (SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    //register pointer sent
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    //data sent
    i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
    //stop command
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    // if (ret == ESP_OK)
    // {
    //   printf("I2C SUCCESSFUL \n");
    // }
    i2c_cmd_link_delete(cmd);
}

// Read register
uint8_t readRegister(uint8_t reg)
{
    uint8_t value;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    //start command
    i2c_master_start(cmd);
    //slave followed by write bit
    i2c_master_write_byte(cmd, (SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    //register pointer sent
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    //stop command
    // i2c_master_stop(cmd);

    //repeated start command
    i2c_master_start(cmd);
    //slave followed by read bit
    i2c_master_write_byte(cmd, (SLAVE_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
    //place data from register into bus
    i2c_master_read_byte(cmd, &value, ACK_CHECK_DIS);
    //stop command
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return value;
}

int distance = 0;
static void measure()
{
    uint8_t reg1 = 0x00;
    uint8_t data1 = 0x04;
    uint8_t data = 0x06;
    vTaskDelay(22);
    while (1)
    {
        writeRegister(reg1, data1);
        data = readRegister(0x01);
        while ((data & 1))
        {
            data = readRegister(0x01);
            vTaskDelay(50);
        }

        uint8_t high = readRegister(0x11);
        uint8_t low = readRegister(0x10);

        distance = (high << 8) + low;

        printf("Distance is: %d\n", distance);
        vTaskDelay(100);
    }
}

/* A sample structure to pass events from the PCNT
 * interrupt handler to the main program.
 */
typedef struct
{
    int unit;        // the PCNT unit that originated an interrupt
    uint32_t status; // information on the event type that caused the interrupt
} pcnt_evt_t;

/* Decode what PCNT's unit originated an interrupt
 * and pass this information together with the event type
 * the main program using a queue.
 */
static void IRAM_ATTR pcnt_example_intr_handler(void *arg)
{
    uint32_t intr_status = PCNT.int_st.val;
    int i;
    pcnt_evt_t evt;
    portBASE_TYPE HPTaskAwoken = pdFALSE;

    for (i = 0; i < PCNT_UNIT_MAX; i++)
    {
        if (intr_status & (BIT(i)))
        {
            evt.unit = i;
            /* Save the PCNT event type that caused an interrupt
               to pass it to the main program */
            evt.status = PCNT.status_unit[i].val;
            PCNT.int_clr.val = BIT(i);
            xQueueSendFromISR(pcnt_evt_queue, &evt, &HPTaskAwoken);
            if (HPTaskAwoken == pdTRUE)
            {
                portYIELD_FROM_ISR();
            }
        }
    }
}

/* Configure LED PWM Controller
 * to output sample pulses at 1 Hz with duty of about 10%
 */
static void ledc_init(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer;
    ledc_timer.speed_mode = LEDC_LOW_SPEED_MODE;
    ledc_timer.timer_num = LEDC_TIMER_1;
    ledc_timer.duty_resolution = LEDC_TIMER_10_BIT;
    ledc_timer.freq_hz = 1; // set output frequency at 1 Hz
    ledc_timer.clk_cfg = LEDC_AUTO_CLK;
    ledc_timer_config(&ledc_timer);

    // Prepare and then apply the LEDC PWM channel configuration
    // ledc_channel_config_t ledc_channel;
    // ledc_channel.speed_mode = LEDC_LOW_SPEED_MODE;
    // ledc_channel.channel    = LEDC_CHANNEL_1;
    // ledc_channel.timer_sel  = LEDC_TIMER_1;
    // ledc_channel.intr_type  = LEDC_INTR_DISABLE;
    // ledc_channel.gpio_num   = LEDC_OUTPUT_IO;
    // ledc_channel.duty       = 100; // set duty at about 10%
    // ledc_channel.hpoint     = 0;
    // ledc_channel_config(&ledc_channel);
}

/* Initialize PCNT functions:
 *  - configure and initialize PCNT
 *  - set up the input filter
 *  - set up the counter events to watch
 */
static void pcnt_example_init(void)
{
    /* Prepare configuration for the PCNT unit */
    pcnt_config_t pcnt_config = {
        // Set PCNT input signal and control GPIOs
        .pulse_gpio_num = PCNT_INPUT_SIG_IO,
        .ctrl_gpio_num = PCNT_INPUT_CTRL_IO,
        .channel = PCNT_CHANNEL_0,
        .unit = PCNT_TEST_UNIT,
        // What to do on the positive / negative edge of pulse input?
        .pos_mode = PCNT_COUNT_INC, // Count up on the positive edge
        .neg_mode = PCNT_COUNT_DIS, // Keep the counter value on the negative edge
        // What to do when control input is low or high?
        .lctrl_mode = PCNT_MODE_REVERSE, // Reverse counting direction if low
        .hctrl_mode = PCNT_MODE_KEEP,    // Keep the primary counter mode if high
        // Set the maximum and minimum limit values to watch
        .counter_h_lim = PCNT_H_LIM_VAL,
        .counter_l_lim = PCNT_L_LIM_VAL,
    };
    /* Initialize PCNT unit */
    pcnt_unit_config(&pcnt_config);

    /* Configure and enable the input filter */
    pcnt_set_filter_value(PCNT_TEST_UNIT, 100);
    pcnt_filter_enable(PCNT_TEST_UNIT);

    /* Set threshold 0 and 1 values and enable events to watch */
    pcnt_set_event_value(PCNT_TEST_UNIT, PCNT_EVT_THRES_1, PCNT_THRESH1_VAL);
    pcnt_event_enable(PCNT_TEST_UNIT, PCNT_EVT_THRES_1);
    pcnt_set_event_value(PCNT_TEST_UNIT, PCNT_EVT_THRES_0, PCNT_THRESH0_VAL);
    pcnt_event_enable(PCNT_TEST_UNIT, PCNT_EVT_THRES_0);
    /* Enable events on zero, maximum and minimum limit values */
    pcnt_event_enable(PCNT_TEST_UNIT, PCNT_EVT_ZERO);
    pcnt_event_enable(PCNT_TEST_UNIT, PCNT_EVT_H_LIM);
    pcnt_event_enable(PCNT_TEST_UNIT, PCNT_EVT_L_LIM);

    /* Initialize PCNT's counter */
    pcnt_counter_pause(PCNT_TEST_UNIT);
    pcnt_counter_clear(PCNT_TEST_UNIT);

    /* Register ISR handler and enable interrupts for PCNT unit */
    pcnt_isr_register(pcnt_example_intr_handler, NULL, 0, &user_isr_handle);
    pcnt_intr_enable(PCNT_TEST_UNIT);

    /* Everything is set up, now go to counting */
    pcnt_counter_resume(PCNT_TEST_UNIT);
}

static void mcpwm_example_gpio_initialize(void)
{
    printf("initializing mcpwm gpio...\n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM0A_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, GPIO_PWM0B_OUT);
}

/**
 * @brief motor moves in forward direction, with duty cycle = duty %
 */
static void brushed_motor_forward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num, float duty_cycle)
{
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_B);
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_A, duty_cycle);
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
}

/**
 * @brief motor moves in backward direction, with duty cycle = duty %
 */
static void brushed_motor_backward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num, float duty_cycle)
{
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_A);
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_B, duty_cycle);
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_B, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
}

/**
 * @brief motor stop
 */
static void brushed_motor_stop(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num)
{
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_A);
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_B);
}

/**
 * @brief Configure MCPWM module for brushed dc motor
 */
static void mcpwm_example_brushed_motor_control(void *arg)
{
    //1. mcpwm gpio initialization
    mcpwm_example_gpio_initialize();

    //2. initial mcpwm configuration
    printf("Configuring Initial Parameters of mcpwm...\n");
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 1000; //frequency = 500Hz,
    pwm_config.cmpr_a = 0;       //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;       //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config); //Configure PWM0A & PWM0B with above settings
    // while (1) {
    //     brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, 50.0);
    //     vTaskDelay(2000 / portTICK_RATE_MS);
    //     brushed_motor_backward(MCPWM_UNIT_0, MCPWM_TIMER_0, 30.0);
    //     vTaskDelay(2000 / portTICK_RATE_MS);
    //     brushed_motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
    //     vTaskDelay(2000 / portTICK_RATE_MS);
    // }
}

int dt = 100;
int setpoint = 50;
float previous_error = 0.00; // Set up PID loop
float integral = 0.00;
float derivative;
float output;
float error;
float Kp = 2, Ki = 0, Kd = 1;

void PID()
{
    integral = 0.00;
    previous_error = 0.00;
    while (1)
    {
        error = setpoint - distance;
        printf("Error is %.2f\n", error);

        if (error > 0)
        {
            gpio_set_level(25, 0);
            gpio_set_level(4, 0);
            gpio_set_level(26, 1);
        }
        else if (error < 0)
        {
            gpio_set_level(25, 1);
            gpio_set_level(4, 0);
            gpio_set_level(26, 0);
        }
        else if (error == 0)
        {
            gpio_set_level(25, 0);
            gpio_set_level(4, 1);
            gpio_set_level(26, 0);
        }

        integral = integral + error * dt;
        derivative = (error - previous_error) / dt;
        output = Kp * error + Ki * integral + Kd * derivative;
        if (output > 0)
        {
            gpio_set_level(GPIO_A_IN0, 0);
            gpio_set_level(GPIO_A_IN1, 0);
            gpio_set_level(GPIO_B_IN0, 0);
            gpio_set_level(GPIO_B_IN1, 0);
        }
        previous_error = error;
        printf("output is %.2f\n", output);

        vTaskDelay(100);
    }
}

void stop()
{
    while (1)
    {
        if (output > 0)
        {
        }
    }
}
static xQueueHandle gpio_evt_queue = NULL;

static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t)arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void gpio_task_example(void *arg)
{
    uint32_t io_num;
    for (;;)
    {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY))
        {
            // printf("GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(io_num));
            if (gpio_get_level(io_num) && (io_num == 23))
            {
                gpio_pad_select_gpio(GPIO_A_IN0);
                gpio_set_direction(GPIO_A_IN0, GPIO_MODE_OUTPUT);
                // gpio_set_level(GPIO_A_IN0, 0);

                gpio_pad_select_gpio(GPIO_A_IN1);
                gpio_set_direction(GPIO_A_IN1, GPIO_MODE_OUTPUT);
                gpio_set_level(GPIO_A_IN1, 0);
            }
            if (gpio_get_level(io_num) && (io_num == 22))
            {
                gpio_pad_select_gpio(GPIO_A_IN0);
                gpio_set_direction(GPIO_A_IN0, GPIO_MODE_OUTPUT);
                gpio_set_level(GPIO_A_IN0, 0);

                gpio_pad_select_gpio(GPIO_A_IN1);
                gpio_set_direction(GPIO_A_IN1, GPIO_MODE_OUTPUT);
                // gpio_set_level(GPIO_A_IN1, 0);
            }
        }
    }
}

void toggleLock(int state)
{
    if (state == 0)
    {
        printf("Close...\n");

        gpio_pad_select_gpio(GPIO_A_IN0);
        gpio_set_direction(GPIO_A_IN0, GPIO_MODE_OUTPUT);
        gpio_set_level(GPIO_A_IN0, 0);

        gpio_pad_select_gpio(GPIO_A_IN1);
        gpio_set_direction(GPIO_A_IN1, GPIO_MODE_OUTPUT);
        gpio_set_level(GPIO_A_IN1, 1);
    }
    else
    {
        printf("Open...\n");

        gpio_pad_select_gpio(GPIO_A_IN0);
        gpio_set_direction(GPIO_A_IN0, GPIO_MODE_OUTPUT);
        gpio_set_level(GPIO_A_IN0, 1);

        gpio_pad_select_gpio(GPIO_A_IN1);
        gpio_set_direction(GPIO_A_IN1, GPIO_MODE_OUTPUT);
        gpio_set_level(GPIO_A_IN1, 0);
    }
}
int ledState = 0;
static void udp_client_task(void *pvParameters)
{
    char rx_buffer[128];
    char addr_str[128];
    int addr_family;
    int ip_protocol;

    while (1)
    {

        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;
        inet_ntoa_r(dest_addr.sin_addr, addr_str, sizeof(addr_str) - 1);
        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        while (1)
        {
            if (sock < 0)
            {
                ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
                break;
            }

            sprintf(buff, "{\"temp\": %.2f, \"x\": %.2f, \"y\": %.2f, \"z\": %.2f, \"state\": %d}", tempGlobal, xVal, yVal, zVal, ledState);
            payload = buff;
            printf("sending\n");
            int err = sendto(sock, payload, strlen(payload), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
            struct sockaddr_in source_addr; // Large enough for both IPv4 or IPv6
            socklen_t socklen = sizeof(source_addr);
            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);
            rx_buffer[len] = 0;

            if (len == 1)
            {
                ledState ^= 1;
                led(ledState);
            }
            printf("rx: %s\n", rx_buffer);
            memset(rx_buffer, 0, sizeof(rx_buffer));
            printf("State: %d\n", ledState);

            vTaskDelay(500 / portTICK_PERIOD_MS);
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
void app_main(void)
{
    gpio_config_t io_conf;

    //interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_down_en = 1;
    gpio_config(&io_conf);

    // //change gpio intrrupt type for one pin
    gpio_set_intr_type(GPIO_INPUT_IO_0, GPIO_INTR_ANYEDGE);

    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    //start gpio task
    xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL);

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void *)GPIO_INPUT_IO_0);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_1, gpio_isr_handler, (void *)GPIO_INPUT_IO_1);

    //remove isr handler for gpio number.
    gpio_isr_handler_remove(GPIO_INPUT_IO_0);
    //hook isr handler for specific gpio pin again
    gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void *)GPIO_INPUT_IO_0);
    // gpio_pad_select_gpio(25);
    // gpio_set_direction(25, GPIO_MODE_OUTPUT);
    // gpio_set_level(25, 0);

    // gpio_pad_select_gpio(4);
    // gpio_set_direction(4, GPIO_MODE_OUTPUT);
    // gpio_set_level(4, 0);

    // gpio_pad_select_gpio(26);
    // gpio_set_direction(26, GPIO_MODE_OUTPUT);
    // gpio_set_level(26, 0);

    gpio_set_direction(4, GPIO_MODE_INPUT);
    gpio_pad_select_gpio(GPIO_A_IN0);
    gpio_set_direction(GPIO_A_IN0, GPIO_MODE_OUTPUT);
    // gpio_set_level(GPIO_A_IN0, 0);

    gpio_pad_select_gpio(GPIO_A_IN1);
    gpio_set_direction(GPIO_A_IN1, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_A_IN1, 0);

    // gpio_pad_select_gpio(GPIO_B_IN0);
    // gpio_set_direction(GPIO_B_IN0, GPIO_MODE_OUTPUT);
    // gpio_set_level(GPIO_B_IN0, 0);

    // gpio_pad_select_gpio(GPIO_B_IN1);
    // gpio_set_direction(GPIO_B_IN1, GPIO_MODE_OUTPUT);
    // gpio_set_level(GPIO_B_IN1, 1);

    mcpwm_config_t pwm_config;
    pwm_config.frequency = 1000; //frequency = 500Hz,
    pwm_config.cmpr_a = 0;       //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;       //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);

    printf("Testing brushed motor...\n");
    toggleLock(1);
    vTaskDelay(5000 / portTICK_RATE_MS);
    toggleLock(0);
    vTaskDelay(5000 / portTICK_RATE_MS);
    // mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 50.0);

    // mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 0.0);
    // ledc_init();
    pcnt_evt_queue = xQueueCreate(10, sizeof(pcnt_evt_t));
    pcnt_example_init();
    xTaskCreate(udp_client_task, "udp_client", 4096, NULL, 5, NULL);
    // i2c_master_init();
    // i2c_scanner();
    // xTaskCreate(measure, "measure", 4096, NULL, 5, NULL);
    // vTaskDelay(200);
    // xTaskCreate(PID, "PID", 4096, NULL, configMAX_PRIORITIES, NULL);
    //xTaskCreate(stop, "stop",2038, NULL, 4, NULL);

    // int16_t count = 0;
    // pcnt_evt_t evt;
    // portBASE_TYPE res;
    // while (1)
    // {
    //     /* Wait for the event information passed from PCNT's interrupt handler.
    //      * Once received, decode the event type and print it on the serial monitor.
    //      */
    //     res = xQueueReceive(pcnt_evt_queue, &evt, 1000 / portTICK_PERIOD_MS);
    //     pcnt_get_counter_value(PCNT_TEST_UNIT, &count);
    //     printf("Current counter value :%d\n", count);
    // }
}
