
//Sam Krasnoff, Abd El Aziz Hussein, Yanni Peng
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/adc.h"
#include "driver/gpio.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"
#include "driver/ledc.h"
#include "driver/pcnt.h"
#include "freertos/semphr.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_adc_cal.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/rmt.h"
#include "driver/periph_ctrl.h"
#include "soc/rmt_reg.h"
#include <sys/time.h>
#include <sys/param.h>
#include "sdkconfig.h"
#include "esp_intr_alloc.h"
#include "esp_attr.h"
#include "driver/i2c.h"
#include "esp_vfs_dev.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#if CONFIG_IDF_TARGET_ESP32
#include "esp_adc_cal.h"
#endif

#define SERVO_MIN_PULSEWIDTH 1000 //Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH 2000 //Maximum pulse width in microsecond
#define SERVO_MAX_DEGREE 1


#define RMT_TX_CHANNEL 1 /* RMT channel for transmitter */
#define RMT_TX_GPIO_NUM PIN_TRIGGER /* GPIO number for transmitter signal */
#define RMT_RX_CHANNEL 0 /* RMT channel for receiver */
#define RMT_RX_GPIO_NUM PIN_ECHO /* GPIO number for receiver */
#define RMT_CLK_DIV 100 /* RMT counter clock divider */
#define RMT_TX_CARRIER_EN 0 /* Disable carrier */
#define rmt_item32_tIMEOUT_US 9500 /*!< RMT receiver timeout value(us) */

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

#define I2C_EXAMPLE_MASTER_SCL_IO1         27   // gpio number for i2c clk
#define I2C_EXAMPLE_MASTER_SDA_IO1          33   // gpio number for i2c data
#define I2C_EXAMPLE_MASTER_NUM1             I2C_NUM_1  // i2c port
#define I2C_EXAMPLE_MASTER_FREQ_HZ1         400000

#define SLAVE_ADDR1 (0x62)     // 0x53
#define RegisterMeasure 0x00  // Register to write to initiate ranging.
#define MeasureValue 0x04     // Value to initiate ranging.
#define RegisterHighLowB 0x8f // Register to get both High and Low bytes in 1 call.

#define PCNT_TEST_UNIT PCNT_UNIT_0
#define PCNT_H_LIM_VAL 1000
#define PCNT_L_LIM_VAL -10
#define PCNT_THRESH1_VAL 5
#define PCNT_THRESH0_VAL -5
#define PCNT_INPUT_SIG_IO 39 // Pulse Input GPIO
//A2 34
#define PCNT_INPUT_CTRL_IO 5 // Control GPIO HIGH=count up, LOW=count down
#define LEDC_OUTPUT_IO 18  

#define TIMER_DIVIDER 16                             //  Hardware timer clock divider
#define TIMER_SCALE (TIMER_BASE_CLK / TIMER_DIVIDER) // convert counter value to seconds
#define TIMER_INTERVAL0_SEC (0.5)                    // sample test interval for the first timer
#define TIMER_INTERVAL1_SEC (5.78)                   // sample test interval for the second timer
#define TEST_WITHOUT_RELOAD 0                        // testing will be done without auto reload
#define TEST_WITH_RELOAD 1    

char addr_str[128];
char rx_buffer[128];
char buff[512];
static const char *TAG = "example";

uint16_t displaybuffer[8];
int ret;
int err;
int flag = 0;
uint16_t lidarDistance = 0;
int dt_complete = 0;
float distance = 0;
int dt = 100;
int setpoint = 20;
float Ki = .33, Kd = .33, Kp = .33;
float previous_error = 0.00;		// Set up PID loop
float integral = 0.00;
float derivative;
float output;
float error;
typedef struct
{
    int type; // the type of timer's event
    int timer_group;
    int timer_idx;
    uint64_t timer_counter_value;
} timer_event_t;

xQueueHandle timer_queue;
xQueueHandle pcnt_evt_queue;              // A queue to handle pulse counter events
pcnt_isr_handle_t user_isr_handle = NULL; //user's ISR service handle



#define RMT_TICK_10_US (80000000/RMT_CLK_DIV/100000) /* RMT counter value for 10 us.(Source clock is APB clock) */
#define ITEM_DURATION(d) ((d & 0x7fff)*10/RMT_TICK_10_US)

#define PIN_TRIGGER 18
#define PIN_ECHO 19

#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisampling

uint32_t voltage;
#if CONFIG_IDF_TARGET_ESP32
static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel = ADC_CHANNEL_6;     //GPIO34 if ADC1, GPIO14 if ADC2
#elif CONFIG_IDF_TARGET_ESP32S2BETA
static const adc_channel_t channel = ADC_CHANNEL_6;     // GPIO7 if ADC1, GPIO17 if ADC2
#endif 
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;
static const uint16_t alphafonttable[] = {
    0b0100110000111111, // 0
    0b0000000000000110, // 1
    0b0000000011011011, // 2
    0b0000000010001111, // 3
    0b0000000011100110, // 4
    0b0010000001101001, // 5
    0b0000000011111101, // 6
    0b0000000000000111, // 7
    0b0000000011111111, // 8
    0b0000000011101111, // 9
};


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
                printf("receiving %c\n", rx_buffer[0]);
                flag = rx_buffer[0] - '0';
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
}

static void pcnt_example_init(void)
{
    /* Prepare configuration for the PCNT unit */
    pcnt_config_t pcnt_config = {
        // Set PCNT input signal and control GPIOs
        .pulse_gpio_num = PCNT_INPUT_SIG_IO,
        .ctrl_gpio_num = PCNT_INPUT_CTRL_IO,
        .channel = PCNT_CHANNEL_0,
        .unit = PCNT_TEST_UNIT,
        .pos_mode = PCNT_COUNT_DIS, // Count up on the positive edge
        .neg_mode = PCNT_COUNT_INC, // Keep the counter value on the negative edge

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

void IRAM_ATTR timer_group0_isr(void *para)
{
    timer_spinlock_take(TIMER_GROUP_0);
    int timer_idx = (int)para;

    /* Retrieve the interrupt status and the counter value
       from the timer that reported the interrupt */
    uint32_t timer_intr = timer_group_get_intr_status_in_isr(TIMER_GROUP_0);
    uint64_t timer_counter_value = timer_group_get_counter_value_in_isr(TIMER_GROUP_0, timer_idx);

    /* Prepare basic event data
       that will be then sent back to the main program task */
    timer_event_t evt;
    evt.timer_group = 0;
    evt.timer_idx = timer_idx;
    evt.timer_counter_value = timer_counter_value;

    /* Clear the interrupt
       and update the alarm time for the timer with without reload */
    if (timer_intr & TIMER_INTR_T0)
    {
        evt.type = TEST_WITHOUT_RELOAD;
        timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_0);
        timer_counter_value += (uint64_t)(TIMER_INTERVAL0_SEC * TIMER_SCALE);
        timer_group_set_alarm_value_in_isr(TIMER_GROUP_0, timer_idx, timer_counter_value);
    }
    else if (timer_intr & TIMER_INTR_T1)
    {
        evt.type = TEST_WITH_RELOAD;
        timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_1);
    }
    else
    {
        evt.type = -1; // not supported even type
    }

    /* After the alarm has been triggered
      we need enable it again, so it is triggered the next time */
    timer_group_enable_alarm_in_isr(TIMER_GROUP_0, timer_idx);

    /* Now just send the event data back to the main program task */
    xQueueSendFromISR(timer_queue, &evt, NULL);
    timer_spinlock_give(TIMER_GROUP_0);
}

static void example_tg0_timer_init(int timer_idx,
                                   bool auto_reload, double timer_interval_sec)
{
    /* Select and initialize basic parameters of the timer */
    timer_config_t config = {
        .divider = TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = auto_reload,
    }; // default clock source is APB
    timer_init(TIMER_GROUP_0, timer_idx, &config);

    /* Timer's counter will initially start from value below.
       Also, if auto_reload is set, this value will be automatically reload on alarm */
    timer_set_counter_value(TIMER_GROUP_0, timer_idx, 0x00000000ULL);

    /* Configure the alarm value and the interrupt on alarm. */
    timer_set_alarm_value(TIMER_GROUP_0, timer_idx, timer_interval_sec * TIMER_SCALE);
    timer_enable_intr(TIMER_GROUP_0, timer_idx);
    timer_isr_register(TIMER_GROUP_0, timer_idx, timer_group0_isr,
                       (void *)timer_idx, ESP_INTR_FLAG_IRAM, NULL);

    timer_start(TIMER_GROUP_0, timer_idx);
}
int16_t count = 0;
int16_t lastcount = 0;
float speed = 0;
static void timer_example_evt_task(void *arg)
{
    while (1)
    {
        lastcount = count;
        timer_event_t evt;
        xQueueReceive(timer_queue, &evt, portMAX_DELAY);
        speed = (float)(count - lastcount) * 2.0 / 6.0 * 21.99114858 / 100.0;
        /* Print information that the timer reported an event */
        printf("Speed: %f\n", speed);
    }
}
void PID() {
    integral = 0.00;
    previous_error = 0.00;
    while (1) {
        error = setpoint - (distance*100);

        if (error > 3) {
            mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1000);
        } else if (error < -3) {
            mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1600);
        } else if (abs(error) < 3) {
            mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1370);
        }

        integral = integral + error * dt;
        derivative = (error - previous_error) / dt;
        output = Kp * error + Ki * integral + Kd * derivative;
        previous_error = error;
        vTaskDelay(dt);
    }
}
void speed_PID() {
    float setspeed = .3;
    float speederror;
    integral = 0.00;
    previous_error = 0.00;
    while (1) {
        speederror = setspeed - speed;

        if (speederror > .1) {
            mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 1500);
        } else if (speederror < -.1) {
            mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 1500);
        } else if (abs(speederror) < .1) {
            mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 1550);
        }

        integral = integral + error * dt;
        derivative = (error - previous_error) / dt;
        output = Kp * error + Ki * integral + Kd * derivative;
        previous_error = error;
        vTaskDelay(dt);
    }
}
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
static void mcpwm_example_gpio_initialize(void)
{
    printf("initializing mcpwm servo control gpio......\n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, 25);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, 26);
    
}
void calibrateESC()
{
    printf("Configuring ESC for Crawler\n");
    vTaskDelay(3000 / portTICK_PERIOD_MS);                                // Give yourself time to turn on crawler
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 2100); // HIGH signal in microseconds
    printf("HIGH\n");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 700); // LOW signal in microseconds
    printf("LOW\n");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 1400); // NEUTRAL signal in microseconds
    printf("NEUTRAL\n");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 1400); // reset the ESC to neutral (non-moving) value
    printf("RESET\n");
}
static void HCSR04_tx_init()
{
  rmt_config_t rmt_tx;
  rmt_tx.channel = RMT_TX_CHANNEL;
  rmt_tx.gpio_num = RMT_TX_GPIO_NUM;
  rmt_tx.mem_block_num = 1;
  rmt_tx.clk_div = RMT_CLK_DIV;
  rmt_tx.tx_config.loop_en = false;
  rmt_tx.tx_config.carrier_duty_percent = 50;
  rmt_tx.tx_config.carrier_freq_hz = 3000;
  rmt_tx.tx_config.carrier_level = 1;
  rmt_tx.tx_config.carrier_en = RMT_TX_CARRIER_EN;
  rmt_tx.tx_config.idle_level = 0;
  rmt_tx.tx_config.idle_output_en = true;
  rmt_tx.rmt_mode = 0;
  rmt_config(&rmt_tx);
  rmt_driver_install(rmt_tx.channel, 0, 0);
}

static void HCSR04_rx_init()
{
  rmt_config_t rmt_rx;
  rmt_rx.channel = RMT_RX_CHANNEL;
  rmt_rx.gpio_num = RMT_RX_GPIO_NUM;
  rmt_rx.clk_div = RMT_CLK_DIV;
  rmt_rx.mem_block_num = 1;
  rmt_rx.rmt_mode = RMT_MODE_RX;
  rmt_rx.rx_config.filter_en = true;
  rmt_rx.rx_config.filter_ticks_thresh = 100;
  rmt_rx.rx_config.idle_threshold = rmt_item32_tIMEOUT_US / 10 * (RMT_TICK_10_US);
  rmt_config(&rmt_rx);
  rmt_driver_install(rmt_rx.channel, 1000, 0);
}
void IRAM_ATTR timer_isr()
{
    // Clear interrupt
    TIMERG0.int_clr_timers.t0 = 1;
    // Indicate timer has fired
    dt_complete = 1;
}

void writeRegister(uint8_t reg, uint8_t data)
{

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SLAVE_ADDR1 << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM1, cmd, 1 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    printf("Wrote %d to %d \n", data, reg);
}
uint8_t readRegister(uint8_t reg)
{
    uint8_t value;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SLAVE_ADDR1 << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SLAVE_ADDR1 << 1) | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, &value, ACK_CHECK_DIS);
    //stop command
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM1, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return value;
}
int getDistance()
{
    uint8_t val;
    uint8_t val2;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SLAVE_ADDR1 << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, (0x8f), ACK_CHECK_EN);
    i2c_master_stop(cmd);

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SLAVE_ADDR1 << 1) | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, &val, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, &val2, ACK_CHECK_DIS);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM1, cmd, 1 / portTICK_RATE_MS);

    i2c_cmd_link_delete(cmd);

    val = val << 8; // shift high byte to be high 8 bits
    val |= val2;    // receive low byte as lower 8 bits
    return val;
}
static void lidarRead()
{
    uint8_t initReg = 0x00;
    uint8_t initData = 0x04;
    uint8_t data = 0x00;
    vTaskDelay(20);
    while (1)
    {

        writeRegister(initReg, initData);
        data = readRegister(0x01);
        while ((data & 1) != 0x00)
        {
            data = readRegister(0x01);
            vTaskDelay(10);
        }
        uint8_t distHigh = readRegister(0x0f);
        uint8_t distLow = readRegister(0x10);

        printf("distHigh is: %x \n", distHigh);
        printf("distLow is: %x \n", distLow);

        lidarDistance = (distHigh << 8) + distLow;

        printf("Distance is: %d \n", lidarDistance);
        // }
        vTaskDelay(10);
    }
}
void app_main() {
  mcpwm_example_gpio_initialize();
  ESP_ERROR_CHECK(nvs_flash_init());
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  ESP_ERROR_CHECK(example_connect());
  timer_queue = xQueueCreate(10, sizeof(timer_event_t));
    example_tg0_timer_init(TIMER_0, TEST_WITHOUT_RELOAD, TIMER_INTERVAL0_SEC);

    xTaskCreate(timer_example_evt_task, "timer_evt_task", 2048, NULL, 5, NULL);
    xTaskCreate(PID, "PID", 1024*2, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(speed_PID, "speed_PID", 1024*2, NULL, configMAX_PRIORITIES, NULL);
    /* Initialize LEDC to generate sample pulse signal */
    ledc_init();

    /* Initialize PCNT event queue and PCNT functions */
    pcnt_evt_queue = xQueueCreate(10, sizeof(pcnt_evt_t));
    pcnt_example_init();
    pcnt_evt_t evt;
    portBASE_TYPE res;
    int i2c_master_port = I2C_EXAMPLE_MASTER_NUM;
    int i2c_master_port1 = I2C_EXAMPLE_MASTER_NUM1;
  i2c_config_t conf;
  conf.mode = I2C_MODE_MASTER;                              // Master mode
  conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;              // Default SDA pin
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;                  // Internal pullup
  conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;              // Default SCL pin
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;                  // Internal pullup
  conf.master.clk_speed = I2C_EXAMPLE_MASTER_FREQ_HZ;       // CLK frequency
  err = i2c_param_config(i2c_master_port, &conf);           // Configure
  if (err == ESP_OK)
    {
        printf("- parameters: ok\n");
    }
  err = i2c_driver_install(i2c_master_port, conf.mode,
                      I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
                      I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0);
  if (err == ESP_OK)
    {
        printf("- initialized: yes\n\n");
    }
  i2c_config_t conf1;
  conf1.mode = I2C_MODE_MASTER;                              // Master mode
  conf1.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO1;              // Default SDA pin
  conf1.sda_pullup_en = GPIO_PULLUP_ENABLE;                  // Internal pullup
  conf1.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO1;              // Default SCL pin
  conf1.scl_pullup_en = GPIO_PULLUP_ENABLE;                  // Internal pullup
  conf1.master.clk_speed = I2C_EXAMPLE_MASTER_FREQ_HZ;       // CLK frequency
  err = i2c_param_config(i2c_master_port1, &conf1);           // Configure
  if (err == ESP_OK)
    {
        printf("- parameters: ok\n");
    }
  err = i2c_driver_install(i2c_master_port1, conf1.mode,
                      I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
                      I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0);     
  if (err == ESP_OK)
    {
        printf("- initialized: yes\n\n");
    }              
  // i2c_set_data_mode(i2c_master_port,I2C_DATA_MODE_LSB_FIRST,I2C_DATA_MODE_LSB_FIRST);
  // Dat in MSB mode
  i2c_set_data_mode(i2c_master_port, I2C_DATA_MODE_MSB_FIRST, I2C_DATA_MODE_MSB_FIRST);
  i2c_set_data_mode(i2c_master_port1, I2C_DATA_MODE_MSB_FIRST, I2C_DATA_MODE_MSB_FIRST);
  xTaskCreate(receive_task, "receive_task", 4096, (void *)AF_INET, 5, NULL);
  mcpwm_config_t pwm_config;
    pwm_config.frequency = 50; //frequency = 50Hz, i.e. for every servo motor time period should be 20ms
    pwm_config.cmpr_a = 0;     //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;     //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);

    calibrateESC();
/// Define I2C configurations
  ret = alpha_oscillator();
  // Set display blink off
  ret = no_blink();
  ret = set_brightness_max(0xF);
  HCSR04_tx_init();
  HCSR04_rx_init();
  rmt_item32_t item;
  item.level0 = 1;
  item.duration0 = RMT_TICK_10_US;
  item.level1 = 0;
  item.duration1 = RMT_TICK_10_US; // for one pulse this doesn't matter
  rmt_item32_t item1;
  item1.level0 = 1;
  item1.duration0 = RMT_TICK_10_US;
  item1.level1 = 0;
  item1.duration1 = RMT_TICK_10_US;
  float distance = 0;
  size_t rx_size = 0;
  RingbufHandle_t rb = NULL;
  if (unit == ADC_UNIT_1) {
        adc1_config_width(ADC_WIDTH_BIT_10);
        adc1_config_channel_atten(channel, atten);
    } else {
        adc2_config_channel_atten((adc2_channel_t)channel, atten);
    }
  rmt_get_ringbuf_handle(RMT_RX_CHANNEL, &rb);
  rmt_rx_start(RMT_RX_CHANNEL, 1);
  adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
  esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
  xTaskCreate(lidarRead, "lidarRead", 4096, NULL, 5, NULL);

while(1){
    
    if (flag == 1)
    {
        printf("flag is 1\n");
        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 1650);
    } else {
        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 1400);
        printf("flag is 0\n");
    }
    pcnt_evt_queue = xQueueCreate(10, sizeof(pcnt_evt_t));

    res = xQueueReceive(pcnt_evt_queue, &evt, 500 / portTICK_PERIOD_MS);
    pcnt_get_counter_value(PCNT_TEST_UNIT, &count);

    uint32_t adc_reading = 0;
   for (int i = 0; i < NO_OF_SAMPLES; i++) {
            if (unit == ADC_UNIT_1) {
                adc_reading += adc1_get_raw((adc1_channel_t)channel);
            } else {
                int raw;
                adc2_get_raw((adc2_channel_t)channel, ADC_WIDTH_BIT_12, &raw);
                adc_reading += raw;
            }
        }
        adc_reading /= NO_OF_SAMPLES;
        // uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
        // //float x = (float)(voltage-75)*2.54/6.4;
        // //float y = adc_reading*5.0/10.0;
        // float y = adc_reading * .5;
        // printf("Right Distance: %f cm\t\n", y);
  rmt_write_items(RMT_TX_CHANNEL, &item, 1, true);
  rmt_wait_tx_done(RMT_TX_CHANNEL, portMAX_DELAY);
  rmt_write_items(3, &item1, 1, true);
  rmt_wait_tx_done(2, portMAX_DELAY);

  rmt_item32_t* item = (rmt_item32_t*)xRingbufferReceive(rb, &rx_size, 1000);
  distance = 340.29 * ITEM_DURATION(item->duration0) / (1000 * 1000 * 2); // distance in meters
  if (distance*100 < 3)
  {
    distance = .3;
  }
  
  printf("Left Distance is %f cm\n", distance * 100); // distance in centimeters
  vRingbufferReturnItem(rb, (void*) item);
  vTaskDelay(30 / portTICK_RATE_MS);
  // rmt_item32_t* item1 = (rmt_item32_t*)xRingbufferReceive(rb1, &rx_size1, 1000);
  // distance1 = 340.29 * ITEM_DURATION(item1->duration0) / (1000 * 1000 * 2); // distance in meters
  // printf("Right Distance is %f cm\n", distance1 * 100); // distance in centimeters
  // vRingbufferReturnItem(rb1, (void*) item1);
  // 
 if (distance*100 < 15 && flag==1) {
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1000);
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 1550);
    displaybuffer[0] = alphafonttable[0];
  displaybuffer[1] = alphafonttable[1]; 
  displaybuffer[2] = alphafonttable[5]; 
  displaybuffer[3] = 0b0000000000000000; 
  } else {
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1370);
    displaybuffer[0] = alphafonttable[0];
    displaybuffer[1] = alphafonttable[3]; 
    displaybuffer[2] = alphafonttable[1]; 
    displaybuffer[3] = 0b0000000000000000; 
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
  ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd4, 500 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd4);
  vTaskDelay(100 / portTICK_PERIOD_MS);
}

}