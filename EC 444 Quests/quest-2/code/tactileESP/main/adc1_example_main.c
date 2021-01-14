//Yanni Pang Sam Krasnoff Abd el Hussain

#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/i2c.h"
#include "driver/uart.h"
#include "esp_vfs_dev.h"
#include <math.h>

#if CONFIG_IDF_TARGET_ESP32
#include "esp_adc_cal.h"
#endif

#define DEFAULT_VREF 1100 //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES 20  //Multisampling

float tempGlobal = 0;
float irGlobal = 0;
float usGlobal = 0;

static esp_adc_cal_characteristics_t *adc_chars;

#if CONFIG_IDF_TARGET_ESP32
static void check_efuse(void)
{
    //Check TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK)
    {
        printf("eFuse Two Point: Supported\n");
    }
    else
    {
        printf("eFuse Two Point: NOT supported\n");
    }

    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK)
    {
        printf("eFuse Vref: Supported\n");
    }
    else
    {
        printf("eFuse Vref: NOT supported\n");
    }
}

static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP)
    {
        printf("Characterized using Two Point Value\n");
    }
    else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF)
    {
        printf("Characterized using eFuse Vref\n");
    }
    else
    {
        printf("Characterized using Default Vref\n");
    }
}
#endif
void thermistor()
{
    while (1)
    {
#if CONFIG_IDF_TARGET_ESP32
        //Check if Two Point or Vref are burned into eFuse
        check_efuse();
#endif

        //Configure ADC
        adc_channel_t channel = ADC1_CHANNEL_3;
        adc_atten_t atten = ADC_ATTEN_DB_11;

        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(channel, atten);

#if CONFIG_IDF_TARGET_ESP32
        //Characterize ADC
        adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
        esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
        print_char_val_type(val_type);
#endif

        //Continuously sample ADC1
        while (1)
        {
            uint32_t adc_reading = 0;
            //Multisampling
            for (int i = 0; i < NO_OF_SAMPLES; i++)
            {
                adc_reading += adc1_get_raw(channel);
            }
            adc_reading /= NO_OF_SAMPLES;
#if CONFIG_IDF_TARGET_ESP32
            //Convert adc_reading to voltage in mV
            uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);

            //Thermistor Begin
            float resistance = (10000 * (3.3 - voltage / 1000.0)) / (voltage / 1000.0);
            float temp = ((1.0 / ((1.0 / 298.15) + (1.0 / 3435.0) * log(resistance / 10000.0))) - 273.15);
            // printf("Temp: %.2f °C\n", temp);

            tempGlobal = temp;
            //Thermistor End

#endif
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}
void irfinder()
{
    while (1)
    {
#if CONFIG_IDF_TARGET_ESP32
        //Check if Two Point or Vref are burned into eFuse
        check_efuse();
#endif

        //Configure ADC
        adc_channel_t channel = ADC1_CHANNEL_6;
        adc_atten_t atten = ADC_ATTEN_DB_11;

        adc1_config_width(ADC_WIDTH_BIT_10);
        adc1_config_channel_atten(channel, atten);

#if CONFIG_IDF_TARGET_ESP32
        //Characterize ADC
        adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
        esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, atten, ADC_WIDTH_BIT_10, DEFAULT_VREF, adc_chars);
        print_char_val_type(val_type);
#endif

        //Continuously sample ADC1
        while (1)
        {
            uint32_t adc_reading = 0;
            //Multisampling
            for (int i = 0; i < NO_OF_SAMPLES; i++)
            {

                adc_reading += adc1_get_raw(channel);
            }
            adc_reading /= NO_OF_SAMPLES;
#if CONFIG_IDF_TARGET_ESP32

            //IR Rangefinder Begin
            float dist = 6797.0 / (((float)adc_reading) - 3.0) - 4.0;
            // printf("Distance: %fcm\n", dist);
            irGlobal = dist;
            //IR Rangefinder End

#endif
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}
void ultrasonic()
{
    while (1)
    {
#if CONFIG_IDF_TARGET_ESP32
        //Check if Two Point or Vref are burned into eFuse
        check_efuse();
#endif

        //Configure ADC
        adc_channel_t channel = ADC1_CHANNEL_0;

        adc1_config_width(ADC_WIDTH_BIT_10);
        adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_0);

        while (1)
        {
            uint32_t adc_reading = 0;
            //Multisampling
            for (int i = 0; i < 100; i++)
            {

                adc_reading += adc1_get_raw(channel);
            }
            adc_reading /= NO_OF_SAMPLES;
#if CONFIG_IDF_TARGET_ESP32

            float distance = adc1_get_raw(channel) / 2 * 2.54;
            usGlobal = distance;
#endif

            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}
void printToConsole()
{
    while (1)
    {
        printf("{ \"temp\":\"%.2f\",", tempGlobal);
        printf(" \"ir\":\"%.2f\",", irGlobal/100);
        printf(" \"us\":\"%.2f\"}\n", usGlobal/100);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}
void app_main(void)
{
    xTaskCreate(thermistor, "thermistor", 4096, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(irfinder, "irfinder", 4096, NULL, configMAX_PRIORITIES - 1, NULL);
    xTaskCreate(ultrasonic, "ultrasonic", 4096, NULL, configMAX_PRIORITIES - 2, NULL);
    xTaskCreate(printToConsole, "printToConsole", 4096, NULL, configMAX_PRIORITIES - 3, NULL);
}
