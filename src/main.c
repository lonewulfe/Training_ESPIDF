#include <stdio.h>
#include <esp_log.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "driver/gpio.h"
#include "driver/ledc.h"

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_TIMER_BIT          LEDC_TIMER_10_BIT
#define LEDC_CHANNEL_BIT        LEDC_CHANNEL_10_BIT

#define LED_OUTPUT              (19)                // Might wanna change this
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT   // Set duty resolution to 13 bits
#define LEDC_DUTY               (4095)              // Set duty to 50%. ((2 ** 13) - 1) * 50% = 4095
#define LEDC_FREQUENCY          (5000)              // Frequency in Hertz. Set frequency at 5 kHz


#define BUILTIN_LED 0

// Enable this for using the LED controls with struct
// #define LOGGING         1
// #define DIGITAL_OUTPUT  1
#define ANALOG_OUTPUT   1
// #define DIGITAL_INPUT   1
// #define ANALOG_INPUT    1
// #define INTERRUPT       1
// #define TIMERS          1
// #define WIFI_CON        1
// #define RTOS            1

static const char *logger = "LOG";

// 27 elements

void app_main()
{
    ESP_LOGI(logger, "Started");

#ifdef DIGITAL_OUTPUT
    // This code block using the normal API methods to configure 
    // the pin
    gpio_config_t led = {};
    led.intr_type = GPIO_INTR_DISABLE;
    led.mode = GPIO_MODE_OUTPUT;
    led.pin_bit_mask = GPIO_SEL_0;
    led.pull_up_en = 0;
    led.pull_down_en = 0;

    if(gpio_config(&led) != 0)
    {
        ESP_LOGI(logger, "GPIO CONFIG ERROR");
    }

    // This code block makes it blink
    for(int i = 0; i < 10; i++){
        vTaskDelay(1000/portTICK_PERIOD_MS);
        ESP_LOGI(logger, "Switching On");
        gpio_set_level(GPIO_NUM_0, 1);
        vTaskDelay(1000/portTICK_PERIOD_MS);
        ESP_LOGI(logger, "Switching Off");
        gpio_set_level(GPIO_NUM_0, 0);
    }

#elif ANALOG_OUTPUT
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_BIT,
        .freq_hz = 5000,
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .channel = LEDC_CHANNEL,
        .duty = 0,
        .gpio_num = GPIO_NUM_0,
        .speed_mode = LEDC_MODE,
        .timer_sel = LEDC_TIMER
    };
    ledc_channel_config(&ledc_channel);

    while (1) {
        for(int duty = 0; duty <= (1 << LEDC_TIMER_BIT) - 1; duty++){
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
        
        for(int duty = (1 << LEDC_TIMER_BIT) - 1; duty >= 0; duty--){
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
#elif ANALOG_INPUT

#endif
} 