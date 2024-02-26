#include <stdio.h>
#include <esp_log.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "freertos/semphr.h"
#include "freertos/queue.h"
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
// #define ANALOG_OUTPUT   1
#define DIGITAL_INPUT   1
// #define ANALOG_INPUT    1
// #define INTERRUPT       1
// #define TIMERS          1
// #define WIFI_CON        1
// #define RTOS            1

static const char *logger = "LOG";

// 27 elements

// This is the button that works
#define BUTTON_PIN GPIO_NUM_34

static xQueueHandle gpio_evt_queue = NULL; // gpio event queue

static void IRAM_ATTR gpio_isr_handler(void *arg) {
    uint32_t gpio_num = (uint32_t)arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void button_task(void* arg) {
    uint32_t io_num;
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            ESP_LOGI("BUTTON", "Button pressed on GPIO %d", io_num);
        }
    }
}

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
    
#elif DIGITAL_INPUT

// GPIO_34 was used as the pin for it to work
// Internal pull up resistors were used
// Note: GPIO_0 is used for reset

    gpio_config_t io_conf={
        .pin_bit_mask=(1ULL << BUTTON_PIN),
        .mode=GPIO_MODE_INPUT,
        .intr_type=GPIO_INTR_NEGEDGE,
        .pull_up_en=GPIO_PULLUP_ENABLE,
        .pull_down_en=GPIO_PULLDOWN_DISABLE,
    };
    gpio_config(&io_conf);

    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));

    xTaskCreate(button_task, "button_task", 2048, NULL, 10, NULL);
    
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_PIN, gpio_isr_handler, (void*) BUTTON_PIN);

    ESP_LOGI("MAIN", "Interrupt-based button");

#elif ANALOG_INPUT
#endif
} 