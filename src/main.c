#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/gpio.h"

// Define PIN
#define LED_1 18
#define LED_2 19
#define LED_3 21

// Initialization function
void initPort();
void blink_task_LED1(void *pvParameter);
void blink_task_LED2(void *pvParameter);
void blink_task_LED3(void *pvParameter);

void app_main()
{
    // Init Port
    initPort();
    // Init task
    xTaskCreate(&blink_task_LED1, "blink LED 1", 512, NULL, 1, NULL);
    xTaskCreate(&blink_task_LED2, "blink LED 2", 512, NULL, 2, NULL);
    xTaskCreate(&blink_task_LED3, "blink LED 3", 512, NULL, 3, NULL);
}

void initPort() {
    // PIN 18
    esp_rom_gpio_pad_select_gpio(LED_1);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(LED_1, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_1, 0);
    // PIN 19
    esp_rom_gpio_pad_select_gpio(LED_2);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(LED_2, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_2, 0);
    // PIN 21
    esp_rom_gpio_pad_select_gpio(LED_3);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(LED_3, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_3, 0);
}
void blink_task_LED1(void *pvParameter)
{
    while(1) {
        /* Blink off (output low) */
        gpio_set_level(LED_1, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        /* Blink on (output high) */
        gpio_set_level(LED_1, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
void blink_task_LED2(void *pvParameter)
{
    while(1) {
        /* Blink off (output low) */
        gpio_set_level(LED_2, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        /* Blink on (output high) */
        gpio_set_level(LED_2, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
void blink_task_LED3(void *pvParameter)
{
    while(1) {
        /* Blink off (output low) */
        gpio_set_level(LED_3, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        /* Blink on (output high) */
        gpio_set_level(LED_3, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}