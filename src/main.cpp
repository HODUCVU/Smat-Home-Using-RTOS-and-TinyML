#include <Arduino.h>
// #include <TaskScheduler.h>
#include <FreeRTOSConfig.h>
// #include <freertos/FreeRTOS.h>
// #include "freertos/task.h"
// Define PIN
#define LED_1 18
#define LED_2 19
#define LED_3 21

// Initialization function
void initPort();
void blink_task_LED1(void *pvParameter);
void blink_task_LED2(void *pvParameter);
void blink_task_LED3(void *pvParameter);

void setup() {

  Serial.begin(115200);
  pinMode(LED_1, OUTPUT);
  pinMode(LED_2, OUTPUT);
  pinMode(LED_3, OUTPUT);

  xTaskCreate(&blink_task_LED1, "Blink led 1", 512,NULL,1,NULL);
  xTaskCreate(&blink_task_LED2, "Blink led 2", 512,NULL,2,NULL);
  xTaskCreate(&blink_task_LED3, "Blink led 2", 512,NULL,3,NULL);
}

void loop() {}

void blink_task_LED1(void *pvParameter)
{
    while(1) {
        /* Blink off (output low) */
        digitalWrite(LED_1, HIGH);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        /* Blink on (output high) */
        digitalWrite(LED_1, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
void blink_task_LED2(void *pvParameter)
{
    while(1) {
        /* Blink off (output low) */
        digitalWrite(LED_2, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        /* Blink on (output high) */
        digitalWrite(LED_2, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
void blink_task_LED3(void *pvParameter)
{
    while(1) {
        /* Blink off (output low) */
        digitalWrite(LED_3, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        /* Blink on (output high) */
        digitalWrite(LED_3, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}