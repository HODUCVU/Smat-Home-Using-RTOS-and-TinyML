#include <Arduino.h>
// #include <TaskScheduler.h>
#include <FreeRTOSConfig.h>
// Define PIN
#define LED_1 2

// Initialization function
void initPort();
void blink_task_LED1(void *pvParameter);
void printS(void *pvParameter);

void setup() {

  Serial.begin(9600);
  pinMode(LED_1, OUTPUT);

  xTaskCreate(&blink_task_LED1, "Blink led 1", 128,NULL,2,NULL);
  xTaskCreate(&printS, "Hello", 128, NULL, 1, NULL);
}

void loop() {}

void blink_task_LED1(void *pvParameter)
{
    while(1) {
        /* Blink off (output, low); */
        digitalWrite(LED_1, HIGH);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        /* Blink on (output, high);  */
        digitalWrite(LED_1, LOW);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
void printS(void * pvParameter) {
  while(1) {
    Serial.print("Hello\n");
    vTaskDelay(1500/portTICK_PERIOD_MS);
  }
} 
