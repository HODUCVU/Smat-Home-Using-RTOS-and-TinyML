#include <Arduino.h>
#include <Wire.h>
#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif
#ifndef FAN
#define FAN 5
#endif
#ifndef MQ135
#define MQ135 25
#endif
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif

/* Using core 1 of ESP32 */
#if CONFIG_FREERTOS_UNICORE
static const BaseType_t app_cpu = 0;
#else
static const BaseType_t app_cpu = 1;
#endif

void setup() {
  Serial.begin(115200);
  pinMode(FAN, OUTPUT);
  Serial.println("Hello, ESP32!");
}

void loop() {
  int smoke = analogRead(MQ135);
  Serial.print("Smoke: ");
  Serial.println(smoke);
  if(smoke > 2700) {
    Serial.println("Smoke active");
    digitalWrite(FAN, HIGH);
  } else {
    Serial.println("Smoke deactive");
    digitalWrite(FAN, LOW);
  }
  delay(1000);
}
