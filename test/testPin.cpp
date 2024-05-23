#include <Arduino.h>
#include <Wire.h>
#ifndef LED_BUILTIN
    #define LED_BUILTIN 2
#endif
#ifndef FAN
    #define FAN 5
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
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.println("Hello, ESP32!");
}

void loop() {
    digitalWrite(FAN, LOW);
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.printf("OFF\n");
    delay(5000);
    digitalWrite(FAN, HIGH);
    digitalWrite(LED_BUILTIN, LOW);
    Serial.printf("ON\n");
    delay(5000);
}

// ; PlatformIO Project Configuration File
// ;
// ;   Build options: build flags, source filter
// ;   Upload options: custom upload port, speed and extra flags
// ;   Library options: dependencies, extra library storages
// ;   Advanced options: extra scripting
// ;
// ; Please visit documentation for the other options and examples
// ; https://docs.platformio.org/page/projectconf.html

// [env:esp32doit-devkit-v1]
// platform = espressif32
// board = esp32doit-devkit-v1
// framework = arduino
// monitor_speed = 115200
// lib_deps = 
// 	adafruit/DHT sensor library@^1.4.6
// 	phoenix1747/MQ135@^1.1.1
// 	tanakamasayuki/TensorFlowLite_ESP32@^1.0.0
// 	adafruit/Adafruit NeoPixel@^1.12.0
// 	kosme/arduinoFFT@^2.0.1
