#include <Arduino.h>
#include <Wire.h>
#include <DHT.h>
#include <Adafruit_Sensor.h>

#ifndef LED_BUILTIN
    #define LED_BUILTIN 2
#endif
#ifndef FAN
    #define FAN 5
#endif
#ifndef DHTPIN
    #define DHTPIN 26
#endif
#ifndef DHTTYPE
    #define DHTTYPE 22
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

DHT dht(DHTPIN, DHTTYPE);

void setup() {
    Serial.begin(115200);
    pinMode(FAN, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    dht.begin(); 
    Serial.println("Hello, ESP32!");
}

void loop() {

    float h = dht.readHumidity(); //Đọc độ ẩm
    float t = dht.readTemperature(); //Đọc nhiệt độ
    Serial.print("Nhiet do: ");
    Serial.println(t); //Xuất nhiệt độ
    Serial.print("Do am: ");
    Serial.println(h); //Xuất độ ẩm
 
    Serial.println(); //Xuống hàng
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(1000);
}
