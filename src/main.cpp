#include <Arduino.h>
#include <Wire.h>
#include <FreeRTOSConfig.h>
#include <Ticker.h>
#include <DHT.h>
#include <Adafruit_Sensor.h>
// Voice regconition
// #include <driver/i2s.h>
// #include <arduinoFFT.h>
// #include <Adafruit_NeoPixel.h>
// #include <TensorFlowLite_ESP32.h>
// #include <tensorflow/lite/micro/all_ops_resolver.h>
// #include <tensorflow/lite/micro/micro_error_reporter.h>
// #include <tensorflow/lite/micro/micro_interpreter.h>
// #include <tensorflow/lite/schema/schema_generated.h>
// #include <tensorflow/lite/version.h>
// #include <model.h>
//
// #define I2S_PORT I2S_NUM_1
// Output pin
#ifndef LIGHTPIN 
#define LIGHTPIN 4
#endif
#ifndef FANPIN 
#define FANPIN 5
#endif
// Input pin
#ifndef DHTPIN 
#define DHTPIN 26
#endif
#ifndef DHTTYPE 
#define DHTTYPE DHT22
#endif
#ifndef MQ135PIN
#define MQ135PIN 25
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
// Initialization global values
DHT dht(DHTPIN, DHTTYPE);

// =========================================================
/* 
 * Sensors 
 => temp < 25 (save into queues) -> light on else light off 
 => temp > 35 -> fan on.
 => smoking > 2700 (save into queues) -> fan on.
 => If sensor error in reading values => suppend task of light and fan.

 * Voice
*/
// =========================================================
class Sensors {
protected:
  int _pin;
  QueueHandle_t _queuesReading = NULL;
public:
  Sensors() {}
  Sensors(int pin) {
    Serial.println("Initialization Sensor");
    _pin = pin;
    _queuesReading = xQueueCreate(10, sizeof(float));  
    Serial.println("Complete init Sensor");
  }
  Sensors(int pin, int sizeQueues) {
    Serial.print("Initialization Sensor");
    _pin = pin;
    _queuesReading = xQueueCreate(sizeQueues, sizeof(float));  
    Serial.println("Complete init Sensor");
  }
  ~Sensors() {
    // print delete Sensors
  }
  void virtual readValue() = 0;
  void virtual taskHandleSensor(void *pvParameter) = 0;

  void sendQueuesHandle(float value) {
    try {
    if(_queuesReading != NULL) {
      xQueueSend(_queuesReading, (void*)&value,(TickType_t) 0);
    }
    } catch (std::exception &e) {
      Serial.print("Error in 95: ");
      Serial.print(e.what());
    }
  }
  /* void virtual receiveQueuesHandle(); */
  QueueHandle_t getQueuesHandle() {
    return _queuesReading;
  }
};
class Objects {
protected:
  int _pin;
  bool _state = false;
  TaskHandle_t _handle = NULL;
public:
  Objects(int pin){
    _pin = pin;
    pinMode(_pin, OUTPUT);
  }
  /* void virtual setup(); */
  void virtual taskHandle(void *pvParameter) = 0;
  void virtual read() = 0;
  void virtual detectVoice() = 0;
  void virtual controllHandle() = 0;
};

class Temperature : public Sensors {
private:
  float _temp = 0.0;
public:
  Temperature(){}
  Temperature(int pin) : Sensors(pin) {
    Serial.println("Initialization Temperature");
    dht.begin();
    Serial.println("Complete Temperature");
  }
  Temperature(int pin, int sizeQueues) : Sensors(pin, sizeQueues) {
    Serial.println("Initialization Temperature");
    dht.begin();
    Serial.println("Complete Temperature");
  }
  void readValue() override {
    _temp = dht.readTemperature();
  }
  float getTemp() const {
    return _temp;
  }
  void taskHandleSensor(void *pvParameter) override {
    try {
      Serial.println("9");
      while(true) {
        _temp = dht.readTemperature();
        Serial.print("Temperature: ");
        Serial.println(_temp);
        Serial.println("10");
        if(!isnan(_temp)) {
          Serial.println("11");
          sendQueuesHandle(_temp);
          Serial.println("12");
        }
        vTaskDelay(500/portTICK_PERIOD_MS);
      }
    }
    catch (std::exception &e) {
      Serial.print("Error in 127: ");
      Serial.print(e.what());
    }
  }
  ~Temperature() {}
};


class Smoking : public Sensors {
public:
  Smoking(int pin) : Sensors(pin) {}
  Smoking(int pin, int sizeQueues) : Sensors(pin, sizeQueues) {}
  ~Smoking() {}

  void readValue() {}
  void taskHandleSensor(void *pvParameter) {

  }
  void sendQueuesHandle() {}
  /* void receiveQueuesHandle() {} */
};

class Light : public Objects {
public:
  Light(int pin) : Objects(pin) {
    // setup pin mode and so on.
  }
  ~Light() {
    // free QueueHandle_t and so on.
  }

  void taskHandle(void *pvParameter) {}
  void read(){}
  void detectVoice(){}
  void controllHandle(){}

};
class Fan : public Objects {
public:
  Fan(int pin) : Objects(pin) {
    // setup pin mode and so on.
  }
  ~Fan() {
    // free QueueHandle_t and so on.
  }

  void taskHandle(void *pvParameter) {}
  void read(){}
  void detectVoice(){}
  void controllHandle(){}
};

Temperature* tempOB = NULL;
static void taskWrapper(void *pvParameter) {
    try {
      Serial.println("6");
      if (tempOB == NULL) {
                Serial.println("Error: globalTempSensor is NULL");
                vTaskDelete(NULL);
                return;
      }
    Serial.println("7");
    tempOB->taskHandleSensor(pvParameter);
    Serial.println("8");
    } catch (std::exception &e) {
      Serial.print("Error in 160: ");
      Serial.print(e.what());
    }
  }
void setup() {
  Serial.begin(115200);
  Serial.println("1");
  try {
  tempOB = new Temperature(DHTPIN);
  Serial.println("2");
  if (tempOB->getQueuesHandle() == NULL) { // || smokingReading == NULL
    Serial.println("queue NULL");
  } else {
    Serial.println("3");
    xTaskCreate(taskWrapper, "Reading temperature", 2048, NULL, 3, NULL);
    /* xTaskCreate(taskSmokeDetect, "Smoking Detect", 2048, NULL, 2, NULL); */
    /* xTaskCreate(taskControlFan, "Controller Fan", 1024, NULL, 1, &handle_Fan ); */
    /* xTaskCreate(taskControlLight, "Controller Light", 1024, NULL, 1, &handle_Light); */
    Serial.println("4");
  }
  } catch (std::exception &e) {
      Serial.println("Error in 232: ");
      Serial.println(e.what());
  }
  Serial.println("5");

}

void loop() {
  vTaskDelay(1000/portTICK_PERIOD_MS);
}

// void taskSmokeDetect(void *pvParameter) {
// // Read humidity
// sensors_event_t event;
// float temperature = 25.0, humidity = 25.0; 
// float correctedRZero =0.0, resistance = 0.0, correctedPPM = 0.0;
// while(true) {
//   dht.humidity().getEvent(&event);
//   if(isnan(event.relative_humidity)) {
//     Serial.println(F("Error reading humidity"));
//     // return; //stop 
//     vTaskSuspend(handle_Fan);
//     vTaskDelay(1000/portTICK_PERIOD_MS);
//     continue;
//   }
//   Serial.println("Readed humidity, Waiting for tempReading!");
//   humidity = event.relative_humidity;
//   // tempQueues take
//   if(tempReading != NULL) {
//     if(xQueueReceive(tempReading, &(temperature), (TickType_t) 10 ) ==pdPASS)
//     {
//       correctedRZero = mq135_sensor.getCorrectedRZero(temperature, humidity);
//       resistance = mq135_sensor.getResistance();
//       correctedPPM = mq135_sensor.getCorrectedPPM(temperature, humidity);
//       // Queues Give values (I think we will use ppm for detect smoke)
//       xQueueSend(smokingReading, (void*)&correctedPPM, (TickType_t) 0);
//     }
//   }
//   Serial.println("Smoking Detect: ");
//   Serial.print("Rzero: ");
//   Serial.println(correctedRZero);
//   Serial.print("Resistance: ");
//   Serial.println(resistance);
//   Serial.print("PPM: ");
//   Serial.println(correctedPPM);
//   vTaskDelay(500/portTICK_PERIOD_MS);
// }
// }
// void taskControlLight(void *pvParameter) {
//   // When temperature is cool -> On else Off
//   float temperature;
//   while(true){
//     if(tempReading != NULL) {
//       if(xQueueReceive(tempReading, &(temperature), (TickType_t) 10 ) ==pdPASS)
//       {
//         if(temperature < float(20)) {
//           if(!lightStatus) {
//             digitalWrite(LIGHTPIN, HIGH);
//             lightStatus = true;
//             Serial.println(F("Light on"));
//           }
//         } else {
//           if(lightStatus) {
//             digitalWrite(LIGHTPIN, LOW);
//             lightStatus = false;
//             Serial.println(F("Light off"));
//           }
//         }
//       }
//     }
//     // When voice "On" and "Off" and "Stop"
//     // "Stop" to suppend TaskHandle 
//     vTaskDelay(200/portTICK_PERIOD_MS);
//   }
// }
// void taskControlFan(void *pvParameter) {
//   // Air is polluted or temperature is hot -> On else Off
//   float temperature, airCondition;
//   while(true){
//     // Take temperature
//     if(tempReading != NULL) {
//       if(xQueueReceive(tempReading, &(temperature), (TickType_t) 10 ) ==pdPASS)
//       {
//         // temperature in home higher 30 °C
//         if(temperature > float(30)) {
//           if(!fanStatus) {
//             digitalWrite(FANPIN, HIGH);
//             fanStatus = true;
//             Serial.println(F("Fan on"));
//           }
//         } else {
//           if (fanStatus) {
//             digitalWrite(FANPIN, LOW);
//             fanStatus = false;
//             Serial.println(F("Fan off"));
//           }
//         }
//       }

//     }
//     // Take air condition in home
//     // Code same with temperature, but I don't want to wait to wear both tempReading and smokingReading at the same time
//     if(smokingReading != NULL) {
//       if(xQueueReceive(smokingReading, &(airCondition), (TickType_t) 10) == pdPASS) {
//         if(airCondition >= float(0.5)) {
//           if(!fanStatus) {
//             digitalWrite(FANPIN, HIGH);
//             fanStatus = true;
//             Serial.println(F("Fan on"));
//           }       
//         } else {
//           if (fanStatus) {
//             digitalWrite(FANPIN, LOW);
//             fanStatus = false;
//             Serial.println(F("Fan off"));
//           }
//         }
//       }
//     }
//     // When voice "Activate" and "Deactivate"
//     // "Stop" to suppend TaskHandle 
//     vTaskDelay(200/portTICK_PERIOD_MS);
//   }
// }
