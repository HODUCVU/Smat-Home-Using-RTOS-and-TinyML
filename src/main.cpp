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
#define LIGHTPIN 5
#endif
#ifndef FANPIN 
#define FANPIN 4
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
 => smoking > 2700 (save into queues) -> fan on. -> light blink
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
    // Serial.println("Initialization Sensor");
    _pin = pin;
    _queuesReading = xQueueCreate(10, sizeof(float));  
    // Serial.println("Complete init Sensor");
  }
  Sensors(int pin, int sizeQueues) {
    // Serial.print("Initialization Sensor");
    _pin = pin;
    _queuesReading = xQueueCreate(sizeQueues, sizeof(float));  
    // Serial.println("Complete init Sensor");
  }
  ~Sensors() {
    // print delete Sensors
  }
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
  QueueHandle_t getQueuesHandle() {
    return _queuesReading;
  }
};
class Temperature : public Sensors {
private:
  float _temp = 0.0;
public:
  Temperature(){}
  Temperature(int pin) : Sensors(pin) {
    // Serial.println("Initialization Temperature");
    dht.begin();
    // Serial.println("Complete Temperature");
  }
  Temperature(int pin, int sizeQueues) : Sensors(pin, sizeQueues) {
    // Serial.println("Initialization Temperature");
    dht.begin();
    // Serial.println("Complete Temperature");
  }
  float getTemp() {
    return _temp;
  }
  void taskHandleSensor(void *pvParameter) override {
    try {
      while(true) {
        _temp = dht.readTemperature();
        Serial.print("Temperature: ");
        Serial.println(_temp);
        if(!isnan(_temp)) {
          // Serial.println("queue-temp");
          sendQueuesHandle(_temp);
        }
        vTaskDelay(500/portTICK_PERIOD_MS);
      }
    }
    catch (std::exception &e) {
      Serial.print("Error in 159: ");
      Serial.println(e.what());
    }
  }
  ~Temperature() {
    // delete queues
  }
};
class Smoking : public Sensors {
private:
  float _smoking = 0.0;
public:
  Smoking() {}
  Smoking(int pin) : Sensors(pin) {
    // Serial.println("Initialization Smoking");
  }
  Smoking(int pin, int sizeQueues) : Sensors(pin, sizeQueues) {
    // Serial.println("Initialization Smoking");
  }
  ~Smoking() {
    // delete queues
  }
  float getSmoking() {
    return _smoking;
  }
  void taskHandleSensor(void *pvParameter) override {
    try {
      while(true) {
        _smoking = analogRead(MQ135PIN);
        Serial.print("Smoking: ");
        Serial.println(_smoking);
        if(!isnan(_smoking)) {
          // Serial.println("queue-smoking");
          sendQueuesHandle(_smoking);
        }
        vTaskDelay(500/portTICK_PERIOD_MS);
      }
    } catch(std::exception& e) {
      Serial.print("Error in 187: ");
      Serial.println(e.what());
    }

  }
};

Temperature* tempOB = NULL;
Smoking *smokingOB = NULL;

class Objects {
protected:
  int _pin;
  bool _stateForTemp = false;
  bool _stateForSmoking = false;
public:
  TaskHandle_t _handle = NULL;
  Objects(){}
  Objects(int pin){
    _pin = pin;
    pinMode(_pin, OUTPUT);
    digitalWrite(_pin, LOW);
  }
  // void recievequeue(Temperature temp) and Smoking smoking.
  void virtual taskHandleObjectTemp(void *pvParameter) = 0;
  void virtual taskHandleObjectSmoking(void *pvParameter) = 0;
  void virtual detectVoice() = 0;
  void virtual controllHandle() = 0;
};
class Light : public Objects {
private:
  float _tempForLight = 0.0;
  float _smokingForLight = 0.0;
public:
  Light() {}
  Light(int pin) : Objects(pin) {
    // setup pin mode and so on.
    // Serial.println("Initialization Light");
  }
  ~Light() {
    // free QueueHandle_t and so on.
  }

  void taskHandleObjectTemp(void *pvParameter) override {
    try {
      while(true) {
        if (tempOB == NULL) {
                Serial.println("Error: globalTempSensor is NULL");
                vTaskDelete(NULL);
                return;
        }
        xQueueReceive(tempOB->getQueuesHandle(),(void *)&_tempForLight,portMAX_DELAY);
        // Serial.print("Value _tempForLight: ");
        // Serial.println(_tempForLight);
        // temp < 25
        if(_tempForLight <= 31 && !_stateForTemp) {
          digitalWrite(_pin, HIGH);
          Serial.println("Light is on");
          _stateForTemp = true;
        } else if (_stateForTemp && !_stateForSmoking) {
          digitalWrite(_pin, LOW);
          Serial.println("Light is off");
          _stateForTemp = false;
        }
        vTaskDelay(200/portTICK_PERIOD_MS);
      }
    }catch (std::exception& e) {
      Serial.print("Error in LIGHT: ");
      Serial.println(e.what());
    }
  }
  void taskHandleObjectSmoking(void *pvParameter) override {
    try {
      while(true) {
        if (smokingOB == NULL) {
                Serial.println("Error: globalSmokingSensor is NULL");
                vTaskDelete(NULL);
                return;
        }
        xQueueReceive(smokingOB->getQueuesHandle(),(void *)&_smokingForLight,portMAX_DELAY);
        // Serial.print("Value _smokingForLight: ");
        // Serial.println(_smokingForLight);
        if(_smokingForLight >= 1700 && !_stateForSmoking) {
          digitalWrite(_pin, !digitalRead(_pin));
          Serial.println("Light is blinking");
          _stateForSmoking = true;
        } else if (_stateForSmoking && !_stateForTemp) {
          digitalWrite(_pin, LOW);
          Serial.println("Light is off");
          _stateForSmoking = false;
        }
        vTaskDelay(500/portTICK_PERIOD_MS);
      }
    }catch (std::exception& e) {
      Serial.print("Error in LIGHT: ");
      Serial.println(e.what());
    }
  }
  void detectVoice() override{}
  void controllHandle() override {}

};
class Fan : public Objects {
private:
  float _tempForFan = 0.0;
  float _smokingForFan = 0.0;
public:
  Fan(){}
  Fan(int pin) : Objects(pin) {
    // setup pin mode and so on.
    //  Serial.println("Initialization Fan");
  }
  ~Fan() {
    // free QueueHandle_t and so on.
  }

  void taskHandleObjectTemp(void *pvParameter) override {
    try {
      while(true) {
        if (tempOB == NULL) {
                Serial.println("Error: globalTempSensor is NULL");
                vTaskDelete(NULL);
                return;
        }
        xQueueReceive(tempOB->getQueuesHandle(),(void *)&_tempForFan,portMAX_DELAY);
        // Serial.print("Value _tempForFan: ");
        // Serial.println(_tempForFan);
        // temp < 25
        if(_tempForFan >= 35 && !_stateForTemp) {
          digitalWrite(_pin, HIGH);
          Serial.println("Fan is on");
          _stateForTemp = true;
        } else if (_stateForTemp && !_stateForSmoking) {
          digitalWrite(_pin, LOW);
          Serial.println("Light is off");
          _stateForTemp = false;
        }
        vTaskDelay(200/portTICK_PERIOD_MS);
      }
    }catch (std::exception& e) {
      Serial.print("Error in FAN: ");
      Serial.println(e.what());
    }
  }
  void taskHandleObjectSmoking(void *pvParameter) override {
    try {
      while(true) {
        if (smokingOB == NULL) {
                Serial.println("Error: globalSmokingSensor is NULL");
                vTaskDelete(NULL);
                return;
        }
        xQueueReceive(smokingOB->getQueuesHandle(),(void *)&_smokingForFan,portMAX_DELAY);
        // Serial.print("Value _smokingForFan: ");
        // Serial.println(_smokingForFan);
        if(_smokingForFan >= 1700 && !_stateForSmoking) {
          digitalWrite(_pin, HIGH);
          Serial.println("Fan is on");
          _stateForSmoking = true;
        } else if (_stateForSmoking && !_stateForTemp) {
          digitalWrite(_pin, LOW);
          Serial.println("Fan is off");
          _stateForSmoking = false;
        } 
        vTaskDelay(500/portTICK_PERIOD_MS);
      }
    }catch (std::exception& e) {
      Serial.print("Error in LIGHT: ");
      Serial.println(e.what());
    }
  }
  void detectVoice() override {}
  void controllHandle() override {}
};

Light *lightOB = NULL;
Fan *fanOB = NULL;
static void taskTemp(void *pvParameter) {
    try {
      if (tempOB == NULL) {
                Serial.println("Error: globalTempSensor is NULL");
                vTaskDelete(NULL);
                return;
      }
    tempOB->taskHandleSensor(pvParameter);
    } catch (std::exception &e) {
      Serial.print("Error in 251: ");
      Serial.print(e.what());
    }
}
static void taskSmoking(void *pvParameter) {
    try {
      if (smokingOB == NULL) {
                Serial.println("Error: globalSmokingSensor is NULL");
                vTaskDelete(NULL);
                return;
      }
    smokingOB->taskHandleSensor(pvParameter);
    } catch (std::exception &e) {
      Serial.print("Error in 267: ");
      Serial.print(e.what());
    }
}
static void taskLightForTemp(void *pvParameter) {
    try {
      if (lightOB == NULL) {
                Serial.println("Error: globalLightSensor is NULL");
                vTaskDelete(lightOB->_handle);
                return;
      }
    lightOB->taskHandleObjectTemp(pvParameter);
    } catch (std::exception &e) {
      Serial.print("Error in 300: ");
      Serial.print(e.what());
    }
}
static void taskLightForSmoking(void *pvParameter) {
    try {
      if (lightOB == NULL) {
                Serial.println("Error: globalLightSensor is NULL");
                vTaskDelete(lightOB->_handle);
                return;
      }
    lightOB->taskHandleObjectSmoking(pvParameter);
    } catch (std::exception &e) {
      Serial.print("Error in 342: ");
      Serial.print(e.what());
    }
}
static void taskFanForTemp(void *pvParameter) {
    try {
      if (fanOB == NULL) {
                Serial.println("Error: globalFanSensor is NULL");
                vTaskDelete(fanOB->_handle);
                return;
      }
    fanOB->taskHandleObjectTemp(pvParameter);
    } catch (std::exception &e) {
      Serial.print("Error in 300: ");
      Serial.print(e.what());
    }
}
static void taskFanForSmoking(void *pvParameter) {
    try {
      if (fanOB == NULL) {
                Serial.println("Error: globalFanSensor is NULL");
                vTaskDelete(fanOB->_handle);
                return;
      }
    fanOB->taskHandleObjectSmoking(pvParameter);
    } catch (std::exception &e) {
      Serial.print("Error in 342: ");
      Serial.print(e.what());
    }
}
void setup() {
  Serial.begin(115200);
  try {
  tempOB = new Temperature(DHTPIN);
  smokingOB = new Smoking(MQ135PIN);
  lightOB = new Light(LIGHTPIN);
  fanOB = new Fan(FANPIN);
  if (tempOB->getQueuesHandle() == NULL || smokingOB->getQueuesHandle() == NULL) {
    Serial.println("queue NULL");
  } else {
    xTaskCreate(taskTemp, "Reading temperature", 2048, NULL, 3, NULL);
    xTaskCreate(taskSmoking, "Smoking Detect", 2048, NULL, 3, NULL); 
    xTaskCreate(taskLightForTemp, "Controller Light temp", 1024, NULL, 2, &(lightOB->_handle)); 
    xTaskCreate(taskLightForSmoking, "Controller Light smoking", 1024, NULL, 2, &(lightOB->_handle)); 
    xTaskCreate(taskFanForTemp, "Controller Fan temp", 1024, NULL, 2, &(fanOB->_handle)); 
    xTaskCreate(taskFanForSmoking, "Controller Fan smoking", 1024, NULL, 2, &(fanOB->_handle)); 
  }
  } catch (std::exception &e) {
      Serial.println("Error in 232: ");
      Serial.println(e.what());
  }
}

void loop() {}
