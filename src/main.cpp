#include <Arduino.h>
#include <Wire.h>
#include <FreeRTOSConfig.h>
#include <Ticker.h>
#include <DHT.h>
#include <Adafruit_Sensor.h>

#include <WiFi.h>
#include <driver/i2s.h>
#include <esp_task_wdt.h>
#include "I2SMicSampler.h"
#include "ADCSampler.h"
#include "config.h"
#include "CommandDetector.h"
#include "CommandProcessor.h"

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
    _pin = pin;
    _queuesReading = xQueueCreate(10, sizeof(float));  
  }
  Sensors(int pin, int sizeQueues) {
    _pin = pin;
    _queuesReading = xQueueCreate(sizeQueues, sizeof(float));  
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
      Serial.print("Error in sendQueuesHandle: ");
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
    dht.begin();
  }
  Temperature(int pin, int sizeQueues) : Sensors(pin, sizeQueues) {
    dht.begin();
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
          sendQueuesHandle(_temp);
        }
        vTaskDelay(500/portTICK_PERIOD_MS);
      }
    }
    catch (std::exception &e) {
      Serial.print("Error in Temp: ");
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
  }
  Smoking(int pin, int sizeQueues) : Sensors(pin, sizeQueues) {
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
          sendQueuesHandle(_smoking);
        }
        vTaskDelay(500/portTICK_PERIOD_MS);
      }
    } catch(std::exception& e) {
      Serial.print("Error in Smoking: ");
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
  void virtual controllHandle() = 0;
};
class Light : public Objects {
private:
  float _tempForLight = 0.0;
  float _smokingForLight = 0.0;
public:
  Light() {}
  Light(int pin) : Objects(pin) {
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
        // temp < 25
        if (!statusLightGB) {
          if(_tempForLight <= 31 && !_stateForTemp) {
            digitalWrite(_pin, HIGH);
            Serial.println("Light is on");
            _stateForTemp = true;
          } else if (_stateForTemp && !_stateForSmoking) {
            digitalWrite(_pin, LOW);
            Serial.println("Light is off");
            _stateForTemp = false;
          }
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
        if (!statusLightGB) {
            if(_smokingForLight >= 1700 && !_stateForSmoking) {
            digitalWrite(_pin, !digitalRead(_pin));
            Serial.println("Light is blinking");
            _stateForSmoking = true;
          } else if (_stateForSmoking && !_stateForTemp) {
            digitalWrite(_pin, LOW);
            Serial.println("Light is off");
            _stateForSmoking = false;
          }
        } 
        vTaskDelay(500/portTICK_PERIOD_MS);
      }
    }catch (std::exception& e) {
      Serial.print("Error in LIGHT: ");
      Serial.println(e.what());
    }
  }
  void controllHandle() override {}

};
class Fan : public Objects {
private:
  float _tempForFan = 0.0;
  float _smokingForFan = 0.0;
public:
  Fan(){}
  Fan(int pin) : Objects(pin) {
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
        // temp < 25
        if(!statusFanGB) {
          if(_tempForFan >= 35 && !_stateForTemp) {
            digitalWrite(_pin, HIGH);
            Serial.println("Fan is on");
            _stateForTemp = true;
          } else if (_stateForTemp && !_stateForSmoking) {
            digitalWrite(_pin, LOW);
            Serial.println("Light is off");
            _stateForTemp = false;
          }
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
        if(!statusFanGB) {
          if(_smokingForFan >= 1700 && !_stateForSmoking) {
            digitalWrite(_pin, HIGH);
            Serial.println("Fan is on");
            _stateForSmoking = true;
          } else if (_stateForSmoking && !_stateForTemp) {
            digitalWrite(_pin, LOW);
            Serial.println("Fan is off");
            _stateForSmoking = false;
          } 
        }
        vTaskDelay(500/portTICK_PERIOD_MS);
      }
    }catch (std::exception& e) {
      Serial.print("Error in FAN: ");
      Serial.println(e.what());
    }
  }
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
    Serial.print("Error in taskTemp: ");
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
    Serial.print("Error in taskSmoking: ");
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
    Serial.print("Error in taskLightForTemp: ");
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
    Serial.print("Error in taskLightForSmoking: ");
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
    Serial.print("Error in taskFanForTemp: ");
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
    Serial.print("Error in taskFanForSmoking: ");
    Serial.print(e.what());
  }
}

//  =================================
// i2s config for using the internal ADC
i2s_config_t adcI2SConfig = {
  .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
  .sample_rate = 16000,
  .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
  .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
  .communication_format = I2S_COMM_FORMAT_I2S_LSB,
  .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
  .dma_buf_count = 4,
  .dma_buf_len = 64,
  .use_apll = false,
  .tx_desc_auto_clear = false,
  .fixed_mclk = 0};

// i2s config for reading from both channels of I2S
i2s_config_t i2sMemsConfigBothChannels = {
  .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
  .sample_rate = 16000,
  .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
  .channel_format = I2S_MIC_CHANNEL,
  .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S),
  .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
  .dma_buf_count = 4,
  .dma_buf_len = 64,
  .use_apll = false,
  .tx_desc_auto_clear = false,
  .fixed_mclk = 0};

// i2s microphone pins
i2s_pin_config_t i2s_mic_pins = {
  .bck_io_num = I2S_MIC_SERIAL_CLOCK,
  .ws_io_num = I2S_MIC_LEFT_RIGHT_CLOCK,
  .data_out_num = I2S_PIN_NO_CHANGE,
  .data_in_num = I2S_MIC_SERIAL_DATA};

// This task does all the heavy lifting for our application
void voiceTask(void *param)
{
  CommandDetector *commandDetector = static_cast<CommandDetector *>(param);

  const TickType_t xMaxBlockTime = pdMS_TO_TICKS(100);
  while (true)
  {
    // wait for some audio samples to arrive
    uint32_t ulNotificationValue = ulTaskNotifyTake(pdTRUE, xMaxBlockTime);
    if (ulNotificationValue > 0)
    {
      commandDetector->run();
    }
  }
}


void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting up");
  try {
    tempOB = new Temperature(DHTPIN);
    smokingOB = new Smoking(MQ135PIN);
    lightOB = new Light(LIGHTPIN);
    fanOB = new Fan(FANPIN);
    // make sure we don't get killed for our long running tasks
    esp_task_wdt_init(10, false);
    // start up the I2S input (from either an I2S microphone or Analogue microphone via the ADC)
#ifdef USE_I2S_MIC_INPUT
    // Direct i2s input from INMP441 or the SPH0645
    I2SSampler *i2s_sampler = new I2SMicSampler(i2s_mic_pins, false);
#else
    // Use the internal ADC
    I2SSampler *i2s_sampler = new ADCSampler(ADC_UNIT_1, ADC_MIC_CHANNEL);
#endif
    // the command processor
    CommandProcessor *command_processor = new CommandProcessor();

    // create our application
    CommandDetector *commandDetector = new CommandDetector(i2s_sampler, command_processor);

    // set up the i2s sample writer task
    TaskHandle_t applicationTaskHandle;
    if (tempOB->getQueuesHandle() == NULL || smokingOB->getQueuesHandle() == NULL) {
      Serial.println("queue NULL");
    } else {
      xTaskCreatePinnedToCore(voiceTask, "Command Detect", 8192, commandDetector, 4, &applicationTaskHandle, 0);
      xTaskCreate(taskTemp, "Reading temperature", 2048, NULL, 3, NULL);
      xTaskCreate(taskSmoking, "Smoking Detect", 2048, NULL, 3, NULL); 
      xTaskCreate(taskLightForTemp, "Controller Light temp", 1024, NULL, 2, &(lightOB->_handle)); 
      xTaskCreate(taskLightForSmoking, "Controller Light smoking", 1024, NULL, 2, &(lightOB->_handle)); 
      xTaskCreate(taskFanForTemp, "Controller Fan temp", 1024, NULL, 2, &(fanOB->_handle)); 
      xTaskCreate(taskFanForSmoking, "Controller Fan smoking", 1024, NULL, 2, &(fanOB->_handle)); 
    }
    // start sampling from i2s device - use I2S_NUM_0 as that's the one that supports the internal ADC
#ifdef USE_I2S_MIC_INPUT
    i2s_sampler->start(I2S_NUM_0, i2sMemsConfigBothChannels, applicationTaskHandle);
#else
    i2s_sampler->start(I2S_NUM_0, adcI2SConfig, applicationTaskHandle);
#endif
  } catch (std::exception &e) {
    Serial.println("Error in setup: ");
    Serial.println(e.what());
  }
}

void loop() {}
