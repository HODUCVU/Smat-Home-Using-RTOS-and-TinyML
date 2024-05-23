#include <Arduino.h>
#include <FreeRTOSConfig.h>
#include <Ticker.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <MQ135.h>
// Voice regconition
#include <driver/i2s.h>
#include <arduinoFFT.h>
#include <Adafruit_NeoPixel.h>
#include <TensorFlowLite_ESP32.h>
#include <tensorflow/lite/micro/all_ops_resolver.h>
#include <tensorflow/lite/micro/micro_error_reporter.h>
#include <tensorflow/lite/micro/micro_interpreter.h>
#include <tensorflow/lite/schema/schema_generated.h>
// #include <tensorflow/lite/version.h>
#include <model.h>
#define I2S_PORT I2S_NUM_1
// Define PIN
// Output pin
#define LIGHTPIN 4
#define FANPIN 5
// Input pin
#define DHTPIN 26
#define DHTTYPE DHT22
#define MQ135PIN 25
// Initialization global values
DHT_Unified dht(DHTPIN, DHTTYPE);
MQ135 mq135_sensor(MQ135PIN);
// Queues for FreeRTOSConfig
/* QueueHandle_t tempReading; */
/* QueueHandle_t smokingReading; */
// Define task task Handle 
/* TaskHandle_t handle_Fan = NULL; */
/* TaskHandle_t handle_Light = NULL; */
// Status On/Off -> True/False
/* bool fanStatus, lightStatus; */
// Task functions
/* void taskTempRead(void *pvParameter); // print values to monitor -> return temperature */
/* void taskSmokeDetect(void *pvParameter); //MQ2 and humidity from DHT sensor, print values to monitor -> return threshold */
/* void taskControlLight(void *pvParameter); */
/* void taskControlFan(void *pvParameter); */
/* void taskControlWithVoice(void *pvParameter); //Use semaphore and mutex to manage data of controller. */

class Sensors {
protected:
  int _pin;
  QueueHandle_t _queuesReading = NULL;
public:
  Sensors(int pin) {
    Serial.print("Initialization Sensor")
      _pin = pin;
    pinMode(_pin, INPUT);
    _queuesReading = xQueueCreate(10, sizeof(float));  
  }
  Sensors(int pin, int sizeQueues) {
    Serial.print("Initialization Sensor")
      _pin = pin;
    pinMode(_pin, INPUT);
    _queuesReading = xQueueCreate(sizeQueues, sizeof(float));  
  }
  ~Sensors() {
    // print delete Sensors
  }
  /* void virtual setup(); */
  void virtual readValue();
  void virtual taskHandle(void *pvParameter);

  void virtual sendQueuesHandle();
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
  Objects(){}
  /* void virtual setup(); */
  void virtual taskHandle(void *pvParameter);
  void virtual read();
  void virtual detectVoice();
  void virtual controllHandle();
};

class Temperature : public Sensors {
public:
  Temperature(int pin) : Sensors(int pin) {
    /* _pin = pin; */
    /* pinMode(_pin, INPUT); */
    /* _queuesReading = xQueueCreate(10, sizeof(float));   */
    dht.begin();
  }
  Temperature(int pin, int sizeQueues) : Sensors(int pin, int sizeQueues) {
    /* _pin = pin; */
    /* pinMode(_pin, INPUT); */
    /* _queuesReading = xQueueCreate(size, sizeof(float));   */
    dht.begin();
  }
  void readValue() {}
  void taskHandle(void *pvParameter) {
    try {
      sensors_event_t event;
      float temperature;
      while (true) {
        // Read Temperature
        dht.temperature().getEvent(&event);
        if(isnan(event.temperature)) {
          Serial.println(F("Error reading temperature"));
          /* return; // stop */
          /* vTaskSuspend(handle_Fan); */
          /* vTaskSuspend(handle_Light); */
          vTaskDelay(1000/portTICK_PERIOD_MS);
          continue;
        }
        Serial.print(F("Temperature: "));
        Serial.print(event.temperature);
        Serial.println(F("°C"));
        // return float(event.temperature); -> Queues send
        temperature = event.temperature;
        /* xQueueSend(tempReading, (void*)&temperature,(TickType_t) 0); */
        sendQueuesHandle(temperature);
        vTaskDelay(500/portTICK_PERIOD_MS);
      }
    } catch (std::exception& e) {
      Serial.print(F("Error: "));
      Serial.println(e.what());
    }

  }
  void sendQueuesHandle(float temperature) {
    xQueueSend(_queuesReading, (void*)&temperature,(TickType_t) 0);
  }
  /* void receiveQueuesHandle() {} */
  void inforDHT(){
    sensor_t sensor;
    dht.temperature().getSensor(&sensor);
    Serial.println(F("Temperature Sensor"));
    Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
    Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
    Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
    Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("°C"));
    Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("°C"));
    Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("°C"));

    dht.humidity().getSensor(&sensor);
    Serial.println(F("Humidity Sensor"));
    Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
    Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
    Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
    Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("%"));
    Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("%"));
    Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("%"));
    vTaskDelay(1000/portTICK_PERIOD_MS);
  }
  ~Temperature() {}

};
class Smoking : public Sensors {
public:
  Smoking(int pin) : Sensors(int pin) {}
  Smoking(int pin, int sizeQueues) : Sensors(int pin, int sizeQueues) {}
  ~Smoking() {}

  void readValue() {}
  void taskHandle(void *pvParameter) {

  }
  void sendQueuesHandle() {}
  /* void receiveQueuesHandle() {} */
};

class Light : public Objects {
public:
  Light() : Objects() {
    // setup pin mode and so on.
  }
  ~Ligh() {
    // free QueueHandle_t and so on.
  }

  void taskHandle(void *pvParameter) {}
  void read(){}
  void detectVoice(){}
  void controllHandle(){}

};
class Fan : public Objects {
public:
  Fan() : Objects() {
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

// Initialization function
/* void initValue(bool permissionInit= false) { */
/*   if (permissionInit) { */
/*     fanStatus = false; */
/*     lightStatus = false; */
/*     dht.begin(); */
/*     tempReading = xQueueCreate(10, sizeof(float)); */
/*     smokingReading = xQueueCreate(10, sizeof(float)); */
/*   } else { */
/*     printf("Initialization values are not permission"); */
/*   } */
/* }  */
/* void definingPinMode(bool permisionConfig = false) { */
/*   if (permisionConfig) { */
/*     // Output pin mode */
/*     pinMode(LIGHTPIN, OUTPUT); */
/*     pinMode(FANPIN, OUTPUT); */
/*     /* pinMode(MQ135PIN, INPUT); */ 
/*     // Start */
/*     digitalWrite(LIGHTPIN, HIGH); */
/*     digitalWrite(FANPIN, LOW); */
/*   } else { */
/*     printf("Config pins mode are not permission"); */
/*   } */
/* } */
void setup() {
  Serial.begin(115200);
  /* initValue(true); */
  /* inforDHT(); */
  /* definingPinMode(true); */

  /* if(tempReading == NULL || smokingReading == NULL) { */
  /* Serial.println("Configuration Failed!"); */
  /* } else { */
  // Create Task 
  /* xTaskCreate(taskTempRead, "Reading Temperature", 2048, NULL, 3, NULL); */
  /* xTaskCreate(taskSmokeDetect, "Smoking Detect", 2048, NULL, 2, NULL); */
  /* xTaskCreate(taskControlFan, "Controller Fan", 1024, NULL, 1, &handle_Fan ); */
  /* xTaskCreate(taskControlLight, "Controller Light", 1024, NULL, 1, &handle_Light); */

  /* } */

  // Turn off fan and light handle 
  /* vTaskSuspend(handle_Fan); */
  /* vTaskSuspend(handle_Light); */
}

void loop() {}

/* void taskTempRead(void *pvParameter) { */
/*   try { */
/*   sensors_event_t event; */
/*   float temperature; */
/*   while (true) { */
/*     // Read Temperature */
/*     dht.temperature().getEvent(&event); */
/*     if(isnan(event.temperature)) { */
/*       Serial.println(F("Error reading temperature")); */
/*       /* return; // stop */ 
/*       vTaskSuspend(handle_Fan); */
/*       vTaskSuspend(handle_Light); */
/*       vTaskDelay(1000/portTICK_PERIOD_MS); */
/*       continue; */
/*     } */
/*     Serial.print(F("Temperature: ")); */
/*     Serial.print(event.temperature); */
/*     Serial.println(F("°C")); */
/*     // return float(event.temperature); -> Queues send */
/*     temperature = event.temperature; */
/*     xQueueSend(tempReading, (void*)&temperature,(TickType_t) 0); */
/*     vTaskDelay(500/portTICK_PERIOD_MS); */
/*   } */
/*   } catch (std::exception& e) { */
/*     Serial.print(F("Error: ")); */
/*     Serial.println(e.what()); */
/*   } */
/* } */
void taskSmokeDetect(void *pvParameter) {
// Read humidity
sensors_event_t event;
float temperature = 25.0, humidity = 25.0; 
float correctedRZero =0.0, resistance = 0.0, correctedPPM = 0.0;
while(true) {
  dht.humidity().getEvent(&event);
  if(isnan(event.relative_humidity)) {
    Serial.println(F("Error reading humidity"));
    // return; //stop 
    vTaskSuspend(handle_Fan);
    vTaskDelay(1000/portTICK_PERIOD_MS);
    continue;
  }
  Serial.println("Readed humidity, Waiting for tempReading!");
  humidity = event.relative_humidity;
  // tempQueues take
  if(tempReading != NULL) {
    if(xQueueReceive(tempReading, &(temperature), (TickType_t) 10 ) ==pdPASS)
    {
      correctedRZero = mq135_sensor.getCorrectedRZero(temperature, humidity);
      resistance = mq135_sensor.getResistance();
      correctedPPM = mq135_sensor.getCorrectedPPM(temperature, humidity);
      // Queues Give values (I think we will use ppm for detect smoke)
      xQueueSend(smokingReading, (void*)&correctedPPM, (TickType_t) 0);
    }
  }
  Serial.println("Smoking Detect: ");
  Serial.print("Rzero: ");
  Serial.println(correctedRZero);
  Serial.print("Resistance: ");
  Serial.println(resistance);
  Serial.print("PPM: ");
  Serial.println(correctedPPM);
  vTaskDelay(500/portTICK_PERIOD_MS);
}
}
void taskControlLight(void *pvParameter) {
  // When temperature is cool -> On else Off
  float temperature;
  while(true){
    if(tempReading != NULL) {
      if(xQueueReceive(tempReading, &(temperature), (TickType_t) 10 ) ==pdPASS)
      {
        if(temperature < float(20)) {
          if(!lightStatus) {
            digitalWrite(LIGHTPIN, HIGH);
            lightStatus = true;
            Serial.println(F("Light on"));
          }
        } else {
          if(lightStatus) {
            digitalWrite(LIGHTPIN, LOW);
            lightStatus = false;
            Serial.println(F("Light off"));
          }
        }
      }
    }
    // When voice "On" and "Off" and "Stop"
    // "Stop" to suppend TaskHandle 
    vTaskDelay(200/portTICK_PERIOD_MS);
  }
}
void taskControlFan(void *pvParameter) {
  // Air is polluted or temperature is hot -> On else Off
  float temperature, airCondition;
  while(true){
    // Take temperature
    if(tempReading != NULL) {
      if(xQueueReceive(tempReading, &(temperature), (TickType_t) 10 ) ==pdPASS)
      {
        // temperature in home higher 30 °C
        if(temperature > float(30)) {
          if(!fanStatus) {
            digitalWrite(FANPIN, HIGH);
            fanStatus = true;
            Serial.println(F("Fan on"));
          }
        } else {
          if (fanStatus) {
            digitalWrite(FANPIN, LOW);
            fanStatus = false;
            Serial.println(F("Fan off"));
          }
        }
      }

    }
    // Take air condition in home
    // Code same with temperature, but I don't want to wait to wear both tempReading and smokingReading at the same time
    if(smokingReading != NULL) {
      if(xQueueReceive(smokingReading, &(airCondition), (TickType_t) 10) == pdPASS) {
        if(airCondition >= float(0.5)) {
          if(!fanStatus) {
            digitalWrite(FANPIN, HIGH);
            fanStatus = true;
            Serial.println(F("Fan on"));
          }       
        } else {
          if (fanStatus) {
            digitalWrite(FANPIN, LOW);
            fanStatus = false;
            Serial.println(F("Fan off"));
          }
        }
      }
    }
    // When voice "Activate" and "Deactivate"
    // "Stop" to suppend TaskHandle 
    vTaskDelay(200/portTICK_PERIOD_MS);
  }
}
