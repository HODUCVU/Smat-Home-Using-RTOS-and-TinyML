#include <Arduino.h>
#include <FreeRTOSConfig.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
// Define PINi
// Output pin
#define LIGHTPIN 25
#define FANPIN 26
// Input pin
#define DHTPIN 12 
#define DHTTYPE DHT22
#define MQ2PIN 13
// Initialization global values
DHT_Unified dht(DHTPIN, DHTTYPE);

// Queues for FreeRTOSConfig
static QueueHandle_t tempReading;
static QueueHandle_t smokingReading;
// Define task task Handle 
TaskHandle_t Fan_handle = NULL;
TaskHandle_t Light_handle = NULL;
// Status On/Off -> True/False
bool fanStatus, lightStatus;
// Task functions
void taskTempRead(void *pvParameter); // print values to monitor -> return temperature
void taskSmokeDetect(void *pvParameter); //MQ2 and humidity from DHT sensor, print values to monitor -> return threshold
void taskControlLight(void *pvParameter);
void taskControlFan(void *pvParameter);
// Initialization function
void initValue(bool permissionInit= false) {
  if (permissionInit) {
    fanStatus = false;
    lightStatus = false;
    pinMode(LIGHTPIN, OUTPUT);
    dht.begin();
    tempReading = xQueueCreate(10, sizeof(int));
    smokingReading = xQueueCreate(10, sizeof(int));
  } else {
    printf("Initialization values are not permission");
  }
} 
void definingPinMode(bool permisionConfig = false) {
  if (permisionConfig) {
    // Output pin mode
    pinMode(LIGHTPIN, OUTPUT);
    pinMode(FANPIN, OUTPUT);
    // Start
    digitalWrite(LIGHTPIN, HIGH);
    digitalWrite(FANPIN, LOW);
  } else {
    printf("Config pins mode are not permission");
  }
}
void inforDHT(){
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  Serial.printf(F("Temperature Sensor"));
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
void setup() {
  Serial.begin(9600);
  initValue(permissionInit=true);
  inforDHT();
  definingPinMode(permisionConfig=true);
  /* xTaskCreate(blink_task_LED1, "Blink led 1", 1024, NULL, 1, NULL); */
  /* xTaskCreate(printS, "Hello", 1024, NULL, 1, NULL); */
  /* xTaskCreate(printTest, "Test", 1024, NULL, 2, NULL); */
}

void loop() {}
/*
void blink_task_LED1(void *pvParameter)
{
  while(1) {
      // Blink off (output, low); 
      digitalWrite(LED_1, HIGH);
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      // Blink on (output, high); 
      digitalWrite(LED_1, LOW);
      vTaskDelay(3000 / portTICK_PERIOD_MS);
  }
}
void printS(void * pvParameter) {
  while(1) {
    Serial.print("Hello\n");
    vTaskDelay(1000/portTICK_PERIOD_MS);
  }
} 
void printTest(void *pvParameter){
  while (1)
  {
    Serial.print("Test\n");
    vTaskDelay(1000/portTICK_PERIOD_MS);
  }
}
*/
