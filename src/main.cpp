#include <Arduino.h>
#include <FreeRTOSConfig.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <MQ135.h>
// Define PINi
// Output pin
#define LIGHTPIN 25
#define FANPIN 26
// Input pin
#define DHTPIN 12 
#define DHTTYPE DHT22
#define MQ135PIN 13
// Initialization global values
DHT_Unified dht(DHTPIN, DHTTYPE);
MQ135 mq135_sensor(MQ135PIN);
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
    dht.begin();
    tempReading = xQueueCreate(10, sizeof(float));
    smokingReading = xQueueCreate(10, sizeof(float));
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
void setup() {
  Serial.begin(9600);
  initValue(true);
  inforDHT();
  definingPinMode(true);

  // Create Task 
  xTaskCreate(taskTempRead, "Reading Temperature", 2048, NULL, 1, NULL);
  xTaskCreate(taskSmokeDetect, "Smoking Detect", 2048, NULL, 1, NULL);
  /* xTaskCreate(taskControlFan, "Controller Fan", 1024, NULL, 1, NULL); */
  /* xTaskCreate(taskControlLight, "Controller Light", 1024, NULL, 1, NULL); */
}

void loop() {}

void taskTempRead(void *pvParameter) {
  sensors_event_t event;
  while (true) {
    // Read Temperature
    dht.temperature().getEvent(&event);
    if(isnan(event.temperature)) {
      Serial.println(F("Error reading temperature"));
      // return 0.0; -> Queues give
    }
    Serial.print(F("Temperature: "));
    Serial.print(event.temperature);
    Serial.println(F("°C"));
    // return float(event.temperature); -> Queues give
    vTaskDelay(500/portTICK_PERIOD_MS);
  }
}
void taskSmokeDetect(void *pvParameter) {
  // Read humidity
  sensors_event_t event;
  while(true) {
    dht.humidity().getEvent(&event);
    if(isnan(event.relative_humidity)) {
      Serial.println(F("Error reading humidity"));
      //return 0.0; -> Queues give
    }
    float humidity = event.relative_humidity;
    // temperature = tempReading(); -> Solve later -> Queues take
    // Use temporary temperature
    float temperature = 25;
    float correctedRZero = mq135_sensor.getCorrectedRZero(temperature, humidity);
    float resistance = mq135_sensor.getResistance();
    float correctedPPM = mq135_sensor.getCorrectedPPM(temperature, humidity);
    // Queues Give values (I think we will use ppm for detect smoke)
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
  // When voice "On" and "Off"
}
void taskControlFan(void *pvParameter) {
  // Air is polluted or temperature is hot -> On else Off
  // When voice "Activate" and "Deactivate"
}
