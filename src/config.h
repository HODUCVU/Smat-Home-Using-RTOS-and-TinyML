// are you using an I2S microphone - comment this out if you want to use an analog mic and ADC input
#define USE_I2S_MIC_INPUT

// I2S Microphone Settings
// Which channel is the I2S microphone on? I2S_CHANNEL_FMT_ONLY_LEFT or I2S_CHANNEL_FMT_ONLY_RIGHT
#define I2S_MIC_CHANNEL I2S_CHANNEL_FMT_ONLY_LEFT
// #define I2S_MIC_CHANNEL I2S_CHANNEL_FMT_ONLY_RIGHT
#define I2S_MIC_SERIAL_CLOCK GPIO_NUM_33
#define I2S_MIC_LEFT_RIGHT_CLOCK GPIO_NUM_26
#define I2S_MIC_SERIAL_DATA GPIO_NUM_25

// Analog Microphone Settings - ADC1_CHANNEL_7 is GPIO35
#define ADC_MIC_CHANNEL ADC1_CHANNEL_7

/*
SCK -> GPIO_33
WS -> GPIO_26
SD -> GPIO 25
VDD and GND -> 3.3V and GND

in CommandProcessor.cpp
    GPIO_NUM_13
    GPIO_NUM_12
*/
// Output pin
#ifndef LIGHTPIN 
#define LIGHTPIN 5
#endif
#ifndef LIGHTPIN_ALARM 
#define LIGHTPIN_ALARM 23
#endif
#ifndef FANPIN 
#define FANPIN 4
#endif
#ifndef DHTPIN 
#define DHTPIN 13
#endif
#ifndef DHTTYPE 
#define DHTTYPE DHT22
#endif
#ifndef MQ135PIN
#define MQ135PIN 12
#endif