# Frameworks like TensorFlow Lite and TiniML for Microcontrollers are designed to run on small devices and can be used with the ESP32.
# Topic: Smart home use FreeRTOS and TinyML on ESP32
Short-explain: Implement smart home, training tiniML module to control light and fan by voice. Use temperator and smoking sensor to control light and fan.
# Testing
## Control with voice
https://github.com/HODUCVU/Smat-Home-Using-RTOS-and-TinyML/assets/73897430/bb15e396-5497-484c-84cf-db7cbdadaa53
## Automatic system with sensors


https://github.com/HODUCVU/Smat-Home-Using-RTOS-and-TinyML/assets/73897430/a2ac8975-1a5b-4979-9845-8154b9d45238




# Hardware
![](/methods/images/board-project_bb-v2.png)
# Usecase Diagram
![](/methods/images/Usecase-Diagram.png) 
# Class Diagram
![](/methods/images/Class-diagram.OVERVIEW.png) 
<!-- # Reports for this project
* [Reports](https://github.com/HODUCVU/Smat-Home-Using-RTOS-and-TinyML/tree/main/methods/reports) -->

# Reference
## Papers
* [Analysis of Free RTOS Vs Bare Metal using ESP32](https://www.iosrjournals.org/iosr-jeee/Papers/Vol16-Issue2/Series-1/E1602013968.pdf)
* [Formally verifying FreeRTOS’ interprocess communication mechanism](https://www.amazon.science/publications/formally-verifying-freertos-interprocess-communication-mechanism)
## Tutorial FreeRTOS on ESP32
* ESP32 with RTOS: [Youtube.com](https://www.youtube.com/watch?v=LLp9T3rgea8)
* Tutorial FreeRTOS: [github.com/neibalch](https://github.com/neilbalch/ESP32-FreeRTOS-Tutorial)
<!-- * Tutorial 2: [github.com/DiegoPaezA](https://github.com/DiegoPaezA/ESP32-freeRTOS?tab=readme-ov-file) -->
## Training model
* [Tensorflow Lite for microcontroller](https://github.com/eloquentarduino/EloquentTinyML.git)
* [TinyML for ESP32](https://github.com/HollowMan6/TinyML-ESP32.git)
* [ESP32\_MICROPHONE](https://github.com/0015/ThatProject/tree/master/ESP32_MICROPHONE)

# Check ESP32 board version
* [esptool-js](https://espressif.github.io/esptool-js/)
* [repository](https://github.com/espressif/esptool-js)
* [Datasheet](https://products.espressif.com/#/product-selector?names=&filter={%22Series%22:[%22ESP32%22]})
<!--
## Videos tutorial train ML on board:
### Video 1
* [Video tutorial train machine learning for Arduino board](https://www.youtube.com/watch?v=BzzqYNYOcWc&list=RDCMUCclJCqMDAkyVGsm5oFOTXIQ&start_radio=1)
* [Detail project in website](https://www.digikey.com/en/maker/projects/intro-to-tinyml-part-1-training-a-model-for-arduino-in-tensorflow/8f1fc8c0b83d417ab521c48864d2a8ec)
* [code colab](https://gist.github.com/ShawnHymel/79237fe6aee5a3653c497d879f746c0c)
### Video 2
* [Video tutorual train recognition speech  with Arduino](https://www.youtube.com/watch?v=fRSVQ4Fkwjc)
* [repo code](https://github.com/ShawnHymel/ei-keyword-spotting)
## Run platformio on terminal
* Build: 
  ```
   /mnt/c/Users/<name>/.platformio/penv/Scripts/platformio.exe run
  ```
* Upload:
  ```
  /mnt/c/Users/<name>/.platformio/penv/Scripts/platformio.exe run --target upload
  ```
* Monitor:
  ```
  /mnt/c/Users/<name>/.platformio/penv/Scripts/platformio.exe device monitor
  ```
-->
