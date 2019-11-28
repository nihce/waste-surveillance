# WasteSurveillance
LoRa based IoT solution for efficient waste management and surveillance

This project consists of several components, i.e. the measurement device, TTN (The Things Network) application and a server application.

## Measurement device

Hardware components:
 - ArduinoMega 2560
 - Dragino LoRa shield for Arduino
 - GY-521 accelerometer
 - HC-SR04 ultrasonic sensor
 - 9V battery

Schematic: ![schematic](https://github.com/nezezime/WasteSurveillance/blob/master/TrashGuard_Wiring_Schematic_PNG.png)

Third party software libraries:
 - https://github.com/Martinsos/arduino-lib-hc-sr04
 - https://github.com/matthijskooijman/arduino-lmic
 
 Setup instructions:
  - connect the hardware components as described in the schematic. Do NOT connect the battery as you will be using the USB power when setting up the measurement device
  - download the third party libraries for Arduino as .zip files and import them in Arduino IDE
  - edit the trash_surveillance_client.ino file: specify the correct values for Network Session Key, Application Session Key and Device address. These values should be provided by the TTN application and allow measurement device pairing and secure communication
  - Compile the Arduino script and load the code to ArduinoMega via USB
  - open up the TTN application and see if the data is being sent from the measurement device and correctly decoded
  - you are now good to go! Replace USB with battery power and pt the device where you want to use it
  
  ## TTN application
  
  Register a new account and create a new application. Add a new device, configure the ABP activation method, which will make available the keys for measurement device configuration. Javascript code for LoRa payload decoding is available in lora_decoder.js file. 
  
LoRa payload format:
 - port: 1 ok; 2 error
 - payload byte 1 and 2: ultrasonic measurement (trash can fullness indicator)
 - payload byte 3: error flags

