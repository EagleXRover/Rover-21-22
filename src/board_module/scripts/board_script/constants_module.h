/*
  constants_module.h - Library for all the constants and general definitions.
  Created by CatBookshelf, April 1, 2022.
*/

#ifndef CONSTANTS_MODULE_H
#define CONSTANTS_MODULE_H

#include <Arduino.h>

// LEDs Constants.
#undef  LED_BUILTIN
#define LED_NOTIFICATION    13          // Notification Led0.
#define LED_NOTIFICATION1   12          // Notification Led1.
#define LED_NOTIFICATION2   11          // Notification Led2.

#define RGB_LED_NOTIFICATION_R A13      // RGB nRED pin.
#define RGB_LED_NOTIFICATION_G A14      // RGB nGREEN pin.
#define RGB_LED_NOTIFICATION_B A15      // RGB nBLUE pin.

// Serial Ports rename.
#define Serial_USB Serial               // Serial pins for the USB.
#define Serial_Arm_Science Serial1      // Serial pins for the arm.
#define Serial_Wheels Serial2           // Serial pins for the wheels.
#define Serial_Sensors_UART Serial3     // Serial pins for the sensors (UART).

// Serial Ports Baudrate.
#define Baudrate_USB 38400              // Baudrate with the Serial_USB.
#define Baudrate_Arm_Science 38400      // Baudrate with the Serial_Arm_Science.
#define Baudrate_Wheels 38400           // Baudrate with the Serial_Wheels.
#define Baudrate_Sensors_UART 9600      // Baudrate with the Serial_Sensors_UART.

// I2C pins. 
#define I2C_SCL SCL     // I2C communication pin
#define I2C_SDA SDA     // I2C communication pin

unsigned long actualTime;


#endif