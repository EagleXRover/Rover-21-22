/*
  minimal_module.h - Library with helpful minimal functions for arduino boards.
  Created by CatBookshelf, April 1, 2022.
  Last updated, September 13, 2022.
*/

#ifndef MINIMAL_MODULE_H
#define MINIMAL_MODULE_H

// Include basic libraries 
#include <Arduino.h>
#include "constants_module.h"



// Defines reboot 
#if BOARD_TYPE == ARDUINO_BOARD
  void(* reboot_funct) (void) = 0; // Reboot
#elif BOARD_TYPE == TEENSY_BOARD
  #define CPU_RESTART_ADDR (uint32_t *)0xE000ED0C
  #define CPU_RESTART_VAL 0x5FA0004
  #define reboot_funct (*CPU_RESTART_ADDR = CPU_RESTART_VAL)
#endif



// Declare functions
void togglePin(uint8_t pin);
void reboot(String message);
void setupSerials(void);
void setupGPIO(void);



// Define auxiliar variables.
String auxString;
uint16_t auxUint16;
unsigned long actualTime;



// Define functions.

// Toggles the given pin to the opposite state.
void togglePin(uint8_t pin){
    digitalWrite(pin, !digitalRead(pin));
}

// Reboots the system.
void reboot(String message){
    if (DEBUG_ON){
      Serial_USB.println(message);
      Serial_USB.println("REBOOTING...");
      delay(100);
    }
    reboot_funct();
}

// Defines the setup configuration of the Serial Ports.
void setupSerials(void){
    if (DEBUG_ON){
      Serial_USB.begin(Baudrate_USB);
    }
    Serial_Sensors_UART.begin(Baudrate_Sensors_UART);
    delay(100);
}

// Defines and configures the mode of some GPIO pins.
void setupGPIO(void){
    pinMode(LED_NOTIFICATION, OUTPUT);
    pinMode(LED_NOTIFICATION1, OUTPUT);
    pinMode(LED_NOTIFICATION2, OUTPUT);
    
    digitalWrite(LED_NOTIFICATION, HIGH);
    digitalWrite(LED_NOTIFICATION1, LOW);
    digitalWrite(LED_NOTIFICATION2, LOW);
}


#endif