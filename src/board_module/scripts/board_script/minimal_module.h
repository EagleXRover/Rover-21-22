/*
  minimal_module.h - Library with helpful minimal functions for arduino.
  Created by CatBookshelf, April 1, 2022.
*/

#ifndef MINIMAL_MODULE_H
#define MINIMAL_MODULE_H

#include <Arduino.h>
#include "constants_module.h"

void(* reboot_funct) (void) = 0; // Reboot

// Toogles the given pin to the opposite state.
void togglePin(uint8_t pin){
    digitalWrite(pin, !digitalRead(pin));
}

// Reboots the system.
void reboot(String message){
    Serial_USB.println(message);
    Serial_USB.println("REBOOTING...");
    delay(100);
    reboot_funct();
}

// Defines the setup configuration of the Serial Ports.
void setupSerials(void){
    Serial_USB.begin(Baudrate_USB);
    Serial_Sensors_UART.begin(Baudrate_Sensors_UART);
    delay(100);
}

void setupGPIO(void){
    pinMode(LED_NOTIFICATION, OUTPUT);
    pinMode(LED_NOTIFICATION1, OUTPUT);
    pinMode(LED_NOTIFICATION2, OUTPUT);
    
    digitalWrite(LED_NOTIFICATION, HIGH);
    digitalWrite(LED_NOTIFICATION1, LOW);
    digitalWrite(LED_NOTIFICATION2, LOW);
}

// Auxiliar variables.
String auxString;
uint16_t auxUint16;

#endif