#include "minimal_module.h"
#include "communication_module.h"
#include "drivers_module.h"
#include "servos_module.h"

void setup(void){
    setupSerials();
    comms_setup();
    setupDrivers();
    setupServos();
}

void loop(void){
    comms_update();
}

// /* Sweep
//  by BARRAGAN <http://barraganstudio.com>
//  This example code is in the public domain.

//  modified 8 Nov 2013
//  by Scott Fitzgerald
//  http://www.arduino.cc/en/Tutorial/Sweep
// */

// #include <Servo.h>

// Servo myservo;  // create servo object to control a servo
// // twelve servo objects can be created on most boards

// #define delayt 100
// int pos = 80;    // variable to store the servo position

// void setup() {
//   Serial.begin(38400);
//   myservo.attach(2);  // attaches the servo on pin 9 to the servo object
// }

// void loop() {
//   for (pos = 30; pos <= 90  ; pos += 1) { // goes from 0 degrees to 180 degrees
//     // in steps of 1 degree
//     myservo.write(pos);              // tell servo to go to position in variable 'pos'
//     Serial.println(pos);
//     delay(delayt);                       // waits 15ms for the servo to reach the position
//   }
//   for (pos = 90; pos >= 30; pos -= 1) { // goes from 180 degrees to 0 degrees
//     myservo.write(pos);              // tell servo to go to position in variable 'pos'
//     Serial.println(pos);
//     delay(delayt);                       // waits 15ms for the servo to reach the position
//   }
// }
