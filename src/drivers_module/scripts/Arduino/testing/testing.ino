/* Libraries */
// Ethernet communication
#include <SPI.h>
#include <Ethernet.h>

// ROS Serial
#define ROSSERIAL_ARDUINO_TCP

// ROS & MSGs
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Empty.h>

// Actuators
#include "RoboClaw.h"
#include <Servo.h>

/* Functions prototyping */
void(* reboot) (void) = 0; // Reboot
void armSubscriber(const std_msgs::UInt16&);
void wheelsSubscriber(const std_msgs::UInt8&);
void timeOutSubscriber(const std_msgs::Empty&);
void setup(void);
void loop(void);
void togglePin(char pin);
void stopWheels(void);
void stopArm(void);
void haltMovement(void);

/* ETHERNET */
// Shield 
const byte mac[6] = {0x80, 0x69, 0x69, 0x69, 0x69, 0x09};
const uint8_t shield_CS = 0x08;

// ROS MASTER
IPAddress MASTER(10, 0, 0, 4);
const uint16_t MASTER_PORT = 11411;

// ROS WATCHDOG
unsigned long arduino_timer = 0;
unsigned long ros_timer = 0;

/* Actuators */
// Drivers
const uint8_t driversPins[][2] = {{0x0B, 0x0A}, {0x0C, 0x09}}; 
const uint8_t armDriversAdresses[] = {0x82, 0x80, 0x81};
const uint8_t leftWheelsDriversAdresses[] = {0x80, 0x82, 0x84};
const uint8_t rightWheelsDriversAdresses[] = {0x81, 0x83, 0x85};
const unsigned int driversBaudrate[] = {38400, 38400};
SoftwareSerial serialDrivers[] = {
    SoftwareSerial(driversPins[0][0],driversPins[0][1]), 
    SoftwareSerial(driversPins[1][0],driversPins[1][1])
};

RoboClaw drivers[] = {
    RoboClaw(&serialDrivers[0],10000),
    RoboClaw(&serialDrivers[1],10000)
};




// Servos
Servo Servos[2] = {Servo(), Servo()};
const uint8_t gripperPins[] = {0x00, 0x00};
const uint8_t gripperLimits[][2] =  {{90, 90}, {10, 170}};
uint8_t gripperPos[] = {gripperLimits[0][0], gripperLimits[1][0]};

// LEDS
const uint8_t LED             = 0x0D;
const uint8_t LED2            = 0x07;

/* ROS Setup*/
// Node Handler definition
ros::NodeHandle  nh;

// Subcriber definitions
ros::Subscriber<std_msgs::UInt16> arm_sub("/drivers/arm", &armSubscriber);
ros::Subscriber<std_msgs::UInt8> wheels_sub("/drivers/wheels", &wheelsSubscriber);
ros::Subscriber<std_msgs::Empty> timeOut_sub("/arduinoWatchdog", &timeOutSubscriber );

// Ros callback/subscribe functions
void armSubscriber(const std_msgs::UInt16& arm){
    Serial.print("ARM : ");
    Serial.println(arm.data);
    uint16_t armValue = arm.data;

    /* Servo instructions */
    for(char i=0; i < 2; i++){
        if (armValue & 0x02){
            if (armValue & 0x01){
                if (gripperPos[i] < gripperLimits[i][1]) {
                    Servos[i].write(++gripperPos[i]);
                }
            } else{
                if (gripperPos[i] > gripperLimits[i][0]) {
                    Servos[i].write(--gripperPos[i]);
                }
            }
        }
        armValue >>= 2;
    }

    /* Motors instructions */
    // wristRoll
    if (armValue & 0x02){
        if (armValue & 0x01){
            drivers[0].ForwardM1(armDriversAdresses[0], 0x20); 
        }else{
            drivers[0].BackwardM1(armDriversAdresses[0], 0x20);
        }
    } else {
        drivers[0].ForwardM1(armDriversAdresses[0], 0x00);
    }
    armValue >>= 2;

    // forearmPitch
    if (armValue & 0x02){
        if (armValue & 0x01){
            drivers[0].ForwardM2(armDriversAdresses[1], 0x40); 
        }else{
            drivers[0].BackwardM2(armDriversAdresses[1], 0x40);
        }
    } else {
        drivers[0].ForwardM2(armDriversAdresses[1], 0x00);
    }
    armValue >>= 2;

    // upperarmPitch
    if (armValue & 0x02){
        if (armValue & 0x01){
            drivers[0].ForwardM1(armDriversAdresses[1], 0x40); 
        }else{
            drivers[0].BackwardM1(armDriversAdresses[1], 0x40);
        }
    } else {
        drivers[0].ForwardM1(armDriversAdresses[1], 0x00); 
    } 
    armValue >>= 2;

    // baseYaw
    if (armValue & 0x02){
        if (armValue & 0x01){
            drivers[0].ForwardM1(armDriversAdresses[2], 0x20); 
        }else{
            drivers[0].BackwardM1(armDriversAdresses[2], 0x20);
        }
    } else {
        drivers[0].ForwardM1(armDriversAdresses[2], 0x00); 
    }
}
void wheelsSubscriber(const std_msgs::UInt8& wheels){
    // left side
    uint8_t wheelsValue = wheels.data & 0xF0;
    int aux = 0;
    aux = (wheelsValue - 64)/2 + 64;
    for (char i=0; i<3; i++){
        drivers[1].ForwardBackwardM1(leftWheelsDriversAdresses[i], aux);
    }

    // rigth side
    wheelsValue = wheels.data & 0xF0;
    aux = 0;
    aux = (wheelsValue - 64)/2 + 64;
    for (char i=0; i<3; i++){
        drivers[1].ForwardBackwardM1(rightWheelsDriversAdresses[i], aux);
    }

}
void timeOutSubscriber(const std_msgs::Empty&){
    Serial.println("WATCHDOG HERE!");
    togglePin(LED2);
    ros_timer = millis();
}


// Set up
void setup(void) {
    pinMode(LED, OUTPUT);
    pinMode(LED2, OUTPUT);
    Ethernet.init(shield_CS);  // Most Arduino shields

    // Open serial communications and wait for port to open:
    Serial.begin(9600);
    while (!Serial); // wait for serial port to connect. Needed for native USB port only



    // start the Ethernet connection:
    Serial.println("Initialize Ethernet with DHCP:");
    // Ethernet.begin(mac, 3000, 4000);
    if (Ethernet.begin(mac, 3000) == 0) {
        Serial.println("Failed to configure Ethernet using DHCP");
        if (Ethernet.hardwareStatus() == EthernetNoHardware) {
            Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
        } else if (Ethernet.linkStatus() == LinkOFF) {
            Serial.println("Ethernet cable is not connected.");
        }
        // no point in carrying on, so do nothing forevermore:
        for(char i=0; i<10; i++){
            delay(100);
            togglePin(LED);
        }
        reboot();
    }
    // print your local IP address:
    Serial.print("My IP address: ");
    Serial.println(Ethernet.localIP());

    nh.getHardware()->setConnection(MASTER, MASTER_PORT);
    nh.initNode();
    nh.subscribe(arm_sub);
    nh.subscribe(wheels_sub);
    nh.subscribe(timeOut_sub);
}

void loop(void) {
    if (Ethernet.maintain()%2) reboot();
    if (Ethernet.linkStatus() != LinkON) reboot();
    // if (!nh.connected()) reboot();

    for(char i=0; i<100; i++){
        arduino_timer = millis();
        if ((arduino_timer - ros_timer) > 500) haltMovement();
        nh.spinOnce();
        delay(1);
    }
        
    togglePin(LED);
}

void togglePin(char pin){
    digitalWrite(pin, !digitalRead(pin));
}

void haltMovement(void){
    stopArm();
    stopWheels();
}

void stopArm(void){
    drivers[0].ForwardM1(armDriversAdresses[0], 0x00);
    drivers[0].ForwardM2(armDriversAdresses[1], 0x00);
    drivers[0].ForwardM1(armDriversAdresses[1], 0x00); 
    drivers[0].ForwardM1(armDriversAdresses[2], 0x00); 
}

void stopWheels(void){
    for (char i=0; i<3; i++){
        drivers[1].ForwardM1(leftWheelsDriversAdresses[i],0);
        drivers[1].ForwardM1(rightWheelsDriversAdresses[i],0);
    }
}
