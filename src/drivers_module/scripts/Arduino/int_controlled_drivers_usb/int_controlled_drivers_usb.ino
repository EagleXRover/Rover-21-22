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

/* ETHERNET */
// Shield 
const byte mac[6] = {0x80, 0x69, 0x69, 0x69, 0x69, 0x09};
const unsigned char shield_CS       = 0x08;

// ROS MASTER
IPAddress MASTER(10, 0, 0, 20);
const unsigned int MASTER_PORT = 11411;

/* Actuators */
// Drivers
const unsigned char drivers[2][2]   = {{0x0B, 0x0A}, {0x0C, 0x09}}; 

// Servos
Servo gripper[2] = {Servo(), Servo()};
const unsigned char gripperServoPin =  0x00;
const unsigned char wristServoPin   =  0x00;

// LEDS
const unsigned char LED             = 0x0D;
const unsigned char LED2            = 0x07;

/* ROS Setup*/
// Node Handler definition
ros::NodeHandle  nh;

// Subcriber definitions
ros::Subscriber<std_msgs::UInt16> arm_sub("/drivers/arm", &armSubscriber);
ros::Subscriber<std_msgs::UInt8> wheels_sub("/drivers/wheels", &wheelsSubscriber);
ros::Subscriber<std_msgs::Empty> timeOut_sub("/arduinoWatchdog", &timeOutSubscriber );

// Ros callback/subscribe functions
void armSubscriber(const std_msgs::UInt16& arm){
    
}
void wheelsSubscriber(const std_msgs::UInt8& wheels){

}
void timeOutSubscriber(const std_msgs::Empty&){
    togglePin(LED2);
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
    if (!nh.connected()) reboot();

    for(char i=0; i<100; i++){
        nh.spinOnce();
        delay(1);
    }
        
    togglePin(LED);
}

void togglePin(char pin){
    digitalWrite(pin, !digitalRead(pin));
}
