/*
    driverController.ino

*/

/* Definitions of libraries. */
#include <SPI.h>
#include <Ethernet.h>
#include "RoboClaw.h"

#define ROSSERIAL_ARDUINO_TCP // To use the TCP version of rosserial_arduino.

/* Include ROS components. */
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt8.h>
#include "std_msgs/Empty.h"

/* Functions prototyping. */
// Special Functions.

void reboot(void);
void togglePin(uint8_t);

// Setup Functions.

void setupDrivers(void);
void setupEthernet(void);
void setupSerials(void);
void setupGPIO(void);
void setupRos(void);

// Minimal Functions.

void setup(void);
void loop(void);

// Control Functions.

void motorsWheelsCb( const std_msgs::UInt16&);
void motorsArmCb( const std_msgs::UInt8&);
void servoArmWristPitchCb( const std_msgs::UInt8&);
void servoArmGripperCb( const std_msgs::UInt8&);
void motorsScienceCb( const std_msgs::UInt8&);
void servoScienceMicroscopeCb( const std_msgs::UInt8&);
void servoScienceDispenserExteriorCb( const std_msgs::UInt8&);
void servoScienceDispenserInteriorCb( const std_msgs::UInt8&);
void watchdogUpdater(void);

void haltMovements(void);

// Testing Functions.

void analogController(void);


/* Definitions of constants. */ 
// Timeout Constants in ms.
#define Timeout_Ethernet 30000           // Timeout for Ethernet connection.
#define Timeout_Wheels 10000            // Timeout for Wheels drivers.
#define Timeout_Arm_Science 10000       // Timeout for Arm & Science drivers.
#define Timeout_Watchdog 3500

// LEDs Constants.
#undef LED_BUILTIN
#define LED_NOTIFICATION 26             // Notification Led0.
#define LED_NOTIFICATION1 27            // Notification Led1.
#define LED_NOTIFICATION2 28            // Notification Led2.

#define RGB_LED_NOTIFICATION_R A22      // RGB nRED pin.
#define RGB_LED_NOTIFICATION_G A21      // RGB nGREEN pin.
#define RGB_LED_NOTIFICATION_B A20      // RGB nBLUE pin.

#define LED_SUCCESS_BLINK_TIME 100      // Time before the led toggles in ms. 

// Ethernet Constants.
#define Ethernet_CS 24                  // Chip Select pin for Ethernet module.
#define Ethernet_RST 25                 // RST pin for the Ethernet modules.
byte mac[6] = {                         // Ethernet MAC adress.
    0x80, 0x69, 0x69, 0x69, 0x69, 0x09 
};   
IPAddress server(10,0,0,20);            // Master IP.
const uint16_t serverPort = 11411;      // Master rosserial socket server port.

// Serial Ports rename.
#define Serial_USB Serial               // Serial pins for the USB.
#define Serial_Arm_Science Serial2      // Serial pins for the arm.
#define Serial_Wheels Serial3           // Serial pins for the wheels.
#define Serial_Sensors_UART Serial5     // Serial pins for the sensors (UART).

// Serial Ports Baudrate.
#define Baudrate_USB 38400              // Baudrate with the Serial_USB.
#define Baudrate_Arm_Science 38400      // Baudrate with the Serial_Arm_Science.
#define Baudrate_Wheels 38400           // Baudrate with the Serial_Wheels.
#define Baudrate_Sensors_UART 9600      // Baudrate with the Serial_Sensors_UART.

// Reboot constants, magic, link: https://forum.pjrc.com/threads/60756-How-do-I-software-reset-the-Teensy-3-6
#define CPU_RESTART_ADDR (uint32_t *)0xE000ED0C
#define CPU_RESTART_VAL 0x5FA0004
#define CPU_RESTART (*CPU_RESTART_ADDR = CPU_RESTART_VAL)

// Motor constants.
#define Motors_HaltSpeed 64
#define Motors_MovementSpeedDiff 0x10
#define Motors_Wheels_Amount_PerSide 3
#define Motors_Arm_Amount 4
#define Motors_Science_Amount 1
const uint8_t Motors_Wheels_Left[Motors_Wheels_Amount_PerSide] = {      // Wheels Left motors array
    0x80, 0x82, 0x84
};
const uint8_t Motors_Wheels_Right[Motors_Wheels_Amount_PerSide] = {     // Wheels Right motors arra
    0x81, 0x83, 0x85
};
const uint8_t Motors_Arm[Motors_Arm_Amount] = {                         // Arm motors array
    0x80, 0x81, 0x82, 0x83
};
const uint8_t Motors_Science[Motors_Science_Amount] = {                 // Science motors array
    0x84
};

// Drivers constants / definitions.
RoboClaw RoboClaw_Wheels = RoboClaw(&Serial_Wheels, Timeout_Wheels);                    // Roboclaw Wheels driver
RoboClaw RoboClaw_Arm_Science = RoboClaw(&Serial_Arm_Science, Timeout_Arm_Science);     // Roboclaw Arm & Science driver

// // test pins .
// #define PushButton 15
// #define Potentiometer 0

// ROS constants.
#define topic_motors_wheels                     "/wheels/drivers"
#define topic_motors_arm                        "/arm/drivers"
#define topic_servo_arm_wristPitch              "/arm/servos/wrist"
#define topic_servo_arm_gripper                 "/arm/servos/gripper"
#define topic_motors_science                    "/science/drivers"
#define topic_servo_science_microscope          "/science/servos/microscope"
#define topic_servo_science_dispenser_exterior  "/science/servos/dispenser/exterior"
#define topic_servo_science_dispenser_interior  "/science/servos/dispenser/interior"

#define topic_watchdog                          "/watchdog_topic"

/* Global variables. */
// ROS NodeHandler.
ros::NodeHandle nh;

// ROS Subscribers
ros::Subscriber<std_msgs::UInt16> sub_motors_wheels(topic_motors_wheels, &motorsWheelsCb);
ros::Subscriber<std_msgs::UInt8> sub_motors_arm(topic_motors_arm, &motorsArmCb);
ros::Subscriber<std_msgs::UInt8> sub_servo_arm_wristPitch (topic_servo_arm_wristPitch, &servoArmWristPitchCb);
ros::Subscriber<std_msgs::UInt8> sub_servo_arm_gripper (topic_servo_arm_gripper, &servoArmGripperCb);
ros::Subscriber<std_msgs::UInt8> sub_motors_science (topic_motors_science, &motorsScienceCb);
ros::Subscriber<std_msgs::UInt8> sub_servo_science_microscope (topic_servo_science_microscope, &servoScienceMicroscopeCb);
ros::Subscriber<std_msgs::UInt8> sub_servo_science_dispenser_exterior (topic_servo_science_dispenser_exterior, &servoScienceDispenserExteriorCb);
ros::Subscriber<std_msgs::UInt8> sub_servo_science_dispenser_interior (topic_servo_science_dispenser_interior, &servoScienceDispenserInteriorCb);

ros::Subscriber<std_msgs::Empty> subWatchdog (topic_watchdog, &watchdogUpdater);

// Watchdog varables.
unsigned long watchPrevTime;
unsigned long ledPrevTime;
unsigned long actualTime;


// Flags.
bool driversReady = false;

// // Testing.
// uint32_t auxUInt32;
// bool auxBool=false;



/* Function definitions. */
// Executes a software reset on the teensy.
void reboot(void){
    haltMovements();
    Serial_USB.println("REBOOTING...");
    delay(100);
    CPU_RESTART;
}

// Toogles the given pin to the opposite state.
void togglePin(uint8_t pin){
    digitalWrite(pin, !digitalRead(pin));
}

// Defines the setup configuration of the drivers.
void setupDrivers(void){
    RoboClaw_Wheels.begin(Baudrate_Wheels);
    RoboClaw_Arm_Science.begin(Baudrate_Arm_Science);
    driversReady = true;
}

// Defines the setup configuration of the Ethernet.
void setupEthernet(void){
    pinMode(Ethernet_RST, OUTPUT);
    digitalWrite(Ethernet_RST, LOW);
    digitalWrite(Ethernet_RST, HIGH);

    Ethernet.init(Ethernet_CS);

    Serial_USB.println("\nInitialize Ethernet : ");

    if (Ethernet.begin(mac, Timeout_Ethernet) == 0){
        Serial_USB.println("Failed to get IP address...");
        if (Ethernet.hardwareStatus() == EthernetNoHardware)
            Serial_USB.println("Ethernet shield was not found.");
        else if (Ethernet.linkStatus() == LinkOFF)
            Serial_USB.println("Ethernet cable is not connected.");
        // No point on continuing, so restart and attempt again.
        delay(100);
        reboot();
    }

    digitalWrite(LED_NOTIFICATION, LOW);
    Serial_USB.print("My IP address is : ");
    Serial_USB.println(Ethernet.localIP());
}

// Defines the setup configuration of the Serial Ports.
void setupSerials(void){
    Serial_USB.begin(Baudrate_USB);
    Serial_Sensors_UART.begin(Baudrate_Sensors_UART);
    delay(400);
}

// Defines the setup configuration of the GPIO pins.
void setupGPIO(void){
    pinMode(LED_NOTIFICATION, OUTPUT);
    pinMode(LED_NOTIFICATION1, OUTPUT);
    pinMode(LED_NOTIFICATION2, OUTPUT);
    
    digitalWrite(LED_NOTIFICATION, HIGH);
    digitalWrite(LED_NOTIFICATION1, LOW);
    digitalWrite(LED_NOTIFICATION2, LOW);
}

// Defines the setup configuration of the ROS environment.
void setupRos(void){
    nh.getHardware()->setConnection(server, serverPort);
    nh.initNode();
    nh.subscribe(sub_motors_wheels);
    nh.subscribe(sub_motors_arm);
    nh.subscribe(sub_servo_arm_wristPitch);
    nh.subscribe(sub_servo_arm_gripper);
    nh.subscribe(sub_motors_science);
    nh.subscribe(sub_servo_science_microscope);
    nh.subscribe(sub_servo_science_dispenser_exterior);
    nh.subscribe(sub_servo_science_dispenser_interior);
    nh.subscribe(subWatchdog);
}

// Minimal setup function.
void setup(void){
    setupGPIO();
    setupSerials();
    setupEthernet();
    setupDrivers();
    delay(1000);
    setupRos();
    watchPrevTime = millis();
    ledPrevTime = millis();
}

// Minimal loop function.
void loop(void){
    actualTime = millis();

    if ((actualTime - watchPrevTime) > Timeout_Watchdog || 
        Ethernet.maintain()%2 || 
        Ethernet.linkStatus() == LinkOFF
    )
        reboot();

    if ((actualTime - ledPrevTime) > LED_SUCCESS_BLINK_TIME){
        ledPrevTime = actualTime;
        togglePin(LED_NOTIFICATION1);
    }

    nh.spinOnce();
    delay(10);
}

// Callback function for wheels motors.
void motorsWheelsCb( const std_msgs::UInt16 &msg){
    int left_speed = msg.data >> 8;
    int right_speed = msg.data & 0xFF;
    for(uint8_t i = 0; i < Motors_Wheels_Amount_PerSide; i++){
        RoboClaw_Wheels.ForwardBackwardM1(Motors_Wheels_Left[i], left_speed);
        RoboClaw_Wheels.ForwardBackwardM1(Motors_Wheels_Right[i], right_speed);
    }
}

// Callback function for arm motors.
void motorsArmCb( const std_msgs::UInt8 &msg){
    uint8_t data = msg.data;
    for (uint8_t i = 0; i < Motors_Arm_Amount; i++){
        if (data & 0x80){
            if (data & 0x40)
                RoboClaw_Arm_Science.ForwardBackwardM1(Motors_Arm[i], Motors_HaltSpeed + Motors_MovementSpeedDiff);
            else 
                RoboClaw_Arm_Science.ForwardBackwardM1(Motors_Arm[i], Motors_HaltSpeed - Motors_MovementSpeedDiff);
        } else 
            RoboClaw_Arm_Science.ForwardBackwardM1(Motors_Arm[i], Motors_HaltSpeed);
        data = data << 2;
    }
}

void servoArmWristPitchCb( const std_msgs::UInt8 &msg){

}

void servoArmGripperCb( const std_msgs::UInt8 &msg){

}

// Callback function for Science motors.
void motorsScienceCb( const std_msgs::UInt8 &msg){
    uint8_t data = msg.data;
    for (uint8_t i = 0; i < Motors_Science_Amount; i++){
        if (data & (0x02 << (Motors_Science_Amount - 1 - i))){
            if (data & (0x01 << (Motors_Science_Amount - 1 - i)))
                RoboClaw_Arm_Science.ForwardBackwardM1(Motors_Science[i], Motors_HaltSpeed + Motors_MovementSpeedDiff);
            else 
                RoboClaw_Arm_Science.ForwardBackwardM1(Motors_Science[i], Motors_HaltSpeed - Motors_MovementSpeedDiff);
        } else
            RoboClaw_Arm_Science.ForwardBackwardM1(Motors_Science[i], Motors_HaltSpeed);
    }
}

void servoScienceMicroscopeCb( const std_msgs::UInt8 &msg){

}

void servoScienceDispenserExteriorCb( const std_msgs::UInt8 &msg){

}
void servoScienceDispenserInteriorCb( const std_msgs::UInt8 &msg){

}

// In case of something failing, it stops everything, and reboots itself. 
void watchdogUpdater(void){
    watchPrevTime = millis();
}

// Stop Movements
void haltMovements(void){
    if (!driversReady) return;
    for (int i = 0; i < Motors_Wheels_Amount_PerSide; i++){
        RoboClaw_Wheels.ForwardBackwardM1(Motors_Wheels_Left[i],Motors_HaltSpeed);
        RoboClaw_Wheels.ForwardBackwardM1(Motors_Wheels_Right[i],Motors_HaltSpeed);
    }
    for (int i = 0; i < Motors_Arm_Amount; i++)
        RoboClaw_Arm_Science.ForwardBackwardM1(Motors_Arm[i], Motors_HaltSpeed);
    for (int i = 0; i < Motors_Science_Amount; i++)
        RoboClaw_Arm_Science.ForwardBackwardM1(Motors_Science[i], Motors_HaltSpeed);
}
