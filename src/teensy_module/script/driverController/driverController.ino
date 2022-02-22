/*
    driverController.ino
*/
/* Definitions of libraries */
#include <SPI.h>
#include <Ethernet.h>
#include "RoboClaw.h"


/* Definitions of constants */ 
// Timeout Constants in ms
#define Timeout_Ethernet 10000           // HI
#define Timeout_Wheels 6000             //
#define Timeout_Arm_Science 6000        // HELO

// LEDs Constants
#undef LED_BUILTIN
#define LED_NOTIFICATION 26             // Notification Led1.
#define LED_NOTIFICATION1 27            // Notification Led1.
#define LED_NOTIFICATION2 28            // Notification Led1.

#define RGB_LED_NOTIFICATION_R 18       // RGB nRED pin.
#define RGB_LED_NOTIFICATION_G 17       // RGB nGREEN pin.
#define RGB_LED_NOTIFICATION_B 16       // RGB nBLUE pin.

#define LED_Toggle_millis 100           // Time before the led toggles in ms. 

// Ethernet Constants
#define Ethernet_CS 24                  // Chip Select pin for Ethernet module.
#define Ethernet_RST 25                 // RST pin for the Ethernet modules.
byte mac[6] = {                         // Ethernet MAC adress.
    0x80, 0x69, 0x69, 0x69, 0x69, 0x09 
};   

// Serial Ports rename
#define Serial_USB Serial               // Serial pins for the USB.
#define Serial_Arm_Science Serial2      // Serial pins for the arm.
#define Serial_Wheels Serial3           // Serial pins for the wheels.
#define Serial_Sensors_UART Serial5     // Serial pins for the sensors (UART).

// Serial Ports Baudrate
#define Baudrate_USB 38400              // Baudrate with the Serial_USB.
#define Baudrate_Arm_Science 38400      // Baudrate with the Serial_Arm_Science.
#define Baudrate_Wheels 38400           // Baudrate with the Serial_Wheels.
#define Baudrate_Sensors_UART 9600      // Baudrate with the Serial_Sensors_UART.

// Reboot constants
#define CPU_RESTART_ADDR (uint32_t *)0xE000ED0C
#define CPU_RESTART_VAL 0x5FA0004
#define CPU_RESTART (*CPU_RESTART_ADDR = CPU_RESTART_VAL)

// Motor constants
#define Motors_HaltSpeed 64
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

// Drivers constants / definitions
RoboClaw RoboClaw_Wheels = RoboClaw(&Serial_Wheels, Timeout_Wheels);                    // Roboclaw Wheels driver
RoboClaw RoboClaw_Arm_Science = RoboClaw(&Serial_Arm_Science, Timeout_Arm_Science);     // Roboclaw Arm & Science driver

// test pins
#define PushButton 15
#define Potentiometer 0



/* Global variables */
// Watchdog varables
unsigned long prevTime;
unsigned long newTime;

// Flags
bool driversReady = false;

// Testing
uint32_t auxUInt32;
bool auxBool=false;



/* Functions prototyping */

// Special Functions

void reboot(void);
void watchdogFunction(void);
void togglePin(uint8_t pin);


// Setup Functions 

void setupDrivers(void);
void setupEthernet(void);
void setupSerials(void);
void setupGPIO(void);

// Minmal Functions

void setup(void);
void loop(void);


// Control Functions

void wheels(void);
void armScience(void);
void haltMovements(void);

// Testing Functions

void analogController(void);



/* Function definitions */
// Executes a software reset on the teensy.
void reboot(void){
    haltMovements();
    Serial_USB.println("REBOOTING...");
    delay(100);
    CPU_RESTART;
}

// In case of something failing, it stops everything, and reboots itself. 
void watchdogFunction(void){
    newTime = millis();
    if ((newTime - prevTime) > LED_Toggle_millis){
        prevTime = newTime;
        togglePin(LED_NOTIFICATION1);
    } if (Ethernet.maintain()%2 || Ethernet.linkStatus() == LinkOFF){
        reboot();
    }
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

// Minimal setup function.
void setup(void){
    setupGPIO();
    setupSerials();
    setupEthernet();
    prevTime = millis();
}

// Minimal loop function.
void loop(void){
    watchdogFunction();
}

// Defines the wheels control function.
void wheels(void){

}

// Defines the arm control function.
void armScience(void){

}

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
