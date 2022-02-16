#include <SoftwareSerial.h>

#define LED_BUILTIN 6       // Built-in Led
#define mySerialRxPin 27    // SoftSerial Rx Pin
#define mySerialTxPin 0     // SoftSerial Tx Pin

#define secondSerial mySerial
#define UsbBaudrate 38400
#define secondSerialBaudrate 38400


SoftwareSerial mySerial = SoftwareSerial(mySerialRxPin, mySerialTxPin);
unsigned long int 

void setup(){
    Serial.begin(UsbBaudrate);
    secondSerial.begin(secondSerialBaudrate);
    pinMode(LED,OUTPUT);
}