#include <SoftwareSerial.h>
#include <ros.h>
#include <std_msgs/Float64.h>

// // Connect the GPS Power pin to 5V
// // Connect the GPS Ground pin to ground
// // Connect the GPS TX (transmit) pin to Digital 2
// // Connect the GPS RX (receive) pin to Digital 3

// // you can change the pin numbers to match your wiring:
SoftwareSerial Serial3(2, 3);
#define Serial_Sensors_UART Serial3     // Serial pins for the sensors (UART).


void setup(void);
void loop(void);

String auxString;
std_msgs::Float64 lat, lon;
int aux;

void setup(void){
    Serial.begin(115200);
    Serial_Sensors_UART.begin(9600);
    delay(100);
}
void loop(void){
    auxString = Serial_Sensors_UART.readStringUntil(',');
    if (auxString == "$GPGGA"){
        Serial.println();
        // Serial_Sensors_UART.readStringUntil(','); // UTC
        // auxString = Serial_Sensors_UART.readStringUntil(',');
        // aux = auxString.toInt()/100;
        // lat.data = auxString.toFloat() - aux*100;
        // lat.data /= 60;
        // lat.data += aux;
        // if (Serial_Sensors_UART.readStringUntil(',') == "S")
        //     lat.data *= -1;

        // auxString = Serial_Sensors_UART.readStringUntil(',');
        // aux = auxString.toInt()/100;
        // lon.data = auxString.toFloat() - aux*100;
        // lon.data /= 60;
        // lon.data += aux;
        // if (Serial_Sensors_UART.readStringUntil(',') == "W")
        //     lon.data *= -1;

        // Serial.println("$GPGGA");
        // Serial.print("lat.data : ");
        // Serial.println(lat.data*1000);
        // Serial.print("lon.data : ");
        // Serial.println(lon.data*1000);
        
        // auxString = Serial_Sensors_UART.readStringUntil('\n');
    }
    Serial.print(auxString);
    Serial.print(",");
    Serial.println(Serial_Sensors_UART.readStringUntil('\n'));
    // else
    //     Serial_Sensors_UART.readStringUntil('\n');
}