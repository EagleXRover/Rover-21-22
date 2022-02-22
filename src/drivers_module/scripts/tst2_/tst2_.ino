// tst2.ino

#define LED_Toggle_millis 500           // Time before the led toggles in ms. 


// Reboot constants
#define CPU_RESTART_ADDR (uint32_t *)0xE000ED0C
#define CPU_RESTART_VAL 0x5FA0004
#define CPU_RESTART (*CPU_RESTART_ADDR = CPU_RESTART_VAL)


/* Global variables */
// Watchdog
unsigned long prevTime;
unsigned long newTime;

// test pins
#define PushButton 15


void setup() {
  // put your setup code here, to run once:
  pinMode(PushButton, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  //pinMode(A21, OUTPUT);
  Serial.begin(38400);
}

void loop() {
  // put your main code here, to run repeatedly:
  newTime = millis();
  if ((newTime - prevTime) >= LED_Toggle_millis){
    prevTime = newTime;
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    analogWrite(A21, (digitalRead(LED_BUILTIN))?0:1024);
    Serial.println(analogRead(A21));
    
  }
  if (digitalRead(PushButton) == LOW) CPU_RESTART;
}
