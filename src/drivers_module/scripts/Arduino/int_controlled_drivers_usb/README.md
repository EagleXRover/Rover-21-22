drivers:
0 = Arm drivers
1 = Wheels drivers

drivers[X]:
0 = drivers S1 / MOSI / ArduinoTx
1 = drivers S2 / MISO / ArduinoRx

In this script the arduino is working as master for the drivers, so instead of calling them drivers S1 and S2, we are going to use the denomination of MISO and MOSI, even if they are not using SPI pins for this.

Servos:
0 = gripper_hand
1 = wrist_pitch



