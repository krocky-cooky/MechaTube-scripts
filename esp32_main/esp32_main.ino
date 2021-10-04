// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include <CAN.h>
#define MOTOR_ID 64
#define DRIVER_ID 0

bool isInMotorControlMode = false;

void setup() {
  Serial.begin(9600);
  while (!Serial);
  
  CAN.setPins(32,33);
  if (!CAN.begin(1000E3)) {
    Serial.println("Starting CAN failed!");
    while (1);
  }
  // modify half speed problem
  // reference: https://github.com/sandeepmistry/arduino-CAN/issues/62
  volatile uint32_t* pREG_IER = (volatile uint32_t*)0x3ff6b010; 
  *pREG_IER &= ~(uint8_t)0x10;
}

void loop() {
  
  delay(1000);

  // enter the motor control mode
  Serial.print("Sending packet ... ");
  CAN.beginPacket(MOTOR_ID);
  uint8_t msgToEnter[8] = {0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xfc};
  CAN.write(msgToEnter, 8);
  CAN.endPacket();
  Serial.println("done");

  delay(1000);

  // constant speed 
  while(1) {
    Serial.print("Sending packet ... ");
    CAN.beginPacket(MOTOR_ID);
    uint8_t msgToEnter[8] = {0x7f, 0xff, 0x84, 0x30, 0x0, 0x33, 0x37, 0xff};
    CAN.write(msgToEnter, 8);
    CAN.endPacket();
    Serial.println("done");
    delay(1000);
  }

  delay(1000);
  
}
