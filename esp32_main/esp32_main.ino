#include <Arduino.h>
#include <CAN.h>

#define MOTOR_ID 64
#define DRIVER_ID 0
#define PIN_CANRX 32
#define PIN_CANTX 33

bool motorPowerCommand = false;
bool motorControlCommand = false;
float torqueCommand = 0.0;
float speedCommand = 0.0;

void setup() {
  Serial.begin(9600);
  while (!Serial);
  
  CAN.setPins(PIN_CANRX, PIN_CANTX);
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
  Serial.print("Sending packet to enter motor control mode... ");
  uint8_t msg[8] = {0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xfc};
  sendCan(msg, 8);
  Serial.println("done");

  delay(1000);

  // constant speed 
  while(1) {
    Serial.print("Sending packet ... ");
    uint8_t msg[8] = {0x7f, 0xff, 0x84, 0x30, 0x0, 0x33, 0x37, 0xff};
    sendCan(msg, 8);
    Serial.println("done");
    delay(1000);
  }

  delay(1000);
  
}

/// @brief CAN送信を行う
/// @param[in] data 送りたいデータ配列
/// @param[in] length 送信するバイト数(上限は8byte)
/// @return 0:fail, 1:success
int sendCan(uint8_t* data, int length) {
  if (length < 1 || length > 8) return 0;

  if (!CAN.beginPacket(MOTOR_ID)) return 0;
  if (!CAN.write(data, length)) return 0;
  if (!CAN.endPacket()) return 0;

  return 1;
}