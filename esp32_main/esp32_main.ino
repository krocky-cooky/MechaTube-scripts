#include <Arduino.h>
#include <CAN.h>
#include <math.h>

#define PIN_CANRX 32
#define PIN_CANTX 33
#define PIN_MOTORPOWER 25
#define KP 0.1
#define KD 1.0

// 特殊なCANコマンド
const uint8_t msgEnter[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc};
const uint8_t msgExit[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd};
const uint8_t msgSetPosToZero[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe};

// ユーザの手元にあるスイッチなど、トルク出力をON/OFFする指令
bool handSwitch = false;

// シリアル通信で受信した指令値
bool motorPowerCommand = false;
bool motorControlCommand = false;
float torqueCommand = 0.0;
float speedCommand = 0.0;

// 実際にモータやコンバータに送信している指令値
bool motorPowerSending = false;
bool motorControlSending = false;
float torqueSending = 0.0;
float speedSending = 0.0;

void setup() {
  Serial.begin(9600);
  while (!Serial)

  pinMode(PIN_MOTORPOWER, OUTPUT);

  CAN.setPins(PIN_CANRX, PIN_CANTX);
  if (!CAN.begin(1000E3)) {
    Serial.println("Starting CAN failed!");
    while (1);
  }
  CAN.onReceive(&can_onReceive);
  // modify half speed problem
  // reference: https://github.com/sandeepmistry/arduino-CAN/issues/62
  volatile uint32_t* pREG_IER = (volatile uint32_t*)0x3ff6b010;
  *pREG_IER &= ~(uint8_t)0x10;
}

void loop() {

  // シリアル通信で指令を受け取る
  serial_decodeIncomingCommand();

  // 指令値に応じて、モータに指令を送信する
    setMotorPower(true);
    setMotorControl(true);

  if (handSwitch) {
    torqueSending = firstOrderDelay(torqueCommand);
    speedSending = firstOrderDelay(speedCommand);
  } else {
    torqueSending = firstOrderDelay(0.0);
    speedSending = firstOrderDelay(0.0);
  }

  if (fabsf(speedSending) < 0.01) {  // 速度指令が0のとき、トルク指令と判断
    uint8_t msg[8];
    can_packCmd(msg, 0.0, 0.0, 0.0, 0.0, torqueSending);
    can_send(msg, 8);
  } else {                           // 速度指令が0より大きいとき、トルク指令値を無視して速度指令と判断
    uint8_t msg[8];
    can_packCmd(msg, 0.0, speedSending, 0.0, KD, 0.0);
    can_send(msg, 8);
  }

  // constant speed
  while (1) {
    Serial.print("Sending packet ... ");
    uint8_t msg[8] = {0x7f, 0xff, 0x84, 0x30, 0x0, 0x33, 0x37, 0xff};
    can_send(msg, 8);
    Serial.println("done");
    delay(1000);
  }

  delay(1000);
}


void setMotorPower(bool command) {
  if (command == 1 && motorPowerSending == 0) {
    Serial.println("[setMotorPower] motor power: ON");
    digitalWrite(PIN_MOTORPOWER, HIGH);
    motorPowerSending = true;
  }
}

void setMotorControl(bool command) {
  if (command == 1 && motorControlSending == 0) {
    if (!motorPowerSending) {
      setMotorPower(true);
      delay(3000);
    }
    Serial.println("[setMotorControl] motor control mode: ON");
    can_send(msgEnter, sizeof(msgEnter));
    motorControlSending = true;
  }
}