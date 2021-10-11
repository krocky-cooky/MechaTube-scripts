#include <Arduino.h>
#include <CAN.h>

#define PIN_CANRX 32
#define PIN_CANTX 33
#define PIN_POWER 26
#define PIN_HANDSWICH 35
#define FORCE_THRESHOLD_OF_HANDSWICH 10.0 //手元スイッチのオンオフを識別するための、スイッチにかかる力の閾値[N]
#define KP 0.1
#define KD 1.0

// 速度制御したいとき0,トルク制御したいとき1になるフラグ
bool torqueCtrlMode = 0;

// ユーザの手元にあるスイッチなど、トルク出力をON/OFFする指令
bool handSwitch = false;

// シリアル通信で受信した指令値
bool powerCommand = false;        // コンバータからモータへの電源供給指令
bool controlCommand = false;      // モータ制御モードに入るかどうかの指令
float torqueCommand = 0.0;        // トルク指令値 [Nm]
float speedCommand = 0.0;         // 速度指令値 [rad/s]

// 実際にモータやコンバータに送信している指令値
bool powerSending = false;
bool controlSending = false;
float torqueSending = 0.0;
float speedSending = 0.0;

// (待ち時間等の処理に使う)時刻の記録
unsigned long timeNow = 0;        // 各loopの開始時刻 [us]
unsigned long timePrev = 0;       // 直前のloopの開始時刻 [us]
unsigned long dtMicros = 0;       // 直前のloopからの経過時間 [us]
unsigned long timePowerOn = 0;    // モータの電源を入れた時刻 [us]
unsigned long timeControlOn = 0;  // モータ制御モードに入った時刻 [us]

// 直近にCANで送受信したデータを記憶しておく変数(表示に利用)
float positionSent, speedSent, kpSent, kdSent, torqueSent;  // 直近のCAN送信データを記録しておく
volatile float positionReceived, speedReceived, torqueReceived;      // 直近のCAN受信データを記録しておく
volatile uint8_t canReceivedMsg[6];  // 直近のCAN受信データそのものを記録しておく

// CAN受信割込みとmainloopの双方からアクセスする変数の排他処理
portMUX_TYPE onCanReceiveMux = portMUX_INITIALIZER_UNLOCKED;


void setup() {
  pinMode(PIN_POWER, OUTPUT);
  digitalWrite(PIN_POWER, LOW);
  
  Serial.begin(9600);
  while (!Serial) delay(1);

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
  
  Serial.println("[setup] setup comleted");
}

void loop() {

  // 開始時刻の記録
  timePrev = timeNow;
  timeNow = micros();
  dtMicros = timeNow - timePrev;

  // シリアル通信で指令値を受け取る
  serial_getIncomingCommand(&powerCommand, &controlCommand, &torqueCtrlMode, &torqueCommand, &speedCommand);

  // 指令値に応じて、モータの電源とモータ制御モードを変更する
  setPower(powerCommand);
  setControl(controlCommand);

  // 手元スイッチのON/OFFを取得する
  float touchSensorValue = analogRead(PIN_HANDSWICH); //手元スイッチのセンサの値
  float force_on_handSwich = (4096 - touchSensorValue) / 4096 * 20; //手元スイッチのセンサにかかる力[N]
  if (force_on_handSwich >= FORCE_THRESHOLD_OF_HANDSWICH){
    handSwitch = true;
  } else {
    handSwitch = false;
  }
  Serial.print("handSwitch=");
  Serial.print(handSwitch);
  Serial.print(" ");
  
  // モータ制御モードに入っているとき、送信値を計算し、CANを送信する
  if (controlSending) {

    // トルク指令モードのとき
    if (torqueCtrlMode) {
      if (handSwitch) {     // 手元スイッチONのとき送信値をゆっくり指令値に近づけ、OFFのときは0に近づける
        torqueSending = firstOrderDelay_torque(torqueCommand, (float)dtMicros/1e6);
      } else {
        torqueSending = firstOrderDelay_torque(0.0, (float)dtMicros/1e6);
      }
      speedSending = 0.0;            // 速度送信値は不要なので0とする
      firstOrderDelay_resetSpeed();  // 速度の1次遅れ計算用変数をリセット

      can_sendCommand(0.0, 0.0, 0.0, 0.0, torqueSending);  // 送信

    // 速度指令モードのとき
    } else {
      if (handSwitch) {
        speedSending = firstOrderDelay_speed(speedCommand, (float)dtMicros/1e6);
      } else {
        speedSending = firstOrderDelay_speed(0.0, (float)dtMicros/1e6);
      }
      torqueSending = 0.0;
      firstOrderDelay_resetTorque();

      can_sendCommand(0.0, speedSending, 0.0, KD, 0.0);
    }
  
  // モータ制御モードに入っていないとき、全ての変数を0にリセットしておく
  } else {
    torqueSending = 0.0;
    speedSending = 0.0;
    firstOrderDelay_resetTorque();
    firstOrderDelay_resetSpeed();
  }

  delay(100);

  portENTER_CRITICAL_ISR(&onCanReceiveMux);  // CAN受信割込みと共有する変数へのアクセスはこの中で行う
  
  // Serial.printf("{\"torque\":%f, \"speed\":%f, \"position\":%f}\n", torqueReceived, speedReceived, positionReceived);
  Serial.printf("%x %x %x %x %x %x\n", canReceivedMsg[0], canReceivedMsg[1], canReceivedMsg[2], canReceivedMsg[3], canReceivedMsg[4], canReceivedMsg[5]);

  portEXIT_CRITICAL_ISR(&onCanReceiveMux);  // CAN受信割込みと共有する変数へのアクセスはこの中で行う
}


void setPower(bool command) {
  if (command == 1 && powerSending == 0) {
    if (digitalRead(PIN_POWER) == LOW) {
      digitalWrite(PIN_POWER, HIGH);
      timePowerOn = micros();
      Serial.println("[setPower] set PIN_POWER HIGH");
    }
    if ((long)(timeNow - timePowerOn) > 2000000) {
      powerSending = 1;
      Serial.println("[setPower] motor power: ON");
    }
  }

  if (command == 0) {
    if (controlSending == 1) {
      setControl(0);
    } else {
      if (digitalRead(PIN_POWER) == HIGH) {
        digitalWrite(PIN_POWER, LOW);
        powerSending = 0;
        Serial.println("[setPower] motor power: OFF");
      }
    }
  }
}

void setControl(bool command) {
  if (command == 1 && powerSending == 0) {
    setPower(1);
  }

  if (command == 1 && powerSending == 1 && controlSending == 0) {
    can_sendControl(1);
    controlSending = 1;
    Serial.println("[setControl] motor control mode: ON");
  }

  if (command == 0 && controlSending == 1) {
    can_sendCommand(0.0, 0.0, 0.0, 0.0, 0.0);
    can_sendControl(0);
    controlSending = 0;
    Serial.println("[setControl] motor control mode: OFF");
  }
}
