#include <Arduino.h>
#include <CAN.h>
#include <ESPAsyncWebServer.h>
#include <WiFi.h>
#include <math.h>
#include <stdio.h>

#include "esp_task.h"

#include "Filter.hpp"
#include "Mode.hpp"
#include "MotorController.hpp"
#include "Secrets.h"
#include "SerialCommunication.hpp"
#include "Tmotor.h"
#include "TouchSwitch.hpp"
#include "websocket.h"

// 定数等
#define MOTOR_ID 64
#define DRIVER_ID 0
#define PIN_CANRX 32
#define PIN_CANTX 33
#define PIN_POWER 26
#define PIN_HANDSWITCH 35
#define KD 1.0
// #define TAU_TRQ 1.0            // 一次遅れ系によるトルク指令の時定数[s]
// #define TAU_SPD 1.0            // 一次遅れ系による速度指令の時定数[s]
#define CONTROL_INTERVAL 10000 // 制御周期[us]

// 閾値等
#define HANDSWITCH_VOLTAGE_THRESHOLD 10.0 // 手元スイッチのオンオフを識別するための、スイッチアナログ入力ピンの電圧閾値 [V]
#define MAX_LOGNUM 1024                   // 筋力測定の最大ログ数

// フラグ等
Mode modeCommand = Mode::TrqCtrl; // 制御対象を表すフラグ. Mode.hppに一覧で記載

// 指令値
bool power = false;        // コンバータ電源ON/OFF
bool motorControl = false; // モータ制御モードON/OFF
float posCommand = 0.0;    // 位置指令値[rad]
float spdCommand = 0.0;    // 速度指令値[rad/s]
float kpCommand = 0.0;     // 位置フィードバックゲイン
float kdCommand = 0.0;     // 速度フィードバックゲイン
float trqCommand = 0.0;    // トルク指令値[Nm]

hw_timer_t *timer0 = NULL;
TaskHandle_t onTimerTaskHandle = NULL;

SerialCommunication serialCommunication;
ESP32BuiltinCAN esp32BuiltinCAN(PIN_CANRX, PIN_CANTX);
Tmotor tmotor(esp32BuiltinCAN, MOTOR_ID, DRIVER_ID);
MotorController motor(tmotor);
TouchSwitch touchSwitch(PIN_HANDSWITCH, HANDSWITCH_VOLTAGE_THRESHOLD);

FirstLPF firstOrderDelayTrq;
FirstLPF firstOrderDelaySpd;

void IRAM_ATTR onTimer()
{
  BaseType_t taskWoken;
  xTaskNotifyFromISR(onTimerTaskHandle, 0, eNoAction, &taskWoken); // おまじない。onTimerTaskに通知を送信する
}

void setup()
{
  pinMode(PIN_POWER, OUTPUT);
  digitalWrite(PIN_POWER, LOW);

  Serial.begin(115200);
  while (!Serial)
    delay(1);

  if (!esp32BuiltinCAN.begin(1000E3)) {
    while (1)
      delay(1);
  }

  tmotor.init();
  motor.init(0.8, 0.8); // motor.init(Pゲイン、Iゲイン)  // 220806:Pゲイン1.0以上だと速度ゼロ指令時に震えた
  
  // WiFIのsetup
  if (!WiFi.config(ESP32_IP_ADDRESS, ESP32_GATEWAY, ESP32_SUBNET_MASK)) {
    Serial.println("Failed to configure!");
  }
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println(WiFi.localIP());

  websocketInit();  // websocketセットアップ

  Serial.println("[setup] setup comleted");
  // firstOrderDelayTrq.setTau(TAU_TRQ); // トルクの1次遅れフィルタを宣言
  // firstOrderDelaySpd.setTau(TAU_SPD); // 速度の1次遅れフィルタを宣言

  xTaskCreatePinnedToCore(onTimerTask, "onTimerTask", 8192, NULL, ESP_TASK_TIMER_PRIO - 1, &onTimerTaskHandle, APP_CPU_NUM); // タイマー割り込みで実行するタスクを登録

  uint16_t prescaler = getApbFrequency() / 1000000; // タイマーのカウントアップ周波数が1MHzとなるようプリスケーラを計算
  timer0 = timerBegin(0, prescaler, true);          // タイマーを初期化
  timerAlarmWrite(timer0, CONTROL_INTERVAL, true);  // 割り込み間隔を設定
  timerAttachInterrupt(timer0, onTimer, true);      // 割り込み関数を登録. edge=trueでエッジトリガ
  timerAlarmEnable(timer0);                         // タイマー割り込みを起動

  Serial.printf("[setup] setup comleted\n");
}

/**
 * @brief CONTROL_INTERVAL [us] おきに実行されるタスク。
 * 一定間隔で処理することが重要なモータ制御などをここに記述する。
 * できるだけ短時間で完了するよう気を付ける。
 * 待ち時間のかかる作業(printf, web接続など)は極力メインループに記述すること。
 */
void onTimerTask(void *pvParameters)
{
  while (1) {
    xTaskNotifyWait(0, 0, NULL, portMAX_DELAY); // おまじない。xTaskNotifyFromISRから通知を受けるまで待機

    // 指令値に応じて、モータの電源とモータ制御モードを変更する
    setPower(power);
    setControl(motorControl);

    // 手元スイッチのON/OFFを取得する
    bool handSwitch = touchSwitch.getState();
    float trqSend = 0.0;
    float spdSend = 0.0;

    // モータ制御モードに入っているとき、送信値を計算し、CANを送信する
    if (motorControl) {
      if (modeCommand == Mode::TrqCtrl) { // トルク制御モードのとき
        motor.startTrqCtrl();             // トルク制御を開始
        motor.setSpdLimit(2.0, 3.0);      // 定トルク制御時の速度制限を設定。2.0rad/sに達したらトルクを減少させはじめ、3.0rad/sでトルク0にする
        motor.setTrqRef(trqCommand);      // トルク目標値を代入

      } else if (modeCommand == Mode::SpdCtrl) { // 速度制御モードのとき
        motor.startSpdCtrl();                    // 速度制御を開始
        motor.setTrqLimit(3.0);                  // 定速制御時のトルク上限を設定
        motor.setSpdRef(spdCommand);             // 速度目標値を代入

      } else {
        motor.stopCtrl();
      }
    } else {            // モータ制御モードに入っていないとき
      motor.stopCtrl(); // 制御を終了
    }

    if (digitalRead(PIN_POWER) == HIGH) { // コンバータ電源ON時(=CAN送信できるとき)のみモータ更新
      motor.update(CONTROL_INTERVAL);
    }
  }
}

void loop()
{
  // シリアル通信でコマンドを受信し、反映
  // 現在はつねにserialCommunicationモジュールが保持する指令値を反映するコードになっているので、毎loopごとにcommandが直近のシリアル受信値で上書きされる
  // retval を確認し、0でないときのみserialCommunicationの値を反映するようにすれば、別の方法(webSocket等)で受信したコマンドと共存できるはず
  char retval = serialCommunication.receive();
  power = serialCommunication.power;
  motorControl = serialCommunication.motorControl;
  modeCommand = serialCommunication.mode;
  trqCommand = serialCommunication.trq;
  spdCommand = serialCommunication.spd;

  // CAN受信ログを1secおきにprint
  static unsigned long time_last_print = 0;
  if (millis() - time_last_print > 100) { // ここの数値をいじるとログ取得間隔[ms]を調整可
    time_last_print = millis();
    Tmotor::Log log;
    while (tmotor.logAvailable() > 0) { // ログが1つ以上たまっていたら
      log = tmotor.logRead();           // ログをひとつ取得
    }
    // 全ログが出るとうるさいので、最新の数値のみを出すようにした
    // Serial.printf(
    //     "{\"timestamp\": %d, \"trq\":%.3f, \"spd\":%.3f, \"pos\":%.3f, \"integratingAngle\": %.3f}\n",
    //     log.timestamp,
    //     log.trq,
    //     log.spd,
    //     log.pos,
    //     log.integratingAngle);
    Serial.printf(
        "\"trq\":%.3f, \"spd\":%.3f\n",
        log.trq,
        log.spd);
    // ログをwebSocketで配信
    char json_data[256];
    sprintf(json_data, "{\"torque\":%.3f, \"speed\":%.3f, \"position\":%.3f, \"integratingAngl\":%.3f}", log.trq, log.spd, log.pos, log.integratingAngle);
    webSocketSend(json_data);
  }

  // コンバータの電圧を表示
  // float voltageOfConverter = analogRead(34) * 3.3 * 21 / 4096; //コンバータの電圧の値
  // Serial.printf("the voltage of converter = %f\n", voltageOfConverter);

  delay(1);
}

void setPower(bool command)
{
  if (digitalRead(PIN_POWER) == LOW && command == 1) {
    digitalWrite(PIN_POWER, HIGH);
    // Serial.println("[setPower] motor power: ON");  // corepanicするので消した
  }
  if (digitalRead(PIN_POWER) == HIGH && command == 0) {
    if (tmotor.getMotorControl() == 1) {
      tmotor.sendMotorControl(0);
    }
    digitalWrite(PIN_POWER, LOW);
    // Serial.println("[setPower] motor power: OFF");
  }
}

void setControl(bool command)
{
  if (command == 1) {
    digitalWrite(PIN_POWER, HIGH);
    if (tmotor.getMotorControl() == 0) {
      tmotor.sendMotorControl(1);
      // Serial.println("[setControl] motor control mode: ON");
    }
  }
  if (command == 0) {
    if (tmotor.getMotorControl() == 1) {
      tmotor.sendCommand(0.0, 0.0, 0.0, 0.0, 0.0);
      tmotor.sendMotorControl(0);
      // Serial.println("[setControl] motor control mode: OFF");
    }
  }
}
