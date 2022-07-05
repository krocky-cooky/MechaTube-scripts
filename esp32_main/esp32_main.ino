#include <Arduino.h>
#include <CAN.h>
#include <math.h>
#include <stdio.h>

#include "esp_task.h"

#include "Filter.hpp"
#include "Mode.hpp"
#include "SerialCommunication.hpp"
#include "Tmotor.h"
#include "TouchSwitch.hpp"

// 定数等
#define MOTOR_ID 64
#define DRIVER_ID 0
#define PIN_CANRX 32
#define PIN_CANTX 33
#define PIN_POWER 26
#define PIN_HANDSWITCH 35
#define KP 0.1
#define KD 1.0
#define TAU_TRQ 1.0             // 一次遅れ系によるトルク指令の時定数[s]
#define TAU_SPD 1.0             // 一次遅れ系による速度指令の時定数[s]
#define CONTROL_INTERVAL 100000 // 制御周期[us]

// 閾値等
#define HANDSWITCH_VOLTAGE_THRESHOLD 10.0 // 手元スイッチのオンオフを識別するための、スイッチアナログ入力ピンの電圧閾値 [V]
#define MAX_LOGNUM 1024                   // 筋力測定の最大ログ数

// フラグ等
Mode mode = Mode::SpdCtrl; // 制御対象を表すフラグ. Mode.hppに一覧で記載

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

  firstOrderDelayTrq.setTau(TAU_TRQ); // トルクの1次遅れフィルタを宣言
  firstOrderDelaySpd.setTau(TAU_SPD); // 速度の1次遅れフィルタを宣言

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
    if (tmotor.getMotorControl()) {
      if (mode == Mode::TrqCtrl) {     // トルク制御モードのとき
        firstOrderDelaySpd.clear(0.0); // 速度の1次遅れ計算用変数は使わないのでリセット
        if (handSwitch) {              // 手元スイッチONのとき送信値をゆっくり指令値に近づけ、OFFのときは0に近づける
          trqSend = firstOrderDelayTrq.update(trqCommand, CONTROL_INTERVAL / 1e6);
        } else {
          trqSend = 0.0;
          firstOrderDelayTrq.clear(0.0);
        }
        tmotor.sendCommand(0, 0, 0, 0, trqSend); // 送信

      } else if (mode == Mode::SpdCtrl) { // 速度指令モードのとき
        firstOrderDelayTrq.clear(0.0);    // トルクの1次遅れ計算用変数は使わないのでリセット
        if (handSwitch) {
          spdSend = firstOrderDelaySpd.update(spdCommand, CONTROL_INTERVAL / 1e6);
        } else {
          spdSend = firstOrderDelaySpd.update(0.0, CONTROL_INTERVAL / 1e6);
        }
        tmotor.sendCommand(0, spdSend, KP, KD, 0); // 送信
      }

    } else { // モータ制御モードに入っていないとき、全ての変数を0にリセットしておく
      firstOrderDelayTrq.clear(0.0);
      firstOrderDelaySpd.clear(0.0);
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
  mode = serialCommunication.mode;
  trqCommand = serialCommunication.trq;
  spdCommand = serialCommunication.spd;

  // CAN受信ログを1secおきにprint
  static unsigned long time_last_print = 0;
  if (millis() - time_last_print > 1000) { // ここの1000をいじるとログ取得間隔を調整可
    time_last_print = millis();
    while (tmotor.logAvailable() > 0) {   // ログが1つ以上たまっていたら
      Tmotor::Log log = tmotor.logRead(); // ログをひとつ取得
      Serial.printf(
          "{\"timestamp\": %d, \"trq\":%.3f, \"spd\":%.3f, \"pos\":%.3f, \"integratingAngle\": %.3f}\n",
          log.timestamp,
          log.trq,
          log.spd,
          log.pos,
          log.integratingAngle);
    }
  }

  // コンバータの電圧を表示
  // float voltageOfConverter = analogRead(34) * 3.3 * 21 / 4096; //コンバータの電圧の値
  // Serial.printf("the voltage of converter = %f\n", voltageOfConverter);
  delay(100);
}

void setPower(bool command)
{
  if (digitalRead(PIN_POWER) == LOW && command == 1) {
    digitalWrite(PIN_POWER, HIGH);
    Serial.println("[setPower] motor power: ON");
  }
  if (digitalRead(PIN_POWER) == HIGH && command == 0) {
    if (tmotor.getMotorControl() == 1) {
      tmotor.sendMotorControl(0);
    }
    digitalWrite(PIN_POWER, LOW);
    Serial.println("[setPower] motor power: OFF");
  }
}

void setControl(bool command)
{
  if (command == 1) {
    digitalWrite(PIN_POWER, HIGH);
    if (tmotor.getMotorControl() == 0) {
      tmotor.sendMotorControl(1);
      Serial.println("[setControl] motor control mode: ON");
    }
  }
  if (command == 0) {
    if (tmotor.getMotorControl() == 1) {
      tmotor.sendCommand(0.0, 0.0, 0.0, 0.0, 0.0);
      tmotor.sendMotorControl(0);
      Serial.println("[setControl] motor control mode: OFF");
    }
  }
}
