#include <Arduino.h>
#include <ArduinoJson.h>
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

hw_timer_t *timer0 = NULL;
TaskHandle_t onTimerTaskHandle = NULL;

SerialCommunication serialCommunication;
ESP32BuiltinCAN esp32BuiltinCAN(PIN_CANRX, PIN_CANTX);
Tmotor tmotor(esp32BuiltinCAN, MOTOR_ID, DRIVER_ID);
MotorController motor(tmotor);
TouchSwitch touchSwitch(PIN_HANDSWITCH, HANDSWITCH_VOLTAGE_THRESHOLD);

// フラグ等
bool power = false;        // コンバータ電源ON/OFF
bool motorControl = false; // モータ制御モードON/OFF

// 指令値
Command command;

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

  websocketInit(); // websocketセットアップ

  // firstOrderDelayTrq.setTau(TAU_TRQ); // トルクの1次遅れフィルタを宣言
  // firstOrderDelaySpd.setTau(TAU_SPD); // 速度の1次遅れフィルタを宣言

  xTaskCreatePinnedToCore(onTimerTask, "onTimerTask", 8192, NULL, ESP_TASK_TIMER_PRIO - 1, &onTimerTaskHandle, APP_CPU_NUM); // タイマー割り込みで実行するタスクを登録

  uint16_t prescaler = getApbFrequency() / 1000000; // タイマーのカウントアップ周波数が1MHzとなるようプリスケーラを計算
  timer0 = timerBegin(0, prescaler, true);          // タイマーを初期化
  timerAlarmWrite(timer0, CONTROL_INTERVAL, true);  // 割り込み間隔を設定
  timerAttachInterrupt(timer0, onTimer, true);      // 割り込み関数を登録. edge=trueでエッジトリガ
  timerAlarmEnable(timer0);                         // タイマー割り込みを起動

  delay(1000);
  power = true;  // 1秒後に電源ON
  delay(2000);
  motorControl = true;  // その2秒後にモータ制御起動
  
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

    // フラグに応じて、モータの電源とモータ制御モードを変更する
    setPower(power);
    setControl(motorControl);

    if (command.target == Target::TrqCtrl) {                       // トルク制御モードのとき
      motor.startTrqCtrl();                                        // トルク制御を開始
      motor.setSpdLimit(command.spdLimit, command.spdLimit + 1.0); // 定トルク制御時の速度制限を設定。2.0rad/sに達したらトルクを減少させはじめ、3.0rad/sでトルク0にする
      motor.setTrqRef(command.trq);                                // トルク目標値を代入

    } else if (command.target == Target::SpdCtrl) { // 速度制御モードのとき
      motor.startSpdCtrl();                         // 速度制御を開始
      motor.setTrqLimit(command.trqLimit);          // 定速制御時のトルク上限を設定
      motor.setSpdRef(command.spd);                 // 速度目標値を代入

    } else {
      motor.stopCtrl();
    }

    if (digitalRead(PIN_POWER) == HIGH) { // コンバータ電源ON時(=CAN送信できるとき)のみモータ更新
      motor.update(CONTROL_INTERVAL);
    }
  }
}

void loop()
{
  // シリアル通信でコマンドを受信し、反映
  char retval = serialCommunication.receive(); // 受信
  if (retval) {
    power = serialCommunication.power;
    motorControl = serialCommunication.motorControl;
    command.target = serialCommunication.target;
    command.trq = serialCommunication.trq;
    command.spdLimit = 2.0; // 速度制限はシリアルで受け取っていないので2.0rad/sに設定
    command.spd = serialCommunication.spd;
    command.trqLimit = 6.0; // トルク制限はシリアルで受け取っていないので6.0Nmに設定
  }

  // websocketからのコマンドを受信し、反映
  if (websocketAvailable()) {
    command = websocketReadCommand();
  }

  // 状態を100msおきに配信
  static unsigned long time_last_print = 0;
  if (millis() - time_last_print > 100) { // ここの数値をいじるとログ取得間隔[ms]を調整可
    time_last_print = millis();
    Tmotor::Log log;
    while (tmotor.logAvailable() > 0) { // ログが1つ以上たまっていたら
      log = tmotor.logRead();           // ログをひとつ取得
    }
    // 状態をwebSocketで配信
    Status status;
    status.target = command.target;
    status.trq = log.trq;
    status.spd = log.spd;
    status.pos = log.pos;
    status.integratingAngle = log.integratingAngle;
    websocketSendStatus(status);
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


// websocket.cppから移植

AsyncWebServer server(80);
AsyncWebSocket ws("ws/");

bool msgAvailable; // 受信すると1になるフラグ
Command commandReceived;   // 受信した指令値

void onWsEvent(AsyncWebSocket *, AsyncWebSocketClient *, AwsEventType, void *, uint8_t *, size_t);
void handleWebSocketMessage(void *, uint8_t *, size_t);

void websocketInit()
{
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.begin();
}

bool websocketAvailable()
{
  return msgAvailable;
}

Command websocketReadCommand()
{
  msgAvailable = false; // 読み出したら受信フラグをクリア
  return commandReceived;
}

void websocketSendStatus(Status& status)
{
  static StaticJsonDocument<256> doc;

  if (status.target == Target::None) {
    doc["target"] = nullptr;
  } else if (status.target == Target::SpdCtrl) {
    doc["target"] = "spd";
  } else if (status.target == Target::TrqCtrl) {
    doc["target"] = "trq";
  } else {
    // pass
  }
  doc["trq"] = status.trq;
  doc["spd"] = status.spd;
  doc["pos"] = status.pos;
  doc["integratingAngle"] = status.integratingAngle;

  String jsonStr;
  serializeJson(doc, jsonStr);

  ws.textAll(jsonStr);     // websocketで送信
  Serial.println(jsonStr); // シリアルモニタにprint
}

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len)
{
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.println("Websocket client connection received");
      break;

    case WS_EVT_DISCONNECT:
      Serial.println("Client disconnected");
      break;

    case WS_EVT_DATA:
      Serial.println("WS_EVT_DATA");
      handleWebSocketMessage(arg, data, len);
      break;

    default:
      break;
  }
}

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len)
{
  AwsFrameInfo *info = (AwsFrameInfo *)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
    String message((char *)data); // 扱いやすいようにString型に変換
    Serial.print("Client Message:");
    Serial.println(message);

    StaticJsonDocument<256> doc;                                // 受信した文字列をjsonにparseするためのバッファ
    DeserializationError error = deserializeJson(doc, message); // JSONにparse
    if (error) {
      Serial.print("[handleWebsocetMessage] deserializeJson() failed: ");
      Serial.println(error.f_str());
      return; // parse時にエラーが出たらそこで終了する
    }

    msgAvailable = true; // parseエラーがなければ、メッセージ到着フラグをセット

    if (doc.containsKey("target")) {
      const char *targetChar = doc["target"];
      const String target(targetChar);

      if (target.equals("spd")) {
        commandReceived.target = Target::SpdCtrl;
        commandReceived.spd = doc["spd"];
        commandReceived.trq = 0.0;
        commandReceived.trqLimit = doc["trqLimit"];

      } else if (target.equals("trq")) {
        commandReceived.target = Target::TrqCtrl;
        commandReceived.trq = doc["trq"];
        commandReceived.spd = 0.0;
        commandReceived.spdLimit = doc["spdLimit"];

      } else {
        commandReceived.target = Target::TrqCtrl;
        commandReceived.trq = 0.0;
        commandReceived.spd = 0.0;
      }
    }
  }
}
