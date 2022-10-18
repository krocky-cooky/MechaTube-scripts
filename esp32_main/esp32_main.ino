#include <Arduino.h>
#include <ArduinoJson.h>
#include <CAN.h>
#include <ESPAsyncWebServer.h>
#include <WiFi.h>
#include <math.h>
#include <stdio.h>

//定数等
#include "esp_task.h"

#include "Filter.hpp"
#include "Mode.hpp"
#include "MotorController.hpp"
#include "Secrets.h"
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
bool powerCommand = false; // コンバータ電源ON/OFF
bool motorCommand = false; // モータ制御モードON/OFF
float posCommand = 0.0;    // 位置指令値[rad]
float spdCommand = 0.0;    // 速度指令値[rad/s]
float kpCommand = 0.0;     // 位置フィードバックゲイン
float kdCommand = 0.0;     // 速度フィードバックゲイン
float trqCommand = 0.0;    // トルク指令値[Nm]
float trqLimit = 6.0;
float spdLimit = 2.0;
unsigned int powerOnTime = 0;       // コンバータ電源を入れた時刻[ms]: モータへのコマンド送信に利用
unsigned int motorCtrlSentTime = 0; // 最後にモータ制御モードのenter/exitコマンドを送った時刻[ms]

hw_timer_t *timer0 = NULL;
TaskHandle_t onTimerTaskHandle = NULL;

SerialCommunication serialCommunication;
ESP32BuiltinCAN esp32BuiltinCAN(PIN_CANRX, PIN_CANTX);
Tmotor tmotor(esp32BuiltinCAN, MOTOR_ID, DRIVER_ID);
MotorController motor(tmotor);
TouchSwitch touchSwitch(PIN_HANDSWITCH, HANDSWITCH_VOLTAGE_THRESHOLD);

FirstLPF firstOrderDelayTrq;
FirstLPF firstOrderDelaySpd;

// websocketのオブジェクト
// AsyncWebServer server(80);
// AsyncWebSocket ws("/ws");

// websocket通信で送るjsonのための文字データ
char json_data[256];

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

  // webserverのセットアップ
  // ws.onEvent(onWsEvent);
  // server.addHandler(&ws);
  // server.begin();

  // firstOrderDelayTrq.setTau(TAU_TRQ); // トルクの1次遅れフィルタを宣言
  // firstOrderDelaySpd.setTau(TAU_SPD); // 速度の1次遅れフィルタを宣言

  xTaskCreatePinnedToCore(onTimerTask, "onTimerTask", 8192, NULL, ESP_TASK_TIMER_PRIO - 1, &onTimerTaskHandle, APP_CPU_NUM); // タイマー割り込みで実行するタスクを登録

  uint16_t prescaler = getApbFrequency() / 1000000; // タイマーのカウントアップ周波数が1MHzとなるようプリスケーラを計算
  timer0 = timerBegin(0, prescaler, true);          // タイマーを初期化
  timerAlarmWrite(timer0, CONTROL_INTERVAL, true);  // 割り込み間隔を設定
  timerAttachInterrupt(timer0, onTimer, true);      // 割り込み関数を登録. edge=trueでエッジトリガ
  timerAlarmEnable(timer0);                         // タイマー割り込みを起動

  powerCommand = 1; // コンバータを起動
  motorCommand = 1; // モータを起動

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

    // 手元スイッチのON/OFFを取得する
    // bool handSwitch = touchSwitch.getState();

    // モータ制御モードに入っているとき、送信値を計算し、CANを送信する
    if (tmotor.getMotorControl() == 1) {
      if (modeCommand == Mode::TrqCtrl) {             // トルク制御モードのとき
        motor.startTrqCtrl();                         // トルク制御を開始
        motor.setSpdLimit(spdLimit * 0.75, spdLimit); // 定トルク制御時の速度制限を設定
        motor.setTrqRef(trqCommand);                  // トルク目標値を代入

      } else if (modeCommand == Mode::SpdCtrl) { // 速度制御モードのとき
        motor.startSpdCtrl();                    // 速度制御を開始
        motor.setTrqLimit(trqLimit);             // 定速制御時のトルク上限を設定
        motor.setSpdRef(spdCommand);             // 速度目標値を代入

      } else {
        motor.stopCtrl();
      }

      motor.update(CONTROL_INTERVAL);

      // モータ制御モードに入っていないとき、制御を終了
    } else {
      motor.stopCtrl();
    }
  }
}

void loop()
{
  // コンバータ電源を指令に応じて入切する
  if (powerCommand == true) {
    digitalWrite(PIN_POWER, HIGH);
    if (powerOnTime == 0) {
      powerOnTime = millis(); // 電源ON時刻を記録
    }
  } else {
    if (tmotor.getMotorControl() == 1) {
      tmotor.sendMotorControl(0);
    }
    digitalWrite(PIN_POWER, LOW);
    powerOnTime = 0; // 電源ON時刻をクリア
  }

  // モータ制御モードを指令に応じて入切する
  if (motorCommand == true && tmotor.getMotorControl() == 0) {
    if (powerOnTime > 0 && millis() - powerOnTime > 3000) { // コンバータ電源ONから3sec経ってからモータ制御モードに移行
      // コンセントが抜けていてモータに電源が供給されていないときにCANコマンドを連続で送り続けるとバグるので、
      // コマンド送信間隔は1secに1回とする
      if (millis() - motorCtrlSentTime > 1000) {
        tmotor.sendMotorControl(1);
        motorCtrlSentTime = millis();
      }
    }
  } else if (motorCommand == false && tmotor.getMotorControl() == 1) {
    if (millis() - motorCtrlSentTime > 1000) {
      tmotor.sendMotorControl(0);
      motorCtrlSentTime = millis();
    }
  }

  // シリアル通信でコマンドを受信し、反映
  if (serialCommunication.check()) { // 有効なコマンドが到着していたら
    String command = serialCommunication.receive();
    if (!applyJsonMessage(command)) {     // JSONとして解釈を試みる
      if (!applyRegacyMessage(command)) { // 失敗したらレガシーコマンドとして解釈を試みる
        Serial.println("[loop] invalid command.");
      }
    }
  }

  // CAN受信ログを1secおきにprint
  static unsigned long time_last_print = 0;
  if (millis() - time_last_print > 100) { // ここの数値をいじるとログ取得間隔[ms]を調整可
    time_last_print = millis();
    Tmotor::Log log;
    while (tmotor.logAvailable() > 0) { // ログが1つ以上たまっていたら
      log = tmotor.logRead();           // ログをひとつ取得
    }

    // ログをwebSocketで配信 & print
    char targetStr[12];
    if (modeCommand == Mode::TrqCtrl) {
      sprintf(targetStr, "\"trq\"");
    } else if (modeCommand == Mode::SpdCtrl) {
      sprintf(targetStr, "\"spd\"");
    } else {
      sprintf(targetStr, "null");
    }
    sprintf(json_data, "{\"timestamp\":%d,\"target\":%s,\"trq\":%f,\"spd\":%f,\"pos\":%f,\"integratingAngle\":%f}", millis(), targetStr, log.trq, log.spd, log.pos, log.integratingAngle);
    Serial.println(json_data);
    // ws.textAll(json_data);
  }

  // コンバータの電圧を表示
  // float voltageOfConverter = analogRead(34) * 3.3 * 21 / 4096; //コンバータの電圧の値
  // Serial.printf("the voltage of converter = %f\n", voltageOfConverter);

  delay(1);
}

// websocketをイベントごとに処理
// void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len)
// {

//   if (type == WS_EVT_CONNECT) {

//     Serial.println("Websocket client connection received");
//   } else if (type == WS_EVT_DISCONNECT) {

//     Serial.println("Client disconnected");
//   } else if (type == WS_EVT_DATA) {
//     handleWebSocketMessage(arg, data, len);
//   }
// }

// クライアントからwebsocketでメッセージを受け取ったら表示
// void handleWebSocketMessage(void *arg, uint8_t *data, size_t len)
// {
//   AwsFrameInfo *info = (AwsFrameInfo *)arg;
//   if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
//     Serial.print("Client Message:");
//     Serial.println((char *)data);
//     applyJsonMessage((char *)data);
//   }
// }

// JSONメッセージからコマンドを読み取って、反映する
// parseに成功したらtrue, 失敗したらfalseを返す
bool applyJsonMessage(String data)
{
  StaticJsonDocument<256> doc;                             // 受信した文字列をjsonにparseするためのバッファ
  DeserializationError error = deserializeJson(doc, data); // JSONにparse
  if (error) {
    Serial.print("[handleWebsocetMessage] deserializeJson() failed: ");
    Serial.println(error.f_str());
    return false; // parse時にエラーが出たらそこで終了する
  }

  if (doc.containsKey("power")) {
    int powerInt = doc["power"];
    powerCommand = (powerInt == 1) ? true : false;
  }

  if (doc.containsKey("motor")) {
    int motorInt = doc["motor"];
    motorCommand = (motorInt == 1) ? true : false;
  }

  if (doc.containsKey("target")) {
    const char *targetChar = doc["target"];
    const String target(targetChar);

    if (target.equals("spd")) {
      modeCommand = Mode::SpdCtrl;
      spdCommand = doc["spd"];
      trqCommand = 0.0;
      trqLimit = doc["trqLimit"];

    } else if (target.equals("trq")) {
      modeCommand = Mode::TrqCtrl;
      trqCommand = doc["trq"];
      spdCommand = 0.0;
      spdLimit = doc["spdLimit"];

    } else {
      modeCommand = Mode::TrqCtrl;
      trqCommand = 0.0;
      spdCommand = 0.0;
    }
  }
  return true;
}

// jsonではない旧来のコマンド文字列（"t0.8" など）を解釈し、反映する。
// 解釈に成功したらtrue, 失敗したらfalseを返す
bool applyRegacyMessage(String data)
{
  char key = 0;
  float value = 0.0;
  sscanf(data.c_str(), "%c%f", &key, &value); // scan the command

  switch (key) { // copy the commanded value
    case 'p':
      powerCommand = (value > 0.5) ? true : false; // float value contains small error so it is not exactly equal to 0 or 1 integer
      return true;
    case 'm':
      motorCommand = (value > 0.5) ? true : false;
      return true;
    case 't':
      modeCommand = Mode::TrqCtrl;
      powerCommand = true;
      motorCommand = true;
      trqCommand = value;
      spdCommand = 0.0;
      return true;
    case 's':
      modeCommand = Mode::SpdCtrl;
      powerCommand = true;
      motorCommand = true;
      trqCommand = 0.0;
      spdCommand = value;
      return true;
    default:
      return false;
  }
  return false;
}
