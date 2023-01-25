#include <Arduino.h>
#include <ArduinoJson.h>
#include <CAN.h>
// #include <ESPAsyncWebServer.h>
// #include <WiFi.h>
#include <esp_task.h>
#include <math.h>
#include <stdio.h>

#include "BTTensionMeter.hpp"
#include "Filter.hpp"
#include "Mode.hpp"
#include "MotorController.hpp"
#include "Secrets.h"
#include "SerialCommunication.hpp"
#include "Tmotor.h"

// 定数等
#define MOTOR_ID 1
#define DRIVER_ID 0
#define PIN_CANRX 32
#define PIN_CANTX 33
#define PIN_POWER 26
#define KD 1.0
#define TAU_TENSION 0.1        // 張力計のLPF時定数[s]
#define CONTROL_INTERVAL 10000 // 制御周期[us]

const char MACHINE_DEVICE_NAME[] = "Machine-ESP32";               // マシンのBluetoothデバイス名
const char TENSIONMETER_DEVICE_NAME[] = "TensionMeter";           // 張力計の device name
const BLEUUID TENSIONMETER_SERVICE_UUID((uint16_t)0x181D);        // Weight Scale (定義済UUID)
const BLEUUID TENSIONMETER_CHARACTERISTIC_UUID((uint16_t)0x2A98); // Weight (定義済UUID)

// 閾値等
#define MAX_LOGNUM 1024 // 筋力測定の最大ログ数

// フラグ等
Mode modeCommand = Mode::TrqCtrl; // 制御対象を表すフラグ. Mode.hppに一覧で記載

// 指令値
float posCommand = 0.0;      // 位置指令値[rad]
float spdCommand = 0.0;      // 速度指令値[rad/s]
float kpCommand = 0.0;       // 位置フィードバックゲイン
float kdCommand = 0.0;       // 速度フィードバックゲイン
float trqCommand = 0.0;      // トルク指令値[Nm]
float trqLimit = 6.0;        // トルク上限[Nm]
float spdLimit = 10.0;       // トルク指令時の、巻取り速度の上限[rad/s]
float spdLimitLiftup = 10.0; // トルク指令時の、挙上速度の上限[rad/s]

hw_timer_t *timer0 = NULL;
TaskHandle_t onTimerTaskHandle = NULL;

SerialCommunication serialCommunication;
ESP32BuiltinCAN esp32BuiltinCAN(PIN_CANRX, PIN_CANTX);
Tmotor tmotor(esp32BuiltinCAN, MOTOR_ID, DRIVER_ID);
MotorController motor(tmotor);

FirstLPF tensionLPF;

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
  motor.init(0.5, 3.0); // motor.init(速度制御のPゲイン、速度制御のIゲイン);

  // 張力計とのBLE通信のsetup
  Serial.println("[setup] Initalizing BLE...");
  BLEDevice::init(MACHINE_DEVICE_NAME);
  Serial.println("[setup] Scanning tension meter...");
  bool connected = tensionMeterBegin(TENSIONMETER_DEVICE_NAME, TENSIONMETER_SERVICE_UUID, TENSIONMETER_CHARACTERISTIC_UUID, 10);
  if (!connected) {
    Serial.println("[setup] Tension meter is not found. Tension log is set to zero");
  }

  // WiFIのsetup
  // if (!WiFi.config(ESP32_IP_ADDRESS, ESP32_GATEWAY, ESP32_SUBNET_MASK)) {
  //   Serial.println("Failed to configure!");
  // }
  // unsigned long wifiBeginTime = millis();
  // WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  // while (WiFi.status() != WL_CONNECTED) {
  //   delay(1000);
  //   Serial.println("Connecting to WiFi..");
  //   if (millis() - wifiBeginTime > 10000) {  // 10sec経ってもWiFiに繋がらなければ再起動
  //     ESP.restart();
  //   }
  // }
  // Serial.println(WiFi.localIP());

  // webserverのセットアップ
  // ws.onEvent(onWsEvent);
  // server.addHandler(&ws);
  // server.begin();

  tensionLPF.setTau(TAU_TENSION);

  xTaskCreatePinnedToCore(onTimerTask, "onTimerTask", 8192, NULL, ESP_TASK_TIMER_PRIO - 1, &onTimerTaskHandle, APP_CPU_NUM); // タイマー割り込みで実行するタスクを登録

  uint16_t prescaler = getApbFrequency() / 1000000; // タイマーのカウントアップ周波数が1MHzとなるようプリスケーラを計算
  timer0 = timerBegin(0, prescaler, true);          // タイマーを初期化
  timerAlarmWrite(timer0, CONTROL_INTERVAL, true);  // 割り込み間隔を設定
  timerAttachInterrupt(timer0, onTimer, true);      // 割り込み関数を登録. edge=trueでエッジトリガ
  timerAlarmEnable(timer0);                         // タイマー割り込みを起動

  digitalWrite(PIN_POWER, HIGH); // コンバータを起動
  delay(5000);
  tmotor.sendMotorControl(1); // モータを起動

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

    // モータ制御モードに入っているとき、送信値を計算し、CANを送信する
    if (tmotor.getMotorControl() == 1) {
      if (modeCommand == Mode::TrqCtrl) {            // トルク制御モードのとき
        motor.startTrqCtrl();                        // トルク制御を開始
        motor.setSpdLimit(spdLimit, spdLimitLiftup); // 定トルク制御時の速度制限を設定
        motor.setTrqRef(trqCommand);                 // トルク目標値を代入
        motor.setTrqLimit(trqLimit);                 // トルク上限を設定

      } else if (modeCommand == Mode::SpdCtrl) { // 速度制御モードのとき
        motor.startSpdCtrl();                    // 速度制御を開始
        motor.setTrqLimit(trqLimit);             // トルク上限を設定
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
  // シリアル通信でコマンドを受信し、反映
  if (serialCommunication.check()) { // 有効なコマンドが到着していたら
    String command = serialCommunication.receive();
    if (!applyJsonMessage(command)) {     // JSONとして解釈を試みる
      if (!applyRegacyMessage(command)) { // 失敗したらレガシーコマンドとして解釈を試みる
        Serial.println("[loop] invalid command.");
      }
    }
  }

  // 張力計からサンプリングを行いLPFにかける
  static unsigned long time_last_sample = 0;
  if (tensionAvailable()) {
    unsigned long now = millis();
    tensionLPF.update(getTension(), 0.001 * (now - time_last_sample));
    time_last_sample = now;
  }

  // CAN受信ログを1secおきにprint
  static unsigned long time_last_print = 0;
  if (millis() - time_last_print > 16) { // ここの数値をいじるとログ取得間隔[ms]を調整可
                                         // if (tensionAvailable()) { // 張力計から受信したタイミングでログを飛ばす場合はこれ
    time_last_print = millis();

    // モーターのログ取得
    Tmotor::Log log;
    while (tmotor.logAvailable() > 0) { // ログが1つ以上たまっていたら
      log = tmotor.logRead();           // ログをひとつ取得
    }
    // 張力計の最新値(LPFによるフィルタリング後)を取得
    float tension = tensionLPF.getLatest();

    // ログをwebSocketで配信 & print
    char targetStr[12];
    if (modeCommand == Mode::TrqCtrl) {
      sprintf(targetStr, "\"trq\"");
    } else if (modeCommand == Mode::SpdCtrl) {
      sprintf(targetStr, "\"spd\"");
    } else {
      sprintf(targetStr, "null");
    }
    sprintf(json_data, "{\"timestamp\":%d,\"target\":%s,\"trq\":%f,\"spd\":%f,\"pos\":%f,\"integratingAngle\":%f,\"tension\":%.6f}", millis(), targetStr, log.trq, log.spd, log.pos, log.integratingAngle, tension);
    // sprintf(json_data, "{\"trq\":%f,\"spd\":%f,\"pos\":%f,\"integratingAngle\":%f,\"tension\":%.6f}", log.trq, log.spd, log.pos, log.integratingAngle, tension);
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
    // Serial.print("[applyJsonMessage] deserializeJson() failed: ");
    // Serial.println(error.f_str());
    return false; // parse時にエラーが出たらそこで終了する
  }

  if (doc.containsKey("power")) {
    int powerInt = doc["power"];
    if (powerInt == 1) {
      digitalWrite(PIN_POWER, HIGH);
    } else {
      digitalWrite(PIN_POWER, LOW);
    }
  }

  if (doc.containsKey("motor")) {
    int motorInt = doc["motor"];
    if (motorInt == 1) {
      tmotor.sendMotorControl(1);
    } else {
      tmotor.sendMotorControl(0);
    }
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
      if (doc["spdLimitLiftup"]) {
        spdLimitLiftup = doc["spdLimitLiftup"];
      }

    } else {
      modeCommand = Mode::TrqCtrl;
      trqCommand = 0.0;
      spdCommand = 0.0;
      return false; // targetがspdでもtrqでもないのはエラー
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
  sscanf(data.c_str(), "%c%f\n", &key, &value); // scan the command

  switch (key) { // copy the commanded value
    case 'p':
      if (value > 0.5) { // value==1のときコンバータ電源ON
        digitalWrite(PIN_POWER, HIGH);
      } else {
        tmotor.sendMotorControl(0); // コンバータ切るときは必ずモータも切る(でないと次にモーターが立ち上がらないことがある)
        digitalWrite(PIN_POWER, LOW);
      }
      return true;
    case 'm':
      if (value > 0.5) {                      // value==1のときモータ制御開始コマンドを送信
        if (digitalRead(PIN_POWER) == HIGH) { // コンバータON時のみモータに起動指令
          tmotor.sendMotorControl(1);
        }
      } else {
        tmotor.sendMotorControl(0);
      }
      return true;
    case 't':
      modeCommand = Mode::TrqCtrl;
      trqCommand = value;
      spdCommand = 0.0;
      return true;
    case 's':
      modeCommand = Mode::SpdCtrl;
      trqCommand = 0.0;
      spdCommand = value;
      return true;
    default:
      return false;
  }

  return false;
}
