#include "websocket.h"

AsyncWebServer server(80);
AsyncWebSocket ws("ws/");

static bool msgAvailable; // 受信すると1になるフラグ
static Command command;   // 受信した指令値

static void onWsEvent(AsyncWebSocket *, AsyncWebSocketClient *, AwsEventType, void *, uint8_t *, size_t);
static void handleWebSocketMessage(void *, uint8_t *, size_t);

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
  return command;
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

static void handleWebSocketMessage(void *arg, uint8_t *data, size_t len)
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
        command.target = Target::SpdCtrl;
        command.spd = doc["spd"];
        command.trq = 0.0;
        command.trqLimit = doc["trqLimit"];

      } else if (target.equals("trq")) {
        command.target = Target::TrqCtrl;
        command.trq = doc["trq"];
        command.spd = 0.0;
        command.spdLimit = doc["spdLimit"];

      } else {
        command.target = Target::TrqCtrl;
        command.trq = 0.0;
        command.spd = 0.0;
      }
    }
  }
}
