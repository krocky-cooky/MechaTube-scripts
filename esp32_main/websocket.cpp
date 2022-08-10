#include "websocket.h"

static AsyncWebServer server(80);
static AsyncWebSocket ws("ws/");
static StaticJsonDocument<256> doc;  // 受信した文字列をjsonにparseするためのバッファ

void websocketInit()
{
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.begin();
}

void websocketSend(char* buffer)
{
  ws.textAll(buffer);
}

static void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len)
{
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.println("Websocket client connection received");
      break;

    case WS_EVT_DISCONNECT:
      Serial.println("Client disconnected");
      break;

    case WS_EVT_DATA:
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
    String message((char*)data);  // 扱いやすいようにString型に変換
    Serial.print("Client Message:");
    Serial.println(message);

    DeserializationError error = deserializeJson(&doc, message);  // JSONにparse
    if (error) {
      Serial.print("[handleWebsocetMessage] deserializeJson() failed: ");
      Serial.println(error.f_str());
      return;  // parse時にエラーが出たらそこで終了する
    }

    String command = doc["command"];  // コマンド


    
  }

}