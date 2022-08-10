/**
 * @file websocket.h
 * @author Hibiki Matsuda
 * @brief Websocketまわりのコードをまとめた
 * @date 2022-08-09
 */

#include <ArduinoJson.h>
#include <ESPAsyncWebServer.h>

/**
 * @brief WebSocket通信の立ち上げと初期化
 * @note ポート番号とURLはwebsocket.cppの中で指定する
 */
void websocketInit();

/**
 * @brief WebSocketで送信する
 * @param buffer 送信したい文字列が入ったchar型配列
 */
void webSocketSend(char* buffer);

