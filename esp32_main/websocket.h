/**
 * @file websocket.h
 * @author Hibiki Matsuda
 * @brief Websocketまわりのコードをまとめた
 * @date 2022-08-09
 */

#pragma once

#include <ArduinoJson.h>
#include <ESPAsyncWebServer.h>
#include "Mode.hpp"

/**
 * @brief WebSocket通信の立ち上げと初期化
 * @note ポート番号とURLはwebsocket.cppの中で指定する
 */
void websocketInit();

/**
 * @brief WebSocketでコマンドを受信しているかどうか
 * @return 読み込んでいないコマンドがあるときtrue, ないときfalse
 */
bool websocketAvailable();

/**
 * @brief WebSocketで受け取った指令を取得する
 * @return Command
 */
Command websocketReadCommand();

/**
 * @brief WebSocketで現在の状態を送信する
 * @param status 現在の状態をStatus型にして代入する
 */
void websocketSendStatus(Status& status);
