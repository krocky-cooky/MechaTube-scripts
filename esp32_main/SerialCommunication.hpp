/**
 * @file SerialCommunication.hpp
 * @author thgcMtdh
 * @brief シリアル通信の送受信と、文字列-->変数へのデコードを行うクラス
 * @date 2022-06-25
 */
#pragma once

#include "Mode.hpp"
#include <Arduino.h>

constexpr size_t SERIAL_BUFSIZE = 256;

class SerialCommunication
{
public:
  /**
   * @brief シリアル通信で取得したコマンドをまとめておくクラス。
   */
  SerialCommunication();

  /**
   * @brief シリアル通信ポートに有効なコマンドが届いているかどうか検出する
   * @return true: コマンドが届いている（receive()で読み出すこと）、false: 届いていない
   */
  bool check();

  /**
   * @brief 最新のコマンド文字列を返す
   * @return 受信したコマンド文字列
   */
  String receive();

private:
  char buf[SERIAL_BUFSIZE];
};
