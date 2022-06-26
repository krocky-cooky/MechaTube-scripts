/**
 * @file SerialCommunication.hpp
 * @author thgcMtdh
 * @brief シリアル通信の送受信と、文字列-->変数へのデコードを行うクラス
 * @date 2022-06-25
 */
#pragma once

#include "Mode.hpp"
#include <Arduino.h>

class SerialCommunication
{
public:
  bool power;        // モータ電源指令値
  bool motorControl; // モータ制御モードに入るかどうか
  Mode mode;         // 制御モード。Mode.hppを参照
  float trq;         // トルク指令値[Nm]
  float spd;         // 速度指令値[rad/s]
  // float increaseOfToraueForEccentricMotion;       // エキセン動作時のトルク増加量
  // float maxSpeedWhileConcentricMotion;            // アイソキネティックトレーニング時の回転スピード
  // float increaseOfToraueWhenPeak;                 // ピーク時のトルクが通常トルクよりどれだけ高いか
  // float rotationAngleFromInitialPositionWhenPeak; // ピーク位置。受信する位置データではなく、初期位置からの回転角
  // float rangeOfTorqueChange;                      // ピーク位置に対して+-いくつの位置まで行けば通常トルクになるか。言い換えれば、ピークの裾野の大きさ

  /**
   * @brief シリアル通信で取得したコマンドをまとめておくクラス。
   */
  SerialCommunication();

  /**
   * @brief シリアル通信ポートからコマンドを受信し、指令値を更新する
   * @return コマンドが送られていない場合は0を、正常に解釈できた場合はコマンドの種類('p','m','t','s')を返す
   */
  char receive();

private:
  static constexpr size_t SERIAL_BUFSIZE = 20;

  /**
   * @brief
   * @param[in] buf 解釈する文字列
   * @retval コマンドが無効な場合は0を、正常に解釈できた場合はコマンドの種類('p','m','t','s')を返す
   */
  char decodeCommand(const char *buf);
};
