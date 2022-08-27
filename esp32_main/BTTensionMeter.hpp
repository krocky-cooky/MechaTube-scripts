/**
 * @file BTTensionMeter.hpp
 * @author Hibiki Matsuda
 * @brief Bluetooth張力計(tension_meter.ino)からデータを受信するライブラリ
 * @date 2022-08-27
 */

#pragma once

#include "BluetoothSerial.h"

/*
 * 想定している受信メッセージの形式
 *  index       0  1  2  3  4  5  6  7  8  9  10         11
 *  content  0x02  1  6  5  0  1  0  0  0  ,   3         \n
 *  meaning  STX   Tension [mg]               checksum   LF
 *                 -> 16.501kg                1+6+5+0+1  end
 */

class BTTensionMeter
{
private:
  static const size_t BT_MSG_BUFSIZE_; // Bluetooth受信メッセージをコピーするバッファのサイズ
  static const unsigned long TIMEOUT_; // 直近のメッセージ到着からこの時間[ms]だけ経過しても次のメッセージが来ないとき接続断とみなす
  BluetoothSerial &SerialBT_;
  unsigned long lastReceivedMillis_; // 最後に受信した時刻[ms]
  bool available_;                   // 最新値到着フラグ。受信した値がまだgetTension()により読まれていないときtrueになる
  int tension_;                      // 受信した張力の最新値[mg]

  int calc_checksum(int val);

public:
  BTTensionMeter(BluetoothSerial &serialBT);

  /**
   * @brief Initialize library
   */
  void begin();

  /**
   * @brief Receive messages from meter. Must call this function in mainloop
   */
  void loop();

  /**
   * @brief Return whether the tension meter is conncted or not.
   * @return true Tension meter is connected.
   * @return false Tension meter is not connected (message hasn't arrived for 50ms or more).
   */
  bool connected();

  /**
   * @brief Return true when a new value has been arrived.
   * @return true when a new value has been arrived but not read by user.
   * @return false when the latest value has also been read. You can still use getTension() to obtain the latest value.
   */
  bool available();

  /**
   * @brief Get the latest tension sent from meter.
   * @return (float) Measured tension [g]
   */
  float getTension();
};
