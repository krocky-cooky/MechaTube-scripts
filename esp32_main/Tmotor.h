/**
 * @file
 * @brief モータとのCANメッセージ送受信を担うモジュール. 1つのCAN通信路に複数のモータを接続しても大丈夫な設計になっている
 * @author thgcMtdh
 * @date 2022/2/13
 * @note serial_communication.ino より移植
 * [usage]
 *
 * ESP32BuiltinCAN esp32BuiltinCAN(PIN_CANRX, PIN_CANTX);
 * Tmotor tmotor(esp32BuiltinCAN, MOTOR_ID, DRIVER_ID);
 *
 * void setup() {
 *   esp32BuiltinCAN.begin(1000E3);
 * }
 *
 * void loop() {
 *   tmotor.sendMotorControl(true);
 *   tmotor.sendCommand(......);
 * }
 */

#pragma once

#include "ESP32BuiltinCAN.hpp"
#include "freertos/ringbuf.h"
#include <Arduino.h>
#include <map>

class Tmotor
{
private:
  /// data boundary ///
  static constexpr float P_MIN = -12.5;
  static constexpr float P_MAX = 12.5;
  static constexpr float V_MIN = -46.57;
  static constexpr float V_MAX = 46.57;
  static constexpr float T_MIN = -6;
  static constexpr float T_MAX = 6;
  static constexpr float Kp_MIN = 0;
  static constexpr float Kp_MAX = 500;
  static constexpr float Kd_MIN = 0;
  static constexpr float Kd_MAX = 5;
  static constexpr size_t RINGBUF_ITEM_NUM = 2048;
  static const uint8_t msgEnter[8];
  static const uint8_t msgExit[8];
  static const uint8_t msgSetPosToZero[8];

  ESP32BuiltinCAN &CAN_;
  const int MOTOR_ID_;
  const int DRIVER_ID_;
  bool motorCtrl_;

  RingbufHandle_t ringbuf_;

  static TaskHandle_t onReceiveTaskHandle_;
  static uint8_t msgReceived_[6];   // CAN受信メッセージ
  static uint64_t msgReceivedTime_; // CAN受信時刻

  /**
   * @brief CAN受信割り込み内で各インスタンスごとに実行する処理. 中身はCANメッセージの転記
   * @param packetSize 受信したバイト数
   */
  void IRAM_ATTR onReceiveMember(int);

  /**
   * @brief CAN受信時に各インスタンスごとに実行するタスク. メッセージのデコードなど(floatを扱うので割り込み内で出来ない)
   */
  void onReceiveTask();

  /**
   * @brief モータから受信した位置を積算位置に換算する
   * @param oldPos ひとつ前の受信位置[rad]
   * @param newPos 最新の受信位置[rad]
   */
  void integratePosition(float oldPos, float newPos);

  static void packCmd(uint8_t *, float, float, float, float, float);
  static int float_to_uint(float, float, float, unsigned int);
  static void unpackReply(uint8_t *, int *, float *, float *, float *);
  static float uint_to_float(int, float, float, int);

public:
  // CAN受信値をためておくログ用構造体
  struct Log {
    uint64_t timestamp;     // マイコン起動からの経過時間[us]
    float pos;              // 位置[rad]
    float spd;              // 速度[rad/s]
    float trq;              // トルク[Nm]
    float integratingAngle; // 積算回転角[rad]
  };

  /// @brief Tmotorオブジェクトを生成する
  /// @param[in] can CANインスタンスへの参照
  /// @param[in] motorId モーターの CAN ID
  /// @param[in] driverId ドライバーのCAN ID
  Tmotor(ESP32BuiltinCAN &, int, int);

  bool operator<(const Tmotor &rhs) const;

  /// @brief トルクや速度指令値をCANで送信する
  /// @param[in] pos 位置指令値 [rad] (位置を指令しない場合は0を指定)
  /// @param[in] spd 速度指令値 [rad/s] (速度を指令しない場合は0を指定)
  /// @param[in] kp 位置のフィードバックゲイン[Nm/rad] (位置を指令しない場合は0を指定)
  /// @param[in] kd 速度のフィードバックゲイン[Nm/(rad/s)] (速度を指令しな場合は0を指定)
  /// @param[in] trq トルク指令値 [Nm] (トルクを指令しない場合は0を指定)
  /// @return 0:fail, 1:success
  int sendCommand(float, float, float, float, float);

  /// @brief モータ制御モードのON/OFFを切り替えるコマンドをCANで送信する
  /// @param[in] command 0:Exit motor control mode, 1:Enter motor control mode
  /// @return 0:fail, 1:success
  int sendMotorControl(bool);

  /// @brief 現在モータ制御モードに入っているかを取得する
  /// @return 0:モータ制御モードOFF, 1:モータ制御モードON
  bool getMotorControl();

  /// @brief 積算位置を、強制的に指定した値にリセットする
  /// @param newIntegratingAngle 新しく指定する積算位置[rad]
  void setIntegratingAngle(float newIntegratingAngle);

  /// @brief バッファにたまっているCAN受信ログの個数を返す
  /// @return ログの個数
  int logAvailable();

  /// @brief バッファにたまっているCAN受信値ログをひとつ返す
  /// @return ログ
  /// @note usage: Tmotor::Log log = tmotor.logRead();
  ///              pos = log.pos;
  ///              spd = log.spd;...
  Log logRead();

  /// @brief CAN受信割り込みに登録するコールバック関数
  /// @param[in] packetSize 受信したバイト数(CAN.onReceiveから渡される)
  /// @param[in] pTmotor Tmotorインスタンスへのポインタ
  /// @return none
  static void IRAM_ATTR onReceive(int, void *);

  // 指令値
  float posSent; // モータに送信した位置指令値[rad]
  float spdSent; // モータに送信した速度指令値[rad/s]
  float kpSent;  // モータに送信したkp
  float kdSent;  // モータに送信したkd
  float trqSent; // モータに送信したトルク指令値[Nm]

  // 受信値
  float posReceived;      // モータから受信した位置[rad]
  float spdReceived;      // モータから受信した速度[rad/s]
  float trqReceived;      // モータから受信したトルク[Nm]
  float integratingAngle; // 積算回転角[rad]
};
