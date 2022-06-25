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
  static const uint8_t msgEnter[8];
  static const uint8_t msgExit[8];
  static const uint8_t msgSetPosToZero[8];

  ESP32BuiltinCAN &CAN_;
  const int MOTOR_ID_;
  const int DRIVER_ID_;
  bool motorCtrl_;
  static TaskHandle_t onReceiveTaskHandle_;
  static std::map<int, Tmotor *> motorIdMap_; // motorIdをkeyとして、Tmotorインスタンスへのポインタを取り出すmap
  static uint8_t msgReceived_[6];             // CAN受信メッセージ
  static void onReceiveTask();

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
  /// @brief Tmotorオブジェクトを生成する
  /// @param[in] can CANインスタンスへの参照
  /// @param[in] motorId モーターの CAN ID
  /// @param[in] driverId ドライバーのCAN ID
  Tmotor(ESP32BuiltinCAN &, int, int);

  /// @brief Tmotorオブジェクトのデストラクタ
  ~Tmotor();

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

  /// @brief この関数を呼び出した瞬間の積算位置を、強制的に指定した値にリセットする
  /// @param newIntegratingAngle 新しく指定する積算位置[rad]
  void setIntegratingAngle(float newIntegratingAngle);

  /// @brief CAN受信割り込みに登録するコールバック関数
  /// @param[in] packetSize 受信したバイト数(CAN.onReceiveから渡される)
  /// @param[in] pTmotor Tmotorインスタンスへのポインタ
  /// @return none
  static void IRAM_ATTR onRecieve(int, void *);

  // 指令値
  float posCommand; // モータに送信した位置指令値[rad]
  float spdCommand; // モータに送信した速度指令値[rad/s]
  float kpCommand;  // モータに送信したkp
  float kdCommand;  // モータに送信したkd
  float trqCommand; // モータに送信したトルク指令値[Nm]

  // 受信値
  float posRecieved; // モータから受信した位置[rad]
  float spdRecieved; // モータから受信した速度[rad/s]
  float trqRecieved; // モータから受信したトルク[Nm]
  float integratingAngle; // 積算回転角[rad]
};
