/// @file 
/// @brief モータとのCANメッセージ送受信を担うモジュール
/// @author thgcMtdh
/// @date 2022/2/13
/// @note serial_communication.ino より移植

#pragma once

#include <Arduino.h>

class Tmotor
{
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

private:  
  const int MOTOR_ID;
  const int DRIVER_ID;
  const uint8_t msgEnter[8];
  const uint8_t msgExit[8];
  const uint8_t msgSetPosToZero[8];

  static void packCmd(uint8_t *, float, float, float, float, float);
  static int float_to_uint(float, float, float, unsigned int);
  static void unpackReply(volatile uint8_t *, volatile int*, volatile float *, volatile float *, volatile float *);
  static float uint_to_float(int, float, float, int);
  
public:
  // コンストラクタ,デストラクタ
  Tmotor(int, int);
  ~Tmotor();

  // CAN送信
  int sendCommand(float, float, float, float, float);
  int sendMotorControl(bool);

  // CAN受信(Tmotorオブジェクトが複数あったとしてもCANの受信口は一つなので、クラス内に一つのstaticメンバ関数として宣言)
  static void onRecieve(int);

  // CAN受信メッセージ
  static uint8_t canRecievedMsg[8];

  // 指令値
  float positionRef;
  float speedRef;
  float kp;
  float kd;
  float torqueRef;

  // 受信値
  float positionRecieved;
  float speedRecieved;
  float torqueRecieved;
};
