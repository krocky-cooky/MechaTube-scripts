/// @file 
/// @brief シリアル通信の送受信と、文字列<->変数間のデコードを行う関数群
/// @author thgcMtdh
/// @date 2021/10/4

#include <Arduino.h>

#define SERIAL_BUFSIZE 20

/// @brief 受信したシリアル通信データを読み取り、コマンドが来ていれば取得して転記する
/// @param[out] pPower モータ電源の指令値を格納する変数へのポインタ
/// @param[out] pControl モータの制御モード指令値を格納する変数へのポインタ
/// @param[out] pMode トルク制御か速度制御かを指定する変数へのポインタ
/// @param[out] pTorque トルク指令値を格納する変数へのポインタ(トルク指令:1, 速度指令:0)
/// @param[out] pSpeed 速度指令値を格納する変数へのポインタ
/// @param[out] pIncreaseOfToraueForEccentricMotion エキセン動作時のトルク増加量を格納する変数へのポインタ
/// @param[out] pMaxSpeedWhileConcentricMotion アイソキネティックトレーニング時の回転スピードを格納する変数へのポインタ
/// @param[out] pIncreaseOfToraueWhenPeak increaseOfToraueWhenPeakへのポインタ
/// @param[out] protationAngleFromInitialPositionWhenPeak rotationAngleFromInitialPositionWhenPeakへのポインタ
/// @param[out] pRangeOfTorqueChange rangeOfTorqueChangeへのポインタ
/// @return 0:コマンドは来ていない, 1: 有効なコマンドが送られてきた
int serial_getIncomingCommand(bool* pPower, bool* pControl, bool* pMode, float* pTorque, float* pSpeed, float* pIncreaseOfToraueForEccentricMotion, float* pMaxSpeedWhileConcentricMotion, float* pIncreaseOfToraueWhenPeak, float* protationAngleFromInitialPositionWhenPeak, float* pRangeOfTorqueChange) {
  char buf[SERIAL_BUFSIZE] = "";
  int i = 0;
  int retval = 0;

  while (Serial.available()) {
    char c = Serial.read();          // read a recieved character
    buf[i] = c;                      // add the character to the buffer
    i++;                             // move the index to the next address
    if (i >= SERIAL_BUFSIZE) i = 0;  // if the index exceeds the length of buffer, reset the index to zero

    if (c == '\n') {  // 改行でコマンドの終了を検知する
      i = 0;
      return decodeCommand(buf, pPower, pControl, pMode, pTorque, pSpeed, pIncreaseOfToraueForEccentricMotion, pMaxSpeedWhileConcentricMotion, pIncreaseOfToraueWhenPeak, protationAngleFromInitialPositionWhenPeak, pRangeOfTorqueChange);
    }
  }
  return 0;
}


/// @brief シリアル通信で送られてきた文字列コマンドをデコードし、与えられた変数に指令値を格納する
/// @param[in] command 解釈する文字列
/// @param[out] pPower モータ電源の指令値を格納する変数へのポインタ
/// @param[out] pControl モータの制御モード指令値を格納する変数へのポインタ
/// @param[out] pMode トルク制御か速度制御かを指定する変数へのポインタ
/// @param[out] pTorque トルク指令値を格納する変数へのポインタ
/// @param[out] pSpeed 速度指令値を格納する変数へのポインタ
/// @param[out] pMaxSpeedWhileConcentricMotion アイソキネティックトレーニング時の回転スピードを格納する変数へのポインタ
/// @retval コマンドが無効な場合は0を、正常に解釈できた場合はコマンドの種類('p','m','t','s')を返す
int decodeCommand(const char* command, bool* pPower, bool* pControl, bool* pMode, float* pTorque, float* pSpeed, float* pIncreaseOfToraueForEccentricMotion, float* pMaxSpeedWhileConcentricMotion, float* pIncreaseOfToraueWhenPeak, float* protationAngleFromInitialPositionWhenPeak, float* pRangeOfTorqueChange) {
  char key;
  float value;
  sscanf(command, "%c%f", &key, &value);  // scan the command

  switch (key) {                          // copy the commanded value
    case 'p':
      *pPower = (value > 0.5)? true : false;  // float value contains small error so it is not exactly equal to 0 or 1 integer
      if (!*pPower && *pControl) {
        *pControl = false;         // if the power command is false but the control is still on, turns the control OFF
      }
      *pTorque = 0.0;
      *pSpeed = 0.0;
      break;
    case 'm':
      *pControl = (value > 0.5)? true : false;
      if (*pControl && !*pPower) {
        *pPower = true;            // if the control command is true but motor power is off, turns the power ON
      }
      *pTorque = 0.0;
      *pSpeed = 0.0;
      break;
    case 't':
      *pMode = 1;
      *pPower = true;
      *pControl = true;
      *pTorque = value;
      *pSpeed = 0.0;
      break;
    case 'e':
      *pIncreaseOfToraueForEccentricMotion = value;
      break;
    case 's':
      *pMode = 0;
      *pPower = true;
      *pControl = true;
      *pSpeed = value;
      *pTorque = 0.0;
      break;
    case 'i':
      *pMaxSpeedWhileConcentricMotion = value;
      break;
    case 'a':
      *pIncreaseOfToraueWhenPeak=value;
      break;
    case 'b':
      *protationAngleFromInitialPositionWhenPeak=value;
      break;
    case 'c':
      *pRangeOfTorqueChange=value;
      break;
    default:
      return 0;
  }
  return key;
}
