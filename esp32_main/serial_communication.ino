/// @file 
/// @brief シリアル通信の送受信と、文字列<->変数間のデコードを行う関数群
/// シリアル通信をBluetooth経由に変更したい場合は、#define SERIAL_USE_BLUETOOTH のコメントアウトを解除してください
/// @author thgcMtdh
/// @date 2021/10/4

#include <Arduino.h>

#define SERIAL_BUFSIZE 20
// #define SERIAL_USE_BLUETOOTH

/// @brief 受信したシリアル通信データを読み取り、コマンドが来ていれば取得して転記する
/// @param[out] pPower モータ電源の指令値を格納する変数へのポインタ
/// @param[out] pControl モータの制御モード指令値を格納する変数へのポインタ
/// @param[out] pMode トルク制御か速度制御かを指定する変数へのポインタ
/// @param[out] pTorque トルク指令値を格納する変数へのポインタ(トルク指令:1, 速度指令:0)
/// @param[out] pSpeed 速度指令値を格納する変数へのポインタ
/// @param[out] pIncreaseOfToraueForEccentricMotion エキセン動作時のトルク増加量を格納する変数へのポインタ
/// @return 0:コマンドは来ていない, 1: 有効なコマンドが送られてきた
int serial_getIncomingCommand(bool* pPower, bool* pControl, bool* pMode, float* pTorque, float* pSpeed, float* pIncreaseOfToraueForEccentricMotion) {
  static char buf[SERIAL_BUFSIZE] = "";
  static int i = 0;
  static char bufBT[SERIAL_BUFSIZE] = "";
  static int iBT = 0;
  int retval = 0;

  while (Serial.available()) {
    char c = Serial.read();          // read a received character
    if (c == '(') i = 0;             // if the character is '(' which indicates the beginnig of the message, move index to zero
    buf[i] = c;                      // add the character to the buffer
    i++;                             // move the index to the next address
    if (i >= SERIAL_BUFSIZE) i = 0;  // if the index exceeds the length of buffer, reset the index to zero

    if (c == ')') {                  // if a command transmission has finished, try to decode it
      if (decodeCommand(buf, pPower, pControl, pMode, pTorque, pSpeed, pIncreaseOfToraueForEccentricMotion) > 0) retval = 1;
    }
  }

  #ifdef SERIAL_USE_BLUETOOTH
  if (!retval) {  // if there is no data available on HardwareSerial, try to read data from Bluetooth Serial

    while (SerialBT.available()) {
      char c = SerialBT.read();            // read a received character
      if (c == '(') i = 0;                 // if the character is '(' which indicates the beginnig of the message, move index to zero
      bufBT[i] = c;                        // add the character to the buffer
      iBT++;                               // move the index to the next address
      if (iBT >= SERIAL_BUFSIZE) iBT = 0;  // if the index exceeds the length of buffer, reset the index to zero

      if (c == ')') {                     // if a command transmission has finished, try to decode it
        if (decodeCommand(bufBT, pPower, pControl, pMode, pTorque, pSpeed, pIncreaseOfToraueForEccentricMotion) > 0) retval = 1;
    }
  }
  #endif

  return retval;
}


/// @brief シリアル通信で送られてきた文字列コマンドをデコードし、与えられた変数に指令値を格納する
/// @param[in] command 解釈する文字列
/// @param[out] pPower モータ電源の指令値を格納する変数へのポインタ
/// @param[out] pControl モータの制御モード指令値を格納する変数へのポインタ
/// @param[out] pMode トルク制御か速度制御かを指定する変数へのポインタ
/// @param[out] pTorque トルク指令値を格納する変数へのポインタ
/// @param[out] pSpeed 速度指令値を格納する変数へのポインタ
/// @retval コマンドが無効な場合は0を、正常に解釈できた場合はコマンドの種類('p','m','t','s')を返す
int decodeCommand(const char* command, bool* pPower, bool* pControl, bool* pMode, float* pTorque, float* pSpeed, float* pIncreaseOfToraueForEccentricMotion) {
  int i = 0;
  if (command[0] == '(') {                // if the command starts form '(', it may be a valid command
    while (command[i] != ')') {           // find ')' which indicates the end of the command. if not found, invalid command
      i++;
      if (i >= SERIAL_BUFSIZE) return 0;
    }

    char key;
    float value;
    sscanf(command, "(%c,%f)", &key, &value);  // scan the command

    switch (key) {                             // copy the commanded value
      case 'p':
        *pPower = (value > 0.5)? true : false;  // float value contains small error so it doesn't exactly eauql to 0 or 1 integer
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
        //*pMode = 1;
        //*pPower = true;
        //*pControl = true;
        //*pTorque = value;
        *pIncreaseOfToraueForEccentricMotion = value;
        //*pSpeed = 0.0;
        break;
      case 's':
        *pMode = 0;
        *pPower = true;
        *pControl = true;
        *pSpeed = value;
        *pTorque = 0.0;
        break;
      default:
        return 0;
    }
    return key;
  }
  return 0;
}
