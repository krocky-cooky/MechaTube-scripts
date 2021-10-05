/// @file 
/// @brief シリアル通信の送受信と、文字列<->変数間のデコードを行う関数群
/// シリアル通信をBluetooth経由に変更したい場合は、#define SERIAL_USE_BLUETOOTH のコメントアウトを解除してください
/// @author thgcMtdh
/// @date 2021/10/4

#include <Arduino.h>

// #define SERIAL_USE_BLUETOOTH
#define SERIAL_BUFSIZE 20

extern bool motorPowerCommand;
extern bool motorControlCommand;
extern float torqueCommand;
extern float speedCommand;


/// @brief 受信したシリアル通信データを読み取り、コマンドが来ていれば反映する
/// @param none
/// @return 0:コマンドは来ていない, 1: 有効なコマンドが送られてきた
int serial_decodeIncomingCommand() {
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
      if (decodeCommand(buf) > 0) retval = 1;
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
        if (decodeCommand(bufBT) > 0) retval = 1;
      }
    }
  }
  #endif

  return retval;
}


/// @brief 内部変数をJSON形式の文字列にしてシリアル通信で送る
/// @param none
/// @return 0: failure, 1: success
int serial_sendVariablesJSON() {
  Serial.println("{\"test\":0.0}");
  return 1;
}


/// @brief シリアル通信で送られてきた文字列コマンドをデコードし、グローバル変数に値を格納する
/// @param[in] command 解釈する文字列
/// @retval コマンドが無効な場合は0を、正常に解釈できた場合はコマンドの種類('p','m','t','s')を返す
int decodeCommand(const char* command) {
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
        motorPowerCommand = (value > 0.5)? true : false;  // float value contains small error so it doesn't exactly eauql to 0 or 1 integer
        if (!motorPowerCommand && motorControlCommand) {
          motorControlCommand = false;         // if the power command is false but the control is still on, turns the control OFF
        }
        torqueCommand = 0.0;
        speedCommand = 0.0;
        break;
      case 'm':
        motorControlCommand = (value > 0.5)? true : false;
        if (motorControlCommand && !motorPowerCommand) {
          motorPowerCommand = true;            // if the control command is true but motor power is off, turns the power ON
        }
        torqueCommand = 0.0;
        speedCommand = 0.0;
        break;
      case 't':
        motorPowerCommand = true;
        motorControlCommand = true;
        torqueCommand = value;
        break;
      case 's':
        motorPowerCommand = true;
        motorControlCommand = true;
        speedCommand = value;
        break;
      default:
        return 0;
    }
    return key;
  }
  return 0;
}