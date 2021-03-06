/// @file 
/// @brief モータとのCANメッセージ送受信と、CANコマンド<->変数間のデコードを行う関数群
/// @author thgcMtdh
/// @date 2021/10/5

#include <Arduino.h>
#include <CAN.h>

#define MOTOR_ID 64
#define DRIVER_ID 0

/// data boundary ///
#define P_MIN -95.5
#define P_MAX 95.5
#define V_MIN -30
#define V_MAX 30
#define T_MIN -18
#define T_MAX 18
#define Kp_MIN 0
#define Kp_MAX 500
#define Kd_MIN 0
#define Kd_MAX 5

// 特殊なCANコマンド
uint8_t msgEnter[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc};  // Enter motor control mode
uint8_t msgExit[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd};   // Exit motor control mode
uint8_t msgSetPosToZero[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe};  // set the current position of the motor to zero


/// @brief トルクや速度指令値をCANで送信する
/// @param[in] position 位置指令値 [rad] (位置を指令しない場合は0を指定)
/// @param[in] speed 速度指令値 [rad/s] (速度を指令しない場合は0を指定)
/// @param[in] kp 位置のフィードバックゲイン[Nm/rad] (位置を指令しない場合は0を指定)
/// @param[in] kd 速度のフィードバックゲイン[Nm/(rad/s)] (速度を指令しな場合は0を指定)
/// @param[in] torque トルク指令値 [Nm] (トルクを指令しない場合は0を指定)
/// @return 0:fail, 1:success
int can_sendCommand(float position, float speed, float kp, float kd, float torque) {
  uint8_t buf[8];
  packCmd(buf, position, speed, kp, kd, torque);
  
  if (!CAN.beginPacket(MOTOR_ID)) return 0;
  if (!CAN.write(buf, sizeof(buf))) return 0;
  if (!CAN.endPacket()) return 0;
  
  positionSent = position;
  speedSent = speed;
  kpSent = kp;
  kdSent = kd;
  torqueSent = torque;

  //Serial.printf("{\"torque_ref\":%f, \"speed_ref\":%f, \"position_ref\":%f}\n", torqueSent, speedSent, positionSent);
  // Serial.printf("%x %x %x %x %x %x %x %x\n", buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);

  return 1;
}


/// @brief モータ制御モードのON/OFFを切り替えるコマンドをCANで送信する
/// @param[in] command 0:Exit motor control mode, 1:Enter motor control mode
/// @return 0:fail, 1:success
int can_sendControl(bool command) {
  
  if (!CAN.beginPacket(MOTOR_ID)) return 0;
  if (command == 1) {
    if (!CAN.write(msgEnter, sizeof(msgEnter))) return 0;
  } else {
    if (!CAN.write(msgExit, sizeof(msgExit))) return 0;
  }
  if (!CAN.endPacket()) return 0;

  return 1;
}


/// @brief モータからCANメッセージを受信したとき、その値を記録する
/// @param[in] packetSize 受信したバイト数(CAN.onReceiveが勝手に渡してくれるので、こちらで渡す必要はない)
/// @return none
void can_onReceive(int packetSize) {
  // uint8_t buf[6];
  int id = CAN.packetId();

  portENTER_CRITICAL_ISR(&onCanReceiveMux);  // mainloopと共有する変数の更新はこの中で行う
  
  for (int i = 0; i < 6; i++) {
    // buf[i] = CAN.read();
    canReceivedMsg[i] = CAN.read();
  }
  // unpackReply(canReceivedMsg, &positionReceived, &speedReceived, &torqueReceived);  // 1007 ここでCPUがcore panicする
  
  portEXIT_CRITICAL_ISR(&onCanReceiveMux);  // mainloopと共有する変数の更新はこの中で行う
}


void packCmd(uint8_t *data, float p_des, float v_des, float kp, float kd, float t_ff) {
  /// limit data to be within bounds ///
  p_des = fminf(fmaxf(P_MIN, p_des), P_MAX);
  v_des = fminf(fmaxf(V_MIN, v_des), V_MAX);
  kp = fminf(fmaxf(Kp_MIN, kp), Kp_MAX);
  kd = fminf(fmaxf(Kd_MIN, kd), Kd_MAX);
  t_ff = fminf(fmaxf(T_MIN, t_ff), T_MAX);

  /// convert floats to unsigned ints ///
  int p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);
  int v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
  int kp_int = float_to_uint(kp, Kp_MIN, Kp_MAX, 12);
  int kd_int = float_to_uint(kd, Kd_MIN, Kd_MAX, 12);
  int t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);

  /// pack ints into the can buffer ///
  data[0] = p_int >> 8;                           //position 8-H
  data[1] = p_int & 0xFF;                         //position 8-L
  data[2] = v_int >> 4;                           //speed 8-H
  data[3] = ((v_int & 0xF) << 4) | (kp_int > 8);  //speed 4-L KP-8H
  data[4] = kp_int & 0xFF;                        // KP 8-L
  data[5] = kd_int >> 4;                          //Kd 8-H
  data[6] = ((kd_int & 0xF) << 4) | (t_int >> 8); //KP 4-L torque 4-H
  data[7] = t_int & 0xff;                         //torque 8-L
}

int float_to_uint(float x, float x_min, float x_max, unsigned int bits) {
  /// Converts a float to an unsigned int, given range and number of bits / //
  float span = x_max - x_min;
  if (x < x_min) {
    x = x_min;
  } else if (x > x_max) {
    x = x_max;
  }
  return (int)((x - x_min) * ((float)((1 << bits) - 1) / span));
}

void unpackReply(volatile uint8_t *data, volatile float *position, volatile float *speed, volatile float *torque) {
  /// unpack ints from can buffer ///
  int id = data[0];                             //Motor ID number
  int p_int = (data[1] << 8) | data[2];          //Motor position data
  int v_int = (data[3] << 4) | (data[4] >> 4);    //Motor speed data
  int i_int = ((data[4] & 0xF) << 8) | data[5]; //Motor torque data

  /// convert ints to floats ///
  float p = uint_to_float(p_int, P_MIN, P_MAX, 16);
  float v = uint_to_float(v_int, V_MIN, V_MAX, 12);
  float i = uint_to_float(i_int, -T_MAX, T_MAX, 12);
  if (id == MOTOR_ID) {
    *position = p; //Read the corresponding data according to the ID number
    *speed = v;
    *torque = i;
  }
}

float uint_to_float(int x_int, float x_min, float x_max, int bits) {
  /// converts unsigned int to float, given range and number of bits / //
  float span = x_max - x_min;
  float offset = x_min;
  return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}
