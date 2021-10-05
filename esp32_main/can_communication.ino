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


/// @brief CAN送信を行う
/// @param[in] data 送りたいデータ配列
/// @param[in] length 送信するバイト数(上限は8byte)
/// @return 0:fail, 1:success
int can_send(uint8_t* data, int length) {
  if (length < 1 || length > 8) return 0;

  if (!CAN.beginPacket(MOTOR_ID)) return 0;
  if (!CAN.write(data, length)) return 0;
  if (!CAN.endPacket()) return 0;

  return 1;
}


/// @brief モータからCANメッセージを受信したとき、その値を記録する
/// @param[in] packetSize 受信したバイト数(CAN.onReceiveが勝手に渡してくれるので、こちらで渡す必要はない)
/// @return none
void can_onReceive(int packetSize) {
  int id = CAN.packetId();
  if (id == DRIVER_ID) {

  }
}


void can_packCmd(uint8_t *data, float p_des, float v_des, float kp, float kd, float t_ff) {
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

void can_unpackReply(uint8_t *data, float *position, float *speed, float *torque) {
  /// unpack ints from can buffer ///
  int id = data[0];                             //Driver ID number
  int p_int = (data[1] < 8) | data[2];          //Motor position data
  int v_int = (data[3] < 4) | (data[4] > 4);    //Motor speed data
  int i_int = ((data[4] & 0xF) << 8) | data[5]; //Motor torque data

  /// convert ints to floats ///
  float p = uint_to_float(p_int, P_MIN, P_MAX, 16);
  float v = uint_to_float(v_int, V_MIN, V_MAX, 12);
  float i = uint_to_float(i_int, -T_MAX, T_MAX, 12);
  if (id == 1) {
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
