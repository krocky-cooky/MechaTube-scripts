#include "Tmotor.h"
#include "ESP32BuiltinCAN.hpp"

// staticメンバ変数の実体を生成
const uint8_t Tmotor::msgEnter[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc};        // Enter motor control mode
const uint8_t Tmotor::msgExit[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd};         // Exit motor control mode
const uint8_t Tmotor::msgSetPosToZero[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe}; // set the preCommand position of the motor to zero
TaskHandle_t Tmotor::onReceiveTaskHandle_ = NULL;
std::map<int, Tmotor *> Tmotor::motorIdMap_;
uint8_t Tmotor::msgReceived_[] = {0, 0, 0, 0, 0, 0};

Tmotor::Tmotor(ESP32BuiltinCAN &can, int motorId, int driverId)
  : CAN_(can),
    MOTOR_ID_(motorId),
    DRIVER_ID_(driverId),
    motorCtrl_(false),
    posCommand(0.0), spdCommand(0.0), kpCommand(0.0), kdCommand(0.0), trqCommand(0.0), posRecieved(0.0), spdRecieved(0.0), trqRecieved(0.0)
{
  CAN_.set_callback(&onRecieve, this); // onReceive関数を、いま新たに生成した自身を指すポインタthisとともにコールバック登録
  motorIdMap_[motorId] = this;
}

Tmotor::~Tmotor()
{
  motorIdMap_.erase(MOTOR_ID_);
}

bool Tmotor::operator<(const Tmotor &rhs) const
{
  return MOTOR_ID_ < rhs.MOTOR_ID_;
}

int Tmotor::sendCommand(float pos, float spd, float kp, float kd, float trq)
{
  // limit data to be within bounds
  posCommand = fminf(fmaxf(P_MIN, pos), P_MAX);
  spdCommand = fminf(fmaxf(V_MIN, spd), V_MAX);
  kpCommand = fminf(fmaxf(Kp_MIN, kp), Kp_MAX);
  kdCommand = fminf(fmaxf(Kd_MIN, kd), Kd_MAX);
  trqCommand = fminf(fmaxf(T_MIN, trq), T_MAX);

  uint8_t buf[8];
  packCmd(buf, posCommand, spdCommand, kpCommand, kdCommand, trqCommand);

  CAN_.send(MOTOR_ID_, buf, sizeof(buf));

  // Serial.printf("{\"trqCommand\":%f, \"spdCommand\":%f, \"posCommand\":%f}\n", trq, spd, pos);
  // Serial.printf("%x %x %x %x %x %x %x %x\n", buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);
  // Serial.printf("trq command = %d\n", (((int)buf[6] & 0xF) << 8) | buf[7]);  // CANコマンドの送信値(0-4095)をprint

  return 1;
}

int Tmotor::sendMotorControl(bool command)
{
  if (command == 1 && motorCtrl_ == 0) {
    if (!CAN_.send(MOTOR_ID_, msgEnter, sizeof(msgEnter))) return 0;
  } else if (command == 0 && motorCtrl_ == 1) {
    if (!CAN_.send(MOTOR_ID_, msgExit, sizeof(msgExit))) return 0;
  }
  motorCtrl_ = command;
  return 1;
}

bool Tmotor::getMotorControl()
{
  return motorCtrl_;
}

void IRAM_ATTR Tmotor::onRecieve(int packetSize, void *pTmotor)
{
  Tmotor *tMotor = reinterpret_cast<Tmotor *>(pTmotor);
  int id = tMotor->CAN_.packetId();
  if (id == tMotor->DRIVER_ID_ && packetSize == 6) { //ドライバー宛のメッセージであれば内容を転記
    for (int i = 0; i < 6; i++) {
      msgReceived_[i] = tMotor->CAN_.read();
    }
  }
  BaseType_t taskWoken;
  xTaskNotifyFromISR(onReceiveTaskHandle_, 0, eNoAction, &taskWoken); // 通知を送信
}

void Tmotor::onReceiveTask()
{
  while (1) {
    xTaskNotifyWait(0, 0, NULL, portMAX_DELAY); // CAN受信割り込み関数が呼ばれるまで待機
    int motorId;
    float pos, spd, trq;
    unpackReply(msgReceived_, &motorId, &pos, &spd, &trq); // 受信メッセージをunpack
    motorIdMap_[motorId]->posRecieved = pos;               // 変数を更新
    motorIdMap_[motorId]->spdRecieved = spd;
    motorIdMap_[motorId]->trqRecieved = trq;
  }
}

void Tmotor::packCmd(uint8_t *data, float p_des, float v_des, float kp, float kd, float t_ff)
{
  /// convert floats to unsigned ints ///
  int p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);
  int v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
  int kp_int = float_to_uint(kp, Kp_MIN, Kp_MAX, 12);
  int kd_int = float_to_uint(kd, Kd_MIN, Kd_MAX, 12);
  int t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);

  /// pack ints into the can buffer ///
  data[0] = p_int >> 8;                           // pos 8-H
  data[1] = p_int & 0xFF;                         // pos 8-L
  data[2] = v_int >> 4;                           // spd 8-H
  data[3] = ((v_int & 0xF) << 4) | (kp_int >> 8); // spd 4-L KP-8H
  data[4] = kp_int & 0xFF;                        // KP 8-L
  data[5] = kd_int >> 4;                          // Kd 8-H
  data[6] = ((kd_int & 0xF) << 4) | (t_int >> 8); // KP 4-L trq 4-H
  data[7] = t_int & 0xff;                         // trq 8-L
}

int Tmotor::float_to_uint(float x, float x_min, float x_max, unsigned int bits)
{
  /// Converts a float to an unsigned int, given range and number of bits / //
  float span = x_max - x_min;
  if (x < x_min) {
    x = x_min;
  } else if (x > x_max) {
    x = x_max;
  }
  return (int)((x - x_min) * ((float)((1 << bits) - 1) / span));
}

void Tmotor::unpackReply(uint8_t *data, int *id, float *pos, float *spd, float *trq)
{
  /// unpack ints from can buffer ///
  int motor_id = data[0];                       // Motor ID number
  int p_int = (data[1] << 8) | data[2];         // Motor pos data
  int v_int = (data[3] << 4) | (data[4] >> 4);  // Motor spd data
  int i_int = ((data[4] & 0xF) << 8) | data[5]; // Motor trq data

  /// convert ints to floats ///
  float p = uint_to_float(p_int, P_MIN, P_MAX, 16);
  float v = uint_to_float(v_int, V_MIN, V_MAX, 12);
  float i = uint_to_float(i_int, -T_MAX, T_MAX, 12);

  /// Read the corresponding data according to the ID number ///
  *id = motor_id;
  *pos = p;
  *spd = v;
  *trq = i;
}

float Tmotor::uint_to_float(int x_int, float x_min, float x_max, int bits)
{
  /// converts unsigned int to float, given range and number of bits / //
  float span = x_max - x_min;
  float offset = x_min;
  return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}
