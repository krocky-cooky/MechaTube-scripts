/// @file 
/// @brief 一次遅れ系
/// @author thgcMtdh
/// @date 2021/10/5          

#define TAU 1.0  // 時定数[s]

// 1次遅れ系の計算に必要な現在と過去のデータをためておく
typedef struct FirstOrderDelayData {
  float y;
  float x;
  float yprev;
  float xprev;
} FirstOrderDelayData_t;

FirstOrderDelayData_t torqueData = {0.0, 0.0, 0.0, 0.0};
FirstOrderDelayData_t speedData = {0.0, 0.0, 0.0, 0.0};


/// @brief トルクについて1次遅れ計算を行う
/// @param[in] input 系への入力 [Nm]
/// @param[in] dt 前回の入力から今回の入力までの経過時間 [s]
/// @return output 系の出力 [Nm]
float firstOrderDelay_torque(float input, float dt) {
  torqueData.xprev = torqueData.x;
  torqueData.yprev = torqueData.y;
  torqueData.x = input;
  torqueData.y = 1/(2*TAU + dt) * ((2*TAU - dt) * torqueData.yprev + dt * (torqueData.x + torqueData.xprev));
  return torqueData.y;
}

float firstOrderDelay_torque_input_tau(float input, float dt, float tau) {
  torqueData.xprev = torqueData.x;
  torqueData.yprev = torqueData.y;
  torqueData.x = input;
  torqueData.y = 1/(2*tau + dt) * ((2*tau - dt) * torqueData.yprev + dt * (torqueData.x + torqueData.xprev));

  if (torqueData.y > input){
    torqueData.y = input;
  }
  
  return torqueData.y;
}


/// @brief 速度について1次遅れ計算を行う
/// @param[in] input 系への入力 [rad/s]
/// @param[in] dt 前回の入力から今回の入力までの経過時間 [s]
/// @return output 系の出力 [rad/s]
float firstOrderDelay_speed(float input, float dt) {
  speedData.xprev = speedData.x;
  speedData.yprev = speedData.y;
  speedData.x = input;
  speedData.y = 1/(2*TAU + dt) * ((2*TAU - dt) * speedData.yprev + dt * (speedData.x + speedData.xprev));
  return speedData.y;
}


/// @brief トルクについて1次遅れ系の変数をリセットする
/// @param none
/// @return none
void firstOrderDelay_resetTorque() {
  torqueData.xprev = 0.0;
  torqueData.yprev = 0.0;
  torqueData.x = 0.0;
  torqueData.y = 0.0;
}


/// @brief 速度について1次遅れ系の変数をリセットする
/// @param none
/// @return none
void firstOrderDelay_resetSpeed() {
  speedData.xprev = 0.0;
  speedData.yprev = 0.0;
  speedData.x = 0.0;
  speedData.y = 0.0;
}

// 1st-order transfer function: Y = 1 / (tau*s + 1) * X
// apply tustin transform:      y = 1/(2*tau + T) * {(2*tau - T) * y' + T * x + T * x'}
//     y,x: current values, y',x': previous values, T: period   
