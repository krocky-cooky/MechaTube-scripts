/// @file 
/// @brief 一次遅れ系
/// @author thgcMtdh
/// @date 2021/10/5          

#define TAU 5.0  // 時定数[s]


/// @brief トルクについて1次遅れ計算を行う
/// @param[in] input 系への入力 [Nm]
/// @param[in] dt 前回の入力から今回の入力までの経過時間 [s]
/// @return output 系の出力 [Nm]
float firstOrderDelay_torque(float input, float dt) {
  static float y = 0.0, x = 0.0, yprev = 0.0, xprev = 0.0;
  xprev = x;
  yprev = y;
  x = input;
  y = 1/(2*TAU + dt) * ((2*TAU - dt) * yprev + dt * (x + xprev));
  return y;
}


/// @brief 速度について1次遅れ計算を行う
/// @param[in] input 系への入力 [rad/s]
/// @param[in] dt 前回の入力から今回の入力までの経過時間 [s]
/// @return output 系の出力 [rad/s]
float firstOrderDelay_speed(float input, float dt) {
  static float y = 0.0, x = 0.0, yprev = 0.0, xprev = 0.0;
  xprev = x;
  yprev = y;
  x = input;
  y = 1/(2*TAU + dt) * ((2*TAU - dt) * yprev + dt * (x + xprev));
  return y;
}


// 1st-order transfer function: Y = 1 / (tau*s + 1) * X
// apply tustin transform:      y = 1/(2*tau + T) * {(2*tau - T) * y' + T * x + T * x'}
//     y,x: current values, y',x': previous values, T: period   