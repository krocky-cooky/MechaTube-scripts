/**
 * @file Filter.hpp
 * @author thgcMtdh
 * @brief LPF, HPFなどの一時遅れ系フィルタを実装
 * @date 2022-06-25
 * @note https://github.com/thgcMtdh/PortableMoha より移植
 */

#pragma once

class FirstLPF
{
private:
  float tau_;  // 1次遅れ時定数[s]
  float x_[2]; // 現在およびひとつ前のx. 0が最新、1がひとつ前
  float y_[2]; // 現在およびひとつ前のy

public:
  FirstLPF()
    : tau_(0.0), x_{0.0, 0.0}, y_{0.0, 0.0}
  {
  }
  ~FirstLPF(){};

  /**
   * @brief
   * @param[in] tau 一次遅れ時定数[s]
   */
  void setTau(const float tau)
  {
    tau_ = tau;
  }

  /**
   * @brief 内部変数をリセット
   * @param[in] init xとyをリセットする初期値
   */
  void clear(const float init)
  {
    x_[0] = init;
    x_[1] = init;
    y_[0] = init;
    y_[1] = init;
  }

  /**
   * @brief 入力から出力を計算
   * @param[in] x 入力
   * @param[in] dt 直前の入力からの経過時間[s]
   * @retval y 出力
   */
  inline float update(const float x, const float dt)
  {
    x_[1] = x_[0];
    y_[1] = y_[0];
    x_[0] = x;
    y_[0] = 1.0 / (2.0 * tau_ + dt) * ((2.0 * tau_ - dt) * y_[1] + dt * (x_[0] + x_[1]));
    return y_[0];
  }
};

class FirstHPF
{
private:
  float tau_;  // 1次進み時定数[s]
  float x_[2]; // 現在およびひとつ前のx. 0が最新、1がひとつ前
  float y_[2]; // 現在およびひとつ前のy

public:
  FirstHPF()
    : tau_(0.0), x_{0.0, 0.0}, y_{0.0, 0.0}
  {
  }
  ~FirstHPF(){};

  /**
   * @brief
   * @param[in] tau 一次進み時定数[s]
   */
  void setTau(const float tau)
  {
    tau_ = tau;
  }

  /**
   * @brief 内部変数をリセット
   * @param[in] init xとyをリセットする初期値
   */
  void clear(const float init)
  {
    x_[0] = init;
    x_[1] = init;
    y_[0] = init;
    y_[1] = init;
  }

  /**
   * @brief 入力から出力を計算
   * @param[in] x 入力
   * @param[in] dt 直前の入力からの経過時間[s]
   * @retval y 出力
   */
  inline float update(const float x, const float dt)
  {
    x_[1] = x_[0];
    y_[1] = y_[0];
    x_[0] = x;
    y_[0] = 1.0 / (2.0 * tau_ + dt) * ((2.0 * tau_ - dt) * y_[1] + 2.0 * tau_ * (x_[0] - x_[1]));
    return y_[0];
  }
};

/**
 * @brief 高域を遅延させるオールパスフィルタ
 */
class FirstAPF
{
private:
  float omegab_; // ブレイク角周波数[rad/s]: 遅延が90°になる角周波数
  float x_[2];   // 現在およびひとつ前のx. 0が最新、1がひとつ前
  float y_[2];   // 現在およびひとつ前のy

public:
  FirstAPF()
    : omegab_(0.0), x_{0.0, 0.0}, y_{0.0, 0.0}
  {
  }
  ~FirstAPF(){};

  /**
   * @brief ブレイク周波数を設定
   * @param[in] freqb ブレイク周波数[Hz]
   */
  void setTau(const float freqb)
  {
    omegab_ = 2.0 * PI * freqb;
  }

  /**
   * @brief 内部変数をリセット
   * @param[in] init xとyをリセットする初期値
   */
  void clear(const float init)
  {
    x_[0] = init;
    x_[1] = init;
    y_[0] = init;
    y_[1] = init;
  }

  /**
   * @brief 入力から出力を計算
   * @param[in] x 入力
   * @param[in] dt 直前の入力からの経過時間[s]
   * @retval y 出力
   */
  inline float update(const float x, const float dt)
  {
    x_[1] = x_[0];
    y_[1] = y_[0];
    x_[0] = x;
    y_[0] = (2 - dt * omegab_) / (2 + dt * omegab_) * (y_[1] - x_[0]) + x_[1];
    return y_[0];
  }
};
