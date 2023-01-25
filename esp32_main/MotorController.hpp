/**
 * @file MotorController.hpp
 * @author thgcMtdh
 * @brief tmotorを制御するモータ制御系
 * @date 2022-07-27
 */

/*
[使い方]

Tmotor tmotor;
MotorController motor(tmotor);

void setup() {
  motor.init(0.2, 0.1);  // 初期化。引数は1つめがPゲイン、2つめがIゲイン
}

void loop() {
  // トルク制御
  motor.setSpdLimit(5.0, 6.0); // 速度制限を設定。5.0rad/sに達したら速度を減少させはじめ、6.0rad/sでトルク0にする
  motor.startTrqCtrl();  // トルク制御を開始
  motor.setTrqRef(2.0);  // トルク目標値2.0Nm
  motor.setTrqRef(3.0);  // トルク目標値3.0Nmに変更
  motor.stopCtrl();  // 制御を終了

  // 速度制御
  motor.startSpdCtrl();  // トルク制御を開始
  motor.setSpdRef(2.0);  // 速度目標値2.0rad/s
  motor.setSpdRef(3.0);  // 速度目標値を3.0rad/sに変更
  motor.stopCtrl();  // 制御を終了
}
*/

#pragma once

#include "Tmotor.h"
#include <Arduino.h>

class MotorController
{
public:
  float kp;
  float ki;

  // ----------
  // Common API
  // ----------

  /**
   * @brief Construct a new Motor Controller object
   * @param tmotor Tmotorインスタンス
   */
  MotorController(Tmotor &tmotor);

  /**
   * @brief 変数初期化処理。setup()内と、制御を完全にリセットしたいときに実行する
   * @param spdKp 速度制御のPゲイン
   * @param spdKi 速度制御のIゲイン
   */
  void init(float spdKp, float spdKi);

  /**
   * @brief トルクの上限値を指定する
   * @param trqLimit トルク上限値。制御モードにかかわらず、これ以上のトルクは出ない
   */
  void setTrqLimit(float trqLimit);

  /**
   * @brief 入出力の更新処理。タイマ割り込み等で定期的にこの関数を呼び出すこと
   * @param interval 制御周期[us]。直前update実行からの経過時間を入力する
   */
  void update(unsigned long interval);

  /**
   * @brief 制御を停止する
   */
  void stopCtrl();

  // --------------
  // Torque Control
  // --------------

  /**
   * @brief 定トルク制御を開始する
   */
  void startTrqCtrl();

  /**
   * @brief トルク指令値を設定・変更する
   * @param trqRef トルク指令値[Nm]
   */
  void setTrqRef(float trqRef);

  /**
   * @brief 定トルク制御中の、速度上限値を設定・変更する
   * @param spdMaxWindin 巻取り速度最大値[rad/s]。
   *                     巻取り中にこの速度を超えると、トルク指令値が減少し、加速しすぎないようにする
   * @param spdMaxLiftup 挙上速度最大値[rad/s]。
   *                     挙上中にこの速度を超えると、トルク指令値が増加し、挙上できないようにする。
   *                     0.0を指定した場合はこの制御が適用されない
   * @param dTdV [任意] 速度最大値に達したときのトルク減少/増加度合い[Nm/(rad/s)]。
   *             大きいほど強く制限が効くが振動が出やすい。デフォルト値は1.0。デフォルト値を変える際は、
   *             cppファイルの先頭に DEFAULT_DTDV という変数があるので、ここも変更してほしい
   */
  void setSpdLimit(float spdMaxWindin, float spdMaxLiftup, float dTdV = 1.0);

  // -------------
  // Speed Control
  // -------------

  /**
   * @brief 定速制御を開始する
   */
  void startSpdCtrl();

  /**
   * @brief 速度指令値を設定・変更する
   * @param spdRef 速度指令値[rad/s]
   */
  void setSpdRef(float spdRef);

private:
  enum class CtrlObject // 制御対象
  {
    None = 0,
    SpdLimitedTrq, // 速度上限付きトルク制御モード
    Spd            // 速度制御モード
  };

  Tmotor &tmotor_;
  CtrlObject object_; // 制御対象

  float trqLimit_; // トルク最大値[Nm]。モータトルクをこの値より大きくしない

  // トルク制御で使う変数

  float trqRef_;         // トルク指令値[rad/s]
  float spdMaxLiftup_;   // トルク制御中の挙上速度最大値[rad/s]
  float spdMaxWindin_;   // トルク制御中の巻取り速度最大値[rad/s]
  float dTdV_;           // 速度最大値に達したときのトルク減少/増加度合い[Nm/(rad/s)]。大きいほど強く制限が効くが振動が出やすい
  float spdLimitLiftup_; // トルク制御中の挙上速度上限値[rad/s]。この速度でトルク指令値はtrqLimitに等しくなり、これ以上挙上できないようにする
  float spdLimitWindin_; // トルク制御中の巻取り速度上限値[rad/s]。この速度でトルク指令値はゼロになり、巻取り中にこれ以上加速しないようにする

  // 速度制御で使う変数

  float spdRef_;         // 速度指令値[rad/s]
  float spdDevIntegral_; // 速度のPID制御で使う、速度偏差の積分値
  float calculatedTrq_;  // 制御器で計算されたトルク[Nm]

  void clear_(); // 制御器の内部変数や指令値を初期化する。上限値は初期化されない
};
