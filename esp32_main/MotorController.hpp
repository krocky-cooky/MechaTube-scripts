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
  motor.init();  // 初期化
}

void loop() {
  // トルク制御
  motor.setSpdLimit(5.0, 6.0); // 速度制限を設定。5.0rad/sに達したら速度を減少させはじめ、6.0rad/sでトルク0にする
  motor.startTrqCtrl();  // トルク制御を開始
  motor.setTrqRef(2.0);  // トルク目標値2.0Nm
  motor.setTrqRef(3.0);  // トルク目標値3.0Nmに変更
  motor.endCtrl();  // 制御を終了

  // 速度制御
  motor.startSpdCtrl();  // トルク制御を開始
  motor.setSpdRef(2.0);  // 速度目標値2.0rad/s
  motor.setSpdRef(3.0);  // 速度目標値を3.0rad/sに変更
  motor.endCtrl();  // 制御を終了
}
*/

#pragma once

#include "Tmotor.h"
#include <Arduino.h>

class MotorController
{
public:
  const float POLE_OMEGA = 2 * PI * 10; // 極配置法の設計パラメータ
  const float M = 0.01;  // モータの慣性モーメント
  const float D = 0.01;  // モータの粘性定数
  float kp;
  float ki;

  /**
   * @brief Construct a new Motor Controller object
   * @param tmotor Tmotorインスタンス
   */
  MotorController(Tmotor &tmotor);

  /**
   * @brief 初期化処理
   */
  void init();

  /**
   * @brief トルク制御を開始する。開始したときの指令値初期値は0Nm
   */
  void startTrqCtrl();

  /**
   * @brief トルク指令値を設定・変更する
   * @param trqRef トルク指令値[Nm]
   */
  void setTrqRef(float trqRef);

  /**
   * @brief 速度上限値を設定・変更する。トルク制御中のみ有効
   * @param spdLimit 速度上限値[rad/s]。トルク制御中にこの速度を超えると、トルク指令値を減少させはじめる
   * @param spdMax 速度最大値[rad/s]。この速度でトルク指令値はゼロになる
   */
  void setSpdLimit(float spdLimit, float spdMax);

  /**
   * @brief 速度制御を開始する。開始したときの速度初期値は0rad/s
   */
  void startSpdCtrl();

  /**
   * @brief 速度指令値を設定・変更する
   * @param spdRef 速度指令値[rad/s]
   */
  void setSpdRef(float spdRef);

  /**
   * @brief 制御を停止する
   */
  void stopCtrl();

  /**
   * @brief 入出力の更新処理。タイマ割り込み等で定期的にこの関数を呼び出すこと
   * @param interval 制御周期[us]。直前update実行からの経過時間を入力する
   */
  void update(unsigned long interval);

private:
  enum class CtrlObject // 制御対象
  {
    None = 0,
    SpdLimitedTrq, // 速度上限付きトルク制御モード
    Spd            // 速度制御モード
  };

  Tmotor &tmotor_;
  CtrlObject object_; // 制御対象

  // トルク制御で使う変数

  float trqRef_;      // トルク指令値[rad/s]
  float spdLimit_;    // トルク制御中の速度上限値[rad/s]。トルク制御中にこの速度を超えると、トルク指令値を減少させはじめる
  float spdMax_;      // トルク制御中の速度上限値2[rad/s]。この速度でトルク指令値はゼロになる

  // 速度制御で使う変数

  float spdRef_;      // 速度指令値[rad/s]
  float trqLimit_;    // 速度制御中のトルク最大値[Nm]。速度制御中のトルクはこの値より大きくならない
  float spdDevIntegral_; // 速度のPID制御で使う、速度偏差の積分値
  float calculatedTrq_;  // 制御器で計算されたトルク[Nm]

  void clear_(); // 制御器の内部変数や指令値を初期化する。上限値は初期化されない
};
