#include "MotorController.hpp"

#include "Tmotor.h"
#include <Arduino.h>

constexpr float DEFAULT_DTDV = 1.0;

MotorController::MotorController(Tmotor &tmotor)
: tmotor_(tmotor),
  object_(CtrlObject::None),
  kp(0.0),
  ki(0.0),
  trqLimit_(0.0),
  trqRef_(0.0),
  spdMaxWindin_(0.0),
  spdMaxLiftup_(0.0),
  dTdV_(DEFAULT_DTDV),
  spdLimitWindin_(0.0),
  spdLimitLiftup_(0.0),
  spdRef_(0.0),
  spdDevIntegral_(0.0),
  calculatedTrq_(0.0)
{
}

void MotorController::init(float spdKp, float spdKi)
{
  // モータのMとDが分かれば極配置法で設計できるが、分からないので直接指定することにした
  kp = spdKp; // 2 * M * POLE_OMEGA - D;
  ki = spdKi; // M * POLE_OMEGA * POLE_OMEGA;
  trqLimit_ = 0.0;
  trqRef_ = 0.0;
  spdMaxWindin_ = 0.0;
  spdMaxLiftup_ = 0.0;
  dTdV_ = DEFAULT_DTDV;
  spdLimitWindin_ = 0.0;
  spdLimitLiftup_ = 0.0;
  spdRef_ = 0.0;
  spdDevIntegral_ = 0.0;
  calculatedTrq_ = 0.0;
}

void MotorController::clear_()
{
  trqRef_ = 0.0;
  spdRef_ = 0.0;
  spdDevIntegral_ = 0.0;
  calculatedTrq_ = 0.0;
}

void MotorController::startTrqCtrl()
{
  if (object_ != CtrlObject::SpdLimitedTrq) { // 現在トルク制御モードでない場合のみ、変数をクリアしモードを変更
    clear_();
    object_ = CtrlObject::SpdLimitedTrq;
  }
}

void MotorController::setTrqRef(float trqRef)
{
  // トルク指令値はゼロ以上（巻取り方向）に限定
  if (trqRef >= 0.0) {
    trqRef_ = trqRef;
  } else {
    trqRef_ = 0.0;
  }
}

void MotorController::setSpdLimit(float spdMaxWindin, float spdMaxLiftup, float dTdV)
{
  // 0または負の値を渡されたら制御を無効化
  if (spdMaxWindin > 0.0) {
    spdMaxWindin_ = spdMaxWindin;
  } else {
    spdMaxWindin_ = 100.0;  // 制御が効かない大きな値
  }

  // 0または負の値を渡されたら制御を無効化
  if (spdMaxLiftup > 0.0) {
    spdMaxLiftup_ = spdMaxLiftup;
  } else {
    spdMaxLiftup_ = 100.0;  // 制御が効かない大きな値
  }
  
  dTdV_ = dTdV;
}

void MotorController::startSpdCtrl()
{
  if (object_ != CtrlObject::Spd) { // 現在速度制御モードでない場合のみ、変数をクリアしモードを変更
    clear_();
    object_ = CtrlObject::Spd;
  }
}

void MotorController::setSpdRef(float spdRef)
{
  spdRef_ = spdRef;
}

void MotorController::setTrqLimit(float trqLimit)
{
  trqLimit_ = trqLimit;
}

void MotorController::stopCtrl()
{
  object_ = CtrlObject::None;
  clear_();
}

void MotorController::update(unsigned long interval)
{
  if (object_ == CtrlObject::None) {
    calculatedTrq_ = 0.0;
    tmotor_.sendCommand(0, 0, 0, 0, calculatedTrq_);

  } else if (object_ == CtrlObject::SpdLimitedTrq) {
    // if Tref < 0, reagard 0
    if (trqRef_ < 0) {
      calculatedTrq_ = 0.0;

    } else {
      // avoid zero division
      if (dTdV_ == 0.0) {
        calculatedTrq_ = 0.0;

      } else {
        spdLimitWindin_ = spdMaxWindin_ + trqRef_ / dTdV_;
        spdLimitLiftup_ = spdMaxLiftup_ + (trqLimit_ - trqRef_) / dTdV_;

        if (tmotor_.spdReceived > spdLimitWindin_) {
          calculatedTrq_ = 0.0;
        } else if (tmotor_.spdReceived > spdMaxWindin_) {
          calculatedTrq_ = (spdLimitWindin_ - tmotor_.spdReceived) * dTdV_;
        } else if (tmotor_.spdReceived > -spdMaxLiftup_) {
          calculatedTrq_ = trqRef_;
        } else if (tmotor_.spdReceived > -spdLimitLiftup_) {
          calculatedTrq_ = (- spdMaxLiftup_ - tmotor_.spdReceived) * dTdV_ + trqRef_;
        } else {
          calculatedTrq_ = trqLimit_;
        }
      }
    }

    tmotor_.sendCommand(0, 0, 0, 0, calculatedTrq_);

  } else if (object_ == CtrlObject::Spd) {
    // 速度の目標値からの偏差を計算
    float spdDev = spdRef_ - tmotor_.spdReceived;
    // 速度偏差を積分 (trqLimit-0.1 Nm に達すると積分を停止する Anti-windup つき)
    spdDevIntegral_ += (abs(calculatedTrq_) < trqLimit_ - 0.1) ? spdDev * interval / 1e6 : 0;
    // PI制御によりトルクを計算
    calculatedTrq_ = kp * spdDev + ki * spdDevIntegral_;
    // トルク上下限設定
    if (calculatedTrq_ > trqLimit_) {
      calculatedTrq_ = trqLimit_;
    } else if (calculatedTrq_ < -trqLimit_) {
      calculatedTrq_ = -trqLimit_;
    }
     // トルク送信
    tmotor_.sendCommand(0, 0, 0, 0, calculatedTrq_);

  } else {
    calculatedTrq_ = 0.0;
    tmotor_.sendCommand(0, 0, 0, 0, calculatedTrq_);
  }
}
