#include "MotorController.hpp"

#include "Tmotor.h"
#include <Arduino.h>

MotorController::MotorController(Tmotor &tmotor)
  : kp(0.0), ki(0.0), tmotor_(tmotor), target_(Target::None), trqRef_(0.0), spdLimit_(0.0), spdMax_(0.0), spdRef_(0.0), trqLimit_(0.0), spdDevIntegral_(0.0), calculatedTrq_(0.0)
{
}

void MotorController::init(float spdKp, float spdKi)
{
  // モータのMとDが分かれば極配置法で設計できるが、分からないので直接指定することにした
  kp = spdKp; // 2 * M * POLE_OMEGA - D;
  ki = spdKi; // M * POLE_OMEGA * POLE_OMEGA;
  trqRef_ = 0.0;
  spdLimit_ = 0.0;
  spdMax_ = 0.0;
  spdRef_ = 0.0;
  trqLimit_ = 0.0;
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
  if (target_ != Target::TrqCtrl) { // 現在トルク制御モードでない場合のみ、変数をクリアしモードを変更
    clear_();
    target_ = Target::TrqCtrl;
  }
}

void MotorController::setTrqRef(float trqRef)
{
  trqRef_ = trqRef;
}

void MotorController::setSpdLimit(float spdLimit, float spdMax)
{
  spdLimit_ = spdLimit;
  spdMax_ = spdMax;
}

void MotorController::startSpdCtrl()
{
  if (target_ != Target::SpdCtrl) { // 現在速度制御モードでない場合のみ、変数をクリアしモードを変更
    clear_();
    target_ = Target::SpdCtrl;
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
  target_ = Target::None;
  clear_();
}

void MotorController::update(unsigned long interval)
{
  if (target_ == Target::None) {
    calculatedTrq_ = 0.0;

  } else if (target_ == Target::TrqCtrl) {
    if ((tmotor_.trqSent >= 0.0 && tmotor_.spdReceived > spdMax_) ||
        (tmotor_.trqSent <= 0.0 && tmotor_.spdReceived < -spdMax_)) { // 正転トルク指令時にspdMaxを上回る & 逆転トルク指令時に-spdMaxを下回る
      calculatedTrq_ = 0.0;
    } else if ((tmotor_.trqSent >= 0.0 && tmotor_.spdReceived > spdLimit_) ||
               (tmotor_.trqSent <= 0.0 && tmotor_.spdReceived < -spdLimit_)) { // 正転トルク指令時spdLimitを上回る & 逆転トルク指令時に-spdLimitを下回る
      calculatedTrq_ = (spdMax_ - abs(tmotor_.spdReceived)) / (spdMax_ - spdLimit_) * trqRef_;
    } else {
      calculatedTrq_ = trqRef_;
    }

  } else if (target_ == Target::SpdCtrl) {
    float spdDev = spdRef_ - tmotor_.spdReceived;                                       // 速度の目標値からの偏差
    spdDevIntegral_ += (abs(calculatedTrq_) < trqLimit_) ? spdDev * interval / 1e6 : 0; // 速度偏差を積分(Anti-windupつき)
    calculatedTrq_ = kp * spdDev + ki * spdDevIntegral_;

  } else {
    calculatedTrq_ = 0.0;
  }

  tmotor_.sendCommand(0, 0, 0, 0, calculatedTrq_);
}
