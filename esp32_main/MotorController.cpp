#include "MotorController.hpp"

#include "BTTensionMeter.hpp"
#include "Tmotor.h"
#include <Arduino.h>

MotorController::MotorController(Tmotor &tmotor)
  : kp(0.0), ki(0.0), tmotor_(tmotor), object_(CtrlObject::None), trqRef_(0.0), spdLimit_(0.0), spdMax_(0.0), spdRef_(0.0), trqLimit_(0.0), spdDevIntegral_(0.0), calculatedTrq_(0.0)
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
  if (object_ != CtrlObject::SpdLimitedTrq) { // 現在トルク制御モードでない場合のみ、変数をクリアしモードを変更
    clear_();
    object_ = CtrlObject::SpdLimitedTrq;
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

  } else if (object_ == CtrlObject::SpdLimitedTrq) {
    if ((tmotor_.trqSent >= 0.0 && tmotor_.spdReceived > spdMax_) ||
        (tmotor_.trqSent <= 0.0 && tmotor_.spdReceived < -spdMax_)) { // 正転トルク指令時にspdMaxを上回る & 逆転トルク指令時に-spdMaxを下回る
      calculatedTrq_ = 0.0;
    } else if ((tmotor_.trqSent >= 0.0 && tmotor_.spdReceived > spdLimit_) ||
               (tmotor_.trqSent <= 0.0 && tmotor_.spdReceived < -spdLimit_)) { // 正転トルク指令時spdLimitを上回る & 逆転トルク指令時に-spdLimitを下回る
      calculatedTrq_ = (spdMax_ - abs(tmotor_.spdReceived)) / (spdMax_ - spdLimit_) * trqRef_;
    } else {
      calculatedTrq_ = trqRef_;
    }

  } else if (object_ == CtrlObject::Spd) {
    float spdDev = spdRef_ - tmotor_.spdReceived;                                       // 速度の目標値からの偏差
    spdDevIntegral_ += (abs(calculatedTrq_) < trqLimit_) ? spdDev * interval / 1e6 : 0; // 速度偏差を積分(Anti-windupつき)
    calculatedTrq_ = kp * spdDev + ki * spdDevIntegral_;

  } else {
    calculatedTrq_ = 0.0;
  }

  tmotor_.sendCommand(0, 0, 0, 0, calculatedTrq_);
}
