#include "MotorController.hpp"

#include "Tmotor.h"
#include <Arduino.h>

MotorController::MotorController(Tmotor &tmotor)
  : kp(0.0), ki(0.0), tmotor_(tmotor), object_(CtrlObject::None), trqRef_(0.0), spdLimit_(0.0), spdMax_(0.0), spdRef_(0.0),trqLimit_(0.0), spdDevIntegral_(0.0), calculatedTrq_(0.0)
{
}

void MotorController::init()
{
  kp = 2 * M * POLE_OMEGA - D;
  ki = M * POLE_OMEGA * POLE_OMEGA;
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
  spdRef_= 0.0;
  spdDevIntegral_ = 0.0;
  calculatedTrq_ = 0.0;
}

void MotorController::startTrqCtrl()
{
  clear_();
  object_ = CtrlObject::SpdLimitedTrq;
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
  clear_();
  object_ = CtrlObject::Spd;
}

void MotorController::setSpdRef(float spdRef)
{
  spdRef_ = spdRef;
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
    if (tmotor_.spdReceived < spdLimit_) {
      calculatedTrq_ = trqRef_;
    } else if (tmotor_.spdReceived < spdMax_) {
      calculatedTrq_ = (spdMax_ - tmotor_.spdReceived) / (spdMax_ - spdLimit_) * trqRef_;
    } else {
      calculatedTrq_ = 0.0;
    }

  } else if (object_ == CtrlObject::Spd) {
    float spdDev = spdRef_ - tmotor_.spdReceived;                 // 速度の目標値からの偏差
    spdDevIntegral_ += (calculatedTrq_ < trqLimit_) ? spdDev : 0; // 速度偏差を積分(Anti-windupつき)
    calculatedTrq_ = kp * spdDev + ki * spdDevIntegral_;
  }
  
  tmotor_.sendCommand(0, 0, 0, 0, calculatedTrq_);
}
