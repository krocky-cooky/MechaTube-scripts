/**
 * @file Mode.hpp
 * @author thgcMtdh
 * @brief 制御モードを表す定数をまとめておく
 * @date 2022-06-25
 */

#pragma once

enum class Target {
  None = 0,
  SpdCtrl, // 速度制御モード
  TrqCtrl  // トルク制御モード
};

struct Command {
  Target target;
  float trq;
  float spd;
  float trqLimit;
  float spdLimit;
  Command() : target(Target::None), trq(0.0), spd(0.0), trqLimit(0.0), spdLimit(0.0) {}
};

struct Status {
  Target target;
  float trq;
  float spd;
  float pos;
  float integratingAngle;
  Status() : target(Target::None), trq(0.0), spd(0.0), pos(0.0), integratingAngle(0.0) {}
};
