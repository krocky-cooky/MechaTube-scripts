/**
 * @file Mode.hpp
 * @author thgcMtdh
 * @brief 制御モードを表す定数をまとめておく
 * @date 2022-06-25
 */

#pragma once

enum class Mode
{
  SpdCtrl = 0, // 速度制御モード
  TrqCtrl  // トルク制御モード
  // ゆくゆくは「可変負荷モード」などが入ってくる
};
