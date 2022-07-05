/**
 * @file TouchSwitch.hpp
 * @author thgcMtdh
 * @brief 手元のタッチスイッチの状態を取得する
 * @version
 * @date 2022-06-25
 */

#pragma once

#include <Arduino.h>

class TouchSwitch
{
private:
  const uint8_t sensorPin_;
  const float voltageThreshold_;
  const uint8_t onLevel_;

public:
  /**
   * @brief タッチスイッチの生成と初期設定
   * @param sensorPin 圧力センサと抵抗の分圧出力を接続したピン番号
   * @param voltageThreshold ON/OFFの境界となる電圧閾値
   * @param onLevel 閾値より電圧が高いときにONとみなす場合HIGH、低いときONとみなす場合LOW
   */
  TouchSwitch(uint8_t sensorPin, float voltageThreshold, uint8_t onLevel = LOW);

  /**
   * @brief タッチスイッチのON/OFFを取得する
   * @return true when ON, false when OFF
   */
  bool getState();
};
