#include "TouchSwitch.hpp"

TouchSwitch::TouchSwitch(uint8_t sensorPin, float voltageThreshold, uint8_t onLevel)
  : sensorPin_(sensorPin), voltageThreshold_(voltageThreshold), onLevel_(onLevel)
{
}

bool TouchSwitch::getState()
{
  float voltage = analogRead(sensorPin_) / 4096.0 * 3.3; // 手元スイッチのセンサの電圧
  if (onLevel_ == HIGH) {
    return (voltage >= voltageThreshold_);
  } else {
    return (voltage <= voltageThreshold_);
  }
}
