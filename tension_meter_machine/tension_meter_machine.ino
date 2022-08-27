#include "BTTensionMeter.hpp"
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;
BTTensionMeter tensionMeter(SerialBT);

void setup()
{
  Serial.begin(115200);
  Serial.println("Serial begin");
  SerialBT.begin("Machine-ESP32"); // Bluetooth device name
  Serial.println("SerialBT begin");
  tensionMeter.begin();
}

void loop()
{
  tensionMeter.loop();
  if (tensionMeter.available()) {
    Serial.println(tensionMeter.getTension());
  }
}
