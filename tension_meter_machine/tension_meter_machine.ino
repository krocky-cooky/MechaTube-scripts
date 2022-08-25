#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

void setup() {
  Serial.begin(115200);
  Serial.println("Serial begin");
  SerialBT.begin("Machine-ESP32"); //Bluetooth device name
  Serial.println("SerialBT begin");
}

void loop() {
  while (Serial.available()) {
    SerialBT.write(Serial.read());
  }
  while (SerialBT.available()) {
    SerialBT.write(SerialBT.read());
  }
}