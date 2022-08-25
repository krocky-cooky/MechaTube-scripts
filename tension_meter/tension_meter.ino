//ファイル>スケッチ例>BluetoothSerial>SerialToSerialBTM

#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

String name = "Machine-ESP32";

const int ledPin = 25;
const int ledcChannel = 0;

void setup() {
  Serial.begin(115200);
  ledcSetup(ledcChannel, 1.0, 16);
  ledcAttachPin(ledPin, ledcChannel);
  ledcWrite(ledcChannel, 32768);
  Serial.println("SerialBT begin");
  SerialBT.begin("TensionMeter-ESP32", true); 
  Serial.println("The device started in master mode, make sure remote BT device is on!");
  bool connected = SerialBT.connect(name);
  
  if(connected) {
    Serial.println("Connected Succesfully!");
  } else {
    while(!SerialBT.connected(1000)) {
      Serial.println("Failed to connect. Make sure remote device is available and in range, then restart app."); 
    }
  }
  ledcWrite(ledcChannel, 0);
  ledcDetachPin(ledPin);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);
  
  static const char* buf = "98765\n";
  unsigned long start = millis();
  for (int i=0; i<1000; i++) {  // 1000回同じメッセージを送って、1往復にかかる時間を取得
    SerialBT.write((uint8_t*)buf, 6);
    while (!SerialBT.available()) {}  // wait until reply
    while (SerialBT.available()) {// read all reply
      SerialBT.read();
    }
  }
  unsigned long end = millis();
  Serial.println(end - start);
}

void loop() {
  delay(1);
}