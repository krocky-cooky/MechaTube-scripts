//ファイル>スケッチ例>BluetoothSerial>SerialToSerialBTM

#include "BluetoothSerial.h"
#include "HX711.h"

BluetoothSerial SerialBT;
HX711 hx711;

String name = "Machine-ESP32";

const int PIN_LED = 25;
const int CHANNEL_LED = 0;
const int PIN_DAT = 32;
const int PIN_CLK = 33;

void setup() {
  Serial.begin(115200);
  ledcSetup(CHANNEL_LED, 1.0, 16);
  ledcAttachPin(PIN_LED, CHANNEL_LED);
  ledcWrite(CHANNEL_LED, 32768);
  hx711.begin(PIN_DAT, PIN_CLK, 128);
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
  ledcWrite(CHANNEL_LED, 0);
  ledcDetachPin(PIN_LED);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);
}

void loop() {
  long value = hx711.read();
  Serial.println(value);
  delay(1);
}