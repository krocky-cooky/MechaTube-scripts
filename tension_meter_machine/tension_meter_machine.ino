#include <BLEDevice.h>
#include <string>

// #include "BTPeripheral.hpp"
#include "BTTensionMeter.hpp"

const char TENSIONMETER_DEVICE_NAME[] = "TensionMeter";           // 張力計に書き込んだ device name
const BLEUUID TENSIONMETER_SERVICE_UUID((uint16_t)0x181D);        // Weight Scale (定義済UUID)
const BLEUUID TENSIONMETER_CHARACTERISTIC_UUID((uint16_t)0x2A98); // Weight (定義済UUID)

const char MACHINE_DEVICE_NAME[] = "ESP32-Machine"; // マシンのデバイス名
// const BLEUUID MACHINE_SERVICE_UUID("91bad492-b950-4226-aa2b-4ede9fa42f59");  // マシンがサーバとして振舞う際のService UUID
// const BLEUUID MACHINE_LOG_CHARACTERISTIC_UUID("f78ebbff-c8b7-4107-93de-889a6a06d408");  // マシンがサーバとして振舞う際の、ログ送信Characteristic UUID
// const BLEUUID MACHINE_COMMAND_CHARACTERISTIC_UUID("f78ebbff-c8b7-4107-93de-889a6a06d409");  // マシンがサーバとして振舞う際の、指令受信Characteristic UUID

void setup()
{
  Serial.begin(115200);
  Serial.println("Serial began");
  delay(5000);

  Serial.println("Initalizing BLE...");
  BLEDevice::init(MACHINE_DEVICE_NAME);

  bool connected = tensionMeterBegin(TENSIONMETER_DEVICE_NAME, TENSIONMETER_SERVICE_UUID, TENSIONMETER_CHARACTERISTIC_UUID, 10);
  if (!connected) {
    Serial.println("Tension meter is not found. Rebooting...");
    ESP.restart();
  }

  // Serial.println("BLE peripheral starting...");
  // peripheralBegin(MACHINE_DEVICE_NAME, MACHINE_SERVICE_UUID, MACHINE_LOG_CHARACTERISTIC_UUID, MACHINE_COMMAND_CHARACTERISTIC_UUID);
}

void loop()
{
  // 張力計から getTension() で張力を受信し、sendText() でPC側に送信
  if (tensionAvailable()) {
    char buf[64];
    sprintf(buf, "{\"tension\": %.6f}", getTension());
    Serial.println(buf); // sendText(buf); 有線化したので張力はSerialにプリント
  }
  // PCから指令を getCommand() で取得（PCとの通信を有線化したので不要）
  // if (commandAvailable()) {
  //   Serial.println(getCommand());
  // }
}
