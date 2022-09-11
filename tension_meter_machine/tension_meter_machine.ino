#include <BLEDevice.h>
#include <string>

#include "BTTensionMeter.hpp"

const char TENSIONMETER_DEVICE_NAME[] = "TensionMeter"; // 張力計に書き込んだ device name
const BLEUUID TENSIONMETER_SERVICE_UUID((uint16_t)0x181D);        // Weight Scale (定義済UUID)
const BLEUUID TENSIONMETER_CHARACTERISTIC_UUID((uint16_t)0x2A98); // Weight (定義済UUID)

const char MACHINE_DEVICE_NAME[] = "ESP32-Machine";  // セントラルに表示されるマシンのデバイス名
const BLEUUID MACHINE_SERVICE_UUID("91bad492-b950-4226-aa2b-4ede9fa42f59");  // マシンがサーバとして振舞う際のService UUID
const BLEUUID MACHINE_LOG_CHARACTERISTIC_UUID("f78ebbff-c8b7-4107-93de-889a6a06d408");  // マシンがサーバとして振舞う際の、ログ送信Characteristic UUID
const BLEUUID MACHINE_COMMAND_CHARACTERISTIC_UUID("f78ebbff-c8b7-4107-93de-889a6a06d409");  // マシンがサーバとして振舞う際の、指令受信Characteristic UUID

BLEServer* pServer = NULL;
BLEService* pService = NULL;
BLECharacteristic* pCharacteristic = NULL;

void sendText(String str)
{
  pCharacteristic->setValue((uint8_t*)(str.c_str()), str.length());
  pCharacteristic->notify();
}
void sendText(const char* str)
{
  pCharacteristic->setValue((uint8_t*)str, strlen(str));
  pCharacteristic->notify();
}

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

  Serial.println("BLE peripheral starting...");
  pServer = BLEDevice::createServer();
  Serial.println("setup 1");
  // pServer->setCallbacks(new FuncServerCallbacks());
  pService = pServer->createService(MACHINE_SERVICE_UUID);
  Serial.println("setup 2");
  pCharacteristic = pService->createCharacteristic(MACHINE_LOG_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  Serial.println("setup 3");
  pService->start();
  Serial.println("setup 4");
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  Serial.println("setup 5");
  pAdvertising->addServiceUUID(MACHINE_SERVICE_UUID);
  Serial.println("setup 6");
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06); // iPhone接続の問題に役立つ
  pAdvertising->setMinPreferred(0x12);
  Serial.println("setup 7");
  BLEDevice::startAdvertising();
  Serial.println("setup 8");
}

void loop()
{
  Serial.println("loop");
  char buf[64];
  sprintf(buf, "{\"tension\": %d}", getTension());
  sendText(buf);
  delay(1000);
}
