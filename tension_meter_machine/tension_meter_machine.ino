#include <BLEDevice.h>
#include <string>

const char tensionMeterDevicename[] = "TensionMeter"; // 張力計に書き込んだ device name
const BLEUUID SERVICE_UUID((uint16_t)0x181D);        // Weight Scale (定義済UUID)
const BLEUUID CHARACTERISTIC_UUID((uint16_t)0x2A98); // Weight (定義済UUID)

bool tensionMeterFound = false;
BLEAddress address("00:00:00:00:00:00");
BLEClient* pBleClient = NULL;
BLERemoteService* pBleRemoteService = NULL;
BLERemoteCharacteristic* pBleRemoteCharacteristic = NULL;
class FoundCallbacks : public BLEAdvertisedDeviceCallbacks
{
  void onResult(BLEAdvertisedDevice advertisedDevice)
  {
    if (advertisedDevice.haveName()) {
      if (advertisedDevice.getName().compare(tensionMeterDevicename) == 0) { // 発見したデバイス名が張力計と一致していたら
        address = advertisedDevice.getAddress();
        tensionMeterFound = true;
        advertisedDevice.getScan()->stop();
      }
    }
  }
};

void tensionMeterNotifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* data, size_t length, bool isNotify) {
  Serial.print(length);
  Serial.printf("%d, %d\n", length, *reinterpret_cast<int*>(data));
}

void setup()
{
  Serial.begin(115200);
  Serial.println("Serial began");
  delay(5000);

  Serial.println("Initalizing BLE...");
  BLEDevice::init("Machine-ESP32");

  Serial.println("Start scanning for BLE devices for 10sec...");
  BLEScan *pBleScan = BLEDevice::getScan();
  pBleScan->setAdvertisedDeviceCallbacks(new FoundCallbacks());
  pBleScan->setActiveScan(true); // デバイス名を取得するためにはactiveScanが必要
  BLEScanResults bleScanResults = pBleScan->start(10);
  
  Serial.printf("Tension meter found. Address: %s\n", address.toString().c_str());
  pBleClient = BLEDevice::createClient();  // 張力計と通信するためのクライアントを生成
  pBleClient->connect(address);
  pBleRemoteService = pBleClient->getService(SERVICE_UUID);
  pBleRemoteCharacteristic = pBleRemoteService->getCharacteristic(CHARACTERISTIC_UUID);
  pBleRemoteCharacteristic->registerForNotify(tensionMeterNotifyCallback);
  Serial.println("Successfully got charactersitic");
}

void loop()
{
  Serial.println("loop");
  delay(1000);
}
