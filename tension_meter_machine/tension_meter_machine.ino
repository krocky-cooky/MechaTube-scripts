#include <BLEDevice.h>
#include <string>

const char TENSIONMETER_DEVICE_NAME[] = "TensionMeter"; // 張力計に書き込んだ device name
const BLEUUID TENSIONMETER_SERVICE_UUID((uint16_t)0x181D);        // Weight Scale (定義済UUID)
const BLEUUID TENSIONMETER_CHARACTERISTIC_UUID((uint16_t)0x2A98); // Weight (定義済UUID)

const char MACHINE_DEVICE_NAME[] = "ESP32-Machine";  // セントラルに表示されるマシンのデバイス名
const BLEUUID MACHINE_SERVICE_UUID("91bad492-b950-4226-aa2b-4ede9fa42f59");  // マシンがサーバとして振舞う際のService UUID
const BLEUUID MACHINE_CHARACTERISTIC_UUID("f78ebbff-c8b7-4107-93de-889a6a06d408");  // マシンがサーバとして振舞う際のCharacteristic UUID

bool tensionMeterFound = false; // Scanで張力計を発見したらtrueとなるフラグ
BLEAddress tensionMeterAddress("00:00:00:00:00:00"); // Scanで張力計を発見したら張力計のアドレスが歳入される
BLEClient* pBleClient = NULL;
BLERemoteService* pBleRemoteService = NULL;
BLERemoteCharacteristic* pBleRemoteCharacteristic = NULL;

BLEServer* pServer = NULL;
BLEService* pService = NULL;
BLECharacteristic* pCharacteristic = NULL;
class FoundCallbacks : public BLEAdvertisedDeviceCallbacks
{
  void onResult(BLEAdvertisedDevice advertisedDevice)
  {
    if (advertisedDevice.haveName()) {
      if (advertisedDevice.getName().compare(TENSIONMETER_DEVICE_NAME) == 0) { // 発見したデバイス名が張力計と一致していたら
        tensionMeterAddress = advertisedDevice.getAddress();
        tensionMeterFound = true;
        advertisedDevice.getScan()->stop();
      }
    }
  }
};

void tensionMeterNotifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* data, size_t length, bool isNotify) {
  Serial.println(*reinterpret_cast<int*>(data));
}

void setup()
{
  Serial.begin(115200);
  Serial.println("Serial began");
  delay(5000);

  Serial.println("Initalizing BLE...");
  BLEDevice::init(MACHINE_DEVICE_NAME);

  Serial.println("Start scanning for BLE devices for 10sec...");
  BLEScan *pBleScan = BLEDevice::getScan();
  pBleScan->setAdvertisedDeviceCallbacks(new FoundCallbacks());
  pBleScan->setActiveScan(true); // デバイス名を取得するためにはactiveScanが必要
  BLEScanResults bleScanResults = pBleScan->start(10);

  if (!tensionMeterFound) { // 張力計を発見できなければ再起動し、先に進まない
    Serial.println("Tension meter is not found. Rebooting...");
    ESP.restart();
  }
  
  Serial.printf("Tension meter found. Address: %s\n", tensionMeterAddress.toString().c_str());
  pBleClient = BLEDevice::createClient();  // 張力計と通信するためのクライアントを生成
  pBleClient->connect(tensionMeterAddress);
  pBleRemoteService = pBleClient->getService(TENSIONMETER_SERVICE_UUID);
  pBleRemoteCharacteristic = pBleRemoteService->getCharacteristic(TENSIONMETER_CHARACTERISTIC_UUID);
  pBleRemoteCharacteristic->registerForNotify(tensionMeterNotifyCallback);
  Serial.println("Successfully created a client to communicate with tension meter.");

  Serial.println("BLE peripheral starting...");
  pServer = BLEDevice::createServer();
  Serial.println("setup 1");
  // pServer->setCallbacks(new FuncServerCallbacks());
  pService = pServer->createService(MACHINE_SERVICE_UUID);
  Serial.println("setup 2");
  pCharacteristic = pService->createCharacteristic(MACHINE_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_NOTIFY);
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
  delay(1000);
}
