#include "BTTensionMeter.hpp"

static const unsigned long CONN_TIMEOUT_ = 50; // 直近のメッセージ到着からこの時間[ms]だけ経過しても次のメッセージが来ないとき接続断とみなす
static BLEClient *pClient_;
static BLERemoteService *pRemoteService_;
static BLERemoteCharacteristic *pRemoteCharacteristic_;
static std::string deviceName_;                  // 張力計のBLEデバイス名
static BLEAddress address_("00:00:00:00:00:00"); // 張力計のアドレス。Scanで張力計が見つかったときに代入される
static bool found_ = false;                      // Scanで張力計が見つかったらtrueになるフラグ
static bool available_ = false;                  // 最新値到着フラグ。受信した値がまだgetTension()により読まれていないときtrueになる
static int tension_ = 0;                         // 受信した張力の最新値[mg]
static unsigned long lastReceivedMillis_ = 0;    // 最後に張力を受信した時刻[ms]

class FoundCallbacks : public BLEAdvertisedDeviceCallbacks // 張力計を発見した際に呼び出されるコールバック関数
{
  void onResult(BLEAdvertisedDevice advertisedDevice)
  { // Scanでデバイスを発見したときに呼び出される関数
    if (advertisedDevice.haveName()) {
      if (advertisedDevice.getName().compare(deviceName_) == 0) { // 発見したデバイス名が張力計と一致していたら
        address_ = advertisedDevice.getAddress();
        found_ = true;
        advertisedDevice.getScan()->stop();
      }
    }
  }
};

static void notifyCallback(BLERemoteCharacteristic *pRemoteCharacteristic, uint8_t *data, size_t length, bool isNotify)
{
  tension_ = *((int *)data);      // 受信したデータから張力を取得
  lastReceivedMillis_ = millis(); // 受信時刻を記録
  available_ = true;
}

bool tensionMeterBegin(const char *deviceName, BLEUUID serviceUUID, BLEUUID characteristicUUID, int scanTimeout)
{
  deviceName_ = std::string(deviceName);

  Serial.println("[tensionMeterBegin] Start scanning for BLE devices for 10sec...");
  BLEScan *pBleScan = BLEDevice::getScan();
  pBleScan->setAdvertisedDeviceCallbacks(new FoundCallbacks());
  pBleScan->setActiveScan(true); // デバイス名を取得するためにはactiveScanが必要
  BLEScanResults bleScanResults = pBleScan->start(scanTimeout);

  if (!found_) { // 張力計を発見できなければ再起動し、先に進まない
    Serial.println("[tensionMeterBegin] Tension meter was not found.");
    return false;
  }

  Serial.printf("[tensionMeterBegin] Tension meter found. Address: %s\n", address_.toString().c_str());
  pClient_ = BLEDevice::createClient(); // 張力計と通信するためのクライアントを生成
  if (!pClient_->connect(address_)) {
    return false;
  }
  pRemoteService_ = pClient_->getService(serviceUUID);
  pRemoteCharacteristic_ = pRemoteService_->getCharacteristic(characteristicUUID);
  pRemoteCharacteristic_->registerForNotify(notifyCallback);
  Serial.println("[tensionMeterBegin] Successfully created a client to communicate with tension meter.");
  return true;
}

bool tensionMeterConnected()
{
  if (millis() - lastReceivedMillis_ <= CONN_TIMEOUT_) {
    return true;
  }
  return false;
}

bool tensionAvailable()
{
  return available_;
}

int getTension()
{
  available_ = false; // 値が読まれたら、最新値到着フラグをクリア
  return tension_;    // mg単位なので、gに換算
}
