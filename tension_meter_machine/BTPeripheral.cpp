#include "BTPeripheral.hpp"

static const unsigned long CONN_TIMEOUT_ = 200; // 直近のメッセージ到着からこの時間[ms]だけ経過しても次のメッセージが来ないとき接続断とみなす
static BLEServer *pServer = NULL;
static BLEService *pService = NULL;
static BLECharacteristic *pLogCharacteristic = NULL;     // ログの送信を行うCharacteristic (property: Notify)
static BLECharacteristic *pCommandCharacteristic = NULL; // 指令の受信を行うCharactersitic (Property: Write)
static bool connected_ = false;                          // 接続されたらtrueになるフラグ
static bool available_ = false;                          // 最新値到着フラグ。受信した指令がまだgetCommand()により読まれていないときtrueになる
static std::string command_;                             // 受信した指令を格納するバッファ
static unsigned long lastReceivedMillis_ = 0;            // 最後に張力を受信した時刻[ms]

// PCが接続された時に実行される処理。
// ただ、張力計が接続/切断された時にもこれが実行されてしまうっぽい
class FuncServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    Serial.println("[onConnect] Client connected!");
    connected_ = true;
  };
  void onDisconnect(BLEServer *pServer)
  {
    Serial.println("[onDisconnect] Client disconnected!");
    connected_ = false;
  }
};

// 指令を受信した時の処理
class CommandCharacteristicCallbacks : public BLECharacteristicCallbacks
{
  // 受信した指令をcommand_に転記する
  void onWrite(BLECharacteristic *pCharacteristic)
  {
    command_ = pCharacteristic->getValue();
    available_ = true;
  }
};

void peripheralBegin(const char *deviceName, BLEUUID serviceUUID, BLEUUID logCharacteristicUUID, BLEUUID commandCharacteristicUUID)
{
  pServer = BLEDevice::createServer();
  delay(500); // 少し待たないと、張力計の接続によりonConnectが実行されてしまう
  pServer->setCallbacks(new FuncServerCallbacks());
  pService = pServer->createService(serviceUUID);
  pLogCharacteristic = pService->createCharacteristic(logCharacteristicUUID, BLECharacteristic::PROPERTY_NOTIFY);
  pCommandCharacteristic = pService->createCharacteristic(commandCharacteristicUUID, BLECharacteristic::PROPERTY_WRITE);
  pCommandCharacteristic->setCallbacks(new CommandCharacteristicCallbacks());
  pService->start();
  Serial.println("[peripheralBegin] Service started.");
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(serviceUUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06); // iPhone接続の問題に役立つ
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.println("[peripheralBegin] Advertising started. Waiting for pc connection...");
  while (!connected_) {
    delay(1000);
  }
}

bool pcConnected()
{
  if (connected_ && millis() - lastReceivedMillis_ <= CONN_TIMEOUT_) {
    return true;
  }
  return false;
}

void sendText(const char *str)
{
  pLogCharacteristic->setValue((uint8_t *)str, strlen(str));
  pLogCharacteristic->notify();
}

void sendText(const String str)
{
  pLogCharacteristic->setValue((uint8_t *)(str.c_str()), str.length());
  pLogCharacteristic->notify();
}

bool commandAvailable()
{
  return available_;
}

String getCommand()
{
  available_ = false;              // 指令が読まれたら、最新値到着フラグをクリア
  return String(command_.c_str()); // 文字列を新たなオブジェクトとしてコピーして返す
}
