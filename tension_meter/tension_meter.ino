#include <BLEDevice.h>
#include <HX711.h>

BLECharacteristic *pCharacteristic;
HX711 hx711;

#define DEVICE_NAME "TensionMeter"
#define SERVICE_UUID (uint16_t)0x181D        // Weight Scale (定義済UUID)
#define CHARACTERISTIC_UUID (uint16_t)0x2A98 // Weight (定義済UUID)

const int PIN_LED = 25;
const int CHANNEL_LED = 0;
const int PIN_DAT = 32;
const int PIN_CLK = 33;
const long OFFSET = 27468;        // ロードセルの、ゼログラムのときの読み値
const float VAL_TO_GRAM = 17.765; // ロードセルの読み値からミリグラムに変換する係数

class FuncServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    Serial.println("Client connected!");
    ledcWrite(CHANNEL_LED, 0); // 接続完了したらLEDを点灯させる
    ledcDetachPin(PIN_LED);
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, HIGH);
  };
  void onDisconnect(BLEServer *pServer)
  {
    Serial.println("Client disconnected!");
    ESP.restart(); // 切断されたら再起動
  }
};

void setup()
{
  Serial.begin(115200);

  hx711.begin(PIN_DAT, PIN_CLK, 128);
  Serial.println("HX711 began");

  BLEDevice::init(DEVICE_NAME);
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new FuncServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06); // iPhone接続の問題に役立つ
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

  ledcSetup(CHANNEL_LED, 1.0, 16); // LEDをPWMを使って1sec周期で点滅させる
  ledcAttachPin(PIN_LED, CHANNEL_LED);
  ledcWrite(CHANNEL_LED, 32768);
  Serial.println("BLE advertise started");
}

void loop()
{
  long value = hx711.read();                                        // ロードセルの読み値を取得
  int milligram = static_cast<int>(VAL_TO_GRAM * (value - OFFSET)); // ミリグラム単位に換算
  pCharacteristic->setValue(milligram);                             // Charactersiticに値をセットし送信
  pCharacteristic->notify();
  Serial.println(milligram);
}
