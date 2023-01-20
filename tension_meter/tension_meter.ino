#include <BLE2902.h>
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
// const long OFFSET = 27468;            // (張力計)ロードセルの、ゼログラムのときの読み値
// const float VAL_TO_KG = 1.7765e-5;    // (張力計)ロードセルの読み値からキログラムに変換する係数
const long OFFSET = -2028;          // (握力計)ロードセルの、ゼログラムのときの読み値
const float VAL_TO_KG = 4.78146e-5; // (握力計)ロードセルの読み値からキログラムに変換する係数

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
  pCharacteristic->addDescriptor(new BLE2902()); // BleWinrtDllでnotifyを受信するために必要
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
  long value = hx711.read();                                          // ロードセルの読み値を取得
  float killogram = static_cast<float>(VAL_TO_KG * (value - OFFSET)); // キログラム単位に換算
  // int valueInt = static_cast<int>(value);
  char jsonText[32];
  sprintf(jsonText, "{\"force\": %.6f}", killogram); // json形式で張力を文字列にprint
  pCharacteristic->setValue(reinterpret_cast<uint8_t *>(jsonText), strlen(jsonText));
  pCharacteristic->notify();
  Serial.println(jsonText);
}
