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
const float VAL_TO_GRAM = 0.01;  // ロードセルの読み値からgに変換する係数

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
    Serial.println("Failed to connect. Make sure remote device is available and in range, then restart app."); 
    if (!SerialBT.connected(10000)) {
      Serial.println("Connection failure continued. Rebooting...");
      ESP.restart();
    }
  }
  ledcWrite(CHANNEL_LED, 0);
  ledcDetachPin(PIN_LED);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);
}

void loop() {
  long value = hx711.read();  // ロードセルの読み値を取得
  int gram = static_cast<int>(VAL_TO_GRAM * value);  // グラム単位に換算

  int checksum = calc_checksum(gram);  // チェックサム計算

  if (SerialBT.connected(1000)) {
    SerialBT.write(0x02);  // STX (Start of TX)
    SerialBT.printf("%05d,%d\n", gram, checksum);  // Bluetoothに送信(gramは見やすさのため0を入れて5桁に揃えている)
  } else {
    Serial.println("Connection lost! Rebooting...");
    ESP.restart();
  }
}

/**
 * @brief 数値を10進数でprintしたときの各桁の和を計算し、下1桁を返す
 * @param val 数値 (例: 16501）
 * @return 各桁の和（例: 3）
 */
int calc_checksum(int val) {
  constexpr size_t INT_MAX_DIGIT = 12;  // intは最長11文字(-2147483648) + null文字
  char val_str[INT_MAX_DIGIT];
  memset(val_str, 0x00, INT_MAX_DIGIT);  // ゼロ埋めする
  sprintf(val_str, "%d", val);  // valを10進数の文字列としてval_strにprint
  int sum = 0;
  for (size_t i = 0; i < INT_MAX_DIGIT; i++) {
    if (val_str[i] < '0' || val_str[i] > '9') {  // '0'-'9'以外の文字の場合、'0'に置き換える
      val_str[i] = '0';
    }
    sum += static_cast<int>(val_str[i] - '0');  // 和を計算
  }
  return (sum - (sum / 10) * 10);  // sum の1桁目を返す
}