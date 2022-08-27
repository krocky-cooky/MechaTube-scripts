#include "BluetoothSerial.h"

BluetoothSerial SerialBT;
constexpr size_t BT_MSG_BUFSIZE = 32; // Bluetooth受信メッセージのバッファ長

/*
 * 想定している受信メッセージの形式
 *  index       0  1  2  3  4  5  6         7   8
 *  content  0x02  1  6  5  0  1  ,         3  \n
 *  meaning  STX   Tension [gram]    checksum  LF
 *                 -> 16501g         1+6+5+0+1 end
 */

void setup()
{
  Serial.begin(115200);
  Serial.println("Serial begin");
  SerialBT.begin("Machine-ESP32"); // Bluetooth device name
  Serial.println("SerialBT begin");
}

void loop()
{
  if (SerialBT.available()) {
    if (SerialBT.read() == 0x02) { // 開始コード STX で始まる場合、有効なメッセージなので読み込む

      char msg[BT_MSG_BUFSIZE];          // 受信メッセージをコピーするバッファ
      memset(msg, 0x00, BT_MSG_BUFSIZE); // ゼロ埋め
      SerialBT.readBytesUntil('\n', msg, BT_MSG_BUFSIZE);

      int val = 0;
      int checksum = 0;
      sscanf(msg, "%d,%d", &val, &checksum); // 受信メッセージから張力とチェックサムを読み取る

      int mychecksum = calc_checksum(val); // valからチェックサムを計算
      if (mychecksum == checksum) {        // チェックサム照合
        Serial.printf("%d,%d\n", val, checksum);
      } else {
        Serial.printf("checksum failure!\n");
      }

    } else { // 開始コード以外を検出した場合、捨てる
      while (SerialBT.available()) {
        SerialBT.read();
      }
    }
  }
}

/**
 * @brief 数値を10進数でprintしたときの各桁の和を計算し、下1桁を返す
 * @param val 数値 (例: 16501）
 * @return 各桁の和（例: 3）
 */
int calc_checksum(int val)
{
  constexpr size_t INT_MAX_DIGIT = 12; // intは最長11文字(-2147483648) + null文字
  char val_str[INT_MAX_DIGIT];
  memset(val_str, 0x00, INT_MAX_DIGIT); // ゼロ埋めする
  sprintf(val_str, "%d", val);          // valを10進数の文字列としてval_strにprint
  int sum = 0;
  for (size_t i = 0; i < INT_MAX_DIGIT; i++) {
    if (val_str[i] < '0' || val_str[i] > '9') { // '0'-'9'以外の文字の場合、'0'に置き換える
      val_str[i] = '0';
    }
    sum += static_cast<int>(val_str[i] - '0'); // 和を計算
  }
  return (sum - (sum / 10) * 10); // sum の1桁目を返す
}
