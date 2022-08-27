#include "BTTensionMeter.hpp"

const size_t BTTensionMeter::BT_MSG_BUFSIZE_ = 32; // Bluetooth受信メッセージをコピーするバッファのサイズ
const unsigned long BTTensionMeter::TIMEOUT_ = 50; // 直近のメッセージ到着からこの時間[ms]だけ経過しても次のメッセージが来ないとき接続断とみなす

BTTensionMeter::BTTensionMeter(BluetoothSerial &serialBT)
  : SerialBT_(serialBT), lastReceivedMillis_(0), available_(false), tension_(0)
{
}

void BTTensionMeter::begin()
{
}

void BTTensionMeter::loop()
{
  if (SerialBT_.available()) {
    if (SerialBT_.read() == 0x02) { // 開始コード STX で始まる場合、有効なメッセージなので読み込む

      char msg[BT_MSG_BUFSIZE_];                            // 受信メッセージをコピーするバッファを確保
      memset(msg, 0x00, BT_MSG_BUFSIZE_);                   // ゼロ埋め
      SerialBT_.readBytesUntil('\n', msg, BT_MSG_BUFSIZE_); // LFまで読み込む

      int val = 0;
      int checksum = 0;
      sscanf(msg, "%d,%d", &val, &checksum); // 受信メッセージから張力とチェックサムを読み取る

      int mychecksum = calc_checksum(val); // valからチェックサムを計算
      if (mychecksum == checksum) {        // チェックサム照合が正常であれば
        lastReceivedMillis_ = millis();    // 受信時刻を記録
        available_ = true;                 // 最新値到着フラグをセット
        tension_ = val;                    // 最新値を反映
      } else {
        Serial.printf("checksum failure!\n");
      }

    } else { // 開始コード以外を検出した場合、捨てる
      while (SerialBT_.available()) {
        SerialBT_.read();
      }
    }
  }
}

bool BTTensionMeter::connected()
{
  if (SerialBT_.connected() && millis() - lastReceivedMillis_ <= TIMEOUT_) {
    return true;
  }
  return false;
}

bool BTTensionMeter::available()
{
  return available_;
}

int BTTensionMeter::getTension()
{
  available_ = false; // 値が読まれたら、最新値到着フラグをクリア
  return tension_;
}

/**
 * @brief 数値を10進数でprintしたときの各桁の和を計算し、下1桁を返す
 * @param val 数値 (例: 16501）
 * @return 各桁の和（例: 3）
 */
int BTTensionMeter::calc_checksum(int val)
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
