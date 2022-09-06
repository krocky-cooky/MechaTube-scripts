/*
#include "BTTensionMeter.hpp"

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

float BTTensionMeter::getTension()
{
  available_ = false;                          // 値が読まれたら、最新値到着フラグをクリア
  return static_cast<float>(tension_) * 0.001; // mg単位なので、gに換算
}
*/