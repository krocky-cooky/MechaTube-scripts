#include "ESP32BuiltinCAN.hpp"

#include <Arduino.h>

// staticメンバ変数の実体を生成
std::vector<ESP32BuiltinCAN::Cb *> ESP32BuiltinCAN::cbs_;
portMUX_TYPE ESP32BuiltinCAN::canMutex_ = portMUX_INITIALIZER_UNLOCKED;

ESP32BuiltinCAN::ESP32BuiltinCAN(uint8_t rx, uint8_t tx)
  : rx_(rx), tx_(tx)
{
}

int ESP32BuiltinCAN::begin(long baudrate)
{
  CAN.setPins(rx_, tx_);
  if (!CAN.begin(baudrate)) return 0;
  // CAN速度修正
  volatile uint32_t *REG_IER = (volatile uint32_t *)0x3ff6b010;
  *REG_IER &= ~(uint8_t)0x10;
  // 割り込み関数登録
  CAN.onReceive(&invoke_);
  return 1;
}

int ESP32BuiltinCAN::send(int id, const uint8_t *data, size_t size) const
{
  portENTER_CRITICAL(&canMutex_);
  if (CAN.beginPacket(id)) {
    CAN.write(data, size);
    if (CAN.endPacket()){
      portEXIT_CRITICAL(&canMutex_);
      return 1;
    }
  }
  portEXIT_CRITICAL(&canMutex_);
  return 0;
}

void ESP32BuiltinCAN::set_callback(void (*callback)(int, void *), void *arg)
{
  Cb *cb = new Cb();
  cb->func = callback;
  cb->arg = arg;
  cbs_.push_back(cb);
}

void ESP32BuiltinCAN::clear_callback()
{
  for (auto cb : cbs_) { // cb_funcが指し示す先にある構造体をすべて削除
    delete cb;
  }
  cbs_.clear(); // 配列cb_funcs_を空にする
}

long ESP32BuiltinCAN::packetId()
{
  return CAN.packetId();
}

int ESP32BuiltinCAN::read()
{
  return CAN.read();
}

void IRAM_ATTR ESP32BuiltinCAN::invoke_(int packet_size)
{
  portENTER_CRITICAL_ISR(&canMutex_);
  for (auto cb_ : cbs_) { // 登録されているコールバック関数をすべて実行
    cb_->func(packet_size, cb_->arg);
  }
  portEXIT_CRITICAL_ISR(&canMutex_);
}
