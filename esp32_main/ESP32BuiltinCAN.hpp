/**
 * @file
 * @brief CAN.h(arduino-CANライブラリ)のラッパーとして、複数バイトの送信や、受信割り込みの管理といった機能を追加したCAN処理層
 * @author thgcMtdh
 * @date 2022-06-23
 * @note **ESP32BuiltinCANインスタンスを2つ以上生成しないこと!!**
 */

#pragma once

#include <Arduino.h>
#include <CAN.h>
#include <vector>

class ESP32BuiltinCAN
{
private:
  struct Cb {
    void (*func)(int, void *); //コールバック関数へのポインタ
    void *arg;                 //クラスインスタンスへのポインタなど、コールバック関数に追加で渡したい変数
  };
  static std::vector<Cb *> cbs_; //コールバックを格納する配列
  static portMUX_TYPE canMutex_; //CANインスタンスへの同時アクセスを防止するためのミューテックス
  static void IRAM_ATTR invoke_(int packet_size);
  const uint8_t rx_;
  const uint8_t tx_;

public:
  /**
   * @brief construct a new ESP32BuiltinCAN object
   * @param rx CAN Rx Pin
   * @param tx CAN Tx Pin
   */
  ESP32BuiltinCAN(uint8_t rx, uint8_t tx);

  /**
   * @brief CANバスを立ち上げて、通信可能な状態にする
   * @param baudrate 通信速度[kbps]
   * @return 0:fail, 1:success
   */
  int begin(long baudrate);

  /**
   * @brief 指定したバイト数分CANバスにデータを送信する
   * @param id CANのメッセージID
   * @param data 送信したいデータのバッファ
   * @param size 送信するバイト数[byte]
   * @return 0:fail, 1:success
   */
  int send(int id, const uint8_t *data, size_t size) const;

  /**
   * @brief CAN受信割り込みで実行されるコールバック関数を登録する
   * @param callback コールバック関数へのポインタ
   * @param arg コールバック関数に追加で渡したい変数
   * @note Class内での使用例: set_callback(&Class::static_callback_func, this);
   */
  void set_callback(void (*callback)(int, void *), void *arg);

  /**
   * @brief CAN受信割り込みで実行されるコールバック関数をすべて削除する
   */
  void clear_callback();

  /**
   * @brief 受信したメッセージのIDを取得する
   * @return 受信したメッセージのID
   */
  long packetId();

  /**
   * @brief 受信バッファから1バイト読む
   * @return 受信した文字. なにも受信していない場合は-1を返す
   */
  int read();
};
