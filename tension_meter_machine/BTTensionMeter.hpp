/**
 * @file BTTensionMeter.hpp
 * @author Hibiki Matsuda
 * @brief Bluetooth張力計(tension_meter.ino)からデータを受信する関数群
 * @date 2022-08-27
 */
#include <Arduino.h>
#include <BLEDevice.h>
#include <string>

/**
 * @brief Scan the tension meter and begin connection. This function is blocking until the meter is found
 * @param[in] deviceName 張力計に書き込んだ device name
 * @param[in] serviceUUID 張力計の service UUID
 * @param[in] characteristicUUID 張力計測値を通知する characteristic UUID
 * @param[in] scanTimeout 張力計が見つかるまで待機する秒数[s]。これ以上待っても張力計を発見できなければfalseを返す
 * @return true: 張力計の接続に成功したとき、false: 張力計に接続できなかったとき
 */
bool tensionMeterBegin(const char *deviceName, BLEUUID serviceUUID, BLEUUID characteristicUUID, int scanTimeout);

/**
 * @brief Return whether the tension meter is conncted or not.
 * @return true Tension meter is connected.
 * @return false Tension meter is not connected (message hasn't arrived for 50ms or more).
 */
bool tensionMeterConnected();

/**
 * @brief Return true when a new value has been arrived.
 * @return true when a new value has been arrived but not read by user.
 * @return false when the latest value has also been read. You can still use getTension() to obtain the latest value.
 */
bool tensionAvailable();

/**
 * @brief Get the latest tension sent from meter.
 * @return (int) Measured tension [mg]
 */
int getTension();
