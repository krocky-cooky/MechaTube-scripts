/**
 * @file BTPeripheral.hpp
 * @author Hibiki Matsuda
 * @brief PCとのBLE通信を担う関数群。マシンのESP32が、PCに対してペリフェラルとして振舞う
 * @date 2022-09-11
 */
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLE2902.h>
#include <string>

/**
 * @brief Initialize BLE peripheral. This function blocks until PC is connected.
 * @param[in] deviceName マシンの device name
 * @param[in] serviceUUID マシンの service UUID
 * @param[in] logCharacteristicUUID ログをPCに通知する characteristic UUID
 * @param[in] commandCharacteristicUUID PCからの指令を受信する characteristic UUID
 */
void peripheralBegin(const char *deviceName, BLEUUID serviceUUID, BLEUUID logCharacteristicUUID, BLEUUID commandCharacteristicUUID);

/**
 * @brief Return whether the machine is conncted to central PC.
 * @return true PC is connected.
 * @return false PC is not connected. (message hasn't arrived for 200ms or more).
 */
bool pcConnected();

/**
 * @brief Send message to PC
 * @param str message
 */
void sendText(const char* str);
/**
 * @brief Send message to PC
 * @param str message
 */
void sendText(const String str);

/**
 * @brief Return true when a new command has been arrived.
 * @return true when a new command has been arrived but not read by user.
 * @return false when the latest value has been read. You can still use getCommand() to obtain the latest command.
 */
bool commandAvailable();

/**
 * @brief Get the latest command sent from PC.
 * @return Command string. example: "{"target": "trq", "trq": 1.0, "spdLimit": 5.0}"
 */
String getCommand();
