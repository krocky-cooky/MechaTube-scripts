/**
 * @file WiFiSetup.hpp
 * @author Hibiki Matsuda
 * @brief
 * @version
 * @date 2022-07-12
 */

#pragma once

#include <DNSServer.h>
#include <ESPAsyncWebServer.h>

class WiFiSetup
{
private:
  static const size_t SSIDLIMIT = 30;

  char *AP_SSID_;   // ESP32自身がルーターとしてWeb設定画面を配信するときのSSID
  char *AP_PASS_;   // ESP32自身がルーターとしてWeb設定画面を配信するときのパスワード
  IPAddress AP_IP_; // ESP32自身がルーターとしてWeb設定画面を配信するときのIPアドレス

  AsyncWebServer &webServer_;
  DNSServer &dnsServer_;

  String ssid_;   // WiFi接続先ルーターのSSID
  String passwd_; // WiFi接続先ルーターのパスワード

  size_t ssidNum_;                  // 発見したSSIDの数
  String ssid_rssi_str_[SSIDLIMIT]; // scaned SSID
  String ssid_str_[SSIDLIMIT];      // scaned SSID

  void webConfig();
  void configServer();
  void wifi_scan();
  String maskpasswd(String passwd);
  String wifimgr_top();
  String Headder_str();
  String wifiinput();
  String WIFI_Form_str();
  String webconnect();
  String status();

public:
  /**
   * @brief Construct a new WiFiSetup object
   * @param ap_ssid 自身がWi-FiアクセスポイントになるときのSSID
   * @param ap_pass 自身がWi-Fiアクセスポイントになるときのパスワード
   * @param ap_ip 自身がWi-FiアクセスポイントになるときのIPアドレス
   */
  WiFiSetup(AsyncWebServer &webServer, DNSServer &dnsServer);

  /**
   * @brief 初期化処理を行う。connectより前に実行すること
   * @param ap_ssid 自身がWi-FiアクセスポイントになるときのSSID
   * @param ap_pass 自身がWi-Fiアクセスポイントになるときのパスワード
   * @param ap_ip 自身がWi-FiアクセスポイントになるときのIPアドレス
   */
  void config(char *ap_ssid, char *ap_pass, IPAddress ap_ip);

  /**
   * @brief Wi-Fi接続を開始する。まず保存済みのSSIDに対して接続を試み、接続できなければ自身がアクセスポイントになってweb設定画面を起動する
   */
  void connect();

  /**
   * @brief ESP32の不揮発性メモリ領域(NVS)に保存されているWi-Fi設定情報を削除する
   */
  void clear();
};
