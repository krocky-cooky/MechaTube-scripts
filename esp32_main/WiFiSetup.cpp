#include "WiFiSetup.hpp"
#include <Arduino.h>
#include <DNSServer.h>
#include <ESPAsyncWebServer.h>
#include <WiFi.h>
#include <nvs.h>
#include <nvs_flash.h>

WiFiSetup::WiFiSetup(AsyncWebServer &webServer, DNSServer &dnsServer)
  : webServer_(webServer), dnsServer_(dnsServer)
{
}

void WiFiSetup::config(char *ap_ssid, char *ap_pass, IPAddress ap_ip)
{
  AP_SSID_ = ap_ssid;
  AP_PASS_ = ap_pass;
  AP_IP_ = ap_ip;
}

void WiFiSetup::connect()
{
  uint8_t retry = 0;
  WiFi.mode(WIFI_AP_STA); // アクセスポイント(親機) + ステーション(子機)モード
  WiFi.begin();           // すでに保存されている情報を使って、ステーションとしてアクセスポイントに接続してみる
  Serial.print("wifi connecting");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(200); // 200ms
    retry++;
    if (retry > 50) { // 200ms x 50 = 10 sec
      Serial.println();
      Serial.println("wifi connection timeout");
      webConfig(); // 接続できなかったのでwebconfigを開始
    }
  }
  Serial.println();
  Serial.print("Connected to ");
  Serial.print(WiFi.SSID());
  Serial.print(", IP address: ");
  Serial.println(WiFi.localIP());
  delay(500);
}

void WiFiSetup::clear()
{
  WiFi.disconnect(true, true);
  delay(100);
  Serial.println("WiFi disconnected and AP info is erased");
}

void WiFiSetup::webConfig()
{
  Serial.println("WebConfig start");
  WiFi.disconnect(); // いったん切断
  delay(100);
  wifi_scan();    // 利用可能なSSIDをスキャン
  configServer(); // Web設定画面を起動
  while (WiFi.status() != WL_CONNECTED) {  // つながるまで待機
    Serial.print('.');
    delay(1000);
  }
}

void WiFiSetup::wifi_scan()
{
  Serial.println("wifi scan start. please wait for a while...");

  ssidNum_ = WiFi.scanNetworks(); // WiFi.scanNetworks will return the number of networks found
  Serial.print("scan done. ");

  if (ssidNum_ == 0) {
    Serial.println("no networks found");
  } else {
    Serial.printf("%d networks found\r\n\r\n", ssidNum_);
    if (ssidNum_ > SSIDLIMIT) {
      ssidNum_ = SSIDLIMIT;
    }
    for (size_t i = 0; i < ssidNum_; i++) {
      ssid_str_[i] = WiFi.SSID(i);
      String wifi_auth_open = ((WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? " " : "*");
      ssid_rssi_str_[i] = ssid_str_[i] + " (" + WiFi.RSSI(i) + "dBm)" + wifi_auth_open;
      Serial.printf("%d: %s\r\n", i, ssid_rssi_str_[i].c_str());
      delay(10);
    }
  }
}

void WiFiSetup::configServer()
{
  if (AP_PASS_ == "") {
    WiFi.softAP(AP_SSID_); // no password
  } else {
    WiFi.softAP(AP_SSID_, AP_PASS_); // with password
  }
  delay(200); // Important! This delay is necessary
  WiFi.softAPConfig(AP_IP_, AP_IP_, IPAddress(255, 255, 255, 0));
  Serial.println();
  Serial.print("Access ");
  Serial.print(AP_IP_);
  Serial.println(" to setup Wi-Fi connection");

  // dnsServer_.setErrorReplyCode(DNSReplyCode::NoError);
  // dnsServer_.start(53, "*", AP_IP_);

  webServer_.on("/", HTTP_GET, [=](AsyncWebServerRequest *request) {
    request->send(200, "text/html", this->wifimgr_top());
  });
  webServer_.on("/wifiinput", HTTP_GET, [=](AsyncWebServerRequest *request) {
    request->send(200, "text/html", this->wifiinput());
  });
  webServer_.on("/wifiset", HTTP_POST, [=](AsyncWebServerRequest *request) {
    if (request->hasParam("ssid", true)) {
      ssid_ = request->getParam("ssid", true)->value();
      ssid_.trim();
    }
    if (request->getParam("passwd", true)) {
      passwd_ = request->getParam("passwd", true)->value();
      passwd_.trim();
    }
    request->redirect("/");
    Serial.print("selected ssid: ");
    Serial.print(ssid_);
    Serial.print(", passwd: ");
    Serial.println(passwd_);
  });
  webServer_.on("/webconnect", [=](AsyncWebServerRequest *request) {
    request->send(200, "text/html", this->webconnect());
    WiFi.begin(this->ssid_.c_str(), this->passwd_.c_str());
    Serial.println();
    Serial.print("connecting to ");
    Serial.println(this->ssid_);
  });
  webServer_.on("/status", [=](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", this->status());
  });
  webServer_.onNotFound([=](AsyncWebServerRequest *request) {
    request->send(200, "text/html", this->wifimgr_top());
  });
  webServer_.begin();
}

String WiFiSetup::maskpasswd(String passwd)
{
  String maskpasswd = "";

  for (int i = 0; i < passwd.length(); i++)
    maskpasswd = maskpasswd + "*";
  if (passwd.length() == 0) maskpasswd = "(null)";

  return maskpasswd;
}

String WiFiSetup::wifimgr_top()
{
  String html = Headder_str();
  html += "<a href='/wifiinput'>WIFI setup</a>";
  html += "<hr><h3>Current Settings</h3>";
  html += "SSID: " + ssid_ + "<br>";
  html += "passwd: " + maskpasswd(passwd_) + "<br>";
  html += "<hr><p><center><a href='/webconnect'>Connect</a></center>";
  html += "</body></html>";
  return html;
}

String WiFiSetup::Headder_str()
{
  String html = "";
  html += "<!DOCTYPE html><html><head>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1.3'>";
  html += "<meta http-equiv='Pragma' content='no-cache'>";
  html += "<meta http-equiv='Cache-Control' content='no-cache'></head>";
  html += "<meta http-equiv='Expires' content='0'>";
  html += "<style>";
  html += "a:link, a:visited { background-color: #009900; color: white; padding: 5px 15px;";
  html += "text-align: center; text-decoration: none;  display: inline-block;}";
  html += "a:hover, a:active { background-color: green;}";
  html += "bo32{ background-color: #EEEEEE;}";
  html += "input[type=button], input[type=submit], input[type=reset] {";
  html += "background-color: #000099;  border: none;  color: white;  padding: 5px 20px;";
  html += "text-decoration: none;  margin: 4px 2px;";
  html += "</style>";
  html += "<body>";
  html += "<h2>WIFIMGR</h2>";
  return html;
}

String WiFiSetup::wifiinput()
{
  String html = Headder_str();
  html += "<a href='/'>TOP</a> ";
  html += "<hr><p>";
  html += "<h3>WiFi Selector</h3>";
  html += WIFI_Form_str();
  html += "<br><hr><p><center><a href='/'>Cancel</a></center>";
  html += "</body></html>";
  return html;
}

String WiFiSetup::WIFI_Form_str()
{
  String str = "";
  str += "<form action='/wifiset' method='post'>";
  str += "<select name='ssid' id ='ssid'>";
  for (int i = 0; i < ssidNum_; i++) {
    str += "<option value=" + ssid_str_[i] + ">" + ssid_rssi_str_[i] + "</option>";
  }
  str += "<option value=" + ssid_ + ">" + ssid_ + "(current)</option>";
  str += "</select><br>\r\n";
  str += "Password<br><input type='password' name='passwd' value='" + passwd_ + "'>";
  str += "<br><input type='submit' value='set'>";
  str += "</form><br>";
  str += "<script>document.getElementById('ssid').value = '" + ssid_ + "';</script>";
  return str;
}

String WiFiSetup::webconnect()
{
  const char *html = R"rawliteral(
      <hr>
      <h3>Status</h3>
      <span id='status'>Connecting to (SSID)</span>
      <hr>
    </body>
    <script>
      var getStatus = function () {  // ajaxで現在の接続状況を取りに行く関数
        var xhr = new XMLHttpRequest();
        xhr.onreadystatechange = function() {
          if (this.readyState == 4 && this.status == 200) {
            document.getElementById("status").innerHTML = this.responseText;
          }
        };
        xhr.open("GET", "/status", true);
        xhr.send(null);
      }
      setInterval(getStatus, 1000);  // statusは2秒おきに更新される
    </script>
  </html>)rawliteral";
  return Headder_str() + String(html);
}

String WiFiSetup::status()
{
  String text;
  switch (WiFi.status())
  {
  case WL_IDLE_STATUS:
    text = "Connecting to " + ssid_ + "...";
    break;
  case WL_CONNECTED:
    text = "Connected to " + ssid_ + "!\n";
    text += "Now you can disconnect from " + String(AP_SSID_) + " and connect to " + ssid_ + " to communicate with ESP32.\n";
    text += "IP address of ESP32 is " + String(WiFi.localIP());
    break;
  case WL_CONNECT_FAILED:
    text = "Connection failed. Click WIFI setup and retry.";
    break;
  default:
    text = "Unexpected wifi status. Click WIFI setup and retry.";
    break;
  }
  return text;
}
