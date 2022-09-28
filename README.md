# MechaTube CAN通信用

## 含まれるソース
- `esp32_main`
  - 筋トレマシン本体のESP32に書き込むコード
- ※ `webserver_client.html` は削除しました
  - WebSocketの疎通を確認したいときはChrome拡張機能の[Browser WebSocket Client](https://chrome.google.com/webstore/detail/browser-websocket-client/mdmlhchldhfnfnkfmljgeinlffmdgkjo?hl=ja)を使ってください
## ダウンロードが必要なArduinoライブラリ
GitHub 上のライブラリについては、Code > Download ZIP でダウンロード後、Arduino IDE > スケッチ > ライブラリをインクルード > .ZIP形式のライブラリをインストール からインストールして下さい

- ESPAsyncWebServer
  - me-no-dev/ESPAsyncWebServer
  - https://github.com/me-no-dev/ESPAsyncWebServer
- AsyncTCP
  - me-no-dev/AsyncTCP
  - https://github.com/me-no-dev/AsyncTCP
- ArduinoJson
  - Arduino IDE のライブラリマネージャーからインストール

## ESP32 Websocket JSON データ形式
以下に示すデータを連続してESP32に送り続けること。データ受信が途中で止まった場合は直近の指令が実行され続ける。なお、トルクと速度の符号は、ロープを巻き取る方向を正、ゆるむ方向を負とする

### ESP32 -> Oculus
|プロパティ|意味|説明|値の範囲|単位|
|---|---|---|---|---|
|target|制御対象|速度指令かトルク指令かを切り替える|"spd" / "trq" / null||
|trq|トルク現在値|出力しているトルク|-6.0 to 6.0|Nm|
|spd|速度現在値|現在の回転速度|-46.57 to 46.57|rad/s|
|pos|位置現在値|現在の位置(モータから送られてくる生の値)|-12.5 to 12.5|rad|
|integratingAngle|回転角|モータから送られてくる位置を、初期値からの絶対回転角に直した値|任意の実数値|rad
```jsonc
{
    "target": "spd",
    "trq": 0.544,
    "spd": 0.171,
    "pos": -4.705,
    "integratingAngle": -4.705
}
```

### Oculus -> ESP32
#### ★速度を制御したいとき
|プロパティ|意味|説明|セットする値|単位|
|---|---|---|---|---|
|target|制御対象|速度指令かトルク指令かを切り替える|"spd"||
|spd|速度指令値|目標とする速度|-46.57 to 46.57|rad/s|
|trqLimit|トルク上限値|速度制御中に出力するトルクの上限値|0.0 to 6.0|Nm|
```jsonc
{
    "target": "spd",
    "spd": 1.0,
    "trqLimit": 2.0
}
```

#### ★トルクを制御したいとき
|プロパティ|意味|説明|セットする値|単位|
|---|---|---|---|---|
|target|制御対象|速度指令かトルク指令かを切り替える|"trq"||
|trq|トルク指令値|出力するトルク|-6.0 to 6.0|Nm|
|spdLimit|速度上限値|トルク制御中の速度上限値。回転速度がこれを超えるとトルクが弱まる|0.0 to 6.0|rad/s|
```jsonc
{
    "target": "trq",
    "trq": 1.0,
    "spdLimit": 2.0
}
```

#### ★なにもしないとき
|プロパティ|意味|説明|セットする値|単位|
|---|---|---|---|---|
|target|制御対象|速度指令かトルク指令かを切り替える|null||
```jsonc
{
   "target": null
}
```

#### ★モータ, コンバータ電源のON/OFF
|プロパティ|型|意味|説明|セットする値|単位|
|---|---|---|---|---|---|
|motor|int|モータ電源|モータからトルク出力を行うかどうか制御する|0:モータOFF, 1:モータON||
|power|int|コンバータ電源|コンバータの電源を制御する|0:コンバータOFF, 1:コンバータON||
```jsonc
{
  "motor": 0  // モータをOFFしたいとき
}
```