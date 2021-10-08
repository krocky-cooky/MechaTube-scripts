#include <Arduino.h>
#include <CAN.h>


void setup() {
  delay(10000);
  Serial.begin(9600);
  Serial.println("setup");

  Serial.println("CAN.begin");
  
  /* 通信速度が半分になってしまう不具合が出る場合、以下に示す2行を追加
   * 1行目 Interrupt enable register (TWAI_INT_ENA_REG) へのポインタを定義
   * 2行目 REG_IERの第4ビット(mask:0x10) を0にする
   * 参考 https://github.com/sandeepmistry/arduino-CAN/issues/62
   */
  volatile uint32_t* pREG_IER = (volatile uint32_t*)0x3ff6b010; 
  *pREG_IER &= ~(uint8_t)0x10;
  
  Serial.println(CAN.begin(1000E3));
  //Serial.println(CAN.setPins(26, 27));
  CAN.setPins(26, 27);
  Serial.println("CAN.beginPacket(0x40)");
  Serial.println(CAN.beginPacket(0x40));  // モータのIDを入力

  uint8_t sendBuffer_init[8];  // 送りたいデータを格納するバッファ

  sendBuffer_init[0] = 0xff;  // 送りたいデータを第0バイトから第7バイトまで入力
  sendBuffer_init[1] = 0xff;
  sendBuffer_init[2] = 0xff;
  sendBuffer_init[3] = 0xff;
  sendBuffer_init[4] = 0xff;
  sendBuffer_init[5] = 0xff;
  sendBuffer_init[6] = 0xff;
  sendBuffer_init[7] = 0xfc;

  Serial.println("CAN.write(sendBuffer_init, 8)");
  Serial.println(CAN.write(sendBuffer_init, 8));  // データを送信。8は送りたいバイト数

  Serial.println("CAN.endPacket()");
  Serial.println(CAN.endPacket());

  Serial.println("motor mode");
  delay(1000);
}

void loop() {
Serial.println("loop function");
  Serial.println("CAN.beginPacket(0x40)");
  Serial.println(CAN.beginPacket(0x40));  // モータのIDを入力

  uint8_t sendBuffer[8];  // 送りたいデータを格納するバッファ

  sendBuffer[0] = 0x7f;  // 送りたいデータを第0バイトから第7バイトまで入力
  sendBuffer[1] = 0xff;
  sendBuffer[2] = 0x84;
  sendBuffer[3] = 0x30;
  sendBuffer[4] = 0x0;
  sendBuffer[5] = 0x33;
  sendBuffer[6] = 0x37;
  sendBuffer[7] = 0xff;

  Serial.println("CAN.write(sendBuffer, 8)");
  Serial.println(CAN.write(sendBuffer, 8));  // データを送信。8は送りたいバイト数

  Serial.println("CAN.endPacket()");
  Serial.println(CAN.endPacket());

  Serial.println("done");
  delay(1000);
}
   /*
void setup() {
  Serial.begin(9600);


  CAN.setPins(26, 27);
 
  while (!Serial);

  Serial.println("CAN Sender");

  CAN.setPins(26, 27);
  if (!CAN.begin(1000E3)) {
    Serial.println("Starting CAN failed!");
    while (1);
  }

}

void loop() {

  CAN.beginPacket(0x40);  // モータのIDを入力

  uint8_t sendBuffer[8];  // 送りたいデータを格納するバッファ

  sendBuffer[0] = 0x7f;  // 送りたいデータを第0バイトから第7バイトまで入力
  sendBuffer[1] = 0xff;
  sendBuffer[2] = 0x84;
  sendBuffer[3] = 0x30;
  sendBuffer[4] = 0x0;
  sendBuffer[5] = 0x33;
  sendBuffer[6] = 0x37;
  sendBuffer[7] = 0xff;

  CAN.write(sendBuffer, 8);  // データを送信。8は送りたいバイト数
  
  CAN.endPacket();

  Serial.println("done");
  delay(1000);
}
*/
