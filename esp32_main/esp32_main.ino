#include <Arduino.h>
#include <CAN.h>
#include <math.h>
#include <stdio.h>

//定数等
#define PIN_CANRX 32
#define PIN_CANTX 33
#define PIN_POWER 26
#define PIN_HANDSWICH 35
#define KP 0.1
#define KD 1.0
#define TAU_WHILE_ECCENTRIC_MOTION 0.5     //エキセン動作時の、一時遅れ系によるトルク指令の、時定数
#define TAU_WHILE_NON_ECCENTRIC_MOTION 1.0 //エキセン動作以外の時の、一時遅れ系によるトルク指令の、時定数
#define P_MIN -12.5
#define P_MAX 12.5

//閾値等
#define FORCE_THRESHOLD_OF_HANDSWICH 10.0                          //手元スイッチのオンオフを識別するための、スイッチにかかる力の閾値 [N]
#define THRESHOLD_OF_MOTOR_SPEED_FOR_DETERMINING_ECCENTRIC_MOTION 0.33 //エキセン動作を判定するための、モータの回転速度の閾値 [rad/s]
#define MAX_TORQUE 4.0                                                //許容する最大トルク [Nm]
#define MAX_SPEED 6.5                                                 //許容する最大回転速さ [rad/s]
#define MAX_LOGNUM 1024                                               //筋力測定の最大ログ数
#define THRESHOLD_OF_COUNT_FOR_SPOTTER_MODE 30.0 //スポッターモードを行うかどうかの閾値
#define THRESHOLD_OF_SPEED_FOR_SPOTTER_MODE 0.7 //スポッターモードのためのカウントをする際の速さの閾値

// フラグ等
bool torqueCtrlMode = 0; // 速度制御したいとき0,トルク制御したいとき1になるフラグ
bool spotterMode = 0; //スポッターモードのフラグ

// ユーザの手元にあるスイッチなど、トルク出力をON/OFFする指令
bool handSwitch = false;

// シリアル通信で受信した指令値
bool powerCommand = false;                       // コンバータからモータへの電源供給指令
bool controlCommand = false;                     // モータ制御モードに入るかどうかの指令
float torqueCommand = 0.0;                       // トルク指令値 [Nm]
float speedCommand = 0.0;                        // 速度指令値 [rad/s]
float increaseOfToraueForEccentricMotion = 0.0;  // エキセン動作時に増加するトルク量 [Nm]
float maxSpeedWhileConcentricMotion = MAX_SPEED; // コンセン動作時に許容する最大回転速さ [rad/s], アイソキネティックトレーニング時は指定値にし、そうでない時はMAX_SPEEDに合わせる
int countForSpotterMode = 0; //スポッターモードのためのカウント。これが閾値以上になったら補助開始
float decreaseOfTorquePerCount = 0.5; //スポッターモードの時、1カウントあたりどれくらいトルクを減らすか

// 実際にモータやコンバータに送信している指令値
bool powerSending = false;
bool controlSending = false;
float torqueSending = 0.0;
float speedSending = 0.0;

// (待ち時間等の処理に使う)時刻の記録
unsigned long timeNow = 0;       // 各loopの開始時刻 [us]
unsigned long timePrev = 0;      // 直前のloopの開始時刻 [us]
unsigned long dtMicros = 0;      // 直前のloopからの経過時間 [us]
unsigned long timePowerOn = 0;   // モータの電源を入れた時刻 [us]
unsigned long timeControlOn = 0; // モータ制御モードに入った時刻 [us]

// 直近にCANで送受信したデータを記憶しておく変数(表示に利用)
float positionSent, speedSent, kpSent, kdSent, torqueSent;      // 直近のCAN送信データを記録しておく
volatile float positionReceived, speedReceived, torqueReceived; // 直近のCAN受信データを記録しておく
volatile uint8_t canReceivedMsg[6];                             // 直近のCAN受信データそのものを記録しておく
volatile float previousPositionReceived = 0.0;                  //直前に受信した位置のデータ
volatile float numberOfTimesYouCrossedOverFromPmaxToPmin = 0.0; //位置=P_MAXから位置が増加して位置=P_MINに移動した回数。逆向きで位置=P_MIMから位置=P_MAXに移動したら-1する。例えば、P_MAX=12.5, P_MIN=-12.5の時、positionReceived=10から、回転位置が5増えると、positionReceivedは15ではなく-10になる。

// 筋力測定用
unsigned long timeLog[MAX_LOGNUM];  // 時刻の保存用配列
float torqueLog[MAX_LOGNUM];  // トルクのログ保存用配列
float positionLog[MAX_LOGNUM];  // 位置のログ保存用配列

//初期位置からの回転角
//例えば、P_MAX=12.5, P_MIN=-12.5の時、positionReceived=10から、回転位置が5増えると、positionReceivedは15ではなく-10になる
//それだと不便なので、下の変数には、累計でどれだけ位置変化したかを記録。上の例では、position=15を記録する事になる
volatile float rotationAngleFromInitialPosition;

// PrimeFittnessのような可変抵抗トレーニングの実装のための変数
// PrimeFittnessのURL:https://www.thinkgroup.co.jp/think_products/products/pdf/strive_catalog.pdf
//一定抵抗ではなく、ピーク位置の位置、ピークの大きさ、ピークの裾野の大きさ(ピークから通常部分までの距離)を指定して、好きなところに、より大きな負荷をかけられる
float increaseOfToraueWhenPeak = 0.0;                 //ピーク時のトルクが通常トルクよりどれだけ高いか
float rotationAngleFromInitialPositionWhenPeak = 0.0; //ピーク位置。受信する位置データではなく、初期位置からの回転角で与える
float rangeOfTorqueChange = 0.0;                      //ピーク位置に対して+-いくつの位置まで行けば通常トルクになるか。言い換えれば、ピークの裾野の大きさ

// CAN受信割込みとmainloopの双方からアクセスする変数の排他処理
portMUX_TYPE onCanReceiveMux = portMUX_INITIALIZER_UNLOCKED;

void setup()
{
  pinMode(PIN_POWER, OUTPUT);
  digitalWrite(PIN_POWER, LOW);

  Serial.begin(115200);
  while (!Serial)
    delay(1);

  CAN.setPins(PIN_CANRX, PIN_CANTX);
  if (!CAN.begin(1000E3))
  {
    Serial.println("Starting CAN failed!");
    while (1)
      ;
  }
  CAN.onReceive(&can_onReceive);
  // modify half speed problem
  // reference: https://github.com/sandeepmistry/arduino-CAN/issues/62
  volatile uint32_t *pREG_IER = (volatile uint32_t *)0x3ff6b010;
  *pREG_IER &= ~(uint8_t)0x10;

  // 直前に受信した位置のデータを初期化
  unpackReply(canReceivedMsg, &positionReceived, &speedReceived, &torqueReceived);
  previousPositionReceived = positionReceived;

  Serial.println("[setup] setup comleted");
}

void loop()
{

  // 開始時刻の記録
  timePrev = timeNow;
  timeNow = micros();
  dtMicros = timeNow - timePrev;

  // シリアル通信で指令値を受け取る
  serial_getIncomingCommand(&powerCommand, &controlCommand, &torqueCtrlMode, &torqueCommand, &speedCommand, &increaseOfToraueForEccentricMotion, &maxSpeedWhileConcentricMotion, &increaseOfToraueWhenPeak, &rotationAngleFromInitialPositionWhenPeak, &rangeOfTorqueChange);

  // 指令値に応じて、モータの電源とモータ制御モードを変更する
  setPower(powerCommand);
  setControl(controlCommand);

  // 手元スイッチのON/OFFを取得する
  float touchSensorValue = analogRead(PIN_HANDSWICH);               //手元スイッチのセンサの値
  float force_on_handSwich = (4096 - touchSensorValue) / 4096 * 20; //手元スイッチのセンサにかかる力[N]
  if (force_on_handSwich >= FORCE_THRESHOLD_OF_HANDSWICH)
  {
    handSwitch = true;
  }
  else
  {
    handSwitch = false;
  }
  //ハンドスイッチを解除
  handSwitch = true;
  // Serial.printf("handSwitch = %d\n", handSwitch);

  //コンバータの電圧を表示
  float voltageOfConverter = analogRead(34) * 3.3 * 21 / 4096; //コンバータの電圧の値
  // Serial.printf("the voltage of converter = %f\n", voltageOfConverter);

  // can通信の受信値を表示
  unpackReply(canReceivedMsg, &positionReceived, &speedReceived, &torqueReceived);
  Serial.printf("{\"torque_received\":%f, \"speed_received\":%f, \"position_received\":%f, \"rotationAngleFromInitialPosition\":%f}\n", torqueReceived, speedReceived, positionReceived, rotationAngleFromInitialPosition);

  //初期位置からの回転角を記録
  //位置=P_MAXから位置が増加して位置=P_MINに移動した回数をカウントする
  //速度が正なのに、位置の符号が正から負に変化したら、位置=P_MAXから位置が増加して位置=P_MINに移動していると判断
  //ただし、位置=0.0前後で小さく動いている時は、速度や位置の符号が不安定になるので、除外する
  //位置の符号が変化し、かつ位置の変化量が一定以上であるかどうか
  if (previousPositionReceived * positionReceived < -2.0)
  {
    if (positionReceived > 0 && speedReceived < 0)
    {
      numberOfTimesYouCrossedOverFromPmaxToPmin -= 1;
    }
    else if (positionReceived < 0 && speedReceived > 0)
    {
      numberOfTimesYouCrossedOverFromPmaxToPmin += 1;
    }
  }
  rotationAngleFromInitialPosition = positionReceived + (P_MAX - P_MIN) * numberOfTimesYouCrossedOverFromPmaxToPmin;
  Serial.printf("rotationAngleFromInitialPosition = %f\n", rotationAngleFromInitialPosition);
  
  Serial.printf("handSwitch = %d\n", handSwitch);

  // モータ制御モードに入っているとき、送信値を計算し、CANを送信する
  if (controlSending)
  {
    if (torqueCtrlMode)
    {
      // トルクの指示値を代入
      if (handSwitch)
      { // 手元スイッチONのとき送信値をゆっくり指令値に近づけ、OFFのときは0に近づける

        // // エキセン動作時(モータの回転速度は正)は指示トルクを大きくする
        // // Serial.printf("increaseOfToraueForEccentricMotion = %f\n", increaseOfToraueForEccentricMotion);

        // if (speedReceived > THRESHOLD_OF_MOTOR_SPEED_FOR_DETERMINING_ECCENTRIC_MOTION)
        // {
        //   torqueSending = firstOrderDelay_torque_controlling_tau(torqueCommand + increaseOfToraueForEccentricMotion, (float)dtMicros / 1e6, TAU_WHILE_NON_ECCENTRIC_MOTION);
        // }
        // else
        // {
          torqueSending = firstOrderDelay_torque_controlling_tau(torqueCommand, (float)dtMicros / 1e6, TAU_WHILE_NON_ECCENTRIC_MOTION);
        // }
      }
      else
      {
        torqueSending = 0.0;
      }

      // //トレーナーの補助機能(スポッターモード)
      // //速さが閾値未満なら、スポッターモードをするかどうかのカウントを増やす
      // //速さが閾値を超えたらカウントをリセット
      // if (fabsf(speedReceived) < THRESHOLD_OF_SPEED_FOR_SPOTTER_MODE){
      //   countForSpotterMode+=1;
      // }else{
      //   countForSpotterMode=0;
      // }
      // //カウントが閾値を超えたらスポッターモードをオンにする
      // if (countForSpotterMode > THRESHOLD_OF_COUNT_FOR_SPOTTER_MODE){
      //   spotterMode = 1;
      // }
      // //トルク指令値が減少量未満なら、カウントとフラグをリセット
      // //例えばt=0を指令すれば、スポッターモードとそのカウントが解除・リセットされる
      // if (torqueSending < decreaseOfTorquePerCount){
      //   countForSpotterMode=0;
      //   spotterMode = 0;
      // }
      // //スポッターモードならトルク減少させる
      // if (spotterMode){
      //   torqueSending = torqueSending - decreaseOfTorquePerCount;
      // }
      // Serial.printf("{\"countForSpotterMode\":%d, \"torqueSending\":%f, \"spotterMode\":%d}\n", countForSpotterMode, torqueSending, spotterMode);

      // //トルクが最大許容値を超える場合は、最大許容値を代入し、それ以上の上昇は許さない
      // if (torqueSending > MAX_TORQUE)
      // {
      //   torqueSending = MAX_TORQUE;
      // }

      speedSending = 0.0;           // 速度送信値は不要なので0とする
      firstOrderDelay_resetSpeed(); // 速度の1次遅れ計算用変数をリセット

      // // 指定条件を満たせば、PrimeFittnessのような可変抵抗トレーニングor等速度トレーニングを実行
      // //どちらも実行するようなことはないようにする
      // //トルク制限つき
      // if (torqueReceived > MAX_TORQUE)
      // {
      //   can_sendCommand(0.0, 0.0, 0.0, 0.0, MAX_TORQUE);
      // }
      // else
      // {
      //   //ピーク時のトルクの増分と、負荷のピークの裾野が指令されており、かつ等速度トレーニングの実行条件が満たされていなければ、可変トレーニングを実行
      //   if (increaseOfToraueWhenPeak * rangeOfTorqueChange && maxSpeedWhileConcentricMotion >= 1.1)
      //   {
      //     //ピークからの距離が指定範囲内かどうか
      //     if (fabsf(rotationAngleFromInitialPosition - rotationAngleFromInitialPositionWhenPeak) < rangeOfTorqueChange)
      //     {
      //       //ピークから離れている分だけ、ピークに比べてトルクを減らす
      //       torqueSending += increaseOfToraueWhenPeak - fabsf(rotationAngleFromInitialPosition - rotationAngleFromInitialPositionWhenPeak) / rangeOfTorqueChange * increaseOfToraueWhenPeak;
      //     }
      //     can_sendCommand(0.0, 0.0, 0.0, 0.0, torqueSending); // 送信
      //   }
      //   //持ち上げるときの制限速度が1.1未満であり、かつ可変トレーニングの実行条件が満たされていなければ、等速性トレーニングにする
      //   else if (maxSpeedWhileConcentricMotion < 1.1 && increaseOfToraueWhenPeak * rangeOfTorqueChange == 0)
      //   {
      //     if (speedReceived < -maxSpeedWhileConcentricMotion)
      //     {
      //       can_sendCommand(0.0, -maxSpeedWhileConcentricMotion, 0.0, KD, 0.0);
      //     }
      //     else
      //     {
      //       can_sendCommand(0.0, 0.0, 0.0, 0.0, torqueSending); // 送信
      //     }
      //   }
      //   else
      //   {
      //    // 等速度トレーニングも可変トレーニングも実行されなければ、そのままトルクを送信
          can_sendCommand(0.0, 0.0, 0.0, 0.0, torqueSending); // 送信
      //   }
      // }

      //直前の位置データを更新
      previousPositionReceived = positionReceived;
    }
    // 速度指令モードのとき
    else
    {
      if (handSwitch)
      {
        speedSending = firstOrderDelay_speed(speedCommand, (float)dtMicros / 1e6);
      }
      else
      {
        speedSending = firstOrderDelay_speed(0.0, (float)dtMicros / 1e6);
      }
      torqueSending = 0.0;
      firstOrderDelay_resetTorque();

      can_sendCommand(0.0, speedSending, 0.0, KD, 0.0);

      // 筋力測定用コード
      static size_t i_measure = 0;  // 筋力測定におけるサンプル番号
      timeLog[i_measure] = micros();  // 現在時刻[us]を記録
      torqueLog[i_measure] = torqueReceived;  // トルクを記録
      positionLog[i_measure] = positionReceived;  // 位置を記録
      i_measure++;  // サンプルを次へ
      if (i_measure > MAX_LOGNUM) {  // 最大個数を超えたらインデックスをリセットし、print
        i_measure = 0;
        Serial.println("[");
        for (int i_print = 0; i_print < MAX_LOGNUM; i_print++) {
          Serial.printf("{time: %d, position: %f, torque: %f},\n", timeLog[i_print], positionLog[i_print], torqueLog[i_print]);
        }
        Serial.println("]");
      }
      
    }
    // モータ制御モードに入っていないとき、全ての変数を0にリセットしておく
  }
  else
  {
    torqueSending = 0.0;
    speedSending = 0.0;
    firstOrderDelay_resetTorque();
    firstOrderDelay_resetSpeed();
  }

  delay(10);

  portENTER_CRITICAL_ISR(&onCanReceiveMux); // CAN受信割込みと共有する変数へのアクセスはこの中で行う
  // Serial.printf("{\"torque\":%f, \"speed\":%f, \"position\":%f}\n", torqueReceived, speedReceived, positionReceived);
  // Serial.printf("%x %x %x %x %x %x\n", canReceivedMsg[0], canReceivedMsg[1], canReceivedMsg[2], canReceivedMsg[3], canReceivedMsg[4], canReceivedMsg[5]);
  portEXIT_CRITICAL_ISR(&onCanReceiveMux); // CAN受信割込みと共有する変数へのアクセスはこの中で行う
}

void setPower(bool command)
{
  if (command == 1 && powerSending == 0)
  {
    if (digitalRead(PIN_POWER) == LOW)
    {
      digitalWrite(PIN_POWER, HIGH);
      timePowerOn = micros();
      Serial.println("[setPower] set PIN_POWER HIGH");
    }
    if ((long)(timeNow - timePowerOn) > 2000000)
    {
      powerSending = 1;
      Serial.println("[setPower] motor power: ON");
    }
  }

  if (command == 0)
  {
    if (controlSending == 1)
    {
      setControl(0);
    }
    else
    {
      if (digitalRead(PIN_POWER) == HIGH)
      {
        digitalWrite(PIN_POWER, LOW);
        powerSending = 0;
        Serial.println("[setPower] motor power: OFF");
      }
    }
  }
}

void setControl(bool command)
{
  if (command == 1 && powerSending == 0)
  {
    setPower(1);
  }

  if (command == 1 && powerSending == 1 && controlSending == 0)
  {
    can_sendControl(1);
    controlSending = 1;
    Serial.println("[setControl] motor control mode: ON");
  }

  if (command == 0 && controlSending == 1)
  {
    can_sendCommand(0.0, 0.0, 0.0, 0.0, 0.0);
    can_sendControl(0);
    controlSending = 0;
    Serial.println("[setControl] motor control mode: OFF");
  }
}
