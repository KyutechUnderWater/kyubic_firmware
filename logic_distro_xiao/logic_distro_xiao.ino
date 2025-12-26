/*
 * Seeeduino Xiao SAMD21 電源分配基板 制御プログラム
 * * 仕様:
 * - Seeeduino XIAO SAMD21 を使用
 * - I2Cピンはデフォルト (D4=SDA, D5=SCL) を使用
 * - 2025/11/17 版 ピン配置
 * - D0: OUT1
 * - D1: OUT2
 * - D2: OUT3
 * - D3: OUT4
 * - D4: SDA (I2C)
 * - D5: SCL (I2C)
 * - D6: OUT5
 * - D7: Act_SW (出力)
 * - D8: ActTemp (DS18B20)
 * - D9: 5V_SW (出力)
 * - D10: Logi_TEMP (DS18B20)
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <PacketSerial.h>
#include "pb_encode.h"
#include "pb_decode.h"

// --- Protoファイルから生成されたヘッダ ---
// 環境に合わせてファイル名を変更してください
#include "driver_msgs__PowerState.pb.h"
#include "driver_msgs__SystemSwitch.pb.h"

// --- メッセージ型のエイリアス (長いため短縮) ---
// 生成された構造体名に合わせて調整してください
using PowerStateMsg = protolink__driver_msgs__PowerState_driver_msgs__PowerState;
using SystemSwitchMsg = protolink__driver_msgs__SystemSwitch_driver_msgs__SystemSwitch;

// --- ピン定義 ---
// 出力ピン (SystemStatusのフィールドに対応)
const int PIN_JETSON = 0; // OUT1
const int PIN_DVL    = 1; // OUT2
const int PIN_COM    = 2; // OUT3
const int PIN_EX1    = 3; // OUT4
const int PIN_EX8    = 6; // OUT5
const int PIN_ACT_SW = 7; // Act_SW
const int PIN_5V_SW  = 9; // 5V_SW (メッセージには含まれていないが制御用として維持)

// センサ・入力ピン
const int PIN_ACT_TEMP = 8;  // ActTemp (DS18B20)
const int PIN_LOG_TEMP = 10; // Logi_TEMP (DS18B20)

// --- I2Cアドレス ---
const uint8_t INA_LOG_ADDR = 0x40; // LogPow (INA219)
const uint8_t INA_ACT_ADDR = 0x45; // ActPow (INA219)

// --- シャント抵抗値による補正係数
const uint8_t SHUNT_COEFFICIENT = 200;

// --- オブジェクト生成 ---
Adafruit_INA219 ina_log(INA_LOG_ADDR);
Adafruit_INA219 ina_act(INA_ACT_ADDR);

OneWire oneWire_log(PIN_LOG_TEMP);
OneWire oneWire_act(PIN_ACT_TEMP);

DallasTemperature sensor_log(&oneWire_log);
DallasTemperature sensor_act(&oneWire_act);

PacketSerial myPacketSerial;

// --- グローバル変数 ---
unsigned long lastSendTime = 0;
const long sendInterval = 1000; // データ送信間隔 (ms) - 頻度を上げる場合は調整

// バッファサイズ (Protoメッセージの最大サイズに合わせて調整)
const size_t MAX_PACKET_SIZE = 256;

// LED制御用変数
const int PIN_LED_USER = 13;  // XIAOのオンボード黄色LED (LED_BUILTINと同等)
unsigned long ledOffTime = 0; // LEDを消す予定時刻
const int LED_DURATION = 200;  // 光らせる時間 (ms)

// --- 関数プロトタイプ ---
void onPacketReceived(const uint8_t* buffer, size_t size);
void readAndPublishData();
void updateOutputPin(int pin, bool state);

// --- セットアップ ---
void setup() {
  // PacketSerial用にSerialを開始
  Serial.begin(115200);
  myPacketSerial.setStream(&Serial);
  myPacketSerial.setPacketHandler(&onPacketReceived);

  // Wire初期化
  Wire.begin();

  // --- ピンモード設定 ---
  pinMode(PIN_JETSON, OUTPUT);
  pinMode(PIN_DVL, OUTPUT);
  pinMode(PIN_EX1, OUTPUT);
  pinMode(PIN_EX8, OUTPUT);
  pinMode(PIN_COM, OUTPUT);
  pinMode(PIN_ACT_SW, OUTPUT);
  pinMode(PIN_5V_SW, OUTPUT);

  // 初期状態: すべてHIGH (ON) または安全側に設定
  digitalWrite(PIN_JETSON, HIGH);
  digitalWrite(PIN_DVL, LOW);
  digitalWrite(PIN_EX1, LOW);
  digitalWrite(PIN_EX8, LOW);
  digitalWrite(PIN_COM, HIGH);
  digitalWrite(PIN_ACT_SW, LOW);
  digitalWrite(PIN_5V_SW, HIGH);

  // LED
  pinMode(PIN_LED_USER, OUTPUT);
  digitalWrite(PIN_LED_USER, LOW);

  // --- センサ初期化 ---
  // begin()に引数は不要 (デフォルトWire使用)
  if (!ina_log.begin()) {
    // エラーハンドリング (必要であればLED点滅など)
  }
  if (!ina_act.begin()) {
    // エラーハンドリング
  }

  sensor_log.begin();
  sensor_act.begin();
  
  // 初回の温度測定要求
  sensor_log.setWaitForConversion(false); // 非ブロッキング設定
  sensor_act.setWaitForConversion(false);
  sensor_log.requestTemperatures();
  sensor_act.requestTemperatures();
}

// --- メインループ ---
void loop() {
  // 受信データの処理
  myPacketSerial.update();

  // 定期的なデータ送信
  unsigned long currentTime = millis();
  if (currentTime - lastSendTime >= sendInterval) {
    lastSendTime = currentTime;
    readAndPublishData();
  }

  if (currentTime > ledOffTime && digitalRead(PIN_LED_USER) == HIGH) {
    digitalWrite(PIN_LED_USER, LOW);
  }
}

// --- パケット受信コールバック (SystemSwitch) ---
void onPacketReceived(const uint8_t* buffer, size_t size) {
  SystemSwitchMsg message = protolink__driver_msgs__SystemSwitch_driver_msgs__SystemSwitch_init_zero;
  pb_istream_t stream = pb_istream_from_buffer(buffer, size);

  // デコード実行
  // ※フィールド定義構造体名は生成されたコードを確認してください
  bool status = pb_decode(&stream, protolink__driver_msgs__SystemSwitch_driver_msgs__SystemSwitch_fields, &message);

  if (status) {
    // 受信成功時、各ピンの状態を更新
    // has_xxx フラグを確認してから反映する (optionalのため)
    
    if (message.has_jetson)   updateOutputPin(PIN_JETSON, message.jetson);
    if (message.has_dvl)      updateOutputPin(PIN_DVL, message.dvl);
    if (message.has_com)      updateOutputPin(PIN_COM, message.com);
    if (message.has_ex1)      updateOutputPin(PIN_EX1, message.ex1);
    if (message.has_ex8)      updateOutputPin(PIN_EX8, message.ex8);
    if (message.has_actuator) updateOutputPin(PIN_ACT_SW, message.actuator);
    
    // Ack等はPacketSerialなので基本不要だが、必要ならここで返信
  } else {
    // デコードエラー処理
  }
}

// --- ピン出力更新ヘルパー ---
void updateOutputPin(int pin, bool state) {
  // state: true -> ON (High), false -> OFF (Low)
  digitalWrite(pin, state ? HIGH : LOW);
}

// --- データ読み取り & 送信 (PowerState) ---
void readAndPublishData() {
  // 1. センサデータ取得
  float log_busVoltage = ina_log.getBusVoltage_V();
  float log_shuntVoltage = ina_log.getShuntVoltage_mV();
  float log_current = ina_log.getCurrent_mA() / SHUNT_COEFFICIENT;
  float log_power = ina_log.getPower_mW() / SHUNT_COEFFICIENT;
  float log_voltage = log_busVoltage + (log_shuntVoltage / 1000.0);

  float act_busVoltage = ina_act.getBusVoltage_V();
  float act_shuntVoltage = ina_act.getShuntVoltage_mV();
  float act_current = ina_act.getCurrent_mA() / SHUNT_COEFFICIENT;
  float act_power = ina_act.getPower_mW() / SHUNT_COEFFICIENT;
  float act_voltage = act_busVoltage + (act_shuntVoltage / 1000.0);

  if (log_voltage > 30) log_voltage = 1.0f / 0.0f;
  if (act_voltage > 30) act_voltage = 1.0f / 0.0f;

  // 温度取得 (リクエスト済みの値を取得)
  float log_temp = sensor_log.getTempCByIndex(0);
  if (log_temp == DEVICE_DISCONNECTED_C) log_temp = -127.0;

  float act_temp = sensor_act.getTempCByIndex(0);
  if (act_temp == DEVICE_DISCONNECTED_C) act_temp = -127.0;

  // 次回の温度測定をリクエスト (非ブロッキング)
  sensor_log.requestTemperatures();
  sensor_act.requestTemperatures();

  // 2. Nanopb メッセージ作成
  PowerStateMsg message = protolink__driver_msgs__PowerState_driver_msgs__PowerState_init_zero;

  // ヘッダー情報はArduino側で正確な時刻を持てないため、
  // 必要なら受信側(ROS 2ノードなど)で付与するか、millis()を入れる
  // message.header.stamp... 

  // データをセット (optionalなので has_xxx = true もセット)
  message.log_voltage = log_voltage;     message.has_log_voltage = true;
  message.log_current = log_current;     message.has_log_current = true;
  message.log_power   = log_power;       message.has_log_power = true;
  
  message.act_voltage = act_voltage;     message.has_act_voltage = true;
  message.act_current = act_current;     message.has_act_current = true;
  message.act_power   = act_power;       message.has_act_power = true;

  message.log_temp    = log_temp;        message.has_log_temp = true;
  message.act_temp    = act_temp;        message.has_act_temp = true;

  // 3. エンコード & 送信
  uint8_t buffer[MAX_PACKET_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));

  if (pb_encode(&stream, protolink__driver_msgs__PowerState_driver_msgs__PowerState_fields, &message)) {
    myPacketSerial.send(buffer, stream.bytes_written);
    digitalWrite(PIN_LED_USER, HIGH);
    ledOffTime = millis() + LED_DURATION;
  } else {
    // エンコードエラー
  }
}