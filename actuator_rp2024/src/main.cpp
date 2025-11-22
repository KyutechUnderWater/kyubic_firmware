/*
  Thruster control with Nanopb (driver_msgs/Actuator)
*/
#include <Arduino.h>
#include <Servo.h>
#include <Adafruit_NeoPixel.h>
#include <array>
#include <PacketSerial.h>

#include "config.hpp"

// Nanopb headers
#include "pb_encode.h"
#include "pb_decode.h"
// ※ .protoファイルから生成されたヘッダファイルのパスに合わせて変更が必要
#include "proto/driver_msgs__Actuator.pb.h"

// メッセージ型のエイリアス定義 (生成された構造体名に合わせる)
using Actuator = protolink__driver_msgs__Actuator_driver_msgs__Actuator;

// --- グローバル変数定義 ---
PacketSerial myPacketSerial;
const static uint8_t THRUSTER_NUM = 6;

// 推力制御パラメータ
const static float CCW_POSITIVE[4] = { -0.0015, 0.1724, -11.936, 1465.9 };
const static float CCW_NEGATIVE[4] = { -0.0035, -0.2981, -15.471, 1532.4 };
const static float CW_POSITIVE[4] = { 0.0015, -0.1724, 11.936, 1534.1 };
const static float CW_NEGATIVE[4] = { 0.0035, 0.2981, 15.471, 1467.6 };

const static float LIMIT_SINGLE_FORCE = 30;
const static float LIMIT_TOTAL_FORCE = 80.1430099;
const static uint16_t SERIAL_TIMEOUT = 1000;

float thrust_value[THRUSTER_NUM] = { 0 };
int thrust_pwm[THRUSTER_NUM];
unsigned long last_time = 0;
bool is_update = false;

// ハードウェアオブジェクト
std::array<Servo, 6> escs;
std::array<Servo, 2> leds;

// --- 関数プロトタイプ ---
void onboard_led(uint8_t r, uint8_t g, uint8_t b);
void GetWritePWM(float *thrust);
void LimitThrust();
void onPacketReceived(const uint8_t* buffer, size_t size);

template<std::size_t N>
void init_servos(std::array<Servo, N>& servos, const std::array<uint8_t, N>& pins){
  for(size_t i = 0; i < N; i++) servos[i].attach(pins[i]);
}

template<std::size_t N>
void write_servos(std::array<Servo, N>& servos, int ms){
  for (auto& servo : servos) servo.writeMicroseconds(ms);
}

// --- パケット受信コールバック ---
void onPacketReceived(const uint8_t* buffer, size_t size)
{
  Actuator message = protolink__driver_msgs__Actuator_driver_msgs__Actuator_init_zero;
  pb_istream_t stream = pb_istream_from_buffer(buffer, size);
  
  // デコード実行
  // Actuatorメッセージのフィールド構造（message.thrust 等）に合わせて適宜修正が必要
  bool status = pb_decode(&stream, protolink__driver_msgs__Actuator_driver_msgs__Actuator_fields, &message);

  if (status)
  {
    // 受信データの反映
    // ※proto定義で配列がfixed_sizeかcallbackかによりアクセス方法が異なる点に注意
    // ここでは thrust_value への代入を行う想定
    
    // 例: message.effort (double配列) が存在する場合
    // nanopbのoptionsでmax_countを設定している場合は直接配列アクセス可能
    /* for (int i = 0; i < THRUSTER_NUM; i++) {
       if (i < message.effort_count) {
           thrust_value[i] = (float)message.effort[i];
       }
    }
    */

    // 仮実装: 受信成功フラグのみ
    is_update = true;
    last_time = millis();
    onboard_led(0, 0, 25); // 正常受信: 青
  }
  else
  {
    onboard_led(25, 0, 0); // デコードエラー: 赤
  }
}

// --- セットアップ ---
void setup() {
  // PacketSerial初期化 (Serial.beginも内部で行われるが明示的に指定も可)
  myPacketSerial.begin(115200);
  myPacketSerial.setPacketHandler(&onPacketReceived);
  
  onboard_led(0, 25, 0); // 起動: 緑

  init_servos(escs, ESC_SIGS);
  init_servos(leds, LEDS);
  write_servos(escs, 1500);
  write_servos(leds, 1500);

  pinMode(RELAY_SIG, OUTPUT);
  pinMode(SIG_SPARE, OUTPUT);
  for (auto pin : SIGS) pinMode(pin, OUTPUT);

  delay(7000);
  onboard_led(0, 0, 25); // 待機: 青
}

// --- メインループ ---
void loop() {
  // パケット受信処理の更新
  myPacketSerial.update();

  // タイムアウト監視
  if (SERIAL_TIMEOUT < (millis() - last_time)) {
    for (int i = 0; i < THRUSTER_NUM; i++) {
      thrust_value[i] = 0;
    }
    is_update = true;
    onboard_led(25, 25, 0); // タイムアウト: 黄
  }

  // 推力更新
  if (is_update) {
    LimitThrust();
    GetWritePWM(thrust_value);
    is_update = false;
  }
}

// --- ヘルパー関数 ---
void onboard_led(uint8_t r, uint8_t g, uint8_t b){
  Adafruit_NeoPixel onboardLED(1, ON_BOARD_LED, NEO_GRB + NEO_KHZ800);
  onboardLED.begin();
  onboardLED.setPixelColor(0, onboardLED.Color(r, g, b));
  onboardLED.show();
}

void GetWritePWM(float *thrust) {
  for (int i = 0; i < THRUSTER_NUM; i++) {
    float x = thrust[i];
    if (i < 3) { // CCW
      if (-0.4 < x && x < 0.4) {
        thrust_pwm[i] = 1500;
      } else if (x > 0) {
        thrust_pwm[i] = round(CCW_POSITIVE[0] * pow(x, 3) + CCW_POSITIVE[1] * pow(x, 2) + CCW_POSITIVE[2] * x + CCW_POSITIVE[3]);
      } else {
        thrust_pwm[i] = round(CCW_NEGATIVE[0] * pow(x, 3) + CCW_NEGATIVE[1] * pow(x, 2) + CCW_NEGATIVE[2] * x + CCW_NEGATIVE[3]);
      }
    } else { // CW
      if (-0.4 < x && x < 0.4) {
        thrust_pwm[i] = 1500;
      } else if (x > 0) {
        thrust_pwm[i] = round(CW_POSITIVE[0] * pow(x, 3) + CW_POSITIVE[1] * pow(x, 2) + CW_POSITIVE[2] * x + CW_POSITIVE[3]);
      } else {
        thrust_pwm[i] = round(CW_NEGATIVE[0] * pow(x, 3) + CW_NEGATIVE[1] * pow(x, 2) + CW_NEGATIVE[2] * x + CW_NEGATIVE[3]);
      }
    }
    escs[i].writeMicroseconds(thrust_pwm[i]);
  }
}

void LimitThrust() {
  for (int i = 0; i < THRUSTER_NUM; i++) {
    if (thrust_value[i] > LIMIT_SINGLE_FORCE) thrust_value[i] = LIMIT_SINGLE_FORCE;
    if (thrust_value[i] < -LIMIT_SINGLE_FORCE) thrust_value[i] = -LIMIT_SINGLE_FORCE;
  }
  float sum = 0;
  for (int i = 0; i < THRUSTER_NUM; i++) sum += abs(thrust_value[i]);

  float rate = sum / LIMIT_TOTAL_FORCE;
  if (rate > 1) {
    for (int i = 0; i < THRUSTER_NUM; i++) thrust_value[i] /= rate;
  }
}