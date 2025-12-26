/*
  Thruster control with Nanopb (driver_msgs/Actuator)
  Updated for specific proto definition and Serial safety check.
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
// ※ .protoファイルから生成されたヘッダファイルのパスに合わせて変更してください
#include "proto/driver_msgs__Actuator.pb.h"

// --- メッセージ型のエイリアス定義 ---
// 生成される構造体名は環境によりますが、提示されたパッケージ名に基づき定義
using ActuatorMsg = protolink__driver_msgs__Actuator_driver_msgs__Actuator;

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
int led_pwm[2] = { 1100, 1100 }; // LED制御用 (Right, Left)

unsigned long last_time = 0;
bool is_update = false;

// ハードウェアオブジェクト
std::array<Servo, 6> escs;
std::array<Servo, 2> leds;

// --- 関数プロトタイプ ---
// void onboard_led(uint8_t r, uint8_t g, uint8_t b);
void GetWritePWM(float *thrust);
void WriteLEDs();
void LimitThrust();
void StopAllActuators(); // 全停止用
void StopThrusters();
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
  ActuatorMsg message = protolink__driver_msgs__Actuator_driver_msgs__Actuator_init_zero;
  pb_istream_t stream = pb_istream_from_buffer(buffer, size);
  
  bool status = pb_decode(&stream, protolink__driver_msgs__Actuator_driver_msgs__Actuator_fields, &message);

  if (status)
  {
    // --- Thrusters (driver_msgs__Thruster) ---
    // has_thrustersフラグがあれば値を読み込む
    if (message.has_thrusters) {
      // optionalフィールドのため、has_thrXを確認するのが確実ですが、
      // ここでは簡易的に値が存在すれば更新、なければ0とみなすか、前の値を保持するか
      // Nanopbのinit_zeroで0初期化されているため、値が入っていればそのまま使います
      
      // 提示された定義に基づき各スラスター値を取得
      thrust_value[0] = message.thrusters.thr1;
      thrust_value[1] = message.thrusters.thr2;
      thrust_value[2] = message.thrusters.thr3;
      thrust_value[3] = message.thrusters.thr4;
      thrust_value[4] = message.thrusters.thr5;
      thrust_value[5] = message.thrusters.thr6;
    }

    // --- LEDs (driver_msgs__LED) ---
    if (message.has_leds) {
       // LED制御値 (int32) を取得。
       // ここではPWM値(1100-1900等)が直接送られてくると想定して格納
       led_pwm[0] = message.leds.right; // leds[0] -> Right
       led_pwm[1] = message.leds.left;  // leds[1] -> Left
    }

    is_update = true;
    last_time = millis();
  }
  else
  {
    // デコードエラー
  }
}

// --- セットアップ ---
void setup() {
  // onboardLED.begin();

  // 変更: Serialを開始し、PacketSerialにStreamをセット
  Serial.begin(115200);
  myPacketSerial.setStream(&Serial);
  myPacketSerial.setPacketHandler(&onPacketReceived);

  init_servos(escs, ESC_SIGS);
  init_servos(leds, LEDS);
  
  StopAllActuators(); // 初期化時は停止

  pinMode(RELAY_SIG, OUTPUT);
  pinMode(SIG_SPARE, OUTPUT);
  for (auto pin : SIGS) pinMode(pin, OUTPUT);

  delay(1000);
}

// --- メインループ ---
void loop() {
  // 追加: Serial接続確認
  // USB接続が切れた場合など、Serialが無効なら停止させる
  if (!Serial) {
    StopAllActuators();
    return; // 通信処理をスキップ
  }

  // パケット受信処理
  myPacketSerial.update();

  // タイムアウト監視
  if (SERIAL_TIMEOUT < (millis() - last_time)) {
    StopThrusters();
    is_update = true; // PWM書き込みを実行させるため
  }

  // アクチュエータ更新
  if (is_update) {
    // LimitThrust();             // 推力制限
    GetWritePWM(thrust_value); // 推力 -> PWM変換と書き込み
    WriteLEDs();               // LED PWM書き込み
    is_update = false;
  }
}

// --- ヘルパー関数 ---

// 全アクチュエータを安全な値（停止/中立）にする
void StopAllActuators() {
  for (int i = 0; i < THRUSTER_NUM; i++) {
    thrust_value[i] = 0.0f;
  }
  led_pwm[0] = 1100;
  led_pwm[1] = 1100;
  GetWritePWM(thrust_value);
  WriteLEDs();
}

void StopThrusters() {
  for (int i = 0; i < THRUSTER_NUM; i++) {
    thrust_value[i] = 0.0f;
  }
  GetWritePWM(thrust_value);
}

void WriteLEDs() {
  // LEDへのPWM出力
  // limit (1100 ~ 1900)
  led_pwm[0] = max(1100, min(1900, led_pwm[0]));
  led_pwm[1] = max(1100, min(1900, led_pwm[1]));
  leds[0].writeMicroseconds(led_pwm[0]);
  leds[1].writeMicroseconds(led_pwm[1]);
}

void GetWritePWM(float *thrust) {
  for (int i = 0; i < THRUSTER_NUM; i++) {
    float x = thrust[i];
    if (i < 3) { // CCW (0, 1, 2)
      if (-0.4 < x && x < 0.4) {
        thrust_pwm[i] = 1500;
      } else if (x > 0) {
        thrust_pwm[i] = round(CCW_POSITIVE[0] * pow(x, 3) + CCW_POSITIVE[1] * pow(x, 2) + CCW_POSITIVE[2] * x + CCW_POSITIVE[3]);
      } else {
        thrust_pwm[i] = round(CCW_NEGATIVE[0] * pow(x, 3) + CCW_NEGATIVE[1] * pow(x, 2) + CCW_NEGATIVE[2] * x + CCW_NEGATIVE[3]);
      }
    } else { // CW (3, 4, 5)
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