// main.cpp
#include <Arduino.h>
#include <Servo.h>
#include <Adafruit_NeoPixel.h>

#include "config.hpp"

std::array<Servo, 6> escs;
std::array<Servo, 2> leds;

int count = 1500;
int flag = 1;

// オンボードのRGBLEDを制御する
void onboard_led(uint8_t r, uint8_t g, uint8_t b){
  Adafruit_NeoPixel onboardLED(1, ON_BOARD_LED, NEO_GRB + NEO_KHZ800);
  onboardLED.begin();
  onboardLED.setPixelColor(0, onboardLED.Color(r, g, b));
  onboardLED.show();
}

// 複数のServoオブジェクトを初期化する
template<std::size_t N>
void init_servos(std::array<Servo, N>& servos, const std::array<uint8_t, N>& pins){
  for(size_t i = 0; i < N; i++) servos[i].attach(pins[i]);
}

// 複数のServoオブジェクトに同じ命令をする
template<std::size_t N>
void write_servos(std::array<Servo, N>& servos, int ms){
  for (auto& servo : servos) servo.writeMicroseconds(ms);
}

// サーボ信号のテスト
void test_servo(){
  count += flag;
  if (count == 2000) flag = -1;
  if (count == 1000) flag = 1;
  write_servos(escs, count);
  write_servos(leds, count);
}

// リレー信号のテスト
void test_relay(){
  digitalWrite(RELAY_SIG, HIGH);
  delay(100);
  digitalWrite(RELAY_SIG, LOW);
  delay(100);
}

// 拡張用デジタルピンのテスト
void test_digital(){
  for (auto sig : SIGS) digitalWrite(sig, HIGH);
  digitalWrite(SIG_SPARE, HIGH);
  delay(1);
  
  for (auto sig : SIGS) digitalWrite(sig, LOW);
  digitalWrite(SIG_SPARE, LOW);
  delay(1);
}


void setup() {
  Serial.begin(9600);
  onboard_led(0, 25, 0);

  // Setup PPM signal (default is 1500us)
  init_servos(escs, ESC_SIGS);
  init_servos(leds, LEDS);
  write_servos(escs, 1500);
  write_servos(leds, 1500);
  
  pinMode(RELAY_SIG, OUTPUT);
  pinMode(SIG_SPARE, OUTPUT);  // or INPUT
  for (auto pin : SIGS) pinMode(pin, OUTPUT);
}


void loop() {
  test_servo();
  // test_relay();
  // test_digital();
}