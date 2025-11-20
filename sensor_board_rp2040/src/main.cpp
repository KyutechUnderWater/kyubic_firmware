// com5  2025/11/19
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>

// --- I2Cアドレス設定 (ESP32側の定義名と合わせてもOK) ---
#define RP2040_ADDRESS 0x08 

// --- NeoPixel 設定 (PDFピン14) ---
#define NEOPIXEL_PIN 14
Adafruit_NeoPixel statusPixel(1, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// --- バッテリーチェック用ピン設定 ---
#define BATT_ENABLE_PIN 8     // バックアップボルテージシグナル
#define BATT_LEAK_ADC_PIN 26  // 漏水センサ用バックアップ電池
#define BATT_RTC_ADC_PIN  27  // RTC用バックアップ電池

// --- ブザー用ピン設定 ---
#define BUZZER_PIN 7 

// --- 設定値 ---
const float ADC_CONVERSION_FACTOR = 3.3 / 4095.0;
const float BATT_CHANGE_THRESHOLD_V = 2.5; // 交換基準電圧

// --- タイマー用変数 ---
unsigned long lastLoopTime = 0;
const unsigned long LOOP_INTERVAL_MS = 10; // 10ms間隔 (100Hz) で監視

// --- フラグ ---
volatile bool buzzerShouldBeOn = false; // I2C割り込みで操作されるフラグ

//ステータス保存用 (0=正常, 1=異常)
volatile uint8_t systemStatus = 0;

// ＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝
// --- 関数定義 ---
// ＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝

// 起動時にバックアップ電池の電圧をチェックする関数
void checkBatteryStatus() {
  Serial.println("Checking backup batteries...");
  pinMode(BATT_ENABLE_PIN, OUTPUT);

  // 1. 計測回路ON
  digitalWrite(BATT_ENABLE_PIN, HIGH);
  delay(10); // ADC安定待ち (setup内なのでdelayでOK)

  // 2. 電圧読み取り
  int adcLeak = analogRead(BATT_LEAK_ADC_PIN);
  int adcRTC = analogRead(BATT_RTC_ADC_PIN);
  Serial.println(adcLeak);
  Serial.println(adcRTC);
  // 3. 計測回路OFF (必須)
  digitalWrite(BATT_ENABLE_PIN, LOW); 

  // 4. 電圧変換
  float voltageLeak = adcLeak * ADC_CONVERSION_FACTOR;
  float voltageRTC = adcRTC * ADC_CONVERSION_FACTOR;

  Serial.print("  [BATT] Leak: "); Serial.print(voltageLeak); Serial.println(" V");
  Serial.print("  [BATT] RTC:  "); Serial.print(voltageRTC); Serial.println(" V");

  // 5. 判定とフラグ作成 (ビット演算)
  systemStatus = 0; // まずリセット

  // ビット0: リーク用電池 (1 = 異常)
  if (voltageLeak < BATT_CHANGE_THRESHOLD_V) {
    systemStatus |= 0x01; // 00000001 を足す
    Serial.println("  [BATT] Leak Battery LOW!");
  }

  // ビット1: RTC用電池 (1 = 異常)
  if (voltageRTC < BATT_CHANGE_THRESHOLD_V) {
    systemStatus |= 0x02; // 00000010 を足す
    Serial.println("  [BATT] RTC Battery LOW!");
  }

  // LED表示 (どちらか一方でも異常なら赤)
  if (systemStatus > 0) {
    statusPixel.setPixelColor(0, statusPixel.Color(255, 0, 0)); // 赤
  } else {
    Serial.println("  [BATT] All Batteries OK.");
    statusPixel.setPixelColor(0, statusPixel.Color(0, 255, 0)); // 緑
  }
  statusPixel.show();
}

//ESPからリクエスト来た時に呼ばれる関数
void requestEvent() {
  // ステータスバイトを送信 (0=OK, 1=BattLow)
  Wire.write(systemStatus); 
}

// I2Cデータ受信時の割り込み関数
void receiveEvent(int howMany) {
  while (Wire.available()) {
    char c = Wire.read(); 
    if (c == 'L') {      // 'L' = Leak (鳴らす)
      buzzerShouldBeOn = true;
    } else if (c == 'S') { // 'S' = Safe (止める)
      buzzerShouldBeOn = false;
    }
  }
}

// ＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝
// --- メイン処理 ---
// ＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);    //消す必要あり　　デバック用
  // NeoPixel初期化
  statusPixel.begin();
  statusPixel.clear();
  statusPixel.show();
  
  // ブザー
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  // 起動時バッテリーチェック
  checkBatteryStatus();
  
  // I2Cスレーブ開始 (ESP32と同じ定義名を使用)
  Wire.begin(RP2040_ADDRESS);
  Wire.onReceive(receiveEvent); // 受信
  Wire.onRequest(requestEvent); // 送信
  
  Serial.println("RP2040 Ready.");
}

void loop() {
  // ノンブロッキング・タイマー (10msごとに実行)
  if (millis() - lastLoopTime > LOOP_INTERVAL_MS) {
    lastLoopTime = millis();

    /*// フラグの状態に合わせてブザーを制御
    if (buzzerShouldBeOn) {
      digitalWrite(BUZZER_PIN, HIGH);
    } else {
      digitalWrite(BUZZER_PIN, LOW);
    }
    //Serial.println("buzzer_test");*/
  }
}