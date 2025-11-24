/*
 * RP2040 Firmware V21.x (Power / RGB / ButtonBattery Integrated)
 * - ESP32 からの UART コマンドを使用してサーボ / RGB / 電圧表示を制御
 * - 起動直後にバックアップ電池電圧を測定し、閾値で OK/NG を判定して ESP に送信
 * - RGB コマンド受信時は一時的に PIN15 の LED を全てその色にし、
 *   数秒間信号が無い場合はバッテリーバー表示に戻す
 */

#include <Arduino.h>
#include <Servo.h>
#include <Adafruit_NeoPixel.h>

extern "C"
{
#include <hardware/watchdog.h>
}

// --- Pins ---
#define UART_TX_PIN 28
#define UART_RX_PIN 29
#define PIN_STATUS_LED_1 3

#define PIN_SERVO_1 0
#define PIN_SERVO_POWER 6
#define PIN_BUZZER 7
#define PIN_HEARTBEAT 3

// NeoPixels
#define PIN_LED_EFFECT 14
#define NUM_LEDS_EFFECT 6
#define PIN_LED_STATUS 15
#define NUM_LEDS_STATUS 31

// Power Monitor Pins
#define PIN_BAT_MEASURE_TRIG 8
#define PIN_BAT_LEAK_CHK 26
#define PIN_BAT_RTC_CHK 27
#define PIN_SIG_ACT_RELAY 9
#define PIN_SIG_PC_POW 10
#define PIN_SIG_ACT_POW 11
#define PIN_SIG_LOGIC_RLY 12
#define PIN_SIG_JETSON 13

// --- ADC / 分圧設定 ---
#define ADC_REF_VOLTAGE 3.3f
#define ADC_MAX_COUNT 4095.0f

// Leak用 LR44 x3 （例：4.5V満充電想定）用の分圧抵抗
// 変更したい場合はここを書き換える
#define LEAK_DIV_R_TOP 100000.0f    // 上側抵抗 [Ω]
#define LEAK_DIV_R_BOTTOM 100000.0f // 下側抵抗 [Ω]

// RTC用 CR2032 x1 （約3V）用の分圧抵抗
#define RTC_DIV_R_TOP 100000.0f
#define RTC_DIV_R_BOTTOM 100000.0f

// 閾値 (mV)
// Leak: LR44x3 -> だいたい 4.5V 新品、3.6V を閾値例として使用
// RTC : CR2032x1 -> 約 3.0V 新品、2.5V を閾値例として使用
#define LEAK_OK_THRESHOLD_MV 3600.0f
#define RTC_OK_THRESHOLD_MV 2500.0f

// --- サーボリミット設定 ---
// 180 度コマンドはそのまま 180 度、
// SERVO_MIN_LIMIT_DEG 未満のコマンドは SERVO_MIN_LIMIT_DEG にクランプする
#define SERVO_MIN_LIMIT_DEG 60
#define SERVO_MAX_LIMIT_DEG 180

// --- LED Objects ---
Adafruit_NeoPixel effectStrip(NUM_LEDS_EFFECT, PIN_LED_EFFECT, NEO_RGB + NEO_KHZ800);
Adafruit_NeoPixel statusStrip(NUM_LEDS_STATUS, PIN_LED_STATUS, NEO_RGB + NEO_KHZ800);

Servo tiltServo;

// --- Shared State ---
volatile bool g_buzzerActive = false;
volatile int32_t g_targetServoVal = 90; // サーボ角度（度）
volatile int g_systemState = 0;         // 0:Normal, 1:Leak

// Voltage Data (mV)
volatile int32_t g_logMv = 0; // ロジック側バッテリ電圧(mV)
volatile int32_t g_actMv = 0; // アクチュエータ側バッテリ電圧(mV)

// RGB Override
volatile uint32_t g_overrideColor = 0;   // Adafruit_NeoPixel 互換 32bit Color 値
volatile uint32_t g_lastRgbUpdateMs = 0; // 最終 RGB 受信時刻[ms]

// Button Battery 判定結果（ESP には bool で送られる）
bool g_batLeakOk = false; // LR44 x3
bool g_batRtcOk = false;  // CR2032 x1

// 測定した実電圧(mV)（デバッグ用）
float g_batLeakMvMeasured = 0.0f;
float g_batRtcMvMeasured = 0.0f;

// Locals
unsigned long rxLedTimer = 0;

// Prototypes
void processIncomingSerial();
void sendStatus();
void checkBatteries();
void drawPlasma();
void drawBatteryBar();
void drawLeakAlert();
void startupAnimation();

// ==========================================================================
// ユーティリティ
// ==========================================================================
float readBatteryMv(uint8_t analogPin, float rTop, float rBottom)
{
  uint16_t raw = analogRead(analogPin);
  float vAdc = (float)raw * ADC_REF_VOLTAGE / ADC_MAX_COUNT; // ピン電圧
  float vIn = vAdc * (rTop + rBottom) / rBottom;             // 分圧前の実電圧
  return vIn * 1000.0f;                                      // [mV]
}

// ==========================================================================
// Core 0: Logic & Communication
// ==========================================================================
void setup()
{

  Serial.begin(115200);
  delay(500);
  Serial.println("=== RP2040 BOOT ===");

  watchdog_enable(4000, 1); // 起動アニメーションが長いため余裕を持たせる
  analogReadResolution(12);

  Serial1.setTX(UART_TX_PIN);
  Serial1.setRX(UART_RX_PIN);
  Serial1.begin(115200);

  pinMode(PIN_STATUS_LED_1, OUTPUT);
  pinMode(PIN_SERVO_POWER, OUTPUT);
  digitalWrite(PIN_SERVO_POWER, HIGH);
  delay(100);

  tiltServo.attach(PIN_SERVO_1);
  tiltServo.write(90);

  pinMode(PIN_BUZZER, OUTPUT);

  pinMode(PIN_SIG_ACT_RELAY, INPUT);
  pinMode(PIN_SIG_PC_POW, INPUT);
  pinMode(PIN_SIG_ACT_POW, INPUT);
  pinMode(PIN_SIG_LOGIC_RLY, INPUT);
  pinMode(PIN_SIG_JETSON, INPUT);

  pinMode(PIN_BAT_MEASURE_TRIG, OUTPUT);
  digitalWrite(PIN_BAT_MEASURE_TRIG, LOW);

  // 起動直後に一度だけボタン電池を測定
  checkBatteries();

  delay(1000);
  Serial.println("RP2040 start");
}

void loop()
{
  watchdog_update();
  processIncomingSerial();

  // UART 受信インジケータ LED
  if (millis() - rxLedTimer < 15)
    digitalWrite(PIN_STATUS_LED_1, LOW);
  else
    digitalWrite(PIN_STATUS_LED_1, HIGH);

  // サーボ角度更新
  tiltServo.write(g_targetServoVal);

  // ブザー
  if (g_buzzerActive)
    digitalWrite(PIN_BUZZER, (millis() / 200) % 2);
  else
    digitalWrite(PIN_BUZZER, LOW);

  // 約100msごとに ESP へステータス送信
  static unsigned long lastStatus = 0;
  if (millis() - lastStatus > 100)
  {
    lastStatus = millis();
    sendStatus();
  }
}

// ==========================================================================
// Core 1: Visuals
// ==========================================================================
void setup1()
{
  delay(200);
  effectStrip.begin();
  effectStrip.setBrightness(128);
  statusStrip.begin();
  statusStrip.setBrightness(255); // 起動時アニメーション用に最大

  effectStrip.show();
  statusStrip.show();

  startupAnimation();

  // 通常動作用に明るさを再設定
  effectStrip.setBrightness(128);
  statusStrip.setBrightness(255);
}

void loop1()
{
  if (g_systemState == 1)
  {
    // Leak 状態：赤点滅
    drawLeakAlert();
  }
  else
  {
    if (effectStrip.getBrightness() != 128)
      effectStrip.setBrightness(128);

    // PIN14 側は常にプラズマエフェクト
    drawPlasma();

    // PIN15 側は RGB コマンド受信から一定時間は上書き表示
    const uint32_t now = millis();
    const uint32_t RGB_OVERRIDE_MS = 3000; // 3秒間有効

    if (g_overrideColor != 0 && (now - g_lastRgbUpdateMs) < RGB_OVERRIDE_MS)
    {
      // 受信した色で全LEDを塗る
      statusStrip.setBrightness(255);
      for (int i = 0; i < NUM_LEDS_STATUS; i++)
      {
        statusStrip.setPixelColor(i, g_overrideColor);
      }
      statusStrip.show();
    }
    else
    {
      // タイムアウト or 未指定：バッテリー電圧バー表示
      drawBatteryBar();
    }

    delay(20);
  }
}

// --------------------------------------------------------------------------
// Visual Functions
// --------------------------------------------------------------------------
void startupAnimation()
{
  // Setup colors
  uint32_t c_cyan = statusStrip.Color(0, 150, 150);
  uint32_t c_blue = statusStrip.Color(0, 100, 255);
  uint32_t c_proj = statusStrip.Color(200, 255, 255);
  uint32_t c_wht = statusStrip.Color(255, 255, 255);

  // --- 1. Awakening: フェードイン ---
  for (int b = 0; b <= 120; b++)
  {
    for (int i = 0; i < NUM_LEDS_EFFECT; i++)
      effectStrip.setPixelColor(i, effectStrip.Color(0, b, b));
    for (int i = 0; i < NUM_LEDS_STATUS; i++)
      statusStrip.setPixelColor(i, statusStrip.Color(0, b, b));

    effectStrip.show();
    statusStrip.show();
    delay(20);
    watchdog_update();
  }
  delay(300);

  // 一旦暗くする
  for (int b = 120; b >= 0; b -= 2)
  {
    for (int i = 0; i < NUM_LEDS_EFFECT; i++)
      effectStrip.setPixelColor(i, effectStrip.Color(0, b, b));
    for (int i = 0; i < NUM_LEDS_STATUS; i++)
      statusStrip.setPixelColor(i, statusStrip.Color(0, b, b));
    effectStrip.show();
    statusStrip.show();
    delay(10);
  }
  delay(500);

  // --- 2. Stacking: 構築 ---
  const int CENTER = 15;

  for (int progress = 0; progress <= 15; progress++)
  {
    int distanceToTravel = 15 - progress;

    for (int p = 0; p <= distanceToTravel; p++)
    {
      statusStrip.clear();

      // 固定部分 (青)
      for (int f = 0; f < progress; f++)
        statusStrip.setPixelColor(f, c_blue);
      for (int f = 0; f < progress; f++)
        statusStrip.setPixelColor(30 - f, c_blue);

      // 飛翔パーティクル
      int pPosLeft = 15 - p;
      int pPosRight = 15 + p;
      statusStrip.setPixelColor(pPosLeft, c_proj);
      statusStrip.setPixelColor(pPosRight, c_proj);
      if (p == 0)
        statusStrip.setPixelColor(CENTER, c_proj);

      // PIN14 同期
      int effFill = map(progress, 0, 15, 0, NUM_LEDS_EFFECT);
      for (int e = 0; e < NUM_LEDS_EFFECT; e++)
      {
        if (e < effFill)
          effectStrip.setPixelColor(e, c_blue);
        else
          effectStrip.setPixelColor(e, 0);
      }
      if (effFill < NUM_LEDS_EFFECT)
        effectStrip.setPixelColor(effFill, c_proj);

      effectStrip.show();
      statusStrip.show();
      delay(35);
    }

    // インパクト後の白フラッシュ
    statusStrip.setPixelColor(progress, c_wht);
    statusStrip.setPixelColor(30 - progress, c_wht);
    statusStrip.show();
    delay(60);
    watchdog_update();
  }

  // --- 3. Final Flash ---
  delay(100);
  for (int i = 0; i < NUM_LEDS_STATUS; i++)
    statusStrip.setPixelColor(i, c_wht);
  for (int i = 0; i < NUM_LEDS_EFFECT; i++)
    effectStrip.setPixelColor(i, c_wht);
  statusStrip.show();
  effectStrip.show();

  delay(400);

  // フェードアウト
  for (int b = 255; b >= 0; b -= 10)
  {
    for (int i = 0; i < NUM_LEDS_STATUS; i++)
      statusStrip.setPixelColor(i, statusStrip.Color(b, b, b));
    for (int i = 0; i < NUM_LEDS_EFFECT; i++)
      effectStrip.setPixelColor(i, effectStrip.Color(b, b, b));
    statusStrip.show();
    effectStrip.show();
    delay(5);
  }
  statusStrip.clear();
  effectStrip.clear();
  statusStrip.show();
  effectStrip.show();
  delay(200);
}

void drawLeakAlert()
{
  statusStrip.setBrightness(255);
  effectStrip.setBrightness(255);

  static bool state = false;
  uint32_t color = state ? 0xFF0000 : 0x000000;

  for (int i = 0; i < NUM_LEDS_EFFECT; i++)
    effectStrip.setPixelColor(i, color);
  effectStrip.show();

  for (int i = 0; i < NUM_LEDS_STATUS; i++)
    statusStrip.setPixelColor(i, color);
  statusStrip.show();

  state = !state;
  delay(100);
}

void drawBatteryBar()
{
  statusStrip.clear();

  // ブレス風フェード
  float breath = (sin(millis() / 2500.0 * PI) + 1.0) / 2.0;
  int dynamicBrightness = 40 + (int)(breath * 215);
  statusStrip.setBrightness(dynamicBrightness);

  bool blinkOn = (millis() / 250) % 2;

  auto getPos = [](int32_t mv) -> float
  {
    if (mv < 1000)
      return 0.0f;
    int cells = (mv > 18000) ? 6 : 4;
    float minV = cells * 3.5f * 1000.0f;
    float maxV = cells * 4.2f * 1000.0f;
    float pct = (float)(mv - minV) / (maxV - minV);
    if (pct < 0.0f)
      pct = 0.0f;
    if (pct > 1.0f)
      pct = 1.0f;
    return pct;
  };

  float logPct = getPos(g_logMv);
  float actPct = getPos(g_actMv);

  const int SIDE_LEDS = 15;
  const int CENTER_IDX = 15;
  const int ACT_START = 16;
  const int ACT_END = 30;

  // Logic Side (0〜14)
  if (logPct < 0.1f && g_logMv > 1000)
  {
    if (blinkOn)
    {
      for (int i = 0; i < SIDE_LEDS; i++)
        statusStrip.setPixelColor(i, 0xFFFF00);
    }
  }
  else
  {
    for (int i = 0; i < SIDE_LEDS; i++)
    {
      int hue = map(i, 0, SIDE_LEDS, 0, 21845);
      statusStrip.setPixelColor(i, statusStrip.ColorHSV(hue, 255, 255));
    }
    int logIdx = (int)(logPct * (float)(SIDE_LEDS - 1));
    if (logIdx >= SIDE_LEDS)
      logIdx = SIDE_LEDS - 1;
    statusStrip.setPixelColor(logIdx, 0x0000FF);
  }

  // Actuator Side (16〜30)
  if (actPct < 0.1f && g_actMv > 1000)
  {
    if (blinkOn)
    {
      for (int i = ACT_START; i <= ACT_END; i++)
        statusStrip.setPixelColor(i, 0xFFFF00);
    }
  }
  else
  {
    for (int i = ACT_START; i <= ACT_END; i++)
    {
      int hue = map(i, ACT_START, ACT_END, 21845, 0);
      statusStrip.setPixelColor(i, statusStrip.ColorHSV(hue, 255, 255));
    }
    int actIdx = ACT_END - (int)(actPct * (float)(SIDE_LEDS - 1));
    if (actIdx < ACT_START)
      actIdx = ACT_START;
    statusStrip.setPixelColor(actIdx, 0x0000FF);
  }

  // センターは白
  statusStrip.setPixelColor(CENTER_IDX, 0xFFFFFF);

  statusStrip.show();
}

void drawPlasma()
{
  unsigned long t = millis();
  for (int i = 0; i < NUM_LEDS_EFFECT; i++)
  {
    float wave = (sin((t / 1000.0) + i * 0.5) + sin((t / 300.0) - i * 0.3) + 2.0) / 4.0;
    long hue = 43000 + (long)(wave * 10000) - 5000;
    int val = (int)(pow(wave, 3) * 255);
    if (val < 30)
      val = 30;
    effectStrip.setPixelColor(i, effectStrip.ColorHSV(hue, 255 - (val / 4), val));
  }
  effectStrip.show();
}

// ==========================================================================
// Core 0 Helpers
// ==========================================================================
void checkBatteries()
{
  // バックアップ電池確認スイッチ ON
  digitalWrite(PIN_BAT_MEASURE_TRIG, HIGH);
  delay(20);

  // 電圧測定（mV）
  g_batLeakMvMeasured = readBatteryMv(PIN_BAT_LEAK_CHK, LEAK_DIV_R_TOP, LEAK_DIV_R_BOTTOM);
  g_batRtcMvMeasured = readBatteryMv(PIN_BAT_RTC_CHK, RTC_DIV_R_TOP, RTC_DIV_R_BOTTOM);

  // スイッチ OFF
  digitalWrite(PIN_BAT_MEASURE_TRIG, LOW);

  // 閾値で OK/NG 判定（ESP には bool で送る）
  g_batLeakOk = (g_batLeakMvMeasured >= LEAK_OK_THRESHOLD_MV);
  g_batRtcOk = (g_batRtcMvMeasured >= RTC_OK_THRESHOLD_MV);

  // デバッグ表示
  Serial.print("[BAT] Leak(LR44x3) = ");
  Serial.print(g_batLeakMvMeasured);
  Serial.print(" mV -> ");
  Serial.println(g_batLeakOk ? "OK" : "LOW");

  Serial.print("[BAT] RTC(CR2032) = ");
  Serial.print(g_batRtcMvMeasured);
  Serial.print(" mV -> ");
  Serial.println(g_batRtcOk ? "OK" : "LOW");
}

void processIncomingSerial()
{
  while (Serial1.available())
  {
    rxLedTimer = millis();

    if (Serial1.read() == '<')
    {
      delay(1);
      if (Serial1.available() >= 5)
      {
        char cmd = Serial1.read();
        int32_t val = 0;
        Serial1.readBytes((char *)&val, 4);
        char end = Serial1.read();

        if (end == '>')
        {
          // 受信内容を USB シリアルに表示
          Serial.print("[RX] cmd=");
          Serial.print(cmd);
          Serial.print(" val=");
          Serial.println(val);

          switch (cmd)
          {
          // サーボ角度（度）: 下限 SERVO_MIN_LIMIT_DEG, 上限 SERVO_MAX_LIMIT_DEG
          case 'T':
          {
            int32_t limited = val;
            if (limited < SERVO_MIN_LIMIT_DEG)
              limited = SERVO_MIN_LIMIT_DEG;
            if (limited > SERVO_MAX_LIMIT_DEG)
              limited = SERVO_MAX_LIMIT_DEG;
            g_targetServoVal = (int)limited;
            break;
          }

          // RGB カラー（Adafruit_NeoPixel 互換 32bit Colorを想定）
          // PIN15 の全LEDをこの色で塗り、一定時間はバッテリバーを上書き
          case 'R':
            g_overrideColor = (uint32_t)val;
            g_lastRgbUpdateMs = millis();
            break;

          // Leak 状態通知（ESP→RP）
          case 'L':
            g_systemState = 1;
            g_buzzerActive = true; // 漏水時はブザーON
            break;

          case 'N':
            g_systemState = 0;
            g_buzzerActive = false;
            break;

          // ロジック / アクチュエータ電圧 (mV)
          case 'V':
            g_logMv = val;
            break;

          case 'A':
            g_actMv = val;
            break;

          // ブザー制御 (ESP側が 'B' / 'Q' を送る場合に対応)
          case 'B':
            g_buzzerActive = (val != 0);
            break;

          case 'Q':
            g_buzzerActive = false;
            break;

          // 電流など追加コマンドが来てもここで拡張可能
          default:
            // 未使用コマンド
            break;
          }
        }
        else
        {
          Serial.println("[RX] invalid frame (no '>')");
        }
      }
    }
  }
}

void sendStatus()
{
  // bit0: Leak電池 OK (LR44x3)
  // bit1: RTC電池 OK  (CR2032x1)
  // bit2: Jetson_POWER_Sig
  // bit3: Actuator_power_Sig
  // bit4: Logic_relay_Sig
  // bit5: PC_Power_Sig
  // bit6: Actuator_Relay_Sig
  byte s = 0;
  if (g_batLeakOk)
    s |= (1 << 0);
  if (g_batRtcOk)
    s |= (1 << 1);
  if (digitalRead(PIN_SIG_JETSON))
    s |= (1 << 2);
  if (digitalRead(PIN_SIG_ACT_POW))
    s |= (1 << 3);
  if (digitalRead(PIN_SIG_LOGIC_RLY))
    s |= (1 << 4);
  if (digitalRead(PIN_SIG_PC_POW))
    s |= (1 << 5);
  if (digitalRead(PIN_SIG_ACT_RELAY))
    s |= (1 << 6);

  Serial1.write('<');
  Serial1.write(s);
  Serial1.write('>');
}
