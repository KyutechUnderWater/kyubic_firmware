/*
 * RP2040 Firmware V22.5 (Voltage Smoothing Added)
 * - Pin14 Effect LED: 18 Cool/Calm Patterns
 * - Pin15 Status LED: Battery Monitor with Moving Average Filter
 * (Prevents flickering caused by sudden voltage drops)
 * - UART Control & Servo Logic included
 */

#include <Arduino.h>
#include <Servo.h>
#include <Adafruit_NeoPixel.h>

extern "C"
{
#include <hardware/watchdog.h>
}

// ==========================================================================
// PINS & DEFINITIONS
// ==========================================================================

// --- UART / System ---
#define UART_TX_PIN 28
#define UART_RX_PIN 29
#define PIN_STATUS_LED_1 3
#define PIN_BUZZER 7
#define PIN_HEARTBEAT 3

// --- Servo ---
#define PIN_SERVO_1 0
#define PIN_SERVO_POWER 6
#define SERVO_MIN_LIMIT_DEG 50
#define SERVO_MAX_LIMIT_DEG 180

// --- Power Monitor Inputs ---
#define PIN_BAT_MEASURE_TRIG 8
#define PIN_BAT_LEAK_CHK 26
#define PIN_BAT_RTC_CHK 27
#define PIN_SIG_ACT_RELAY 9
#define PIN_SIG_PC_POW 10
#define PIN_SIG_ACT_POW 11
#define PIN_SIG_LOGIC_RLY 12
#define PIN_SIG_JETSON 13

// --- NeoPixels ---
#define PIN_LED_EFFECT 14
#define NUM_LEDS_EFFECT 6
#define PIN_LED_STATUS 15
#define NUM_LEDS_STATUS 31

// --- ADC / Voltage Divider ---
#define ADC_REF_VOLTAGE 3.3f
#define ADC_MAX_COUNT 4095.0f

#define LEAK_DIV_R_TOP 100000.0f
#define LEAK_DIV_R_BOTTOM 100000.0f
#define LEAK_OK_THRESHOLD_MV 3600.0f

#define RTC_DIV_R_TOP 100000.0f
#define RTC_DIV_R_BOTTOM 100000.0f
#define RTC_OK_THRESHOLD_MV 2500.0f

// --- Effect Settings ---
#define EFFECT_COUNT 18
#define EFFECT_SWITCH_INTERVAL_MS 15000

// --- Moving Average Settings ---
#define MA_WINDOW_SIZE 8 // 平均化するサンプル数（大きくすると安定するが遅延が増える）

// ==========================================================================
// OBJECTS & GLOBALS
// ==========================================================================

Adafruit_NeoPixel effectStrip(NUM_LEDS_EFFECT, PIN_LED_EFFECT, NEO_RGB + NEO_KHZ800);
Adafruit_NeoPixel statusStrip(NUM_LEDS_STATUS, PIN_LED_STATUS, NEO_RGB + NEO_KHZ800);

Servo tiltServo;

// Shared State
volatile bool g_buzzerActive = false;
volatile int32_t g_targetServoVal = 90;
volatile int g_systemState = 0; // 0:Normal, 1:Leak

// Voltage Data (Smoothed)
volatile int32_t g_logMv = 0;
volatile int32_t g_actMv = 0;

// Moving Average Buffers
int32_t g_logHistory[MA_WINDOW_SIZE];
int32_t g_actHistory[MA_WINDOW_SIZE];
int g_logHistoryIdx = 0;
int g_actHistoryIdx = 0;
int32_t g_logSum = 0;
int32_t g_actSum = 0;
bool g_logInitialized = false;
bool g_actInitialized = false;

// RGB Override (Pin15)
volatile uint32_t g_overrideColor = 0;
volatile uint32_t g_lastRgbUpdateMs = 0;

// Battery Status
bool g_batLeakOk = false;
bool g_batRtcOk = false;
float g_batLeakMvMeasured = 0.0f;
float g_batRtcMvMeasured = 0.0f;

// Timers
unsigned long rxLedTimer = 0;

// Effect Control (Core 1)
int g_currentEffectIdx = 0;
unsigned long g_lastEffectChangeMs = 0;

// ==========================================================================
// PROTOTYPES
// ==========================================================================
void processIncomingSerial();
void sendStatus();
void checkBatteries();
float readBatteryMv(uint8_t analogPin, float rTop, float rBottom);
int32_t updateMovingAvg(int32_t newVal, int32_t *history, int *idx, int32_t *sum, bool *initialized);

// Visuals
void startupAnimation();
void drawBatteryBar();
void drawLeakAlert();

// Effect Patterns
void drawPlasma();
void drawRainbowCycle();
void drawBreathing();
void drawSparkle();
void drawComet();
void drawBlueScanner();
void drawOcean();
void drawAurora();
void drawIceShimmer();
void drawPingPong();
void drawMeteorRain();
void drawStacking();
void drawBioPulse();
void drawTheaterChase();
void drawDoubleScan();
void drawTwinkleMint();
void drawCyberFade();
void drawSoftWhite();

// ==========================================================================
// CORE 0: LOGIC & COMMUNICATION
// ==========================================================================
void setup()
{
  Serial.begin(115200);
  delay(500);
  Serial.println("=== RP2040 BOOT V22.5 (Voltage Smoothing) ===");

  watchdog_enable(4000, 1);
  analogReadResolution(12);

  // Buffer Init
  memset(g_logHistory, 0, sizeof(g_logHistory));
  memset(g_actHistory, 0, sizeof(g_actHistory));

  randomSeed(analogRead(26) + millis());

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

  checkBatteries();
  delay(1000);
  Serial.println("Core 0 Ready");
}

void loop()
{
  watchdog_update();
  processIncomingSerial();

  if (millis() - rxLedTimer < 15)
    digitalWrite(PIN_STATUS_LED_1, LOW);
  else
    digitalWrite(PIN_STATUS_LED_1, HIGH);

  tiltServo.write(g_targetServoVal);

  if (g_buzzerActive)
    digitalWrite(PIN_BUZZER, (millis() / 200) % 2);
  else
    digitalWrite(PIN_BUZZER, LOW);

  static unsigned long lastStatus = 0;
  if (millis() - lastStatus > 100)
  {
    lastStatus = millis();
    sendStatus();
  }
}

// ==========================================================================
// CORE 1: VISUALS
// ==========================================================================
void setup1()
{
  delay(200);
  effectStrip.begin();
  effectStrip.setBrightness(128);
  statusStrip.begin();
  statusStrip.setBrightness(255);

  effectStrip.show();
  statusStrip.show();

  startupAnimation();

  effectStrip.setBrightness(128);
  statusStrip.setBrightness(255);
  

  g_lastEffectChangeMs = millis();
  g_currentEffectIdx = 0;
}

void loop1()
{
  if (g_systemState == 1)
  {
    drawLeakAlert();
  }
  else
  {
    unsigned long now = millis();
    if (now - g_lastEffectChangeMs > EFFECT_SWITCH_INTERVAL_MS)
    {
      g_lastEffectChangeMs = now;
      int nextIdx = random(0, EFFECT_COUNT);
      if (nextIdx == g_currentEffectIdx)
      {
        nextIdx = (nextIdx + 1) % EFFECT_COUNT;
      }
      g_currentEffectIdx = nextIdx;
    }

    if (effectStrip.getBrightness() != 128)
      effectStrip.setBrightness(128);

    switch (g_currentEffectIdx)
    {
    case 0:
      drawPlasma();
      break;
    case 1:
      drawRainbowCycle();
      break;
    case 2:
      drawBreathing();
      break;
    case 3:
      drawSparkle();
      break;
    case 4:
      drawComet();
      break;
    case 5:
      drawBlueScanner();
      break;
    case 6:
      drawOcean();
      break;
    case 7:
      drawAurora();
      break;
    case 8:
      drawIceShimmer();
      break;
    case 9:
      drawPingPong();
      break;
    case 10:
      drawMeteorRain();
      break;
    case 11:
      drawStacking();
      break;
    case 12:
      drawBioPulse();
      break;
    case 13:
      drawTheaterChase();
      break;
    case 14:
      drawDoubleScan();
      break;
    case 15:
      drawTwinkleMint();
      break;
    case 16:
      drawCyberFade();
      break;
    case 17:
      drawSoftWhite();
      break;
    default:
      drawPlasma();
      break;
    }

    const uint32_t RGB_OVERRIDE_MS = 3000;
    if (g_overrideColor != 0 && (now - g_lastRgbUpdateMs) < RGB_OVERRIDE_MS)
    {
      statusStrip.setBrightness(255);
      for (int i = 0; i < NUM_LEDS_STATUS; i++)
      {
        statusStrip.setPixelColor(i, g_overrideColor);
      }
      statusStrip.show();
    }
    else
    {
      drawBatteryBar();
    }

    delay(20);
  }
}

// ==========================================================================
// HELPERS (Smoothing)
// ==========================================================================

// Updates circular buffer and returns moving average
int32_t updateMovingAvg(int32_t newVal, int32_t *history, int *idx, int32_t *sum, bool *initialized)
{
  // First time initialization: Fill buffer with the first value to avoid ramp-up
  if (!(*initialized))
  {
    for (int i = 0; i < MA_WINDOW_SIZE; i++)
    {
      history[i] = newVal;
    }
    *sum = newVal * MA_WINDOW_SIZE;
    *initialized = true;
    return newVal;
  }

  // Update sum: subtract oldest, add newest
  *sum -= history[*idx];
  history[*idx] = newVal;
  *sum += newVal;

  // Advance index
  *idx = (*idx + 1) % MA_WINDOW_SIZE;

  // Return Average
  return *sum / MA_WINDOW_SIZE;
}

// ==========================================================================
// DATA PROCESSING
// ==========================================================================

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
          switch (cmd)
          {
          case 'T':
          {
            int32_t limited = val;
            if (limited < SERVO_MIN_LIMIT_DEG)
              limited = SERVO_MIN_LIMIT_DEG;
            if (limited > SERVO_MAX_LIMIT_DEG)
              limited = SERVO_MAX_LIMIT_DEG;
            g_targetServoVal = (int)limited;
          }
          break;
          case 'R':
            g_overrideColor = (uint32_t)val;
            g_lastRgbUpdateMs = millis();
            break;
          case 'L':
            g_systemState = 1;
            g_buzzerActive = true;
            break;
          case 'N':
            g_systemState = 0;
            g_buzzerActive = false;
            break;

          // Voltage: Update via Moving Average
          case 'V':
            g_logMv = updateMovingAvg(val, g_logHistory, &g_logHistoryIdx, &g_logSum, &g_logInitialized);
            break;
          case 'A':
            g_actMv = updateMovingAvg(val, g_actHistory, &g_actHistoryIdx, &g_actSum, &g_actInitialized);
            break;

          case 'B':
            g_buzzerActive = (val != 0);
            break;
          case 'Q':
            g_buzzerActive = false;
            break;
          }
        }
      }
    }
  }
}

// ==========================================================================
// PIN 15: BATTERY BAR & STATUS
// ==========================================================================

void drawBatteryBar()
{
  statusStrip.clear();

  float breath = (sin(millis() / 2500.0 * PI) + 1.0) / 2.0;
  float waitBreath = (sin(millis() / 1000.0 * PI) + 1.0) / 2.0;
  int dynamicBrightness = 40 + (int)(breath * 215);
  statusStrip.setBrightness(dynamicBrightness);

  auto getPos = [](int32_t mv) -> float
  {
    if (mv < 1000)
      return 0.0f;
    int cells = (mv > 18000) ? 6 : 4;
    float minV = cells * 3.35f * 1000.0f;
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

  // --- Logic Side (0-14) ---
  if (g_logMv < 1000)
  {
    for (int i = 0; i < SIDE_LEDS; i++)
      statusStrip.setPixelColor(i, (int)(150 * waitBreath), 0, (int)(255 * waitBreath));
  }
  else if (logPct < 0.1f)
  {
    int blinkInterval = 50 + (int)(logPct * 4500);
    bool blinkOn = (millis() / blinkInterval) % 2;
    int gVal = (int)(255.0 * (logPct / 0.10));
    if (gVal < 0)
      gVal = 0;
    if (gVal > 255)
      gVal = 255;
    if (blinkOn)
      for (int i = 0; i < SIDE_LEDS; i++)
        statusStrip.setPixelColor(i, 255, gVal, 0);
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

  // --- Actuator Side (16-30) ---
  if (g_actMv < 1000)
  {
    for (int i = ACT_START; i <= ACT_END; i++)
      statusStrip.setPixelColor(i, (int)(150 * waitBreath), 0, (int)(255 * waitBreath));
  }
  else if (actPct < 0.1f)
  {
    int blinkInterval = 50 + (int)(actPct * 4500);
    bool blinkOn = (millis() / blinkInterval) % 2;
    int gVal = (int)(255.0 * (actPct / 0.10));
    if (gVal < 0)
      gVal = 0;
    if (gVal > 255)
      gVal = 255;
    if (blinkOn)
      for (int i = ACT_START; i <= ACT_END; i++)
        statusStrip.setPixelColor(i, 255, gVal, 0);
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

  int centerVal = 100 + (int)(breath * 155);
  statusStrip.setPixelColor(CENTER_IDX, centerVal, centerVal, centerVal);
  statusStrip.show();
}

// ==========================================================================
// PIN 14: EFFECT PATTERNS (SAFE & COOL)
// ==========================================================================
// (Effects functions are same as V22.4 - Omitted for brevity, but include all 18 functions here in actual compile)
// Copy-paste effects from previous V22.4 code:
// drawPlasma, drawRainbowCycle, drawBreathing, drawSparkle, drawComet,
// drawBlueScanner, drawOcean, drawAurora, drawIceShimmer, drawPingPong,
// drawMeteorRain, drawStacking, drawBioPulse, drawTheaterChase,
// drawDoubleScan, drawTwinkleMint, drawCyberFade, drawSoftWhite

// ... [PASTE 18 EFFECTS HERE] ...

// 0. Plasma
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
// 1. Rainbow
void drawRainbowCycle()
{
  unsigned long t = millis();
  long firstPixelHue = (t * 256 / 20);
  for (int i = 0; i < NUM_LEDS_EFFECT; i++)
  {
    int pixelHue = firstPixelHue + (i * 65536L / NUM_LEDS_EFFECT);
    effectStrip.setPixelColor(i, effectStrip.ColorHSV(pixelHue, 255, 255));
  }
  effectStrip.show();
}
// 2. Breathing
void drawBreathing()
{
  unsigned long t = millis();
  float val = (exp(sin(t / 2000.0 * PI)) - 0.36787944) * 108.0;
  if (val < 10)
    val = 10;
  if (val > 255)
    val = 255;
  uint32_t color = effectStrip.ColorHSV(35000, 255, (int)val);
  for (int i = 0; i < NUM_LEDS_EFFECT; i++)
    effectStrip.setPixelColor(i, color);
  effectStrip.show();
}
// 3. Sparkle
void drawSparkle()
{
  for (int i = 0; i < NUM_LEDS_EFFECT; i++)
  {
    uint32_t c = effectStrip.getPixelColor(i);
    uint8_t r = (uint8_t)(c >> 16);
    uint8_t g = (uint8_t)(c >> 8);
    uint8_t b = (uint8_t)c;
    r = (uint8_t)(r * 0.85);
    g = (uint8_t)(g * 0.85);
    b = (uint8_t)(b * 0.85);
    if (b < 20)
      b = 20;
    effectStrip.setPixelColor(i, r, g, b);
  }
  if (random(10) < 3)
  {
    int pos = random(NUM_LEDS_EFFECT);
    effectStrip.setPixelColor(pos, 200, 255, 255);
  }
  effectStrip.show();
}
// 4. Comet
void drawComet()
{
  for (int i = 0; i < NUM_LEDS_EFFECT; i++)
  {
    uint32_t c = effectStrip.getPixelColor(i);
    uint8_t r = (uint8_t)(c >> 16);
    uint8_t g = (uint8_t)(c >> 8);
    uint8_t b = (uint8_t)c;
    effectStrip.setPixelColor(i, r * 0.6, g * 0.6, b * 0.6);
  }
  long pos = (millis() / 150) % NUM_LEDS_EFFECT;
  effectStrip.setPixelColor(pos, 100, 255, 255);
  effectStrip.show();
}
// 5. Blue Scanner
void drawBlueScanner()
{
  for (int i = 0; i < NUM_LEDS_EFFECT; i++)
  {
    uint32_t c = effectStrip.getPixelColor(i);
    effectStrip.setPixelColor(i, (c >> 16) * 0.5, (c >> 8) * 0.5, c * 0.5);
  }
  int speed = 200;
  int range = NUM_LEDS_EFFECT - 1;
  int pos = abs(((long)millis() / speed) % (2 * range) - range);
  effectStrip.setPixelColor(pos, 0, 100, 255);
  effectStrip.show();
}
// 6. Ocean
void drawOcean()
{
  for (int i = 0; i < NUM_LEDS_EFFECT; i++)
  {
    int flicker = random(0, 55);
    long hue = 30000 + random(0, 15000);
    int val = 255 - flicker;
    if (val < 0)
      val = 0;
    effectStrip.setPixelColor(i, effectStrip.ColorHSV(hue, 255, val));
  }
  effectStrip.show();
}
// 7. Aurora
void drawAurora()
{
  unsigned long t = millis();
  for (int i = 0; i < NUM_LEDS_EFFECT; i++)
  {
    float wave = (sin((t / 1500.0) + i * 0.4) + 1.0) / 2.0;
    long hue = 21000 + (long)(wave * 29000);
    effectStrip.setPixelColor(i, effectStrip.ColorHSV(hue, 255, 200));
  }
  effectStrip.show();
}
// 8. Ice Shimmer
void drawIceShimmer()
{
  for (int i = 0; i < NUM_LEDS_EFFECT; i++)
  {
    if (random(10) > 8)
    {
      effectStrip.setPixelColor(i, 200, 240, 255);
    }
    else
    {
      uint32_t c = effectStrip.getPixelColor(i);
      uint8_t r = (uint8_t)(c >> 16);
      uint8_t g = (uint8_t)(c >> 8);
      uint8_t b = (uint8_t)c;
      if (r > 0)
        r--;
      if (g > 0)
        g--;
      if (b > 50)
        b--;
      else
        b = 50;
      effectStrip.setPixelColor(i, r, g, b);
    }
  }
  effectStrip.show();
}
// 9. Ping Pong
void drawPingPong()
{
  effectStrip.clear();
  int range = NUM_LEDS_EFFECT - 1;
  int pos = abs(((long)millis() / 180) % (2 * range) - range);
  effectStrip.setPixelColor(pos, 255, 255, 0);
  if (pos > 0)
    effectStrip.setPixelColor(pos - 1, 50, 50, 0);
  if (pos < range)
    effectStrip.setPixelColor(pos + 1, 50, 50, 0);
  effectStrip.show();
}
// 10. Meteor Rain
void drawMeteorRain()
{
  for (int i = 0; i < NUM_LEDS_EFFECT; i++)
  {
    uint32_t c = effectStrip.getPixelColor(i);
    uint8_t r = (uint8_t)(c >> 16);
    uint8_t g = (uint8_t)(c >> 8);
    uint8_t b = (uint8_t)c;
    effectStrip.setPixelColor(i, r * .7, g * .7, b * .7);
  }
  if (random(10) > 4)
  {
    int pos = (millis() / 100) % NUM_LEDS_EFFECT;
    effectStrip.setPixelColor(NUM_LEDS_EFFECT - 1 - pos, 255, 255, 255);
  }
  effectStrip.show();
}
// 11. Stacking
void drawStacking()
{
  unsigned long cycle = 2000;
  unsigned long t = millis() % cycle;
  int numLit = map(t, 0, cycle, 0, NUM_LEDS_EFFECT + 1);
  for (int i = 0; i < NUM_LEDS_EFFECT; i++)
  {
    if (i < numLit)
      effectStrip.setPixelColor(i, 0, 150, 255);
    else
      effectStrip.setPixelColor(i, 0, 0, 0);
  }
  effectStrip.show();
}
// 12. Bio Pulse
void drawBioPulse()
{
  unsigned long t = millis();
  float val = (exp(sin(t / 800.0 * PI)) - 0.36787944) * 108.0;
  if (val < 5)
    val = 5;
  if (val > 255)
    val = 255;
  for (int i = 0; i < NUM_LEDS_EFFECT; i++)
  {
    effectStrip.setPixelColor(i, (int)(val * 0.2), (int)val, 0);
  }
  effectStrip.show();
}
// 13. Theater Chase
void drawTheaterChase()
{
  int phase = (millis() / 100) % 3;
  for (int i = 0; i < NUM_LEDS_EFFECT; i++)
  {
    if ((i % 3) == phase)
      effectStrip.setPixelColor(i, 0, 0, 255);
    else
      effectStrip.setPixelColor(i, 0, 0, 0);
  }
  effectStrip.show();
}
// 14. Double Scan
void drawDoubleScan()
{
  effectStrip.clear();
  int range = NUM_LEDS_EFFECT - 1;
  int pos1 = abs(((long)millis() / 200) % (2 * range) - range);
  int pos2 = range - pos1;
  effectStrip.setPixelColor(pos1, 0, 255, 255);
  effectStrip.setPixelColor(pos2, 150, 0, 255);
  effectStrip.show();
}
// 15. Twinkle Mint
void drawTwinkleMint()
{
  for (int i = 0; i < NUM_LEDS_EFFECT; i++)
  {
    if (random(20) == 0)
    {
      effectStrip.setPixelColor(i, 100, 255, 150);
    }
    else
    {
      uint32_t c = effectStrip.getPixelColor(i);
      uint8_t r = (uint8_t)(c >> 16);
      uint8_t g = (uint8_t)(c >> 8);
      uint8_t b = (uint8_t)c;
      effectStrip.setPixelColor(i, r * .9, g * .9, b * .9);
    }
  }
  effectStrip.show();
}
// 16. Cyber Fade
void drawCyberFade()
{
  unsigned long t = millis();
  float shift = t / 500.0;
  for (int i = 0; i < NUM_LEDS_EFFECT; i++)
  {
    float wave = sin(i + shift);
    int r = (int)((wave + 1.0) * 127.5);
    int g = (int)((1.0 - wave) * 127.5);
    effectStrip.setPixelColor(i, r, g, 255);
  }
  effectStrip.show();
}
// 17. Soft White
void drawSoftWhite()
{
  int flicker = random(0, 10);
  int val = 200 - flicker;
  for (int i = 0; i < NUM_LEDS_EFFECT; i++)
  {
    effectStrip.setPixelColor(i, val, val * 0.9, val * 0.5);
  }
  effectStrip.show();
}

// ==========================================================================
// UTILITIES
// ==========================================================================

float readBatteryMv(uint8_t analogPin, float rTop, float rBottom)
{
  uint16_t raw = analogRead(analogPin);
  float vAdc = (float)raw * ADC_REF_VOLTAGE / ADC_MAX_COUNT;
  float vIn = vAdc * (rTop + rBottom) / rBottom;
  return vIn * 1000.0f;
}

void checkBatteries()
{
  digitalWrite(PIN_BAT_MEASURE_TRIG, HIGH);
  delay(20);
  g_batLeakMvMeasured = readBatteryMv(PIN_BAT_LEAK_CHK, LEAK_DIV_R_TOP, LEAK_DIV_R_BOTTOM);
  g_batRtcMvMeasured = readBatteryMv(PIN_BAT_RTC_CHK, RTC_DIV_R_TOP, RTC_DIV_R_BOTTOM);
  digitalWrite(PIN_BAT_MEASURE_TRIG, LOW);

  g_batLeakOk = (g_batLeakMvMeasured >= LEAK_OK_THRESHOLD_MV);
  g_batRtcOk = (g_batRtcMvMeasured >= RTC_OK_THRESHOLD_MV);

  Serial.print("[BAT] Leak=");
  Serial.print(g_batLeakMvMeasured);
  Serial.println(g_batLeakOk ? " OK" : " LOW");
  Serial.print("[BAT] RTC=");
  Serial.print(g_batRtcMvMeasured);
  Serial.println(g_batRtcOk ? " OK" : " LOW");
}

void sendStatus()
{
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

void startupAnimation()
{
  uint32_t c_blue = statusStrip.Color(0, 100, 255);
  uint32_t c_proj = statusStrip.Color(200, 255, 255);
  uint32_t c_wht = statusStrip.Color(255, 255, 255);

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

  const int CENTER = 15;
  for (int progress = 0; progress <= 15; progress++)
  {
    int distanceToTravel = 15 - progress;
    for (int p = 0; p <= distanceToTravel; p++)
    {
      statusStrip.clear();
      for (int f = 0; f < progress; f++)
        statusStrip.setPixelColor(f, c_blue);
      for (int f = 0; f < progress; f++)
        statusStrip.setPixelColor(30 - f, c_blue);
      int pPosLeft = 15 - p;
      int pPosRight = 15 + p;
      statusStrip.setPixelColor(pPosLeft, c_proj);
      statusStrip.setPixelColor(pPosRight, c_proj);
      if (p == 0)
        statusStrip.setPixelColor(CENTER, c_proj);
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
    statusStrip.setPixelColor(progress, c_wht);
    statusStrip.setPixelColor(30 - progress, c_wht);
    statusStrip.show();
    delay(60);
    watchdog_update();
  }
  delay(100);
  for (int i = 0; i < NUM_LEDS_STATUS; i++)
    statusStrip.setPixelColor(i, c_wht);
  for (int i = 0; i < NUM_LEDS_EFFECT; i++)
    effectStrip.setPixelColor(i, c_wht);
  statusStrip.show();
  effectStrip.show();
  delay(400);

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