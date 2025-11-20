/* * センサ基板 V2.0 (ESP32) メインコントローラ スケッチ　COM3
* 機能:
* - 100Hz: 深度センサーの値をUDP送信 (micros()で高精度に実行)
* - 1Hz:   OLEDに時刻と漏水アラートを表示
* - 1min:  GNSS(緯度経度), BME, RTC時刻, 漏水センサの値をメインUDPで送信
* - setup(): 起動時にGNSSから時刻を取得し、RTCを同期
*/

#include <Arduino.h>
#include <stdio.h>
#include <Wire.h>
// --- イーサネット (W5500) ---
#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>

// --- MS5837 -----------------------------------------------------------
#include <MS5837.h>
MS5837 depthSensor;
bool depthSensorOK = false;
// --- RTC --------------------------------------------------------------
#include <RTClib.h>
RTC_DS3231 rtc;
bool rtcOK = false;
// --- u-blox GNSS ------------------------------------------------------
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> 
SFE_UBLOX_GNSS myGNSS;
bool gnssOK = false; // GNSSがsetupで初期化成功したか
// --- BME280 -----------------------------------------------------------
#include <SparkFunBME280.h>
BME280 bmeSensor;
bool bmeSensorOK = false;
// --- OLED (SH11106) ---------------------------------------------------
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C
Adafruit_SH1106G display(128, 64, &Wire, OLED_RESET);
bool oledOK = false;
// ---漏水検出 leak -----------------------------------------------------
#define LEAK_PIN 39

// --- I2C ---------------------------------------------------------------
#define I2C_SDA 21
#define I2C_SCL 22

// RP2040-ZERO I2C 
#define RP2040_ADDRESS 0x08 
bool g_leakBuzzerActive = false; //buzzer Flag
bool g_battLeakLow = false; //  リーク電池異常
bool g_battRtcLow = false;  //  RTC電池異常
bool rp2040Exists = false;   // RP2040の確認フラグ

// --- GNSS (UART) ------------------------------------------------------
const int RX_PIN = 16;
const int TX_PIN = 17;

// W5500
#define W5500_SCLK 18
#define W5500_MISO 19
#define W5500_MOSI 23
#define W5500_CS 26
#define W5500_RST 25

// ーーー イーサネットとUDPの設定 ーーーーーーーーーーーーーーーーーーーーーーーーーーー
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress esp32_ip(192, 168, 9, 130);
IPAddress gateway(192, 168, 9, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns(192, 168, 9, 1);

// UDP送信先 (NUC Main PC)
IPAddress nuc_ip(192, 168, 9, 100);
unsigned int nuc_main_port = 12345;  // メインデータ用 (1min)
unsigned int nuc_depth_port = 12346; // 深度データ用 (100Hz)

EthernetUDP udp;

// --- タイマー用 グローバル変数 ---------------------------------------------
// 100Hz (10ms) タイマー (高精度な micros() を使用)
uint64_t lastDepthSendTime_us = 0;
const uint64_t DEPTH_INTERVAL_US = 10000; // 10ms = 10,000µs

// 1Hz (1000ms) タイマー (OLED表示用)
unsigned long lastDisplayUpdateTime_ms = 0;
const unsigned long DISPLAY_INTERVAL_MS = 1000; // 1s = 1000ms

// 1/60Hz (1min) タイマー (メインUDP送信用)
unsigned long lastMainDataSendTime_ms = 0;
const unsigned long MAIN_DATA_INTERVAL_MS = 10000; // 10s = 10,000ms  10秒に変更した


// --- センサーデータ グローバル変数 ------------------------------------------
long g_latitude = 0;
long g_longitude = 0;
long g_altitude_gnss = 0;
byte g_siv = 0;

float g_depth = -1.0;
float g_temp_bme = -999.0;
float g_humidity_bme = -1.0;
float g_pressure_bme = -1.0;
bool g_leakDetected = false;


// ＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝
// --- 関数定義 (Tasks) ---
// ＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝

// I2Cコマンド送信
void sendRP2040Command(char command){
  if (!rp2040Exists) { //フリーズ防止
    return; 
  }

  Wire.beginTransmission(RP2040_ADDRESS);
  Wire.write(command);
  byte error = Wire.endTransmission();//送信終了
  if (error != 0) {
    Serial.print("Failed to send I2C command to RP2040. Error: ");
    Serial.println(error);
  }
}

// --- Task 1 (100Hz): 深度センサーの読み取りとUDP送信 ---
void task_100Hz_SendDepth() {
  if (depthSensorOK) {
    depthSensor.read();
    g_depth = depthSensor.depth(); // グローバル変数を更新
  } else {
    g_depth = -1.0;
  }

  // 深度データのみを送信 (CSV: "Depth,値")
  char depthPacket[32];
  snprintf(depthPacket, sizeof(depthPacket), "Depth,%.3f", g_depth);
  
  udp.beginPacket(nuc_ip, nuc_depth_port);
  udp.print(depthPacket);
  udp.endPacket();
  // デバッグ用にシリアルにも出力
  Serial.print("Sent Main UDP (Depth) -> ");
  Serial.println(depthPacket);
}

// --- Task 2 (1Hz): 漏水検知 と OLED表示 (時刻と漏水のみ) ---
void task_1Hz_UpdateDisplayAndLeak() {
  
  // 漏水検知 (1Hzでポーリング)
  // (地上(安全)がLOW、漏水(異常)がHIGH)
  g_leakDetected = (digitalRead(LEAK_PIN) == HIGH); 

  // 漏水状態が変化した場合、RP2040にI2Cコマンドを送信
  if (g_leakDetected && !g_leakBuzzerActive) {
    // 漏水検知 + ブザーが鳴っていない -> ブザーをONにする
    Serial.println("[I2C] Sending LEAK command 'L' to RP2040.");
    sendRP2040Command('L'); // 'L' = Leak
    g_leakBuzzerActive = true; // 状態を「鳴らしている」に更新
  } else if (!g_leakDetected && g_leakBuzzerActive) {
    // 漏水なし + ブザーが鳴っている -> ブザーをOFFにする
    Serial.println("[I2C] Sending SAFE command 'S' to RP2040.");
    sendRP2040Command('S'); // 'S' = Safe/Silent
    g_leakBuzzerActive = false; // 状態を「鳴らしていない」に更新
  }
  
  // OLED表示
  if (!oledOK) {
    return; // OLEDがNGなら何もしない
  }

  display.clearDisplay();
  DateTime now;
  if (rtcOK) {
    now = rtc.now(); // ★時刻は常にRTCから取得
  }

  // --- 1行目: 時刻 (大) ---
  display.setTextSize(2);
  display.setCursor(0, 0);
  char timeString[10];
  sprintf(timeString, "%02d:%02d:%02d", now.hour(), now.minute(), now.second());
  display.println(timeString);
  
  // (データ自体は1分ごとに task_1Min... で更新されるが、表示は1秒ごと)
  
  // --- 2行目: 日付 (小) ---
  display.setTextSize(1);
  display.setCursor(0, 20); // 1行目の下
  char dateString[12];
  sprintf(dateString, "%04d-%02d-%02d", now.year(), now.month(), now.day());
  display.println(dateString);

  // --- 3行目: Hull内の温度と湿度 (BME280) ---
  display.setCursor(0, 32);   // 2行目の下
  if (bmeSensorOK) {
    // g_temp_bme などのグローバル変数は1分ごとに更新されるが、
    // 表示は1秒ごとに最新の(1分前に読み取った)値を表示し続ける
    char bmeString[30];
    sprintf(bmeString, "T:%.1fC H:%.0f%%", g_temp_bme, g_humidity_bme);
    display.println(bmeString);
  } else {
    display.println(F("BME: N/A"));
  }

  // --- 4行目: 気圧  ---
  display.setCursor(0, 44);   // 3行目の下
  if (bmeSensorOK) {
    display.setTextSize(1);
    char pressureString[20];
    sprintf(pressureString, "P:%.1fhPa", g_pressure_bme);
    display.println(pressureString);
  } else {
    display.setTextSize(1);
    display.println(F("Pressure: N/A"));
  }

  // --- 5行目: 状態 (漏水アラート または System OK) ---
  display.setCursor(0, 56); // 4行目の下
  if (g_leakDetected) {
    // 漏水検知時 (白黒反転表示)
    display.setTextSize(1); // 1行に収めるためサイズ1
    display.setTextColor(SH110X_BLACK, SH110X_WHITE); // (文字色, 背景色)
    display.println(F("! ! ! LEAK ! ! !")); // サイズ1で目立たせる
    display.setTextColor(SH110X_WHITE); // 色を通常に戻す
  }/*else if (g_battRtcLow) {
    // RTC電池切れ
    display.println(F("! RTC Battery LOW !")); 
  }else if (g_battLeakLow) {
    // リーク電池切れ
    display.println(F("! LEAK Battery LOW !"));
  }*/else {
    // 平常時
    display.setTextSize(1);
    display.println(F("System OK"));
  }
  
  display.display();
}

// --- Task 3 (1Min): BME/GNSSの読み取りとメインUDP送信 ---
void task_1Min_ReadSensorsAndSendUDP() {
  
  // 1. BME280の読み取り (1分に1回)
  if (bmeSensorOK) {
    g_temp_bme = bmeSensor.readTempC();
    g_humidity_bme = bmeSensor.readFloatHumidity();
    g_pressure_bme = bmeSensor.readFloatPressure() / 100.0;
  }
  
  // 2. GNSSの読み取り (1分に1回)
  if (gnssOK) {
    g_latitude = myGNSS.getLatitude();
    g_longitude = myGNSS.getLongitude();
    g_altitude_gnss = myGNSS.getAltitude();
    g_siv = myGNSS.getSIV();
  }

  // 3. メインUDP送信
  DateTime now;
  if (rtcOK) {
    now = rtc.now(); // RTCから現在時刻を取得
  }

  // 形式: "YYYY-MM-DD,HH:MM:SS,Lat,Lon,Alt,SIV,Temp,Humid,Pressure,Leak"
  char udpPacket[256];
  snprintf(udpPacket, sizeof(udpPacket),
           "%04d-%02d-%02d,%02d:%02d:%02d,%ld,%ld,%ld,%d,%.2f,%.2f,%.2f,%d",
           now.year(), now.month(), now.day(),
           now.hour(), now.minute(), now.second(),    // 時刻 (RTC)
           g_latitude, g_longitude, g_altitude_gnss, g_siv, // 位置 (GNSS)
           g_temp_bme, g_humidity_bme, g_pressure_bme,    // BME
           g_leakDetected ? 1 : 0                         // Leak
  );

  udp.beginPacket(nuc_ip, nuc_main_port);
  udp.print(udpPacket);
  udp.endPacket();
  
  // デバッグ用にシリアルにも出力
  Serial.print("Sent Main UDP (1Min) -> ");
  Serial.println(udpPacket);
}


/*セットアップーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーー*/
void setup() {
  Serial.begin(115200);
  Serial.println("Starting Sensor Board V2.0 ...");
  Wire.begin(I2C_SDA, I2C_SCL);

  // --- イーサネット (W5500) 初期化 ---
  Serial.println("Initializing Ethernet (W5500)...");
  SPI.begin(W5500_SCLK, W5500_MISO, W5500_MOSI, W5500_CS);
  // (W5500 )
  pinMode(W5500_CS, OUTPUT);
  digitalWrite(W5500_CS, HIGH);
  pinMode(W5500_RST, OUTPUT);
  digitalWrite(W5500_RST, LOW);
  delay(100);
  digitalWrite(W5500_RST, HIGH);
  delay(200);
  Ethernet.init(W5500_CS);
  Ethernet.begin(mac, esp32_ip, dns, gateway, subnet);
  delay(1000);
  if (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("Ethernet cable is not connected.");
  } else {
    Serial.print("Ethernet IP address: ");
    Serial.println(Ethernet.localIP());
  }
  udp.begin(nuc_main_port);

  // --- センサー類 初期化 ---
  if (!depthSensor.init()) { Serial.println("Depth sensor Init failed!"); }
  else { depthSensorOK = true; Serial.println("Depth sensor initialized."); }
  depthSensor.setModel(MS5837::MS5837_30BA);
  depthSensor.setFluidDensity(997);

  bmeSensor.setI2CAddress(0x77);
  if(!bmeSensor.beginI2C(Wire)){ Serial.println("BME280 init failed."); }
  else { bmeSensorOK = true; Serial.println("BME280 initialized."); }

  if (!display.begin(SCREEN_ADDRESS)) { Serial.println(F("SH110X allocation failed.")); }
  else {
    oledOK = true;
    Serial.println(F("OLED initialized."));
    display.setRotation(2); // 180度反転
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(0, 0);
    display.println(F("OLED System Boot..."));
    display.display();
    delay(1000);
  }

  pinMode(LEAK_PIN, INPUT_PULLUP); // 漏水
  Serial.println("Leak Sensor (Pin 39) initialized.");

  if (!rtc.begin()) { Serial.println("RTC (0x68) failed."); }
  else { rtcOK = true; Serial.println("RTC initialized."); }

  // --- GNSS 初期化 (RTC同期専用) ---
  Serial.println("Initializing GNSS...");
  Serial2.begin(38400, SERIAL_8N1, RX_PIN, TX_PIN);
  delay(100);
  if (myGNSS.begin(Serial2) == true) {
    gnssOK = true;
  } else {
    delay(100);
    Serial2.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
    delay(100);
    if (myGNSS.begin(Serial2) == true) {
      Serial.println("GNSS: connected at 9600 baud, switching to 38400");
      myGNSS.setSerialRate(38400);
      delay(100);
      gnssOK = true;
    }
  }

  if (gnssOK) {
    Serial.println("GNSS connected.");
    // (緯度経度を1分に1回取得するため、1Hz設定は不要。デフォルトのままでOK)
    // myGNSS.setNavigationFrequency(1); 
    // myGNSS.saveConfiguration();
    
    
    Serial.println("Waiting for GNSS time sync (max 10s)...");
    unsigned long gnssStartTime = millis();
    bool syncOK = false;
    while (millis() - gnssStartTime < 10000) {
      if (myGNSS.getYear() > 2000 && myGNSS.getSIV() >= 4) {
        int year = myGNSS.getYear();
        int month = myGNSS.getMonth();
        int day = myGNSS.getDay();
        int hourJST = myGNSS.getHour() + 9;
        int minute = myGNSS.getMinute();
        int second = myGNSS.getSecond();
        if (hourJST >= 24) hourJST -= 24;

        if (rtcOK) {
          rtc.adjust(DateTime(year, month, day, hourJST, minute, second));
          Serial.println("\n!!! RTC SYNCED TO GNSS TIME !!!");
        }
        syncOK = true;
        break;
      }
      delay(500);
      Serial.print(".");
    }
    if (!syncOK) Serial.println("\nGNSS sync timeout. Using existing RTC time.");
    
  } else {
    Serial.println("GNSS: Failed to connect. Using existing RTC time.");
  }

  // --- RP2040 (I2C) チェックと電池問い合わせ ---
  Serial.println("Connecting to RP2040 (0x08)...");
  // RR2040存在確認
  Wire.beginTransmission(RP2040_ADDRESS);
  if (Wire.endTransmission() == 0) {
    Serial.println(" -> RP2040 Found!");
    rp2040Exists = true; // 生存確認OK
    // 生きているなら、バッテリー状態を問い合わせる
    delay(50);
    int bytes = Wire.requestFrom(RP2040_ADDRESS, 1);
    if (bytes > 0) {
      byte status = Wire.read();
      g_battLeakLow = (status & 0x01); // 1桁目が1ならtrue
      g_battRtcLow  = (status & 0x02); // 2桁目が1ならtrue
      if (g_battLeakLow) Serial.println(" -> BATTERY LOW WARNING!");
      if (g_battRtcLow) Serial.println(" -> RTC LOW WARNING!");
      if(status==0) Serial.println(" -> Battery OK.");
    }
  } else {
    // 応答なし (RP2040空っぽ or 配線不良????)
    Serial.println(" -> RP2040 NOT FOUND. I2C commands will be disabled.");
    rp2040Exists = false; // 生存確認NG -> 通信をスキップ
  }
  
  Serial.println("Setup Complete. Starting main loops...");
  
  // タイマーの初期時刻を設定
  lastDepthSendTime_us = micros();
  lastDisplayUpdateTime_ms = millis();
  lastMainDataSendTime_ms = millis();
}


/*  ループ=================================================================================================================================================*/
void loop() {
  
  // -----------------------------------------------------------------
  // 100Hz タイマー (10ミリ秒 = 10000マイクロ秒ごと)
  // -----------------------------------------------------------------
  if (micros() - lastDepthSendTime_us > DEPTH_INTERVAL_US) {
    // タイマーをリセット (処理遅延を考慮し、現在時刻ではなく加算する)
    lastDepthSendTime_us += DEPTH_INTERVAL_US; 
    
    task_100Hz_SendDepth(); // 深度センサを読み取りUDP送信
  }

  // -----------------------------------------------------------------
  // 1Hz タイマー (1000ミリ秒ごと)
  // -----------------------------------------------------------------
  if (millis() - lastDisplayUpdateTime_ms > DISPLAY_INTERVAL_MS) {
    // タイマーをリセット
    lastDisplayUpdateTime_ms += DISPLAY_INTERVAL_MS;
    
    task_1Hz_UpdateDisplayAndLeak(); // 漏水検知とOLED表示
    Serial.println("1Hz__ok");
  }
  
  // -----------------------------------------------------------------
  // 1/60Hz タイマー (60000ミリ秒ごと)
  // -----------------------------------------------------------------
  if (millis() - lastMainDataSendTime_ms > MAIN_DATA_INTERVAL_MS) {
    // タイマーをリセット
    lastMainDataSendTime_ms += MAIN_DATA_INTERVAL_MS;
    
    task_1Min_ReadSensorsAndSendUDP(); // メインセンサー読み取りとUDP送信
  }
}