#include <Arduino.h>
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

#include <Wire.h> // 標準のWireライブラリを使用
#include <Adafruit_INA219.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// --- ピン定義 (新配置) ---
// 出力ピン
const int OUT1_PIN = 0;
const int OUT2_PIN = 1;
const int OUT3_PIN = 2;
const int OUT4_PIN = 3;
const int OUT5_PIN = 6;
const int ACTSW_PIN = 7; // Act_SW
const int V5_SW_PIN = 9; // 5V_SW

// センサ・入力ピン
const int ACTTEMP_PIN = 8;  // ActTemp (DS18B20)
const int LOGTEMP_PIN = 10; // Logi_TEMP (DS18B20)

// (I2Cピン D4, D5 は Wire ライブラリが自動的に使用)

// --- I2Cアドレス ---
const uint8_t INA_LOG_ADDR = 0x40; // LogPow (INA219)
const uint8_t INA_ACT_ADDR = 0x45; // ActPow (INA219)

// --- オブジェクト生成 ---
// (標準の Wire を使うため、引数指定は不要)
Adafruit_INA219 ina_log(INA_LOG_ADDR);
Adafruit_INA219 ina_act(INA_ACT_ADDR);

OneWire oneWire_log(LOGTEMP_PIN);
OneWire oneWire_act(ACTTEMP_PIN);

DallasTemperature sensor_log(&oneWire_log);
DallasTemperature sensor_act(&oneWire_act);

// 出力ピンの管理用配列
const int outPins[] = {OUT1_PIN, OUT2_PIN, OUT3_PIN, OUT4_PIN, OUT5_PIN, V5_SW_PIN, ACTSW_PIN};
const int numOutPins = sizeof(outPins) / sizeof(outPins[0]);
// 出力ピンの名前 (シリアルコマンド用)
const String outNames[] = {"OUT1", "OUT2", "OUT3", "OUT4", "OUT5", "5VSW", "ACTSW"};


// --- グローバル変数 ---
unsigned long lastSendTime = 0;
const long sendInterval = 1000; // データ送信間隔 (ms)


// --- 出力制御 ---
// (処理内容に変更なし)
bool controlOutput(String target, int state) {
  int pinToControl = -1;

  for (int i = 0; i < numOutPins; i++) {
    if (target.equalsIgnoreCase(outNames[i])) {
      pinToControl = outPins[i];
      break;
    }
  }

  if (pinToControl == -1) {
    return false;
  }

  if (state == 0) {
    digitalWrite(pinToControl, LOW);
  } else {
    digitalWrite(pinToControl, HIGH);
  }
  
  return true;
}

// --- シリアル入力処理 ---
// (処理内容に変更なし)
void handleSerialInput() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.startsWith("SET")) {
      int firstComma = input.indexOf(',');
      int secondComma = input.indexOf(',', firstComma + 1);

      if (firstComma > 0 && secondComma > 0) {
        String target = input.substring(firstComma + 1, secondComma);
        int state = input.substring(secondComma + 1).toInt();

        bool success = controlOutput(target, state);
        
        if (success) {
          Serial.print("ACK: ");
          Serial.println(input);
        } else {
          Serial.print("NACK: Unknown target ");
          Serial.println(target);
        }
      } else {
        Serial.println("NACK: Invalid command format. Use: SET,TARGET,STATE");
      }
    } else {
      Serial.println("NACK: Unknown command. Use: SET");
    }
  }
}


// --- センサデータ読み取り & シリアル送信 ---
// (処理内容に変更なし)
void readAndPublishData() {
  // --- INA219 (Log) ---
  float log_busVoltage = ina_log.getBusVoltage_V();
  float log_shuntVoltage = ina_log.getShuntVoltage_mV();
  float log_current = ina_log.getCurrent_mA();
  float log_power = ina_log.getPower_mW();
  float log_voltage = log_busVoltage + (log_shuntVoltage / 1000);

  // --- INA219 (Act) ---
  float act_busVoltage = ina_act.getBusVoltage_V();
  float act_shuntVoltage = ina_act.getShuntVoltage_mV();
  float act_current = ina_act.getCurrent_mA();
  float act_power = ina_act.getPower_mW();
  float act_voltage = act_busVoltage + (act_shuntVoltage / 1000);

  // --- DS18B20 (Log) ---
  float log_temp = sensor_log.getTempCByIndex(0);
  if (log_temp == DEVICE_DISCONNECTED_C) {
      log_temp = -127.0;
  }

  // --- DS18B20 (Act) ---
  float act_temp = sensor_act.getTempCByIndex(0);
  if (act_temp == DEVICE_DISCONNECTED_C) {
      act_temp = -127.0;
  }

  // --- シリアル出力 (CSV形式) ---
  Serial.print("DATA,");
  Serial.print(log_voltage);
  Serial.print(",");
  Serial.print(log_current);
  Serial.print(",");
  Serial.print(log_power);
  Serial.print(",");
  Serial.print(act_voltage);
  Serial.print(",");
  Serial.print(act_current);
  Serial.print(",");
  Serial.print(act_power);
  Serial.print(",");
  Serial.print(log_temp);
  Serial.print(",");
  Serial.print(act_temp);
  Serial.println();
}


// --- セットアップ関数 ---
void setup() {
  Serial.begin(115200);

  // ★★★ 標準の Wire (D4, D5) を初期化 ★★★
  Wire.begin();

  // --- 出力ピンの初期化 ---
  // 指定通り、標準HIGHに設定
  for (int i = 0; i < numOutPins; i++) {
    pinMode(outPins[i], OUTPUT);
    digitalWrite(outPins[i], HIGH);
  }

  // --- センサの初期化 ---
  // ★★★ begin() に引数は不要 (デフォルトの Wire を使うため) ★★★
  if (!ina_log.begin()) {
    Serial.println("ERROR: Failed to find INA219 (Log 0x40)");
  }
  if (!ina_act.begin()) {
    Serial.println("ERROR: Failed to find INA219 (Act 0x45)");
  }

  sensor_log.begin();
  sensor_act.begin();
  
  // 最初の温度測定をリクエスト (非ブロッキング)
  sensor_log.requestTemperatures();
  sensor_act.requestTemperatures();
  
  Serial.println("STATUS: Power Distribution Board Ready. (SAMD21 - Default I2C)");
}

// --- メインループ ---
void loop() {
  // 1. シリアルコマンド受信処理
  handleSerialInput();

  // 2. 定期的なセンサデータ送信
  unsigned long currentTime = millis();
  if (currentTime - lastSendTime >= sendInterval) {
    lastSendTime = currentTime;
    
    // センサデータ読み取りとシリアル送信
    readAndPublishData();
    
    // 次の温度測定をリクエスト (非ブロッキング)
    sensor_log.requestTemperatures();
    sensor_act.requestTemperatures();
  }
}