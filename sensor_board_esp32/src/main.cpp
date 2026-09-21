/*
 * ESP32 Firmware V31.0 (Startup and Runtime Hardening)
 * - Bounded I2C operations and safe open-drain bus recovery
 * - Validated UDP commands, sensor values, UART frames, and task creation
 * - WDT coverage for all worker tasks and detailed boot diagnostics
 */

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <esp_task_wdt.h>
#include <esp_system.h>

// --- Nanopb Headers ---
#include "pb_encode.h"
#include "pb_decode.h"
#include "proto/driver_msgs__Depth.pb.h"
#include "proto/driver_msgs__RtcTime.pb.h"
#include "proto/driver_msgs__RtcGnss.pb.h"
#include "proto/driver_msgs__Environment.pb.h"
#include "proto/driver_msgs__ButtonBatteryState.pb.h"
#include "proto/driver_msgs__BoolStamped.pb.h"
#include "proto/driver_msgs__Int32Stamped.pb.h"
#include "proto/driver_msgs__BuzzerSwitch.pb.h"
#include "proto/driver_msgs__SystemStatus.pb.h"
#include "proto/driver_msgs__PowerState.pb.h"

#include <RTClib.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include "SparkFunBME280.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

// --- Typedefs ---
typedef protolink__driver_msgs__Depth_driver_msgs__Depth Msg_Depth;
typedef protolink__driver_msgs__RtcTime_driver_msgs__RtcTime Msg_RtcTime;
typedef protolink__driver_msgs__RtcGnss_driver_msgs__RtcGnss Msg_RtcGnss;
typedef protolink__driver_msgs__Environment_driver_msgs__Environment Msg_Environment;
typedef protolink__driver_msgs__ButtonBatteryState_driver_msgs__ButtonBatteryState Msg_BtnBatt;
typedef protolink__driver_msgs__BoolStamped_driver_msgs__BoolStamped Msg_Bool;
typedef protolink__driver_msgs__Int32Stamped_driver_msgs__Int32Stamped Msg_Int32;
typedef protolink__driver_msgs__BuzzerSwitch_driver_msgs__BuzzerSwitch Msg_Buzzer;
typedef protolink__driver_msgs__SystemStatus_driver_msgs__SystemStatus Msg_SystemStatus;
typedef protolink__driver_msgs__PowerState_driver_msgs__PowerState Msg_Power;

// --- Pin Definitions ---
#define PIN_I2C_A_SDA 32
#define PIN_I2C_A_SCL 33
#define PIN_I2C_B_SDA 21
#define PIN_I2C_B_SCL 22
#define PIN_RP_TX 27
#define PIN_RP_RX 34
#define PIN_GNSS_RX 16
#define PIN_GNSS_TX 17
#define W5500_CS 26
#define W5500_RST 25
#define W5500_SCLK 18
#define W5500_MISO 19
#define W5500_MOSI 23
#define PIN_LEAK 39
#define PIN_IMU_RESET 13
#define PIN_ZED_POWER 14
#define PIN_PPS_IN 35

#define ADDR_GNSS 0x42
#define ADDR_OLED 0x3C

// --- Network Config ---
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
IPAddress myIp(192, 168, 9, 5);
IPAddress pcIp(192, 168, 9, 100);
IPAddress bcastIp(192, 168, 9, 255);

// --- UDP Ports ---
#define PORT_TX_BATTERY 9000
#define PORT_TX_DEPTH 9002
#define PORT_TX_ENV 9003
#define PORT_TX_LEAK 9005
#define PORT_TX_GNSS 9007
#define PORT_TX_TIME 9008
#define PORT_TX_SIGNAL 9012

#define PORT_RX_BUZZER 9001
#define PORT_RX_DEPTH_CFG 9002
#define PORT_RX_IMU 9004
#define PORT_RX_RGB 9006
#define PORT_RX_SERVO 9009
#define PORT_RX_ZED 9010
#define PORT_RX_POWER 9011

// --- Objects ---
EthernetUDP udpSender;
EthernetUDP udpRxBuzzer, udpRxDepthCfg, udpRxImu, udpRxRgb, udpRxServo, udpRxZed, udpRxPower;

// --- High Speed MS5837 (Stabilized) ---
// I2Cバスを手動でトグルしてスタックしたスレーブを解放する
void forceBusReset(int sda, int scl)
{
    // I2CはオープンドレインなのでHIGHを出力せず、INPUTでラインを解放する
    pinMode(sda, INPUT_PULLUP);
    pinMode(scl, INPUT_PULLUP);
    delayMicroseconds(10);

    for (int i = 0; i < 9 && digitalRead(sda) == LOW; i++)
    {
        pinMode(scl, OUTPUT);
        digitalWrite(scl, LOW);
        delayMicroseconds(10);
        pinMode(scl, INPUT_PULLUP);
        delayMicroseconds(10);
    }

    // STOP条件: SDA LOW中にSCLを解放し、その後SDAを解放する
    pinMode(sda, OUTPUT);
    digitalWrite(sda, LOW);
    delayMicroseconds(10);
    pinMode(scl, INPUT_PULLUP);
    delayMicroseconds(10);
    pinMode(sda, INPUT_PULLUP);
    delayMicroseconds(10);
}

class RobustMS5837
{
private:
    uint16_t C[8];
    TwoWire *wirePort;
    uint8_t addr;
    int pin_sda;
    int pin_scl;
    void osDelay(int ms) { vTaskDelay(pdMS_TO_TICKS(ms)); }

    // CRITICAL FIX: ドライバを停止してから手動操作を行う
    void recoverBus()
    {
        // 1. ESP32のI2Cドライバを停止し、ピンを解放する
        wirePort->end(); 
        
        // 2. 手動でクロックを叩いてバスをクリアする
        forceBusReset(pin_sda, pin_scl);
        
        // 3. ドライバを再開する
        wirePort->begin(pin_sda, pin_scl);
        wirePort->setClock(400000);
        // 電源投入時にセンサがSDA/SCLを保持しても永久に待たない
        wirePort->setTimeOut(50);
    }

    bool writeCmd(uint8_t cmd)
    {
        wirePort->beginTransmission(addr);
        wirePort->write(cmd);
        return (wirePort->endTransmission() == 0);
    }

    uint16_t read16()
    {
        wirePort->requestFrom(addr, (uint8_t)2);
        return (wirePort->available() >= 2) ? (wirePort->read() << 8) | wirePort->read() : 0;
    }

    uint32_t read24()
    {
        if (wirePort->requestFrom(addr, (uint8_t)3) == 3)
        {
            uint32_t v = wirePort->read() << 16;
            v |= wirePort->read() << 8;
            v |= wirePort->read();
            return v;
        }
        return 0;
    }

    uint8_t crc4()
    {
        uint16_t prom[8];
        memcpy(prom, C, sizeof(prom));
        uint16_t remainder = 0;
        prom[0] &= 0x0FFF;
        prom[7] = 0;
        for (uint8_t i = 0; i < 16; i++)
        {
            remainder ^= (i & 1) ? (prom[i >> 1] & 0x00FF) : (prom[i >> 1] >> 8);
            for (uint8_t bit = 0; bit < 8; bit++)
                remainder = (remainder & 0x8000) ? (remainder << 1) ^ 0x3000 : remainder << 1;
        }
        return (remainder >> 12) & 0x0F;
    }

public:
    float depthVal = 0.0;
    float tempVal = 0.0;

    // コンストラクタでピン番号を受け取るように変更
    RobustMS5837(TwoWire *w, int sda, int scl, uint8_t a = 0x76)
    {
        wirePort = w;
        pin_sda = sda;
        pin_scl = scl;
        addr = a;
    }

    bool init()
    {
        recoverBus(); // 安全に呼び出せるようになった
        if (!writeCmd(0x1E))
            return false;
        osDelay(20);
        for (uint8_t i = 0; i <= 6; i++)
        {
            if (!writeCmd(0xA0 | (i * 2)))
                return false;
            C[i] = read16();
            osDelay(2);
        }
        const uint8_t expectedCrc = C[0] >> 12;
        return C[1] != 0 && C[2] != 0 && crc4() == expectedCrc;
    }

    bool read(float fluidDensity)
    {
        if (!writeCmd(0x48))
            return false;
        osDelay(11);
        if (!writeCmd(0x00))
            return false;
        uint32_t D1 = read24();
        if (D1 == 0)
            return false;
        if (!writeCmd(0x58))
            return false;
        osDelay(11);
        if (!writeCmd(0x00))
            return false;
        uint32_t D2 = read24();
        if (D2 == 0)
            return false;
        int32_t dT = static_cast<int32_t>(D2) - static_cast<int32_t>(C[5]) * 256;
        int32_t TEMP = 2000 + ((int64_t)dT * C[6]) / 8388608;
        int64_t OFF = (int64_t)C[2] * 65536 + ((int64_t)C[4] * dT) / 128;
        int64_t SENS = (int64_t)C[1] * 32768 + ((int64_t)C[3] * dT) / 256;

        // MS5837-30BA datasheetの二次温度補償
        int64_t Ti = 0;
        int64_t OFFi = 0;
        int64_t SENSi = 0;
        int64_t tempOffset = static_cast<int64_t>(TEMP) - 2000;
        if (TEMP < 2000)
        {
            Ti = (3LL * dT * dT) / 8589934592LL;
            OFFi = (3LL * tempOffset * tempOffset) / 2;
            SENSi = (5LL * tempOffset * tempOffset) / 8;
            if (TEMP < -1500)
            {
                int64_t veryLowOffset = static_cast<int64_t>(TEMP) + 1500;
                OFFi += 7LL * veryLowOffset * veryLowOffset;
                SENSi += 4LL * veryLowOffset * veryLowOffset;
            }
        }
        else
        {
            Ti = (2LL * dT * dT) / 137438953472LL;
            OFFi = (tempOffset * tempOffset) / 16;
        }
        TEMP -= static_cast<int32_t>(Ti);
        OFF -= OFFi;
        SENS -= SENSi;

        int32_t P = (int32_t)((D1 * SENS / 2097152 - OFF) / 8192);
        tempVal = TEMP / 100.0f;
        depthVal = (P / 10.0f - 1013.25f) / (9.80665f * fluidDensity / 100.0f);
        return isfinite(tempVal) && isfinite(depthVal) &&
               tempVal >= -50.0f && tempVal <= 100.0f;
    }
};

// インスタンス化時にピン番号を指定
RobustMS5837 depthSensor(&Wire, PIN_I2C_A_SDA, PIN_I2C_A_SCL, 0x76);
BME280 bmeSensor;
RTC_DS3231 rtc;
SFE_UBLOX_GNSS myGNSS;
Adafruit_SH1106G display(128, 64, &Wire1, -1);

SemaphoreHandle_t wire1Mutex;
SemaphoreHandle_t depthMutex;
SemaphoreHandle_t uartMutex;
SemaphoreHandle_t ethMutex;

bool g_oledFound = false;
bool g_gnssFound = false;
bool g_bmeFound = false;
bool g_rtcFound = false;
bool g_ethernetReady = false;
bool g_rp2040Alive = false;
bool g_leakDetected = false;
volatile bool g_ethLinkStatus = false;
bool g_isSeawater = true;
portMUX_TYPE g_stateMux = portMUX_INITIALIZER_UNLOCKED;

unsigned long g_lastLeakTxMs = 0;

// --- Battery Management Variables ---
const float CAP_LOGIC_AH = 18.0f; // 4S
const float CAP_ACT_AH = 6.0f;    // 6S

float g_logVoltage = 0.0f;
float g_actVoltage = 0.0f;
float g_logCurrent = 0.0f;
float g_actCurrent = 0.0f;

float g_logConsumedAh = 0.0f;
float g_actConsumedAh = 0.0f;
unsigned long g_lastPowerUpdateMs = 0;
bool g_batteryInitialized = false;
unsigned long g_lastRejectedPacketLogMs = 0;

// --- Display Cache Variables ---
int32_t g_dispServo = 90;
int32_t g_dispRgb = -1;
bool g_dispZedPower = false;
bool g_dispImuReset = false;

struct DepthData
{
    float depth;
    float temp;
    bool ok;
} sharedDepth;

struct RP2040State
{
    bool bat_leak;
    bool bat_rtc;
    bool sig_jetson;
    bool sig_act_pow;
    bool sig_logic_rly;
    bool sig_pc_pow;
    bool sig_act_rly;
    unsigned long lastUpdate;
} rp2040State;

enum OledPage
{
    PAGE_SYSTEM,
    PAGE_BATTERY,
    PAGE_INSIDE,
    PAGE_DEPTH,
    PAGE_SIGNALS,
    PAGE_GNSS,
    PAGE_RX_INFO,
    PAGE_MAX
};
int currentPage = PAGE_SYSTEM;
unsigned long lastPageChange = 0;

// --- Prototypes ---
void Task_Depth(void *pvParameters);
void Task_Main(void *pvParameters);
void Task_NetRx(void *pvParameters);
void sendRP2040Uart(char type, int32_t val);
void checkRP2040Incoming();
void updateOLED();
void handleTimeSync();
void sendLeakUDP(bool leak);
void drawGuiHeader(const char *title);
void drawGuiFooter();
void drawCorner(int x, int y, int w, int h);
void drawSegBar(int x, int y, int w, int h, float val, float maxVal, bool usePercent);
float estimateConsumedAhFromVoltage(float voltage, float capacityAh, int cells);
bool readTrustedUdp(EthernetUDP &udp, uint8_t *buf, size_t capacity, int &len);
bool getIsSeawater();

struct SharedStateSnapshot
{
    float logVoltage;
    float actVoltage;
    float logCurrent;
    float actCurrent;
    float logConsumedAh;
    float actConsumedAh;
    int32_t dispServo;
    int32_t dispRgb;
    bool dispZedPower;
    bool dispImuReset;
    bool isSeawater;
};

SharedStateSnapshot getSharedState()
{
    SharedStateSnapshot state;
    portENTER_CRITICAL(&g_stateMux);
    state.logVoltage = g_logVoltage;
    state.actVoltage = g_actVoltage;
    state.logCurrent = g_logCurrent;
    state.actCurrent = g_actCurrent;
    state.logConsumedAh = g_logConsumedAh;
    state.actConsumedAh = g_actConsumedAh;
    state.dispServo = g_dispServo;
    state.dispRgb = g_dispRgb;
    state.dispZedPower = g_dispZedPower;
    state.dispImuReset = g_dispImuReset;
    state.isSeawater = g_isSeawater;
    portEXIT_CRITICAL(&g_stateMux);
    return state;
}

void setup()
{
    Serial.begin(115200);
    Serial.printf("\n[Boot] reset reason=%d\n", static_cast<int>(esp_reset_reason()));

    //ラッチアップ防止
    Serial.println("[Boot] waiting for peripheral power...");
    delay(3000);

    // 通信タイムアウトより十分長く、フリーズを早期復旧できる10秒に設定
    esp_err_t wdtInitResult = esp_task_wdt_init(10, true);
    if (wdtInitResult != ESP_OK)
        Serial.printf("[Boot] watchdog init failed: %s\n", esp_err_to_name(wdtInitResult));

    // Setup実行中の自身の監視
    esp_err_t wdtAddResult = esp_task_wdt_add(NULL);
    if (wdtAddResult != ESP_OK)
        Serial.printf("[Boot] watchdog subscribe failed: %s\n", esp_err_to_name(wdtAddResult));
    Serial.println("[Boot] watchdog ready");

    wire1Mutex = xSemaphoreCreateMutex();
    depthMutex = xSemaphoreCreateMutex();
    uartMutex = xSemaphoreCreateMutex();
    ethMutex = xSemaphoreCreateMutex();
    if (wire1Mutex == nullptr || depthMutex == nullptr || uartMutex == nullptr || ethMutex == nullptr)
    {
        Serial.println("[Boot] FATAL: mutex allocation failed");
        delay(1000);
        ESP.restart();
    }

    pinMode(PIN_LEAK, INPUT);
    pinMode(PIN_PPS_IN, INPUT);
    pinMode(PIN_IMU_RESET, OUTPUT);
    digitalWrite(PIN_IMU_RESET, HIGH);
    pinMode(PIN_ZED_POWER, OUTPUT);
    digitalWrite(PIN_ZED_POWER, LOW);

    Serial1.begin(115200, SERIAL_8N1, PIN_RP_RX, PIN_RP_TX);
    Serial2.begin(38400, SERIAL_8N1, PIN_GNSS_RX, PIN_GNSS_TX);

    // 起動時のバス浄化：両方のバスを一度リセットする
    Wire.end(); 
    Wire1.end();
    forceBusReset(PIN_I2C_A_SDA, PIN_I2C_A_SCL);
    forceBusReset(PIN_I2C_B_SDA, PIN_I2C_B_SCL);

    // FIX: setup内でWireを開始し、状態を確定させる
    Wire.begin(PIN_I2C_A_SDA, PIN_I2C_A_SCL);
    Wire.setClock(400000);
    Wire.setTimeOut(50);

    Wire1.begin(PIN_I2C_B_SDA, PIN_I2C_B_SCL);
    Wire1.setClock(400000);
    Wire1.setTimeOut(50);
    Serial.println("[Boot] I2C buses ready");

    esp_task_wdt_reset(); // WDTリセット

    // OLEDを最初に立ち上げ、後段の初期化で問題が起きても起動状態を見えるようにする
    Serial.println("[Boot] OLED init...");
    if (display.begin(ADDR_OLED, true))
    {
        g_oledFound = true;
        display.setRotation(2);
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SH110X_WHITE);
        display.setCursor(0, 0);
        display.println("V31.0 BOOTING...");
        display.display();
        Serial.println("[Boot] OLED OK");
    }
    else
    {
        Serial.println("[Boot] OLED not found");
    }
    esp_task_wdt_reset();

    Serial.println("[Boot] Ethernet init...");
    SPI.begin(W5500_SCLK, W5500_MISO, W5500_MOSI, W5500_CS);
    SPI.setFrequency(20000000);
    pinMode(W5500_RST, OUTPUT);
    digitalWrite(W5500_RST, LOW);
    delay(10);
    digitalWrite(W5500_RST, HIGH);
    delay(200);

    Ethernet.init(W5500_CS);
    Ethernet.begin(mac, myIp);
    // 合計待ち時間を抑えつつ、ARP等の一時的な取りこぼしには再試行する
    Ethernet.setRetransmissionTimeout(100);
    Ethernet.setRetransmissionCount(2);

    bool udpReady = true;
    udpReady &= udpSender.begin(12345);
    udpReady &= udpRxBuzzer.begin(PORT_RX_BUZZER);
    udpReady &= udpRxDepthCfg.begin(PORT_RX_DEPTH_CFG);
    udpReady &= udpRxImu.begin(PORT_RX_IMU);
    udpReady &= udpRxRgb.begin(PORT_RX_RGB);
    udpReady &= udpRxServo.begin(PORT_RX_SERVO);
    udpReady &= udpRxZed.begin(PORT_RX_ZED);
    udpReady &= udpRxPower.begin(PORT_RX_POWER);
    g_ethernetReady = (Ethernet.hardwareStatus() != EthernetNoHardware) && udpReady;
    Serial.printf("[Boot] Ethernet=%s\n", g_ethernetReady ? "OK" : "not available");

    esp_task_wdt_reset(); // WDTリセット

    Serial.println("[Boot] BME280/RTC init...");
    bmeSensor.setI2CAddress(0x77);
    if (bmeSensor.beginI2C(Wire1))
        g_bmeFound = true;

    g_rtcFound = rtc.begin(&Wire1);
    Serial.printf("[Boot] BME280=%s, RTC=%s\n",
                  g_bmeFound ? "OK" : "not found",
                  g_rtcFound ? "OK" : "not found");

    esp_task_wdt_reset(); // WDTリセット

    // GNSS Init & Explicit PPS (TP1) Configuration
    Serial.println("[Boot] GNSS init...");
    {
        SFE_UBLOX_GNSS configGNSS;
        if (configGNSS.begin(Wire1, ADDR_GNSS))
        {
            bool configOk = true;
            // 1. Disable first to ensure clean state
            configOk &= configGNSS.setVal8(0x10050007, 0, VAL_LAYER_RAM); // TP1_ENA = 0
            esp_task_wdt_reset();

            // 2. Set params
            configOk &= configGNSS.setVal32(0x40050024, 1, VAL_LAYER_RAM);      // FREQ_TP1 = 1Hz
            esp_task_wdt_reset();
            configOk &= configGNSS.setVal32(0x40050004, 100000, VAL_LAYER_RAM); // LEN_TP1 = 100ms
            esp_task_wdt_reset();
            configOk &= configGNSS.setVal8(0x20050030, 1, VAL_LAYER_RAM);       // TIMEGRID = GPS
            esp_task_wdt_reset();
            configOk &= configGNSS.setVal8(0x10050008, 1, VAL_LAYER_RAM);       // USE_LOCKED = 1
            esp_task_wdt_reset();
            configOk &= configGNSS.setVal8(0x1005000a, 1, VAL_LAYER_RAM);       // ALIGN_TO_TOW = 1
            esp_task_wdt_reset();
            configOk &= configGNSS.setVal8(0x10050006, 1, VAL_LAYER_RAM);       // POL_TP1 = 1
            esp_task_wdt_reset();

            // 3. Enable
            configOk &= configGNSS.setVal8(0x10050007, 1, VAL_LAYER_RAM); // TP1_ENA = 1
            esp_task_wdt_reset();
            Serial.printf("[Boot] GNSS PPS config=%s\n", configOk ? "OK" : "failed");
        }
    }
    if (myGNSS.begin(Serial2))
    {
        g_gnssFound = true;
    }
    Serial.printf("[Boot] GNSS=%s\n", g_gnssFound ? "OK" : "not found");

    g_lastPowerUpdateMs = millis();

    // 自身のWDT監視を解除
    esp_task_wdt_delete(NULL);

    // FIX: Depthタスクの優先度を 10 から 5 に下げる
    BaseType_t depthTask = xTaskCreatePinnedToCore(Task_Depth, "Depth", 8192, NULL, 5, NULL, 0);
    BaseType_t rxTask = xTaskCreatePinnedToCore(Task_NetRx, "Rx", 8192, NULL, 3, NULL, 1);
    BaseType_t mainTask = xTaskCreatePinnedToCore(Task_Main, "Main", 8192, NULL, 2, NULL, 1);
    if (depthTask != pdPASS || rxTask != pdPASS || mainTask != pdPASS)
    {
        Serial.printf("[Boot] FATAL: task creation failed (%ld/%ld/%ld)\n",
                      static_cast<long>(depthTask),
                      static_cast<long>(rxTask),
                      static_cast<long>(mainTask));
        delay(1000);
        ESP.restart();
    }
    Serial.println("[Boot] tasks started");
}

void loop() { vTaskDelete(NULL); }

void Task_Depth(void *pvParameters)
{
    // FIX: WDT監視を追加。フリーズ時に再起動させる
    esp_task_wdt_add(NULL);

    // Wire.beginはsetupで行うため削除
    
    // 初期化ループにもWDTリセットとディレイを入れる
    while (!depthSensor.init())
    {
        Serial.println("[Depth] init failed; retrying in 1 s");
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    Serial.println("[Depth] sensor ready");

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(50);
    uint8_t txBuf[128];
    
    // 連続エラーカウンタを追加
    int errorCount = 0;
    const int MAX_ERROR_COUNT = 20; // 50ms * 20 = 1000ms (1秒)

    for (;;)
    {
        // ループごとにWDTをリセット
        esp_task_wdt_reset();

        const bool isSeawater = getIsSeawater();
        bool ok = depthSensor.read(isSeawater ? 1029.0f : 997.0f);

        // 成功したらカウンタをリセット、失敗したらカウントアップ
        if (ok) 
        {
            errorCount = 0;
        }
        else 
        {
            errorCount++;
        }

        // 自動復旧ロジック: 1秒間通信不能ならバスリセットを試みる
        if (errorCount > MAX_ERROR_COUNT)
        {
            // バスリカバリー付き初期化を実行
            // init()内部で wirePort->end() -> forceBusReset() -> wirePort->begin() が走る
            Serial.println("[Depth] !! Sensor Timeout. Recovering Bus... !!");
            if (depthSensor.init()) 
            {
                Serial.println("[Depth] Recovery Success!");
                errorCount = 0; // 復旧成功
            }
            else 
            {
                // 復旧失敗時は少し待ってリトライ（スパム防止）
                Serial.println("[Depth] Recovery Failed.");
                vTaskDelay(pdMS_TO_TICKS(100));
            }
        }
        
        if (xSemaphoreTake(depthMutex, pdMS_TO_TICKS(5)) == pdTRUE)
        {
            sharedDepth.depth = ok ? depthSensor.depthVal : 0.0f;
            sharedDepth.temp = ok ? depthSensor.tempVal : 0.0f;
            sharedDepth.ok = ok;
            xSemaphoreGive(depthMutex);
        }

        if (ok)
        {
            Msg_Depth msg = protolink__driver_msgs__Depth_driver_msgs__Depth_init_zero;
            msg.has_depth = true;
            msg.depth = depthSensor.depthVal;
            msg.has_temperature = true;
            msg.temperature = depthSensor.tempVal;
            msg.has_watertype = true;
            msg.watertype = isSeawater;
            pb_ostream_t stream = pb_ostream_from_buffer(txBuf, sizeof(txBuf));
            if (pb_encode(&stream, protolink__driver_msgs__Depth_driver_msgs__Depth_fields, &msg))
            {
                if (g_ethernetReady)
                {
                    if (xSemaphoreTake(ethMutex, pdMS_TO_TICKS(30)) == pdTRUE)
                    {
                        udpSender.beginPacket(pcIp, PORT_TX_DEPTH);
                        udpSender.write(txBuf, stream.bytes_written);
                        udpSender.endPacket();
                        xSemaphoreGive(ethMutex);
                    }
                    else
                    {
                        static uint32_t lastBusyLogMs = 0;
                        if (millis() - lastBusyLogMs >= 1000)
                        {
                            Serial.println("[Depth] Tx skipped: Ethernet busy");
                            lastBusyLogMs = millis();
                        }
                    }
                }
            }
        }
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

bool readTrustedUdp(EthernetUDP &udp, uint8_t *buf, size_t capacity, int &len)
{
    len = 0;
    if (!g_ethernetReady ||
        xSemaphoreTake(ethMutex, pdMS_TO_TICKS(10)) != pdTRUE)
        return false;

    int packetSize = udp.parsePacket();
    if (packetSize <= 0)
    {
        xSemaphoreGive(ethMutex);
        return false;
    }

    bool trusted = (udp.remoteIP() == pcIp);
    bool validSize = packetSize <= static_cast<int>(capacity);
    if (trusted && validSize)
        len = udp.read(buf, capacity);
    else
        udp.read(static_cast<uint8_t *>(nullptr), packetSize);

    xSemaphoreGive(ethMutex);

    if (!trusted || !validSize)
    {
        uint32_t now = millis();
        if (now - g_lastRejectedPacketLogMs >= 1000)
        {
            Serial.printf("[NetRx] rejected UDP packet (source=%s, size=%d)\n",
                          trusted ? "trusted" : "untrusted", packetSize);
            g_lastRejectedPacketLogMs = now;
        }
        return false;
    }
    return len == packetSize;
}

bool getIsSeawater()
{
    portENTER_CRITICAL(&g_stateMux);
    bool result = g_isSeawater;
    portEXIT_CRITICAL(&g_stateMux);
    return result;
}

void Task_NetRx(void *pvParameters)
{
    uint8_t rxBuf[256];
    unsigned long lastLinkCheck = 0;
    esp_task_wdt_add(NULL);

    for (;;)
    {
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(10));

        if (g_ethernetReady && millis() - lastLinkCheck > 500 &&
            xSemaphoreTake(ethMutex, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            g_ethLinkStatus = (Ethernet.linkStatus() == LinkON);
            xSemaphoreGive(ethMutex);
            lastLinkCheck = millis();
        }

        int len = 0;
        if (readTrustedUdp(udpRxPower, rxBuf, sizeof(rxBuf), len))
        {
            pb_istream_t s = pb_istream_from_buffer(rxBuf, len);
            Msg_Power msg = protolink__driver_msgs__PowerState_driver_msgs__PowerState_init_zero;
            if (pb_decode(&s, protolink__driver_msgs__PowerState_driver_msgs__PowerState_fields, &msg))
            {
                const bool validLogV = msg.has_log_voltage && isfinite(msg.log_voltage) &&
                                       msg.log_voltage >= 0.0f && msg.log_voltage <= 60.0f;
                const bool validActV = msg.has_act_voltage && isfinite(msg.act_voltage) &&
                                       msg.act_voltage >= 0.0f && msg.act_voltage <= 60.0f;
                const bool validLogI = msg.has_log_current && isfinite(msg.log_current) &&
                                       fabsf(msg.log_current) <= 200.0f;
                const bool validActI = msg.has_act_current && isfinite(msg.act_current) &&
                                       fabsf(msg.act_current) <= 200.0f;
                const uint32_t now = millis();
                float dtSec = (now - g_lastPowerUpdateMs) / 1000.0f;
                if (dtSec > 5.0f)
                    dtSec = 0.0f;
                g_lastPowerUpdateMs = now;

                portENTER_CRITICAL(&g_stateMux);
                if (validLogV)
                    g_logVoltage = msg.log_voltage;
                if (validActV)
                    g_actVoltage = msg.act_voltage;
                if (!g_batteryInitialized && validLogV && validActV &&
                    msg.log_voltage > 5.0f && msg.act_voltage > 5.0f)
                {
                    g_logConsumedAh = estimateConsumedAhFromVoltage(msg.log_voltage, CAP_LOGIC_AH, 4);
                    g_actConsumedAh = estimateConsumedAhFromVoltage(msg.act_voltage, CAP_ACT_AH, 6);
                    g_batteryInitialized = true;
                }
                if (validLogI)
                {
                    g_logCurrent = msg.log_current;
                    g_logConsumedAh = constrain(g_logConsumedAh + fmaxf(msg.log_current, 0.0f) *
                                               (dtSec / 3600.0f), 0.0f, CAP_LOGIC_AH);
                }
                if (validActI)
                {
                    g_actCurrent = msg.act_current;
                    g_actConsumedAh = constrain(g_actConsumedAh + fmaxf(msg.act_current, 0.0f) *
                                               (dtSec / 3600.0f), 0.0f, CAP_ACT_AH);
                }
                portEXIT_CRITICAL(&g_stateMux);

                if (validLogV)
                    sendRP2040Uart('V', static_cast<int32_t>(msg.log_voltage * 1000.0f));
                if (validActV)
                    sendRP2040Uart('A', static_cast<int32_t>(msg.act_voltage * 1000.0f));
                if (validLogI)
                    sendRP2040Uart('v', static_cast<int32_t>(msg.log_current * 1000.0f));
                if (validActI)
                    sendRP2040Uart('a', static_cast<int32_t>(msg.act_current * 1000.0f));
            }
        }

        if (readTrustedUdp(udpRxServo, rxBuf, sizeof(rxBuf), len))
        {
            pb_istream_t s = pb_istream_from_buffer(rxBuf, len);
            Msg_Int32 msg = protolink__driver_msgs__Int32Stamped_driver_msgs__Int32Stamped_init_zero;
            if (pb_decode(&s, protolink__driver_msgs__Int32Stamped_driver_msgs__Int32Stamped_fields, &msg) &&
                msg.has_data && msg.data >= 0 && msg.data <= 180)
            {
                sendRP2040Uart('T', msg.data);
                portENTER_CRITICAL(&g_stateMux);
                g_dispServo = msg.data;
                portEXIT_CRITICAL(&g_stateMux);
            }
        }

        if (readTrustedUdp(udpRxRgb, rxBuf, sizeof(rxBuf), len))
        {
            pb_istream_t s = pb_istream_from_buffer(rxBuf, len);
            Msg_Int32 msg = protolink__driver_msgs__Int32Stamped_driver_msgs__Int32Stamped_init_zero;
            if (pb_decode(&s, protolink__driver_msgs__Int32Stamped_driver_msgs__Int32Stamped_fields, &msg) &&
                msg.has_data && msg.data >= 0 && msg.data <= 0xFFFFFF)
            {
                sendRP2040Uart('R', msg.data);
                portENTER_CRITICAL(&g_stateMux);
                g_dispRgb = msg.data;
                portEXIT_CRITICAL(&g_stateMux);
            }
        }

        if (readTrustedUdp(udpRxZed, rxBuf, sizeof(rxBuf), len))
        {
            pb_istream_t s = pb_istream_from_buffer(rxBuf, len);
            Msg_Bool msg = protolink__driver_msgs__BoolStamped_driver_msgs__BoolStamped_init_zero;
            if (pb_decode(&s, protolink__driver_msgs__BoolStamped_driver_msgs__BoolStamped_fields, &msg) &&
                msg.has_data)
            {
                digitalWrite(PIN_ZED_POWER, msg.data ? HIGH : LOW);
                portENTER_CRITICAL(&g_stateMux);
                g_dispZedPower = msg.data;
                portEXIT_CRITICAL(&g_stateMux);
            }
        }

        if (readTrustedUdp(udpRxImu, rxBuf, sizeof(rxBuf), len))
        {
            pb_istream_t s = pb_istream_from_buffer(rxBuf, len);
            Msg_Bool msg = protolink__driver_msgs__BoolStamped_driver_msgs__BoolStamped_init_zero;
            if (pb_decode(&s, protolink__driver_msgs__BoolStamped_driver_msgs__BoolStamped_fields, &msg) &&
                msg.has_data)
            {
                if (msg.data)
                {
                    digitalWrite(PIN_IMU_RESET, LOW);
                    vTaskDelay(pdMS_TO_TICKS(10));
                    digitalWrite(PIN_IMU_RESET, HIGH);
                }
                portENTER_CRITICAL(&g_stateMux);
                g_dispImuReset = msg.data;
                portEXIT_CRITICAL(&g_stateMux);
            }
        }

        if (readTrustedUdp(udpRxBuzzer, rxBuf, sizeof(rxBuf), len))
        {
            pb_istream_t s = pb_istream_from_buffer(rxBuf, len);
            Msg_Buzzer msg = protolink__driver_msgs__BuzzerSwitch_driver_msgs__BuzzerSwitch_init_zero;
            if (pb_decode(&s, protolink__driver_msgs__BuzzerSwitch_driver_msgs__BuzzerSwitch_fields, &msg))
            {
                if (msg.has_buzzer_stop && msg.buzzer_stop)
                    sendRP2040Uart('Q', 0);
                else if (msg.has_buzzer)
                    sendRP2040Uart('B', msg.buzzer ? 1 : 0);
            }
        }

        if (readTrustedUdp(udpRxDepthCfg, rxBuf, sizeof(rxBuf), len))
        {
            pb_istream_t s = pb_istream_from_buffer(rxBuf, len);
            Msg_Bool msg = protolink__driver_msgs__BoolStamped_driver_msgs__BoolStamped_init_zero;
            if (pb_decode(&s, protolink__driver_msgs__BoolStamped_driver_msgs__BoolStamped_fields, &msg) &&
                msg.has_data)
            {
                portENTER_CRITICAL(&g_stateMux);
                g_isSeawater = msg.data;
                portEXIT_CRITICAL(&g_stateMux);
            }
        }
    }
}

void Task_Main(void *pvParameters)
{
    // メインタスクもWDTで保護
    esp_task_wdt_add(NULL);
    uint32_t count = 0;
    uint8_t txBuf[256];
    bool pendingLeakState = (digitalRead(PIN_LEAK) == HIGH);
    uint32_t leakStateChangedMs = millis();

    for (;;)
    {
        esp_task_wdt_reset();

        checkRP2040Incoming();

        if (g_gnssFound)
        {
            myGNSS.checkUblox();
        }

        g_rp2040Alive = (millis() - rp2040State.lastUpdate < 1500);

        bool leakSample = (digitalRead(PIN_LEAK) == HIGH);
        if (leakSample != pendingLeakState)
        {
            pendingLeakState = leakSample;
            leakStateChangedMs = millis();
        }
        if (pendingLeakState != g_leakDetected &&
            millis() - leakStateChangedMs >= 50)
        {
            g_leakDetected = pendingLeakState;
            sendRP2040Uart(g_leakDetected ? 'L' : 'N', 0);
            sendLeakUDP(g_leakDetected);
        }
        if (millis() - g_lastLeakTxMs > 1000)
            sendLeakUDP(g_leakDetected);

        if (xSemaphoreTake(wire1Mutex, pdMS_TO_TICKS(50)) == pdTRUE)
        {
            int slot = count % 5;

            if (slot == 0)
            {
                if (count % 25 == 0){
                    updateOLED();
                }
                esp_task_wdt_reset();
            }
            else if (slot == 1)
            {
                handleTimeSync();
            }
            else if (slot == 2)
            { // Env Tx
                if (g_bmeFound)
                {
                    float temperature = bmeSensor.readTempC();
                    float humidity = bmeSensor.readFloatHumidity();
                    float pressure = bmeSensor.readFloatPressure() / 100.0f;
                    if (!isfinite(temperature) || !isfinite(humidity) || !isfinite(pressure) ||
                        temperature < -50.0f || temperature > 100.0f ||
                        humidity < 0.0f || humidity > 100.0f ||
                        pressure < 100.0f || pressure > 1200.0f)
                    {
                        static uint32_t lastInvalidEnvLogMs = 0;
                        if (millis() - lastInvalidEnvLogMs >= 1000)
                        {
                            Serial.println("[Env] invalid sensor reading");
                            lastInvalidEnvLogMs = millis();
                        }
                        xSemaphoreGive(wire1Mutex);
                        count++;
                        vTaskDelay(pdMS_TO_TICKS(20));
                        continue;
                    }
                    Msg_Environment msg = protolink__driver_msgs__Environment_driver_msgs__Environment_init_zero;
                    msg.has_temperature = true;
                    msg.temperature = temperature;
                    msg.has_humidity = true;
                    msg.humidity = humidity;
                    msg.has_pressure = true;
                    msg.pressure = pressure;
                    pb_ostream_t stream = pb_ostream_from_buffer(txBuf, sizeof(txBuf));
                    if (pb_encode(&stream, protolink__driver_msgs__Environment_driver_msgs__Environment_fields, &msg))
                    {
                        taskYIELD();
                        if (g_ethernetReady && xSemaphoreTake(ethMutex, pdMS_TO_TICKS(5)) == pdTRUE)
                        {
                            udpSender.beginPacket(pcIp, PORT_TX_ENV);
                            udpSender.write(txBuf, stream.bytes_written);
                            udpSender.endPacket();
                            xSemaphoreGive(ethMutex);
                        }
                    }
                    else
                        Serial.println("[Env] Tx NG | nanopb encode faild");
                }
            }
            else if (slot == 3)
            { // GNSS Tx
                if (g_gnssFound)
                {
                    Msg_RtcGnss gMsg = protolink__driver_msgs__RtcGnss_driver_msgs__RtcGnss_init_zero;
                    gMsg.has_latitude = true;
                    gMsg.latitude = myGNSS.getLatitude(0) / 10000000.0;
                    gMsg.has_longitude = true;
                    gMsg.longitude = myGNSS.getLongitude(0) / 10000000.0;
                    gMsg.has_altitude = true;
                    gMsg.altitude = myGNSS.getAltitude(0) / 1000.0;
                    gMsg.has_satellites = true;
                    gMsg.satellites = myGNSS.getSIV(0);
                    pb_ostream_t s1 = pb_ostream_from_buffer(txBuf, sizeof(txBuf));
                    if (pb_encode(&s1, protolink__driver_msgs__RtcGnss_driver_msgs__RtcGnss_fields, &gMsg))
                    {
                        taskYIELD();
                        if (g_ethernetReady && xSemaphoreTake(ethMutex, pdMS_TO_TICKS(5)) == pdTRUE)
                        {
                            udpSender.beginPacket(pcIp, PORT_TX_GNSS);
                            udpSender.write(txBuf, s1.bytes_written);
                            udpSender.endPacket();
                            xSemaphoreGive(ethMutex);
                        }
                    }
                    else
                        Serial.println("[GNSS] Tx NG | nanopb encode faild");
                }
            }
            else if (slot == 4)
            { // Battery/Signal Tx
                if (g_rp2040Alive)
                {
                    Msg_BtnBatt bMsg = protolink__driver_msgs__ButtonBatteryState_driver_msgs__ButtonBatteryState_init_zero;
                    bMsg.has_battery_leak = true;
                    bMsg.battery_leak = rp2040State.bat_leak;
                    bMsg.has_battery_rtc = true;
                    bMsg.battery_rtc = rp2040State.bat_rtc;
                    pb_ostream_t stream = pb_ostream_from_buffer(txBuf, sizeof(txBuf));
                    if (pb_encode(&stream, protolink__driver_msgs__ButtonBatteryState_driver_msgs__ButtonBatteryState_fields, &bMsg))
                    {
                        taskYIELD();
                        if (g_ethernetReady && xSemaphoreTake(ethMutex, pdMS_TO_TICKS(5)) == pdTRUE)
                        {
                            udpSender.beginPacket(pcIp, PORT_TX_BATTERY);
                            udpSender.write(txBuf, stream.bytes_written);
                            udpSender.endPacket();
                            xSemaphoreGive(ethMutex);
                        }
                    }
                    else
                        Serial.println("[Bat] Tx NG | nanopb encode faild");
                    vTaskDelay(pdMS_TO_TICKS(5));

                    Msg_SystemStatus sMsg = protolink__driver_msgs__SystemStatus_driver_msgs__SystemStatus_init_zero;
                    sMsg.has_jetson = true;
                    sMsg.jetson = rp2040State.sig_jetson;
                    sMsg.has_actuator_power = true;
                    sMsg.actuator_power = rp2040State.sig_act_pow;
                    sMsg.has_logic_relay = true;
                    sMsg.logic_relay = rp2040State.sig_logic_rly;
                    sMsg.has_usb_power = true;
                    sMsg.usb_power = rp2040State.sig_pc_pow;
                    stream = pb_ostream_from_buffer(txBuf, sizeof(txBuf));
                    if (pb_encode(&stream, protolink__driver_msgs__SystemStatus_driver_msgs__SystemStatus_fields, &sMsg))
                    {
                        taskYIELD();
                        if (g_ethernetReady && xSemaphoreTake(ethMutex, pdMS_TO_TICKS(5)) == pdTRUE)
                        {
                            udpSender.beginPacket(pcIp, PORT_TX_SIGNAL);
                            udpSender.write(txBuf, stream.bytes_written);
                            udpSender.endPacket();
                            xSemaphoreGive(ethMutex);
                        }
                    }
                }
            }
            xSemaphoreGive(wire1Mutex);
        }
        else{
            Serial.println("[Main] Failed to take Wire1 Mutex!");
        }

        if (millis() - lastPageChange > 4000)
        {
            currentPage = (currentPage + 1) % PAGE_MAX;
            lastPageChange = millis();
        }
        count++;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void sendLeakUDP(bool leak)
{
    // Ethernet停止時にも呼び出し頻度を1Hzに制限する
    g_lastLeakTxMs = millis();
    if (!g_ethernetReady)
        return;

    uint8_t buf[64];
    Msg_Bool msg = protolink__driver_msgs__BoolStamped_driver_msgs__BoolStamped_init_zero;
    msg.has_data = true;
    msg.data = leak;
    pb_ostream_t stream = pb_ostream_from_buffer(buf, sizeof(buf));
    if (pb_encode(&stream, protolink__driver_msgs__BoolStamped_driver_msgs__BoolStamped_fields, &msg))
    {
        if (xSemaphoreTake(ethMutex, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            udpSender.beginPacket(pcIp, PORT_TX_LEAK);
            udpSender.write(buf, stream.bytes_written);
            udpSender.endPacket();
            xSemaphoreGive(ethMutex);
        }
    }
}

void handleTimeSync()
{
    static unsigned long lastSync = 0;
    uint8_t buf[128];
    if (!g_rtcFound)
        return;

    if (g_gnssFound && myGNSS.getSIV(0) >= 4 &&
        myGNSS.getTimeValid(0) && myGNSS.getDateValid(0))
    {
        if (millis() - lastSync > 60000)
        {
            DateTime utc = DateTime(myGNSS.getYear(0), myGNSS.getMonth(0), myGNSS.getDay(0),
                                    myGNSS.getHour(0), myGNSS.getMinute(0), myGNSS.getSecond(0));
            DateTime jst = utc + TimeSpan(0, 9, 0, 0);
            rtc.adjust(jst);
            lastSync = millis();
        }
    }
    DateTime now = rtc.now();
    if (!now.isValid())
        return;
    Msg_RtcTime tMsg = protolink__driver_msgs__RtcTime_driver_msgs__RtcTime_init_zero;
    tMsg.has_year = true;
    tMsg.year = now.year();
    tMsg.has_hour = true;
    tMsg.hour = now.hour();
    tMsg.has_minute = true;
    tMsg.minute = now.minute();
    tMsg.has_second = true;
    tMsg.second = now.second();
    pb_ostream_t s = pb_ostream_from_buffer(buf, sizeof(buf));
    if (pb_encode(&s, protolink__driver_msgs__RtcTime_driver_msgs__RtcTime_fields, &tMsg))
    {
        taskYIELD();
        if (g_ethernetReady && xSemaphoreTake(ethMutex, pdMS_TO_TICKS(5)) == pdTRUE)
        {
            udpSender.beginPacket(bcastIp, PORT_TX_TIME);
            udpSender.write(buf, s.bytes_written);
            udpSender.endPacket();
            xSemaphoreGive(ethMutex);
        }
    }
}

void sendRP2040Uart(char type, int32_t val)
{
    if (xSemaphoreTake(uartMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        uint32_t rawValue = static_cast<uint32_t>(val);
        uint8_t buf[7];
        buf[0] = '<';
        buf[1] = type;
        buf[2] = (rawValue >> 0) & 0xFF;
        buf[3] = (rawValue >> 8) & 0xFF;
        buf[4] = (rawValue >> 16) & 0xFF;
        buf[5] = (rawValue >> 24) & 0xFF;
        buf[6] = '>';
        Serial1.write(buf, 7);
        xSemaphoreGive(uartMutex);
    }
}

void checkRP2040Incoming()
{
    int maxReads = 50;
    while (Serial1.available() && maxReads-- > 0)
    {
        if (Serial1.peek() == '<')
        {
            if (Serial1.available() >= 3)
            {
                Serial1.read();
                byte s = Serial1.read();
                int endMarker = Serial1.read();
                if (endMarker == '>')
                {
                    rp2040State.bat_leak = (s >> 0) & 1;
                    rp2040State.bat_rtc = (s >> 1) & 1;
                    rp2040State.sig_jetson = (s >> 2) & 1;
                    rp2040State.sig_act_pow = (s >> 3) & 1;
                    rp2040State.sig_logic_rly = (s >> 4) & 1;
                    rp2040State.sig_pc_pow = (s >> 5) & 1;
                    rp2040State.sig_act_rly = (s >> 6) & 1;
                    rp2040State.lastUpdate = millis();
                }
            }
            else
                return;
        }
        else
            Serial1.read();
    }
}

float estimateConsumedAhFromVoltage(float voltage, float capacityAh, int cells)
{
    if (cells <= 0)
        return capacityAh;
    float cellV = voltage / (float)cells;
    float pct = 0.0f;
    if (cellV >= 4.20f)
        pct = 1.0f;
    else if (cellV >= 4.00f)
        pct = 0.85f + (cellV - 4.00f) * (0.15f / 0.20f);
    else if (cellV >= 3.75f)
        pct = 0.50f + (cellV - 3.75f) * (0.35f / 0.25f);
    else if (cellV >= 3.60f)
        pct = 0.20f + (cellV - 3.60f) * (0.30f / 0.15f);
    else if (cellV >= 3.30f)
        pct = 0.00f + (cellV - 3.30f) * (0.20f / 0.30f);
    else
        pct = 0.0f;
    return capacityAh * (1.0f - pct);
}

void drawGuiHeader(const char *title)
{
    display.fillRect(0, 0, 128, 10, SH110X_WHITE);
    display.setTextColor(SH110X_BLACK);
    display.setTextSize(1);
    display.setCursor(64 - (strlen(title) * 3), 1);
    display.print(title);
    display.setTextColor(SH110X_WHITE);
    if (g_leakDetected)
    {
        display.fillRect(0, 0, 128, 64, SH110X_WHITE);
        display.setTextColor(SH110X_BLACK);
        display.setTextSize(2);
        display.setCursor(30, 25);
        display.print("! LEAK !");
    }
}

void drawGuiFooter()
{
    SharedStateSnapshot state = getSharedState();
    int y = 55;
    display.drawLine(0, 53, 128, 53, SH110X_WHITE);
    display.setCursor(2, y);
    display.print(g_rp2040Alive ? "RP:OK" : "RP:NO");
    display.setCursor(40, y);
    display.print(g_ethLinkStatus ? "NET:OK" : "NET:NO");

    display.setCursor(85, y);
    float remAh = CAP_LOGIC_AH - state.logConsumedAh;
    if (remAh < 0)
        remAh = 0;
    if (state.logCurrent > 0.1f)
    {
        float hours = remAh / state.logCurrent;
        if (hours > 99.9)
            hours = 99.9;
        display.printf("L:%.1fh", hours);
    }
    else
    {
        display.print("L:---");
    }
}

void drawCorner(int x, int y, int w, int h)
{
    int len = 4;
    display.drawLine(x, y, x + len, y, SH110X_WHITE);
    display.drawLine(x, y, x, y + len, SH110X_WHITE);
    display.drawLine(x + w - 1, y, x + w - 1 - len, y, SH110X_WHITE);
    display.drawLine(x + w - 1, y, x + w - 1, y + len, SH110X_WHITE);
    display.drawLine(x, y + h - 1, x + len, y + h - 1, SH110X_WHITE);
    display.drawLine(x, y + h - 1, x, y + h - 1 - len, SH110X_WHITE);
    display.drawLine(x + w - 1, y + h - 1, x + w - 1 - len, y + h - 1, SH110X_WHITE);
    display.drawLine(x + w - 1, y + h - 1, x + w - 1, y + h - 1 - len, SH110X_WHITE);
}

void drawSegBar(int x, int y, int w, int h, float val, float maxVal, bool usePercent)
{
    display.drawRect(x, y, w, h, SH110X_WHITE);
    int segs = 10;
    int active;
    if (usePercent)
        active = (int)((val / 100.0f) * segs);
    else if (maxVal > 0.0f)
        active = (int)((val / maxVal) * segs);
    else
        active = 0;
    if (active > segs)
        active = segs;
    if (active < 0)
        active = 0;
    int sw = (w - 4) / segs;
    for (int i = 0; i < active; i++)
    {
        display.fillRect(x + 2 + (i * sw), y + 2, sw - 1, h - 4, SH110X_WHITE);
    }
}

void updateOLED()
{
    if (!g_oledFound)
        return;
    SharedStateSnapshot state = getSharedState();
    display.clearDisplay();
    if (g_leakDetected)
    {
        display.fillScreen(SH110X_WHITE);
        display.setTextColor(SH110X_BLACK);
        display.setTextSize(2);
        display.setCursor(10, 25);
        display.print("EMERGENCY");
    }
    else
    {
        display.setTextColor(SH110X_WHITE);
        display.setTextSize(1);
        drawCorner(0, 12, 128, 40);
        switch (currentPage)
        {
        case PAGE_SYSTEM: // 1. System
        {
            drawGuiHeader("SYSTEM");
            display.setCursor(5, 16);
            if (g_rtcFound)
            {
                DateTime now = rtc.now();
                if (now.isValid())
                {
                    display.printf("DATE: %02d/%02d/%02d", now.year() % 100, now.month(), now.day());
                    display.setCursor(5, 26);
                    display.printf("TIME: %02d:%02d:%02d", now.hour(), now.minute(), now.second());
                }
                else
                {
                    display.print("RTC: READ ERROR");
                }
            }
            else
            {
                display.print("RTC: NOT FOUND");
            }
            display.setCursor(5, 36);
            unsigned long upSec = millis() / 1000;
            unsigned long upH = upSec / 3600;
            unsigned long upM = (upSec % 3600) / 60;
            unsigned long upS = upSec % 60;
            display.printf("UP  : %02lu:%02lu:%02lu", upH, upM, upS);
            break;
        }

        case PAGE_BATTERY: // 2. Battery (High Density)
        {
            drawGuiHeader("BATTERY");
            float lPct = (CAP_LOGIC_AH > 0) ? (1.0f - (state.logConsumedAh / CAP_LOGIC_AH)) * 100.0f : 0;
            float aPct = (CAP_ACT_AH > 0) ? (1.0f - (state.actConsumedAh / CAP_ACT_AH)) * 100.0f : 0;
            if (lPct < 0)
                lPct = 0;
            if (aPct < 0)
                aPct = 0;

            // L: 16.8V 1.5A 80%
            display.setCursor(2, 14);
            display.printf("L:%4.1fV %4.1fA %3.0f%%", state.logVoltage, state.logCurrent, lPct);
            drawSegBar(2, 22, 124, 4, lPct, 100.0f, true);

            // A: 25.2V 0.5A 50%
            display.setCursor(2, 30);
            display.printf("A:%4.1fV %4.1fA %3.0f%%", state.actVoltage, state.actCurrent, aPct);
            drawSegBar(2, 38, 124, 4, aPct, 100.0f, true);

            display.setCursor(2, 44);
            float remAh = CAP_LOGIC_AH - state.logConsumedAh;
            if (remAh < 0)
                remAh = 0;
            if (state.logCurrent > 0.1f)
            {
                float h = remAh / state.logCurrent;
                int hh = (int)h;
                int mm = (int)((h - hh) * 60);
                display.printf("Time:%02d:%02d(3.3V/C)", hh, mm);
            }
            else
            {
                display.print("Time:--:--(3.3V/C)");
            }
            break;
        }

        case PAGE_INSIDE: // 3. Hull Sensors
            drawGuiHeader("INSIDE");
            if (g_bmeFound)
            {
                float temperature = bmeSensor.readTempC();
                float pressure = bmeSensor.readFloatPressure() / 100.0f;
                float humidity = bmeSensor.readFloatHumidity();
                if (isfinite(temperature) && isfinite(pressure) && isfinite(humidity))
                {
                    display.setCursor(5, 16);
                    display.printf("Temp: %.1f C", temperature);
                    display.setCursor(5, 26);
                    display.printf("Pres: %.1f hPa", pressure);
                    display.setCursor(5, 36);
                    display.printf("Humi: %.1f %%", humidity);
                }
                else
                {
                    display.setCursor(5, 26);
                    display.print("Sensor Error");
                }
            }
            else
            {
                display.setCursor(5, 26);
                display.print("No Sensor");
            }
            break;

        case PAGE_DEPTH: // 4. Depth Sensor
        {
            drawGuiHeader("DEPTH");
            float d = 0.0f, t = 0.0f;
            if (xSemaphoreTake(depthMutex, pdMS_TO_TICKS(10)) == pdTRUE)
            {
                d = sharedDepth.depth;
                t = sharedDepth.temp;
                xSemaphoreGive(depthMutex);
            }
            display.setCursor(5, 16);
            display.printf("Type: %s", state.isSeawater ? "Sea" : "Fresh");
            display.setCursor(5, 26);
            display.printf("Dep : %.2f m", d);
            display.setCursor(5, 36);
            display.printf("Temp: %.1f C", t);
            break;
        }

        case PAGE_SIGNALS: // 5. Signals (RP2040)
            drawGuiHeader("SIGNALS");
            display.setCursor(5, 16);
            display.printf("Jetson: %d", rp2040State.sig_jetson);
            display.setCursor(64, 16);
            display.printf("L-Rly : %d", rp2040State.sig_logic_rly);
            display.setCursor(5, 26);
            display.printf("ActPow: %d", rp2040State.sig_act_pow);
            display.setCursor(64, 26);
            display.printf("A-Rly : %d", rp2040State.sig_act_rly);
            display.setCursor(5, 36);
            display.printf("PC-Pow: %d", rp2040State.sig_pc_pow);
            break;

        case PAGE_GNSS: // 6. GNSS
        {
            drawGuiHeader("GNSS");
            if (!g_gnssFound)
            {
                display.setCursor(5, 26);
                display.print("No GNSS");
                break;
            }
            display.setCursor(2, 16);
            display.printf("Lat: %.6f", myGNSS.getLatitude(0) / 10000000.0);
            display.setCursor(2, 26);
            display.printf("Lon: %.6f", myGNSS.getLongitude(0) / 10000000.0);

            // Fix Type Logic for Altitude Reliability
            byte fixType = myGNSS.getFixType(0); // 0=No, 1=DR, 2=2D, 3=3D...
            display.setCursor(2, 36);
            if (fixType >= 3)
            {
                display.printf("Alt: %.1f m (3D)", myGNSS.getAltitude(0) / 1000.0);
            }
            else if (fixType == 2)
            {
                display.print("Alt: 2D Fix (Unrel)");
            }
            else
            {
                display.print("Alt: No Fix");
            }
            break;
        }

        case PAGE_RX_INFO: // 7. RX Info
            drawGuiHeader("RX INFO");
            display.setCursor(5, 16);
            display.printf("Servo: %d", state.dispServo);
            display.setCursor(5, 26);
            if (state.dispRgb == -1)
            {
                display.print("RGB  : [-]");
            }
            else
            {
                // Assuming packed RGB (0xRRGGBB)
                int r = (state.dispRgb >> 16) & 0xFF;
                int g = (state.dispRgb >> 8) & 0xFF;
                int b = state.dispRgb & 0xFF;
                display.printf("R:%d G:%d B:%d", r, g, b);
            }
            display.setCursor(5, 36);
            display.printf("ZED:%d  IMU-R:%d", state.dispZedPower, state.dispImuReset);
            break;
        }
        drawGuiFooter();
    }
    display.display();
}
