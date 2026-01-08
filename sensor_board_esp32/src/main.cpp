/*
 * ESP32 Firmware V30.6 (Advisor Refined - Auto Recovery)
 * - CRITICAL UPDATE: Added auto-recovery logic to Task_Depth.
 * If sensor read fails for >1 second, it triggers full bus reset and re-initialization.
 * - FIX: Previous V30.5 fixes (Wire.end usage, WDT, Priority) are preserved.
 */

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <esp_task_wdt.h>

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
    pinMode(sda, INPUT_PULLUP);
    pinMode(scl, OUTPUT);
    for (int i = 0; i < 9; i++)
    {
        digitalWrite(scl, LOW);
        delayMicroseconds(10);
        digitalWrite(scl, HIGH);
        delayMicroseconds(10);
    }
    pinMode(sda, OUTPUT);
    digitalWrite(sda, LOW);
    delayMicroseconds(10);
    digitalWrite(scl, HIGH);
    delayMicroseconds(10);
    digitalWrite(sda, HIGH);
    delayMicroseconds(10);
}

class RobustMS5837
{
private:
    uint16_t C[7];
    TwoWire *wirePort;
    uint8_t addr;
    int pin_sda;
    int pin_scl;
    float fluidDensity = 1029.0;

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

    void setFluidDensity(float density) { fluidDensity = density; }

    bool init()
    {
        recoverBus(); // 安全に呼び出せるようになった
        if (!writeCmd(0x1E))
            return false;
        osDelay(20);
        for (uint8_t i = 1; i <= 6; i++)
        {
            if (!writeCmd(0xA0 | (i * 2)))
                return false;
            C[i] = read16();
            osDelay(2);
        }
        return (C[1] > 0);
    }

    bool read()
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
        int32_t dT = D2 - (uint32_t)C[5] * 256;
        int32_t TEMP = 2000 + ((int64_t)dT * C[6]) / 8388608;
        int64_t OFF = (int64_t)C[2] * 65536 + ((int64_t)C[4] * dT) / 128;
        int64_t SENS = (int64_t)C[1] * 32768 + ((int64_t)C[3] * dT) / 256;
        int32_t P = (int32_t)((D1 * SENS / 2097152 - OFF) / 8192);
        tempVal = TEMP / 100.0f;
        depthVal = (P / 10.0f - 1013.25f) / (9.80665f * fluidDensity / 100.0f);
        return true;
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
bool g_rp2040Alive = false;
bool g_leakDetected = false;
volatile bool g_ethLinkStatus = false;
bool g_isSeawater = true;

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
float estimateConsumedAhFromVoltage(float voltage, float capacityAh);

void setup()
{
    //ラッチアップ防止
    delay(3000); 

    Serial.begin(115200);
    
    // WDTの初期化 (60秒)
    esp_task_wdt_init(60, true);

    // Setup実行中の自身の監視
    esp_task_wdt_add(NULL);

    wire1Mutex = xSemaphoreCreateMutex();
    depthMutex = xSemaphoreCreateMutex();
    uartMutex = xSemaphoreCreateMutex();
    ethMutex = xSemaphoreCreateMutex();

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

    Wire1.begin(PIN_I2C_B_SDA, PIN_I2C_B_SCL);
    Wire1.setClock(400000);
    Wire1.setTimeOut(50);

    esp_task_wdt_reset(); // WDTリセット

    SPI.begin(W5500_SCLK, W5500_MISO, W5500_MOSI, W5500_CS);
    SPI.setFrequency(20000000);
    pinMode(W5500_RST, OUTPUT);
    digitalWrite(W5500_RST, LOW);
    delay(10);
    digitalWrite(W5500_RST, HIGH);
    delay(200);

    Ethernet.init(W5500_CS);
    Ethernet.begin(mac, myIp);
    Ethernet.setRetransmissionTimeout(300);
    Ethernet.setRetransmissionCount(0);

    udpSender.begin(12345);
    udpRxBuzzer.begin(PORT_RX_BUZZER);
    udpRxDepthCfg.begin(PORT_RX_DEPTH_CFG);
    udpRxImu.begin(PORT_RX_IMU);
    udpRxRgb.begin(PORT_RX_RGB);
    udpRxServo.begin(PORT_RX_SERVO);
    udpRxZed.begin(PORT_RX_ZED);
    udpRxPower.begin(PORT_RX_POWER);

    esp_task_wdt_reset(); // WDTリセット

    bmeSensor.setI2CAddress(0x77);
    if (bmeSensor.beginI2C(Wire1))
        g_bmeFound = true;

    rtc.begin(&Wire1);

    if (display.begin(ADDR_OLED, true))
    {
        g_oledFound = true;
        display.setRotation(2);
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SH110X_WHITE);
        display.setCursor(0, 0);
        display.println("V30.6 SYSTEM OK");
        display.display();
    }

    esp_task_wdt_reset(); // WDTリセット

    // GNSS Init & Explicit PPS (TP1) Configuration
    {
        SFE_UBLOX_GNSS configGNSS;
        if (configGNSS.begin(Wire1, ADDR_GNSS))
        {
            // 1. Disable first to ensure clean state
            configGNSS.setVal8(0x10050007, 0); // TP1_ENA = 0

            // 2. Set params
            configGNSS.setVal32(0x40050024, 1);      // FREQ_TP1 = 1Hz
            configGNSS.setVal32(0x40050004, 100000); // LEN_TP1 = 100ms
            configGNSS.setVal8(0x20050030, 1);       // TIMEGRID = GPS
            configGNSS.setVal8(0x10050008, 1);       // USE_LOCKED = 1
            configGNSS.setVal8(0x1005000a, 1);       // ALIGN_TO_TOW = 1
            configGNSS.setVal8(0x10050006, 1);       // POL_TP1 = 1 (Falling/Inverted)

            // 3. Enable
            configGNSS.setVal8(0x10050007, 1); // TP1_ENA = 1

            // 4. Force Save to BBR/Flash
            configGNSS.saveConfiguration();
        }
    }
    if (myGNSS.begin(Serial2))
    {
        g_gnssFound = true;
    }

    g_lastPowerUpdateMs = millis();

    // 自身のWDT監視を解除
    esp_task_wdt_delete(NULL);

    // FIX: Depthタスクの優先度を 10 から 5 に下げる
    xTaskCreatePinnedToCore(Task_Depth, "Depth", 8192, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(Task_NetRx, "Rx", 8192, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(Task_Main, "Main", 8192, NULL, 2, NULL, 1);
}

void loop() { vTaskDelete(NULL); }

void Task_Depth(void *pvParameters)
{
    // FIX: WDT監視を追加。フリーズ時に再起動させる
    esp_task_wdt_add(NULL);

    // Wire.beginはsetupで行うため削除
    
    depthSensor.setFluidDensity(g_isSeawater ? 1029.0f : 997.0f);
    
    // 初期化ループにもWDTリセットとディレイを入れる
    while (!depthSensor.init())
    {
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

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

        bool ok = depthSensor.read();

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
            if (depthSensor.init()) 
            {
                errorCount = 0; // 復旧成功
            }
            else 
            {
                // 復旧失敗時は少し待ってリトライ（スパム防止）
                vTaskDelay(pdMS_TO_TICKS(100));
            }
        }
        
        if (xSemaphoreTake(depthMutex, 5) == pdTRUE)
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
            msg.watertype = g_isSeawater;
            pb_ostream_t stream = pb_ostream_from_buffer(txBuf, sizeof(txBuf));
            if (pb_encode(&stream, protolink__driver_msgs__Depth_driver_msgs__Depth_fields, &msg))
            {
                if (xSemaphoreTake(ethMutex, 30) == pdTRUE)
                {
                    udpSender.beginPacket(pcIp, PORT_TX_DEPTH);
                    udpSender.write(txBuf, stream.bytes_written);
                    udpSender.endPacket();
                    xSemaphoreGive(ethMutex);
                }
            }
        }
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void Task_NetRx(void *pvParameters)
{
    uint8_t rxBuf[256];
    unsigned long lastLinkCheck = 0;

    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(10));

        if (millis() - lastLinkCheck > 500)
        {
            if (xSemaphoreTake(ethMutex, 10) == pdTRUE)
            {
                g_ethLinkStatus = (Ethernet.linkStatus() == LinkON);
                xSemaphoreGive(ethMutex);
                lastLinkCheck = millis();
            }
        }

        // Power
        if (xSemaphoreTake(ethMutex, 5) == pdTRUE)
        {
            int packetSize = udpRxPower.parsePacket();
            if (packetSize)
            {
                int len = udpRxPower.read(rxBuf, sizeof(rxBuf));
                pb_istream_t s = pb_istream_from_buffer(rxBuf, len);
                Msg_Power msg = protolink__driver_msgs__PowerState_driver_msgs__PowerState_init_zero;
                if (pb_decode(&s, protolink__driver_msgs__PowerState_driver_msgs__PowerState_fields, &msg))
                {
                    unsigned long now = millis();
                    float dt_sec = (now - g_lastPowerUpdateMs) / 1000.0f;
                    if (dt_sec > 1.0f)
                        dt_sec = 0.0f;
                    g_lastPowerUpdateMs = now;

                    if (msg.has_log_voltage)
                    {
                        g_logVoltage = msg.log_voltage;
                        sendRP2040Uart('V', (int32_t)(g_logVoltage * 1000));
                    }
                    if (msg.has_act_voltage)
                    {
                        g_actVoltage = msg.act_voltage;
                        sendRP2040Uart('A', (int32_t)(g_actVoltage * 1000));
                    }
                    if (!g_batteryInitialized && msg.has_log_voltage && msg.has_act_voltage && g_logVoltage > 5.0f)
                    {
                        g_logConsumedAh = estimateConsumedAhFromVoltage(g_logVoltage, CAP_LOGIC_AH);
                        g_actConsumedAh = estimateConsumedAhFromVoltage(g_actVoltage, CAP_ACT_AH);
                        g_batteryInitialized = true;
                    }
                    if (msg.has_log_current)
                    {
                        g_logCurrent = msg.log_current; // 200.0f;
                        g_logConsumedAh += g_logCurrent * (dt_sec / 3600.0f);
                        sendRP2040Uart('v', (int32_t)(g_logCurrent * 1000));
                    }
                    if (msg.has_act_current)
                    {
                        g_actCurrent = msg.act_current; // 200.0f;
                        g_actConsumedAh += g_actCurrent * (dt_sec / 3600.0f);
                        sendRP2040Uart('a', (int32_t)(g_actCurrent * 1000));
                    }
                }
            }
            xSemaphoreGive(ethMutex);
        }
        taskYIELD();

        // Servo
        if (xSemaphoreTake(ethMutex, 5) == pdTRUE)
        {
            if (udpRxServo.parsePacket())
            {
                int len = udpRxServo.read(rxBuf, sizeof(rxBuf));
                pb_istream_t s = pb_istream_from_buffer(rxBuf, len);
                Msg_Int32 msg = protolink__driver_msgs__Int32Stamped_driver_msgs__Int32Stamped_init_zero;
                if (pb_decode(&s, protolink__driver_msgs__Int32Stamped_driver_msgs__Int32Stamped_fields, &msg))
                {
                    sendRP2040Uart('T', msg.data);
                    g_dispServo = msg.data;
                }
            }
            xSemaphoreGive(ethMutex);
        }

        // RGB
        if (xSemaphoreTake(ethMutex, 5) == pdTRUE)
        {
            if (udpRxRgb.parsePacket())
            {
                int len = udpRxRgb.read(rxBuf, sizeof(rxBuf));
                pb_istream_t s = pb_istream_from_buffer(rxBuf, len);
                Msg_Int32 msg = protolink__driver_msgs__Int32Stamped_driver_msgs__Int32Stamped_init_zero;
                if (pb_decode(&s, protolink__driver_msgs__Int32Stamped_driver_msgs__Int32Stamped_fields, &msg))
                {
                    sendRP2040Uart('R', msg.data);
                    g_dispRgb = msg.data;
                }
            }
            xSemaphoreGive(ethMutex);
        }

        // ZED
        if (xSemaphoreTake(ethMutex, 5) == pdTRUE)
        {
            if (udpRxZed.parsePacket())
            {
                int len = udpRxZed.read(rxBuf, sizeof(rxBuf));
                pb_istream_t s = pb_istream_from_buffer(rxBuf, len);
                Msg_Bool msg = protolink__driver_msgs__BoolStamped_driver_msgs__BoolStamped_init_zero;
                if (pb_decode(&s, protolink__driver_msgs__BoolStamped_driver_msgs__BoolStamped_fields, &msg))
                {
                    if (msg.has_data)
                    {
                        digitalWrite(PIN_ZED_POWER, msg.data ? HIGH : LOW);
                        g_dispZedPower = msg.data;
                    }
                }
            }
            xSemaphoreGive(ethMutex);
        }

        // IMU
        if (xSemaphoreTake(ethMutex, 5) == pdTRUE)
        {
            if (udpRxImu.parsePacket())
            {
                int len = udpRxImu.read(rxBuf, sizeof(rxBuf));
                pb_istream_t s = pb_istream_from_buffer(rxBuf, len);
                Msg_Bool msg = protolink__driver_msgs__BoolStamped_driver_msgs__BoolStamped_init_zero;
                if (pb_decode(&s, protolink__driver_msgs__BoolStamped_driver_msgs__BoolStamped_fields, &msg))
                {
                    if (msg.has_data && msg.data)
                    {
                        digitalWrite(PIN_IMU_RESET, LOW);
                        vTaskDelay(pdMS_TO_TICKS(10));
                        digitalWrite(PIN_IMU_RESET, HIGH);
                        g_dispImuReset = true;
                    }
                    else
                    {
                        g_dispImuReset = false;
                    }
                }
            }
            xSemaphoreGive(ethMutex);
        }

        // Buzzer
        if (xSemaphoreTake(ethMutex, 5) == pdTRUE)
        {
            if (udpRxBuzzer.parsePacket())
            {
                int len = udpRxBuzzer.read(rxBuf, sizeof(rxBuf));
                pb_istream_t s = pb_istream_from_buffer(rxBuf, len);
                Msg_Buzzer msg = protolink__driver_msgs__BuzzerSwitch_driver_msgs__BuzzerSwitch_init_zero;
                if (pb_decode(&s, protolink__driver_msgs__BuzzerSwitch_driver_msgs__BuzzerSwitch_fields, &msg))
                {
                    if (msg.has_buzzer_stop && msg.buzzer_stop)
                        sendRP2040Uart('Q', 0);
                    else if (msg.has_buzzer && msg.buzzer)
                        sendRP2040Uart('B', 1);
                }
            }
            xSemaphoreGive(ethMutex);
        }

        // Depth Config (Water Type)
        if (xSemaphoreTake(ethMutex, 5) == pdTRUE)
        {
            if (udpRxDepthCfg.parsePacket())
            {
                int len = udpRxDepthCfg.read(rxBuf, sizeof(rxBuf));
                pb_istream_t s = pb_istream_from_buffer(rxBuf, len);
                Msg_Bool msg = protolink__driver_msgs__BoolStamped_driver_msgs__BoolStamped_init_zero;
                if (pb_decode(&s, protolink__driver_msgs__BoolStamped_driver_msgs__BoolStamped_fields, &msg))
                {
                    if (msg.has_data)
                    {
                        g_isSeawater = msg.data;
                        depthSensor.setFluidDensity(g_isSeawater ? 1029.0f : 997.0f);
                    }
                }
            }
            xSemaphoreGive(ethMutex);
        }
    }
}

void Task_Main(void *pvParameters)
{
    // メインタスクもWDTで保護
    esp_task_wdt_add(NULL);
    uint32_t count = 0;
    uint8_t txBuf[256];

    for (;;)
    {
        esp_task_wdt_reset();
        checkRP2040Incoming();

        if (g_gnssFound)
        {
            myGNSS.checkUblox();
            myGNSS.checkUblox();
        }

        g_rp2040Alive = (millis() - rp2040State.lastUpdate < 1500);

        bool currentLeak = (digitalRead(PIN_LEAK) == HIGH);
        if (currentLeak != g_leakDetected)
        {
            g_leakDetected = currentLeak;
            sendRP2040Uart(g_leakDetected ? 'L' : 'N', 0);
            sendLeakUDP(currentLeak);
        }
        if (millis() - g_lastLeakTxMs > 1000)
            sendLeakUDP(g_leakDetected);

        if (xSemaphoreTake(wire1Mutex, 50) == pdTRUE)
        {
            int slot = count % 5;

            if (slot == 0)
            {
                if (count % 10 == 0)
                    updateOLED();
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
                    Msg_Environment msg = protolink__driver_msgs__Environment_driver_msgs__Environment_init_zero;
                    msg.has_temperature = true;
                    msg.temperature = bmeSensor.readTempC();
                    msg.has_humidity = true;
                    msg.humidity = bmeSensor.readFloatHumidity();
                    msg.has_pressure = true;
                    msg.pressure = bmeSensor.readFloatPressure() / 100.0f;
                    pb_ostream_t stream = pb_ostream_from_buffer(txBuf, sizeof(txBuf));
                    if (pb_encode(&stream, protolink__driver_msgs__Environment_driver_msgs__Environment_fields, &msg))
                    {
                        taskYIELD();
                        if (xSemaphoreTake(ethMutex, 5) == pdTRUE)
                        {
                            udpSender.beginPacket(pcIp, PORT_TX_ENV);
                            udpSender.write(txBuf, stream.bytes_written);
                            udpSender.endPacket();
                            xSemaphoreGive(ethMutex);
                        }
                    }
                }
            }
            else if (slot == 3)
            { // GNSS Tx
                if (g_gnssFound)
                {
                    Msg_RtcGnss gMsg = protolink__driver_msgs__RtcGnss_driver_msgs__RtcGnss_init_zero;
                    gMsg.has_latitude = true;
                    gMsg.latitude = myGNSS.getLatitude() / 10000000.0;
                    gMsg.has_longitude = true;
                    gMsg.longitude = myGNSS.getLongitude() / 10000000.0;
                    gMsg.has_altitude = true;
                    gMsg.altitude = myGNSS.getAltitude() / 1000.0;
                    gMsg.has_satellites = true;
                    gMsg.satellites = myGNSS.getSIV();
                    pb_ostream_t s1 = pb_ostream_from_buffer(txBuf, sizeof(txBuf));
                    if (pb_encode(&s1, protolink__driver_msgs__RtcGnss_driver_msgs__RtcGnss_fields, &gMsg))
                    {
                        taskYIELD();
                        if (xSemaphoreTake(ethMutex, 5) == pdTRUE)
                        {
                            udpSender.beginPacket(pcIp, PORT_TX_GNSS);
                            udpSender.write(txBuf, s1.bytes_written);
                            udpSender.endPacket();
                            xSemaphoreGive(ethMutex);
                        }
                    }
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
                        if (xSemaphoreTake(ethMutex, 5) == pdTRUE)
                        {
                            udpSender.beginPacket(pcIp, PORT_TX_BATTERY);
                            udpSender.write(txBuf, stream.bytes_written);
                            udpSender.endPacket();
                            xSemaphoreGive(ethMutex);
                        }
                    }
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
                        if (xSemaphoreTake(ethMutex, 5) == pdTRUE)
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
    uint8_t buf[64];
    Msg_Bool msg = protolink__driver_msgs__BoolStamped_driver_msgs__BoolStamped_init_zero;
    msg.has_data = true;
    msg.data = leak;
    pb_ostream_t stream = pb_ostream_from_buffer(buf, sizeof(buf));
    if (pb_encode(&stream, protolink__driver_msgs__BoolStamped_driver_msgs__BoolStamped_fields, &msg))
    {
        if (xSemaphoreTake(ethMutex, 10) == pdTRUE)
        {
            udpSender.beginPacket(pcIp, PORT_TX_LEAK);
            udpSender.write(buf, stream.bytes_written);
            udpSender.endPacket();
            xSemaphoreGive(ethMutex);
        }
        g_lastLeakTxMs = millis();
    }
}

void handleTimeSync()
{
    static unsigned long lastSync = 0;
    uint8_t buf[128];
    if (g_gnssFound && myGNSS.getSIV() >= 4 && myGNSS.getTimeValid())
    {
        if (millis() - lastSync > 60000)
        {
            DateTime utc = DateTime(myGNSS.getYear(), myGNSS.getMonth(), myGNSS.getDay(),
                                    myGNSS.getHour(), myGNSS.getMinute(), myGNSS.getSecond());
            DateTime jst = utc + TimeSpan(0, 9, 0, 0);
            rtc.adjust(jst);
            lastSync = millis();
        }
    }
    DateTime now = rtc.now();
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
        if (xSemaphoreTake(ethMutex, 5) == pdTRUE)
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
    if (xSemaphoreTake(uartMutex, 10) == pdTRUE)
    {
        uint8_t buf[7];
        buf[0] = '<';
        buf[1] = type;
        buf[2] = (val >> 0) & 0xFF;
        buf[3] = (val >> 8) & 0xFF;
        buf[4] = (val >> 16) & 0xFF;
        buf[5] = (val >> 24) & 0xFF;
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
            if (Serial1.available() >= 2)
            {
                Serial1.read();
                if (Serial1.available())
                {
                    byte s = Serial1.read();
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

float estimateConsumedAhFromVoltage(float voltage, float capacityAh)
{
    int cells = (voltage > 18.0f) ? 6 : 4;
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
    int y = 55;
    display.drawLine(0, 53, 128, 53, SH110X_WHITE);
    display.setCursor(2, y);
    display.print(g_rp2040Alive ? "RP:OK" : "RP:NO");
    display.setCursor(40, y);
    display.print(g_ethLinkStatus ? "NET:OK" : "NET:NO");

    display.setCursor(85, y);
    float remAh = CAP_LOGIC_AH - g_logConsumedAh;
    if (remAh < 0)
        remAh = 0;
    if (g_logCurrent > 0.1f)
    {
        float hours = remAh / g_logCurrent;
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
    display.drawLine(x + w - 1, y + h - 1, x + w - 1, y + h - 1 - len, SH110X_WHITE);
    display.drawLine(x + w - 1, y + h - 1, x + w - 1, y + h - 1 - len, SH110X_WHITE);
}

void drawSegBar(int x, int y, int w, int h, float val, float maxVal, bool usePercent)
{
    display.drawRect(x, y, w, h, SH110X_WHITE);
    int segs = 10;
    int active;
    if (usePercent)
        active = (int)((val / 100.0f) * segs);
    else
        active = (int)((val / maxVal) * segs);
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
            DateTime now = rtc.now();
            display.printf("DATE: %02d/%02d/%02d", now.year() % 100, now.month(), now.day());
            display.setCursor(5, 26);
            display.printf("TIME: %02d:%02d:%02d", now.hour(), now.minute(), now.second());
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
            int lCells = (g_logVoltage > 18.0) ? 6 : 4;
            int aCells = (g_actVoltage > 18.0) ? 6 : 4;
            float lPct = (CAP_LOGIC_AH > 0) ? (1.0f - (g_logConsumedAh / CAP_LOGIC_AH)) * 100.0f : 0;
            float aPct = (CAP_ACT_AH > 0) ? (1.0f - (g_actConsumedAh / CAP_ACT_AH)) * 100.0f : 0;
            if (lPct < 0)
                lPct = 0;
            if (aPct < 0)
                aPct = 0;

            // L: 16.8V 1.5A 80%
            display.setCursor(2, 14);
            display.printf("L:%4.1fV %4.1fA %3.0f%%", g_logVoltage, g_logCurrent, lPct);
            drawSegBar(2, 22, 124, 4, lPct, 100.0f, true);

            // A: 25.2V 0.5A 50%
            display.setCursor(2, 30);
            display.printf("A:%4.1fV %4.1fA %3.0f%%", g_actVoltage, g_actCurrent, aPct);
            drawSegBar(2, 38, 124, 4, aPct, 100.0f, true);

            display.setCursor(2, 44);
            float remAh = CAP_LOGIC_AH - g_logConsumedAh;
            if (remAh < 0)
                remAh = 0;
            if (g_logCurrent > 0.1f)
            {
                float h = remAh / g_logCurrent;
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
                display.setCursor(5, 16);
                display.printf("Temp: %.1f C", bmeSensor.readTempC());
                display.setCursor(5, 26);
                display.printf("Pres: %.1f hPa", bmeSensor.readFloatPressure() / 100.0f);
                display.setCursor(5, 36);
                display.printf("Humi: %.1f %%", bmeSensor.readFloatHumidity());
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
            if (xSemaphoreTake(depthMutex, 10) == pdTRUE)
            {
                d = sharedDepth.depth;
                t = sharedDepth.temp;
                xSemaphoreGive(depthMutex);
            }
            display.setCursor(5, 16);
            display.printf("Type: %s", g_isSeawater ? "Sea" : "Fresh");
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
            display.setCursor(2, 16);
            display.printf("Lat: %.6f", myGNSS.getLatitude() / 10000000.0);
            display.setCursor(2, 26);
            display.printf("Lon: %.6f", myGNSS.getLongitude() / 10000000.0);

            // Fix Type Logic for Altitude Reliability
            byte fixType = myGNSS.getFixType(); // 0=No, 1=DR, 2=2D, 3=3D...
            display.setCursor(2, 36);
            if (fixType >= 3)
            {
                display.printf("Alt: %.1f m (3D)", myGNSS.getAltitude() / 1000.0);
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
            display.printf("Servo: %d", g_dispServo);
            display.setCursor(5, 26);
            if (g_dispRgb == -1)
            {
                display.print("RGB  : [-]");
            }
            else
            {
                // Assuming packed RGB (0xRRGGBB)
                int r = (g_dispRgb >> 16) & 0xFF;
                int g = (g_dispRgb >> 8) & 0xFF;
                int b = g_dispRgb & 0xFF;
                display.printf("R:%d G:%d B:%d", r, g, b);
            }
            display.setCursor(5, 36);
            display.printf("ZED:%d  IMU-R:%d", g_dispZedPower, g_dispImuReset);
            break;
        }
        drawGuiFooter();
    }
    display.display();
}