/*
 * ESP32 Firmware V28.2 (Depth 20Hz / ARP Fix / Time Broadcast / Leak Periodic / IMU-ZED-WaterType)
 * Target: ESP32-D1-Mini + W5500
 *
 * 追加要素:
 * - ROS/PC → ESP32 経由の IMU リセット (9004) を PIN13 (EX_IO_1) で実行
 * - ZED 電源制御 (90010) を PIN14 (EX_IO_2) で実行
 * - 水質 (water_type, 9002) を受信し、MS5837 の fluidDensity に反映
 * - log_current / act_current (9011) を RP2040 へ UART で送信
 * - 必要であれば Depth メッセージに water_type を載せて PC へフィードバック
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
// EX_IO_3 / EX_IO_4 → 深度センサ I2C (A 系)
#define PIN_I2C_A_SDA 32 // EX_IO_3 (AD_IO)
#define PIN_I2C_A_SCL 33 // EX_IO_4 (AD_IO)

// I2C_B: GNSS / OLED / BME など
#define PIN_I2C_B_SDA 21 // SDA
#define PIN_I2C_B_SCL 22 // SCL

// RP2040 UART
#define PIN_RP_TX 27 // EX_IO_6 (UART: TX - RP)
#define PIN_RP_RX 34 // EX_IO_5 (UART: RX - RP)

// GNSS UART
#define PIN_GNSS_RX 16 // GNSS_TXO
#define PIN_GNSS_TX 17 // GNSS_RXI

// W5500
#define W5500_CS 26   // W5500_CS
#define W5500_RST 25  // W5500_RST
#define W5500_SCLK 18 // W5500_SCLK
#define W5500_MISO 19 // W5500_MISO
#define W5500_MOSI 23 // W5500_MOSI

// その他 IO
#define PIN_LEAK 39      // Leak_Sig (D_I)
#define PIN_IMU_RESET 13 // EX_IO_1 (AD_IO, IMU_Reset)
#define PIN_ZED_POWER 14 // EX_IO_2 (AD_IO, ZED_Pow)
#define PIN_PPS_IN 35    // GNSS_PPS (D_I)

#define ADDR_GNSS 0x42
#define ADDR_OLED 0x3C

// --- Network Config ---
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
IPAddress myIp(192, 168, 9, 5);
IPAddress pcIp(192, 168, 9, 100);    // メインPC
IPAddress bcastIp(192, 168, 9, 255); // サブネットブロードキャスト

// --- UDP Ports ---
#define PORT_TX_BATTERY 9000
#define PORT_TX_DEPTH 9002
#define PORT_TX_ENV 9003
#define PORT_TX_LEAK 9005
#define PORT_TX_GNSS 9007
#define PORT_TX_TIME 9008
#define PORT_TX_SIGNAL 9012

#define PORT_RX_BUZZER 9001    // Bool buzzer / buzzer_stop
#define PORT_RX_DEPTH_CFG 9002 // Bool water_type (1=海水, 0=真水)
#define PORT_RX_IMU 9004       // Bool imu_reset
#define PORT_RX_RGB 9006       // Int32 rgb
#define PORT_RX_SERVO 9009     // Int32 servo
#define PORT_RX_ZED 9010       // Bool zed_power
#define PORT_RX_POWER 9011     // Float32 log/act voltage & current

// --- Objects ---
EthernetUDP udpSender;
EthernetUDP udpRxBuzzer, udpRxDepthCfg, udpRxImu, udpRxRgb, udpRxServo, udpRxZed, udpRxPower;

// --- Helper: Force Bus Reset ---
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

// --- High Speed MS5837 ---
class RobustMS5837
{
private:
    uint16_t C[7];
    TwoWire *wirePort;
    uint8_t addr;
    float fluidDensity = 1029.0; // 初期値: 海水
    void osDelay(int ms) { vTaskDelay(pdMS_TO_TICKS(ms)); }
    void recoverBus()
    {
        forceBusReset(PIN_I2C_A_SDA, PIN_I2C_A_SCL);
        wirePort->begin(PIN_I2C_A_SDA, PIN_I2C_A_SCL);
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
        if (wirePort->available() >= 2)
            return (wirePort->read() << 8) | wirePort->read();
        return 0;
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
    RobustMS5837(TwoWire *w, uint8_t a = 0x76)
    {
        wirePort = w;
        addr = a;
    }
    void setFluidDensity(float density) { fluidDensity = density; }
    bool init()
    {
        recoverBus();
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
        float pressure = P / 10.0f;
        depthVal = (pressure - 1013.25f) / (9.80665f * fluidDensity / 100.0f);
        return true;
    }
};

RobustMS5837 depthSensor(&Wire, 0x76);
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

float g_logVoltage = 0.0f;
float g_actVoltage = 0.0f;
float g_logCurrent = 0.0f;
float g_actCurrent = 0.0f;

// water_type: true=海水, false=真水
bool g_isSeawater = true;

unsigned long g_lastLeakTxMs = 0; // Leak 定期送信用

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
    PAGE_STATUS,
    PAGE_ENV,
    PAGE_GNSS,
    PAGE_POWER,
    PAGE_MAX
};
int currentPage = PAGE_STATUS;
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
void drawSegBar(int x, int y, int w, int h, float val, float maxVal);

void setup()
{
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== BOOT V28.2 (IMU/ZED/WATER_TYPE) ===");

    esp_task_wdt_init(30, true);

    wire1Mutex = xSemaphoreCreateMutex();
    depthMutex = xSemaphoreCreateMutex();
    uartMutex = xSemaphoreCreateMutex();
    ethMutex = xSemaphoreCreateMutex();

    pinMode(PIN_LEAK, INPUT);
    pinMode(PIN_PPS_IN, INPUT);
    pinMode(PIN_IMU_RESET, OUTPUT);
    digitalWrite(PIN_IMU_RESET, HIGH); // 常時 HIGH
    pinMode(PIN_ZED_POWER, OUTPUT);
    digitalWrite(PIN_ZED_POWER, LOW); // ZED 初期は OFF

    Serial1.begin(115200, SERIAL_8N1, PIN_RP_RX, PIN_RP_TX);
    Serial2.begin(38400, SERIAL_8N1, PIN_GNSS_RX, PIN_GNSS_TX);

    // I2C_B: GNSS / OLED / BME
    forceBusReset(PIN_I2C_B_SDA, PIN_I2C_B_SCL);
    Wire1.begin(PIN_I2C_B_SDA, PIN_I2C_B_SCL);
    Wire1.setClock(400000);
    Wire1.setTimeOut(50);

    // SPI / W5500
    SPI.begin(W5500_SCLK, W5500_MISO, W5500_MOSI, W5500_CS);
    SPI.setFrequency(20000000);

    pinMode(W5500_RST, OUTPUT);
    digitalWrite(W5500_RST, LOW);
    delay(10);
    digitalWrite(W5500_RST, HIGH);
    delay(200);

    Ethernet.init(W5500_CS);
    Ethernet.begin(mac, myIp);
    Ethernet.setRetransmissionTimeout(300); // 30ms
    Ethernet.setRetransmissionCount(0);

    udpSender.begin(12345);
    udpRxBuzzer.begin(PORT_RX_BUZZER);
    udpRxDepthCfg.begin(PORT_RX_DEPTH_CFG);
    udpRxImu.begin(PORT_RX_IMU);
    udpRxRgb.begin(PORT_RX_RGB);
    udpRxServo.begin(PORT_RX_SERVO);
    udpRxZed.begin(PORT_RX_ZED);
    udpRxPower.begin(PORT_RX_POWER);

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
        display.println("V28.2 SYSTEM OK");
        display.display();
    }

    {
        SFE_UBLOX_GNSS configGNSS;
        if (configGNSS.begin(Wire1, ADDR_GNSS))
        {
            configGNSS.setVal32(0x20050030, 1); // PPS 1Hz 等の設定
            configGNSS.saveConfiguration();
        }
    }
    if (myGNSS.begin(Serial2))
        g_gnssFound = true;

    // Depth / NetRx / Main タスク起動
    xTaskCreatePinnedToCore(Task_Depth, "Depth", 8192, NULL, 10, NULL, 0);
    xTaskCreatePinnedToCore(Task_NetRx, "Rx", 8192, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(Task_Main, "Main", 8192, NULL, 2, NULL, 1);
}

void loop() { vTaskDelete(NULL); }

// ==========================================================================
// Task: Depth (Core 0, Priority 10)
// ==========================================================================
void Task_Depth(void *pvParameters)
{
    Wire.begin(PIN_I2C_A_SDA, PIN_I2C_A_SCL);
    Wire.setClock(400000);
    Wire.setTimeOut(5);

    // 初期 fluidDensity を water_type に合わせる
    depthSensor.setFluidDensity(g_isSeawater ? 1029.0f : 997.0f);

    while (!depthSensor.init())
        vTaskDelay(pdMS_TO_TICKS(1000));

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(50);
    uint8_t txBuf[128];

    for (;;)
    {
        bool ok = depthSensor.read();

        if (xSemaphoreTake(depthMutex, 5) == pdTRUE)
        {
            if (ok)
            {
                sharedDepth.depth = depthSensor.depthVal;
                sharedDepth.temp = depthSensor.tempVal;
                sharedDepth.ok = true;
            }
            else
            {
                sharedDepth.ok = false;
            }
            xSemaphoreGive(depthMutex);
        }

        if (ok)
        {
            Msg_Depth msg = protolink__driver_msgs__Depth_driver_msgs__Depth_init_zero;
            msg.has_depth = true;
            msg.depth = depthSensor.depthVal;
            msg.has_temperature = true;
            msg.temperature = depthSensor.tempVal;

            // ★ water_type を Depth メッセージに載せる場合は .proto で bool water_type を追加しておくこと
            //   例: bool water_type = 3;  // true=海水, false=真水
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

// ==========================================================================
// Task: Net Rx (Core 1, Priority 3)
// ==========================================================================
void Task_NetRx(void *pvParameters)
{
    uint8_t rxBuf[256];
    unsigned long lastLinkCheck = 0;

    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(10));

        // 500ms ごとにリンク状態を確認
        if (millis() - lastLinkCheck > 500)
        {
            if (xSemaphoreTake(ethMutex, 10) == pdTRUE)
            {
                g_ethLinkStatus = (Ethernet.linkStatus() == LinkON);
                xSemaphoreGive(ethMutex);
                lastLinkCheck = millis();
            }
        }

        // --- Power: log/act Voltage & Current (PORT_RX_POWER) ---
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
                    if (msg.has_log_current)
                    {
                        g_logCurrent = msg.log_current;
                        sendRP2040Uart('v', (int32_t)(g_logCurrent * 1000)); // log current (mA 相当)
                    }
                    if (msg.has_act_current)
                    {
                        g_actCurrent = msg.act_current;
                        sendRP2040Uart('a', (int32_t)(g_actCurrent * 1000)); // act current
                    }
                }
            }
            xSemaphoreGive(ethMutex);
        }
        taskYIELD();

        // --- Servo / RGB / Buzzer ---
        if (xSemaphoreTake(ethMutex, 5) == pdTRUE)
        {
            if (udpRxServo.parsePacket())
            {
                int len = udpRxServo.read(rxBuf, sizeof(rxBuf));
                pb_istream_t s = pb_istream_from_buffer(rxBuf, len);
                Msg_Int32 msg = protolink__driver_msgs__Int32Stamped_driver_msgs__Int32Stamped_init_zero;
                if (pb_decode(&s, protolink__driver_msgs__Int32Stamped_driver_msgs__Int32Stamped_fields, &msg))
                    sendRP2040Uart('T', msg.data);
            }
            else if (udpRxRgb.parsePacket())
            {
                int len = udpRxRgb.read(rxBuf, sizeof(rxBuf));
                pb_istream_t s = pb_istream_from_buffer(rxBuf, len);
                Msg_Int32 msg = protolink__driver_msgs__Int32Stamped_driver_msgs__Int32Stamped_init_zero;
                if (pb_decode(&s, protolink__driver_msgs__Int32Stamped_driver_msgs__Int32Stamped_fields, &msg))
                    sendRP2040Uart('R', msg.data);
            }
            else if (udpRxBuzzer.parsePacket())
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

        // --- IMU Reset (PORT_RX_IMU, BoolStamped) ---
        if (xSemaphoreTake(ethMutex, 5) == pdTRUE)
        {
            int packetSize = udpRxImu.parsePacket();
            if (packetSize)
            {
                int len = udpRxImu.read(rxBuf, sizeof(rxBuf));
                pb_istream_t s = pb_istream_from_buffer(rxBuf, len);
                Msg_Bool msg = protolink__driver_msgs__BoolStamped_driver_msgs__BoolStamped_init_zero;
                if (pb_decode(&s, protolink__driver_msgs__BoolStamped_driver_msgs__BoolStamped_fields, &msg))
                {
                    if (msg.has_data && msg.data)
                    {
                        // IMU Reset: LOW → 少し待って → HIGH（常時 HIGH）
                        digitalWrite(PIN_IMU_RESET, LOW);
                        vTaskDelay(pdMS_TO_TICKS(10));
                        digitalWrite(PIN_IMU_RESET, HIGH);
                    }
                }
            }
            xSemaphoreGive(ethMutex);
        }

        // --- ZED Power (PORT_RX_ZED, BoolStamped) ---
        if (xSemaphoreTake(ethMutex, 5) == pdTRUE)
        {
            int packetSize = udpRxZed.parsePacket();
            if (packetSize)
            {
                int len = udpRxZed.read(rxBuf, sizeof(rxBuf));
                pb_istream_t s = pb_istream_from_buffer(rxBuf, len);
                Msg_Bool msg = protolink__driver_msgs__BoolStamped_driver_msgs__BoolStamped_init_zero;
                if (pb_decode(&s, protolink__driver_msgs__BoolStamped_driver_msgs__BoolStamped_fields, &msg))
                {
                    if (msg.has_data)
                    {
                        digitalWrite(PIN_ZED_POWER, msg.data ? HIGH : LOW);
                    }
                }
            }
            xSemaphoreGive(ethMutex);
        }

        // --- Depth Config / water_type (PORT_RX_DEPTH_CFG, BoolStamped) ---
        //      data == true  -> 海水 (1029 kg/m^3)
        //      data == false -> 真水 (約 997 kg/m^3)
        if (xSemaphoreTake(ethMutex, 5) == pdTRUE)
        {
            int packetSize = udpRxDepthCfg.parsePacket();
            if (packetSize)
            {
                int len = udpRxDepthCfg.read(rxBuf, sizeof(rxBuf));
                pb_istream_t s = pb_istream_from_buffer(rxBuf, len);
                Msg_Bool msg = protolink__driver_msgs__BoolStamped_driver_msgs__BoolStamped_init_zero;
                if (pb_decode(&s, protolink__driver_msgs__BoolStamped_driver_msgs__BoolStamped_fields, &msg))
                {
                    if (msg.has_data)
                    {
                        g_isSeawater = msg.data; // 1=海水, 0=真水
                        float density = g_isSeawater ? 1029.0f : 997.0f;
                        depthSensor.setFluidDensity(density);
                    }
                }
            }
            xSemaphoreGive(ethMutex);
        }
    }
}

// ==========================================================================
// Task: Main (Core 1, Priority 2)
// ==========================================================================
void Task_Main(void *pvParameters)
{
    esp_task_wdt_add(NULL);
    uint32_t count = 0;
    uint8_t txBuf[256];

    for (;;)
    {
        esp_task_wdt_reset();
        checkRP2040Incoming();

        if (millis() - rp2040State.lastUpdate < 1500)
            g_rp2040Alive = true;
        else
            g_rp2040Alive = false;

        // Leak 検出（変化時に即送信）
        bool currentLeak = (digitalRead(PIN_LEAK) == HIGH);
        if (currentLeak != g_leakDetected)
        {
            g_leakDetected = currentLeak;
            sendRP2040Uart(g_leakDetected ? 'L' : 'N', 0);
            sendLeakUDP(currentLeak);
        }

        // Leak の 1Hz 定期送信
        if (millis() - g_lastLeakTxMs > 1000)
        {
            sendLeakUDP(g_leakDetected);
        }

        if (xSemaphoreTake(wire1Mutex, 50) == pdTRUE)
        {
            int slot = count % 10;

            if (slot == 0)
            {
                updateOLED();
                esp_task_wdt_reset();
            }
            else if (slot == 1)
            {
                handleTimeSync();
                if (g_gnssFound)
                    myGNSS.checkUblox();
            }
            else if (slot == 2)
            { // Env
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
            { // Battery & Signal
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
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// ==========================================================================
// Helpers
// ==========================================================================
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

// 時刻はブロードキャスト (192.168.9.255:9008)
void handleTimeSync()
{
    static unsigned long lastSync = 0;
    uint8_t buf[128];

    if (g_gnssFound && myGNSS.getSIV() >= 4 && myGNSS.getTimeValid())
    {
        if (millis() - lastSync > 60000)
        {
            rtc.adjust(DateTime(myGNSS.getYear(), myGNSS.getMonth(), myGNSS.getDay(),
                                myGNSS.getHour(), myGNSS.getMinute(), myGNSS.getSecond()));
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
                Serial1.read(); // '<'
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
        {
            Serial1.read();
        }
    }
}

// --- OLED 描画 ---
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
        return;
    }
}

void drawGuiFooter()
{
    int y = 55;
    display.drawLine(0, 53, 128, 53, SH110X_WHITE);
    display.setCursor(2, y);
    display.print(g_rp2040Alive ? "RP:OK" : "RP:NO");
    display.setCursor(45, y);
    display.print(g_ethLinkStatus ? "NET:OK" : "NET:NO");
    display.setCursor(90, y);
    display.print("SAT:");
    display.print(myGNSS.getSIV());
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

void drawSegBar(int x, int y, int w, int h, float val, float maxVal)
{
    display.drawRect(x, y, w, h, SH110X_WHITE);
    int segs = 10;
    int active = (int)((val / maxVal) * segs);
    if (active > segs)
        active = segs;
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
        case PAGE_STATUS:
        {
            drawGuiHeader("STATUS");
            display.setCursor(5, 18);
            DateTime now = rtc.now();
            display.printf("%02d:%02d:%02d", now.hour(), now.minute(), now.second());
            display.setCursor(5, 30);
            display.print("DEPTH:");
            float d = 0.0f;
            if (xSemaphoreTake(depthMutex, 10) == pdTRUE)
            {
                d = sharedDepth.depth;
                xSemaphoreGive(depthMutex);
            }
            display.setTextSize(2);
            display.setCursor(50, 26);
            display.print(d, 1);
            display.setTextSize(1);
            display.print("m");
            break;
        }
        case PAGE_ENV:
        {
            drawGuiHeader("ENVIRON");
            float t = 0.0f;
            if (xSemaphoreTake(depthMutex, 10) == pdTRUE)
            {
                t = sharedDepth.temp;
                xSemaphoreGive(depthMutex);
            }
            display.setCursor(5, 18);
            display.printf("W-Temp: %.1f C", t);
            if (g_bmeFound)
            {
                display.setCursor(5, 30);
                display.printf("Air-T : %.1f C", bmeSensor.readTempC());
                display.setCursor(5, 40);
                display.printf("Air-H : %.0f %%", bmeSensor.readFloatHumidity());
            }
            break;
        }
        case PAGE_GNSS:
        {
            drawGuiHeader("GNSS");
            display.setCursor(5, 18);
            display.print("LAT:");
            display.print(myGNSS.getLatitude() / 10000000.0, 5);
            display.setCursor(5, 28);
            display.print("LON:");
            display.print(myGNSS.getLongitude() / 10000000.0, 5);
            break;
        }
        case PAGE_POWER:
        {
            drawGuiHeader("POWER");
            display.setCursor(5, 16);
            display.printf("L:%.1fV", g_logVoltage);
            drawSegBar(60, 16, 60, 6, g_logVoltage, 16.8f);
            display.setCursor(5, 26);
            display.printf("A:%.1fV", g_actVoltage);
            drawSegBar(60, 26, 60, 6, g_actVoltage, 25.2f);
            display.setCursor(5, 38);
            display.printf("W:%s", g_isSeawater ? "Sea" : "Fresh");
            break;
        }
        }
        drawGuiFooter();
    }
    display.display();
}
