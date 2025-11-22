/*
 * RP2040 Firmware V15.1 (Non-Blocking Fix)
 * Fixes:
 * - Removed delay() and heavy logic from ISR.
 * - Implemented flag-based processing in loop().
 */

#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_NeoPixel.h>

#define I2C_ADDR        0x08

// --- Pin Definitions ---
#define PIN_SERVO_1     0
#define PIN_SERVO_POWER 6    
#define PIN_BAT_MEASURE 8    
#define PIN_IMU_RESET   2    
#define PIN_HEARTBEAT   3    
#define PIN_BUZZER      7

#define PIN_LED_EFFECT  14  
#define NUM_LEDS_EFFECT 6
#define PIN_LED_STATUS  15  
#define NUM_LEDS_STATUS 35 

#define PIN_BAT_LEAK_CHK 26 
#define PIN_BAT_RTC_CHK  27 
#define PIN_SIG_JETSON   10
#define PIN_SIG_ACT      11
#define PIN_SIG_LOGIC    12
#define PIN_SIG_USB      13

// --- Limits ---
#define SERVO_LIM_MIN 45
#define SERVO_LIM_MAX 135

// --- Objects ---
Adafruit_NeoPixel effectStrip(NUM_LEDS_EFFECT, PIN_LED_EFFECT, NEO_RGB + NEO_KHZ800);
Adafruit_NeoPixel statusStrip(NUM_LEDS_STATUS, PIN_LED_STATUS, NEO_GRB + NEO_KHZ800);
Servo tiltServo;

// --- Volatile Data (Shared between ISR and Loop) ---
volatile bool g_buzzerActive = false;
volatile bool g_quietMode = false;

volatile bool g_flagImuReset = false;
volatile bool g_flagServoUpdate = false;
volatile int32_t g_targetServoVal = 90;

// Status Cache for I2C Read
volatile byte g_statusCache = 0;

// Prototypes
void receiveEvent(int howMany);
void requestEvent();

// ==========================================================================
// Setup
// ==========================================================================
void setup() {
    analogReadResolution(12); 

    // Pin Modes
    pinMode(PIN_SERVO_POWER, OUTPUT);
    digitalWrite(PIN_SERVO_POWER, HIGH); 
    delay(100);

    tiltServo.attach(PIN_SERVO_1);
    tiltServo.write(90); 

    pinMode(PIN_BUZZER, OUTPUT);
    pinMode(PIN_IMU_RESET, OUTPUT); digitalWrite(PIN_IMU_RESET, HIGH);
    pinMode(PIN_HEARTBEAT, OUTPUT); 
    pinMode(PIN_BAT_MEASURE, OUTPUT); 

    pinMode(PIN_SIG_JETSON, INPUT);
    pinMode(PIN_SIG_ACT, INPUT);
    pinMode(PIN_SIG_LOGIC, INPUT);
    pinMode(PIN_SIG_USB, INPUT);

    // Init LED (One shot)
    effectStrip.begin();
    effectStrip.setBrightness(100);
    for(int i=0; i<NUM_LEDS_EFFECT; i++) effectStrip.setPixelColor(i, 0x00FFFF);
    effectStrip.show();

    statusStrip.begin();
    statusStrip.setBrightness(100);
    for(int i=0; i<NUM_LEDS_STATUS; i++) statusStrip.setPixelColor(i, 0x00FFFF);
    statusStrip.show();

    // I2C Init (Must be last to ensure vars are ready)
    Wire.begin(I2C_ADDR);
    Wire.onReceive(receiveEvent);
    Wire.onRequest(requestEvent);
}

// ==========================================================================
// Main Loop (Process Flags Here)
// ==========================================================================
void loop() {
    // 1. Handle IMU Reset (Blocking delay is safe here)
    if (g_flagImuReset) {
        g_flagImuReset = false;
        digitalWrite(PIN_IMU_RESET, LOW);
        delay(100); 
        digitalWrite(PIN_IMU_RESET, HIGH);
    }

    // 2. Handle Servo Update
    if (g_flagServoUpdate) {
        g_flagServoUpdate = false;
        tiltServo.write(g_targetServoVal);
    }

    // 3. Update Status Cache (Periodically)
    static unsigned long lastAdcTime = 0;
    if (millis() - lastAdcTime > 100) {
        lastAdcTime = millis();
        
        // Analog Reads take time, do them here
        bool batLeakOk = (analogRead(PIN_BAT_LEAK_CHK) > 2000); 
        bool batRtcOk  = (analogRead(PIN_BAT_RTC_CHK)  > 2000);

        byte s = 0;
        s |= (batLeakOk ? 1 : 0) << 0;
        s |= (batRtcOk  ? 1 : 0) << 1;
        s |= (digitalRead(PIN_SIG_JETSON) ? 1 : 0) << 2;
        s |= (digitalRead(PIN_SIG_ACT) ? 1 : 0) << 3;
        s |= (digitalRead(PIN_SIG_LOGIC) ? 1 : 0) << 4;
        s |= (digitalRead(PIN_SIG_USB) ? 1 : 0) << 5;
        
        // Atomic update
        noInterrupts();
        g_statusCache = s;
        interrupts();
    }

    // 4. Heartbeat & Buzzer
    digitalWrite(PIN_HEARTBEAT, (millis() / 500) % 2);

    if (g_buzzerActive && !g_quietMode) {
        digitalWrite(PIN_BUZZER, (millis() / 200) % 2);
    } else {
        digitalWrite(PIN_BUZZER, LOW);
    }
    
    // Small delay to yield
    delay(1);
}

// ==========================================================================
// I2C ISRs (Keep Extremely Short!)
// ==========================================================================

// Rx: ESP32 -> RP2040
void receiveEvent(int howMany) {
    if (Wire.available()) {
        char cmd = Wire.read();
        int32_t val = 0;
        if (Wire.available() >= 4) {
            val |= ((int32_t)Wire.read() << 24);
            val |= ((int32_t)Wire.read() << 16);
            val |= ((int32_t)Wire.read() << 8);
            val |= (int32_t)Wire.read();
        }

        switch(cmd) {
            case 'T': // Servo
                g_targetServoVal = constrain(val, SERVO_LIM_MIN, SERVO_LIM_MAX);
                g_flagServoUpdate = true;
                break;
            case 'I': // IMU Reset
                if(val == 1) g_flagImuReset = true;
                break;
            case 'B': // Buzzer
                g_buzzerActive = (val > 0);
                if(val > 0) g_quietMode = false;
                break;
            case 'Q': // Quiet
                g_quietMode = true;
                break;
        }
        
        // Flush any garbage
        while(Wire.available()) Wire.read();
    }
}

// Tx: RP2040 -> ESP32
void requestEvent() {
    // Just return the pre-calculated byte
    Wire.write(g_statusCache);
}