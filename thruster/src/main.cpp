/*
  Thruster control for KYUBIC AUV

  Arduino receives 6 thrust values (in N) from main computer via USB.
  Format is *<value 1>;<value 2>;<value 3>;<value 4>;<value 5>;<value 6>#
  Input is to be ignored if invalid characters are detected.
  Program calculates respective PWM values for each thruster value using cubic formula.
  Once calculated, PWM values are written to the respective ESCs.
*/

#include <Arduino.h>
#include <Servo.h>

// #define DEBUG  // Define for debugging purposes

// Hardware Specification
const static uint8_t THRUSTER_NUM = 6;                                   // 搭載スラスター数
const static uint8_t THRUSTER_PIN[THRUSTER_NUM] = { 7, 4, 8, 3, 6, 5 };  // 信号のPIN番号

// 推力(N)からPWM値へ変換するの近似3次曲線の係数
const static float CCW_POSITIVE[4] = { -0.0015, 0.1724, -11.936, 1465.9 };
const static float CCW_NEGATIVE[4] = { -0.0035, -0.2981, -15.471, 1532.4 };
const static float CW_POSITIVE[4] = { 0.0015, -0.1724, 11.936, 1534.1 };
const static float CW_NEGATIVE[4] = { 0.0035, 0.2981, 15.471, 1467.6 };

// Limitation
const static float LIMIT_SINGLE_FORCE = 30;         // 1基あたりの最大推力[N]
const static float LIMIT_TOTAL_FORCE = 80.1430099;  // 合計最大推力[N]
const static uint16_t SERIAL_TIMEOUT = 1000;        // 通信が無ければ停止する時間[ms]

// Variable Declaration
int thrust_pwm[THRUSTER_NUM];              // Calculated PWM input for thrusters
unsigned long last_time = 0;               // 最後に指令値を受け取った時間

bool is_update = false;                    // Boolean to indicate thrust_value update
Servo thruster[THRUSTER_NUM];


bool decode_serial(String data, float thrust_value[]){
  bool is_error = false;                     // Boolean to indicate unexpected character
  String data_divided[THRUSTER_NUM];         // String divided from input
  for (int i = 0; i < THRUSTER_NUM; i++) data_divided[i] = "";
  
  // 区切り文字";"で文字列を分割
  uint8_t thruster_idx = 0;                  // Array indices for data_divided
  for (unsigned int i = 0; i < data.length(); i++) {
    char character = data.charAt(i);

    // 数字,ドット，マイナスのみ格納
    if (character == ';'){
        if(THRUSTER_NUM <= ++thruster_idx){ 
          is_error = true;
          break;
        }
    } 
    else if ((48 <= character && character <= 57) || character == '.' || character == '-') data_divided[thruster_idx] += character;
    else if (character == 0 || character == 10 || character == 13) {}  // ignore null, line feed or carriage return
    else {
      is_error = true;
      break;
    }
  }

  // スラスターの個数以外の指令値が来たらエラー
  if (thruster_idx != (THRUSTER_NUM - 1)) is_error = true;

  // Convert from String to Float. エラーがあれば指令値を変更しない
  if (is_error) {
    Serial.println("Unexpeced data format!!!!!!!!!!!!!!!!!!!!");
    Serial.println("");
    return false;
  }

  for (int i = 0; i < THRUSTER_NUM; i++) thrust_value[i] = data_divided[i].toFloat();
  last_time = millis();  // 最後の時間を記録
  return true;
}

void LimitThrust(float thrust[]) {
  // スラスタの合計推力を LIMIT_TOTAL_FORCE とする
  float sum = 0;
  for (int i = 0; i < THRUSTER_NUM; i++) sum += abs(thrust[i]);
  
  if (sum > LIMIT_TOTAL_FORCE) {
    float rate = sum / LIMIT_TOTAL_FORCE;
    for (int i = 0; i < THRUSTER_NUM; i++) thrust[i] /= rate;
  }

  // 各スラスタ推力の最大，最小値を ± LIMIT_SINGLE_FORCEにする
  for (int i = 0; i < THRUSTER_NUM; i++) {
    thrust[i] = max(-LIMIT_SINGLE_FORCE, min(thrust[i], LIMIT_SINGLE_FORCE));
  }
}

void WritePWM(float *thrust) {
  // Calculate for the PWM and write to thrusters
  for (int i = 0; i < THRUSTER_NUM; i++) {
    float x = thrust[i];
    if (i == 0 || i == 1 || i == 2) {  // for CCW thruster
      if (-0.4 < x && x < 0.4) {
        // 不感帯領域
        thrust_pwm[i] = 1500;
      } else if (x > 0) {
        // positive
        thrust_pwm[i] = round(CCW_POSITIVE[0] * pow(x, 3) + CCW_POSITIVE[1] * pow(x, 2) + CCW_POSITIVE[2] * x + CCW_POSITIVE[3]);
      } else {
        // negative
        thrust_pwm[i] = round(CCW_NEGATIVE[0] * pow(x, 3) + CCW_NEGATIVE[1] * pow(x, 2) + CCW_NEGATIVE[2] * x + CCW_NEGATIVE[3]);
      }
    } else {  // for CW thruster
      if (-0.4 < x && x < 0.4) {
        // 不感帯領域
        thrust_pwm[i] = 1500;
      } else if (x > 0) {
        // positive
        thrust_pwm[i] = round(CW_POSITIVE[0] * pow(x, 3) + CW_POSITIVE[1] * pow(x, 2) + CW_POSITIVE[2] * x + CW_POSITIVE[3]);
      } else {
        // negative
        thrust_pwm[i] = round(CW_NEGATIVE[0] * pow(x, 3) + CW_NEGATIVE[1] * pow(x, 2) + CW_NEGATIVE[2] * x + CW_NEGATIVE[3]);
      }
    }
  }
  for (int i = 0; i < THRUSTER_NUM; i++) {
    thruster[i].writeMicroseconds(thrust_pwm[i]);
  }
}



void setup() {
  Serial.begin(115200);
  Serial.println("I am Thruster Arduino.");
  Serial.println("Data format : *<value 1>;<value 2>;<value 3>;<value 4>;<value 5>;<value 6>#");

  // Initialize pins
  for (int i = 0; i < THRUSTER_NUM; i++) {
    thruster[i].attach(THRUSTER_PIN[i]);
  }

  // Write 1500 to initialize ESCs
  for (int i = 0; i < THRUSTER_NUM; i++) {
    thruster[i].writeMicroseconds(1500);
  }

  // delay to allow the ESC to recognize the stopped signal
  delay(7000);
}

void loop() {
  float thrust_value[THRUSTER_NUM] = {};         // float array for 6 input thrust values

  // Read command value
  if (Serial.read() == '*') {                    // checks if input is "*" to find header
    String input = Serial.readStringUntil('#');  // Read serial data until "#"
    is_update = decode_serial(input, thrust_value);    
  }

  // 規定時間以上モーター指令が無かったらスラスタ停止する
  if (SERIAL_TIMEOUT < (millis() - last_time)) {
    for (int i = 0; i < THRUSTER_NUM; i++) {
      thrust_value[i] = 0;
    }
    is_update = true;
  }

  // if is_update is true, write PWM
  if (is_update) {
    LimitThrust(thrust_value);
    WritePWM(thrust_value);
    is_update = false;
#ifdef DEBUG
    for (int i = 0; i < THRUSTER_NUM; i++) {
      Serial.print("Thruster ");
      Serial.print(i + 1);
      Serial.print(" : ");
      Serial.print("Thrust = ");
      Serial.print(thrust_value[i]);
      Serial.print(" PWM = ");
      Serial.println(thrust_pwm[i]);
    }
    Serial.println();
#endif
  }
}
