/*
  Thruster control for KYUBIC AUV

  Arduino receives 6 thrust values (in N) from main computer via USB.
  Format is *<value 1>;<value 2>;<value 3>;<value 4>;<value 5>;<value 6>#
  Input is to be ignored if invalid characters are detected.
  Program calculates respective PWM values for each thruster value using cubic formula.
  Once calculated, PWM values are written to the respective ESCs.
*/

#include <Servo.h>
// #define DEBUG  // Define for debugging purposes

const static uint8_t THRUSTER_NUM = 6;  // 搭載スラスター数
const static uint8_t THRUSTER_PIN[THRUSTER_NUM] = { 7, 4, 8, 3, 6, 5 };

// 推力(N)からPWM値へ変換するの近似3次曲線の係数
const static float CCW_POSITIVE[4] = { -0.0015, 0.1724, -11.936, 1465.9 };
const static float CCW_NEGATIVE[4] = { -0.0035, -0.2981, -15.471, 1532.4 };
const static float CW_POSITIVE[4] = { 0.0015, -0.1724, 11.936, 1534.1 };
const static float CW_NEGATIVE[4] = { 0.0035, 0.2981, 15.471, 1467.6 };

const static float LIMIT_SINGLE_FORCE = 30;         // 1基あたりの最大推力[N]
const static float LIMIT_TOTAL_FORCE = 80.1430099;  // 合計最大推力[N]
const static uint16_t SERIAL_TIMEOUT = 1000;        // 通信が無ければ停止する時間[ms]

String input;                              // Where to store the serial data
String input_divided[THRUSTER_NUM];        // String divided from input
char input_char;                           // char extracted from input_divided
uint8_t input_index;                       // Array indices for input_divided
float thrust_value[THRUSTER_NUM] = { 0 };  // float array for 6 input thrust values
int thrust_pwm[THRUSTER_NUM];              // Calculated PWM input for thrusters
unsigned long last_time = 0;               // 最後に指令値を受け取った時間
bool is_error = false;                     // Boolean to indicate unexpected character
bool is_update = false;                    // Boolean to indicate thrust_value update

Servo thruster[THRUSTER_NUM];


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
  if (Serial.read() == '*') {             // checks if input is "*" to find header
    input = Serial.readStringUntil('#');  // Read serial data until "#"

    // 区切り文字";"で文字列を分割
    for (int i = 0; i < THRUSTER_NUM; i++) {
      input_divided[i] = "";
    }
    input_index = 0;
    for (int i = 0; i < input.length(); i++) {
      input_char = input.charAt(i);
      if (input_char == ';') {
        input_index++;
        if (THRUSTER_NUM <= input_index) {
          is_error = true;
          break;
        }
      } else if ((48 <= input_char && input_char <= 57) || input_char == '.' || input_char == '-') {  // 数字のみ格納
        input_divided[input_index] += input_char;
      } else if (input_char == 0 || input_char == 10 || input_char == 13) {  // ignore null, line feed or carriage return
      } else {
        is_error = true;
        break;
      }
    }
    if (input_index != (THRUSTER_NUM - 1)) {  // スラスターの個数以外の指令値が来たらエラー
      is_error = true;
    }

    if (is_error) {  // エラーがあれば指令値を変更しない
      Serial.println("Unexpeced data format!!!!!!!!!!!!!!!!!!!!");
      Serial.println("");
      is_error = false;
    } else {  // エラーがなければ指令値を更新
      for (int i = 0; i < THRUSTER_NUM; i++) {
        thrust_value[i] = input_divided[i].toFloat();
      }
      is_update = true;
      last_time = millis();  //最後の時間を記録
    }
  }

  if (SERIAL_TIMEOUT < (millis() - last_time)) {  // 規定時間以上モーター指令が無かったらスラスタ停止する
    for (int i = 0; i < THRUSTER_NUM; i++) {
      thrust_value[i] = 0;
    }
    is_update = true;
  }

  if (is_update) {
    LimitThrust();
    GetWritePWM(thrust_value);
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

void GetWritePWM(float *thrust) {
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


void LimitThrust() {
  for (int i = 0; i < THRUSTER_NUM; i++) {
    if (thrust_value[i] > LIMIT_SINGLE_FORCE) {
      thrust_value[i] = LIMIT_SINGLE_FORCE;
    }
    if (thrust_value[i] < -LIMIT_SINGLE_FORCE) {
      thrust_value[i] = -LIMIT_SINGLE_FORCE;
    }
  }
  float sum = 0;
  float rate = 0;
  for (int i = 0; i < THRUSTER_NUM; i++) {
    sum += abs(thrust_value[i]);
  }

  rate = sum / LIMIT_TOTAL_FORCE;

  if (rate > 1) {
    for (int i = 0; i < THRUSTER_NUM; i++) {
      thrust_value[i] = thrust_value[i] / rate;
    }
  }
}