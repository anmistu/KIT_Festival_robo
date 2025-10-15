/*  _______.ino  (Arduino UNO + USB Host Shield 2.0 + PS4BT + L298N + 2xServo + NeoPixel)
 *
 *  機能:
 *   - PS4コントローラ（BT）で2モータ制御（左右スティックY）。中立帯 & ランプ（スロープ）で滑らかに。
 *   - 十字キー UP/DOWN：サーボ#1 を1クリックごとに ±60°（0..180°クランプ）
 *   - 十字キー LEFT/RIGHT：サーボ#2 を1クリックごとに ±60°（0..180°クランプ）
 *   - 〇ボタン：テープLED（WS2812B）をトグル（点灯/消灯）
 *   - L1：押している間は電気ブレーキ
 *   - PS：非常停止（Coast + 両サーボを初期角へ、LEDは状態維持）
 *
 *  配線:
 *   - L298N モータA（左）： ENA(PWM)=D3, IN1=D4, IN2=D6
 *   - L298N モータB（右）： ENB(PWM)=D5, IN3=D7, IN4=D8
 *   - サーボ#1：D2
 *   - サーボ#2：D9   （ServoライブラリはTimer1を使うがピンは任意可）
 *   - NeoPixel：DIN=A0(D14)  ※USB HostのSPI(10-13)と衝突しないように
 *
 *  注意:
 *   - NeoPixelは更新時にごく短時間割り込みが止まるため、連続更新は避ける（本コードはクリック時のみ更新）
 *   - テープLEDは本数に応じて十分な5V電源を用意し、GNDをArduino/USB Hostと確実に共通化すること
 */

#include <SPI.h>
#include <usbhub.h>
#include <PS4BT.h>
#include <Servo.h>
#include <Adafruit_NeoPixel.h>

// ===================== USB Host / PS4 =====================
USB     Usb;
BTD     Btd(&Usb);
PS4BT   PS4(&Btd);

// ===================== NeoPixel (WS2812B) =================
#define LED_PIN    A0       // DIN
#define LED_COUNT  30       // テープのLED数に合わせて
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

bool g_stripOn = false;     // 〇ボタンでトグル
uint8_t LED_R = 255, LED_G = 64, LED_B = 0; // 点灯色（暖色）
uint8_t LED_BRIGHTNESS = 64;

void stripApply() {
  if (g_stripOn) {
    for (int i = 0; i < LED_COUNT; ++i) {
      strip.setPixelColor(i, strip.Color(LED_R, LED_G, LED_B));
    }
  } else {
    for (int i = 0; i < LED_COUNT; ++i) {
      strip.setPixelColor(i, 0);
    }
  }
  strip.show();
}

// ===================== Servo ==============================
const uint8_t SERVO1_PIN      = 2;    // UP/DOWN で動くサーボ
const uint8_t SERVO2_PIN      = 0;    // LEFT/RIGHT で動くサーボ
const int     SERVO_MIN_DEG   = 0;
const int     SERVO_MAX_DEG   = 180;
const int     SERVO_STEP_DEG  = 60;   // 1クリックで動かす角度
const int     SERVO1_INIT_DEG = 90;   // 非常停止時に戻す角度
const int     SERVO2_INIT_DEG = 90;

Servo g_servo1;
Servo g_servo2;
int   g_servo1Angle = SERVO1_INIT_DEG;
int   g_servo2Angle = SERVO2_INIT_DEG;

// ===================== Motors (L298N) =====================
struct Motor {
  uint8_t pwmPin;   // ENA/ENB (PWM)
  uint8_t in1Pin;
  uint8_t in2Pin;

  int currentPWM;   // -255..+255
  int targetPWM;

  Motor() : pwmPin(255), in1Pin(255), in2Pin(255), currentPWM(0), targetPWM(0) {}
  Motor(uint8_t pwm, uint8_t in1, uint8_t in2)
  : pwmPin(pwm), in1Pin(in1), in2Pin(in2), currentPWM(0), targetPWM(0) {}

  void begin() {
    pinMode(in1Pin, OUTPUT);
    pinMode(in2Pin, OUTPUT);
    pinMode(pwmPin, OUTPUT);
    coast();
  }

  void setTargetPWM(int v) {
    if (v > 255) v = 255;
    if (v < -255) v = -255;
    targetPWM = v;
  }

  void coast() { // 自由回転
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
    analogWrite(pwmPin, 0);
    currentPWM = 0;
    targetPWM  = 0;
  }

  void brake() { // 電気ブレーキ
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, HIGH);
    analogWrite(pwmPin, 0);
    currentPWM = 0;
    targetPWM  = 0;
  }

  void apply(int pwm) {
    if (pwm == 0) {
      digitalWrite(in1Pin, LOW);
      digitalWrite(in2Pin, LOW);
      analogWrite(pwmPin, 0);
      return;
    }
    if (pwm > 0) {
      digitalWrite(in1Pin, HIGH);
      digitalWrite(in2Pin, LOW);
      analogWrite(pwmPin, pwm);
    } else {
      digitalWrite(in1Pin, LOW);
      digitalWrite(in2Pin, HIGH);
      analogWrite(pwmPin, -pwm);
    }
  }

  void updateRamp(int step) {
    if (currentPWM < targetPWM) {
      currentPWM += step;
      if (currentPWM > targetPWM) currentPWM = targetPWM;
    } else if (currentPWM > targetPWM) {
      currentPWM -= step;
      if (currentPWM < targetPWM) currentPWM = targetPWM;
    }
    apply(currentPWM);
  }
};

// ピン割り当て: 左モータA / 右モータB
Motor motorA(3, 4, 6);  // ENA=3(PWM), IN1=4, IN2=6
Motor motorB(5, 7, 8);  // ENB=5(PWM), IN3=7, IN4=8

// ===================== Control params =====================
const uint8_t  NEUTRAL_LO     = 120; // スティック中立帯 下限
const uint8_t  NEUTRAL_HI     = 135; // スティック中立帯 上限
const uint8_t  RAMP_STEP      = 7;   // ランプ1ステップのPWM変化量
const uint16_t UPDATE_PERIOD  = 12;  // ランプ更新周期[ms]
const uint16_t FAILSAFE_MS    = 600; // 入力喪失でCoastへ[ms]

uint32_t lastUpdateMs = 0;
uint32_t lastRxMs     = 0;

// ===================== Helpers ============================
int mapStickToPWM(uint8_t v) {
  // 0..255, 中立はおおよそ 127
  if (v >= NEUTRAL_LO && v <= NEUTRAL_HI) return 0;

  if (v > NEUTRAL_HI) {
    float t = (float)(v - NEUTRAL_HI) / (255.0f - (float)NEUTRAL_HI);
    int pwm = (int)(t * 255.0f + 0.5f);
    if (pwm > 255) pwm = 255;
    return pwm;
  } else { // v < NEUTRAL_LO
    float t = (float)(NEUTRAL_LO - v) / (float)NEUTRAL_LO;
    int pwm = (int)(t * 255.0f + 0.5f);
    if (pwm > 255) pwm = 255;
    return -pwm;
  }
}

void emergencyStopAll() {
  motorA.coast();
  motorB.coast();

  g_servo1Angle = SERVO1_INIT_DEG;
  g_servo2Angle = SERVO2_INIT_DEG;
  g_servo1.write(g_servo1Angle);
  g_servo2.write(g_servo2Angle);
  // LEDの状態は維持（必要ならここで g_stripOn=false; stripApply();）
}

void handleServo1Clicks() {
  // 十字キー UP/DOWN → サーボ#1 を ±60°
  if (PS4.getButtonClick(UP)) {
    g_servo1Angle += SERVO_STEP_DEG;
    if (g_servo1Angle > SERVO_MAX_DEG) g_servo1Angle = SERVO_MAX_DEG;
    g_servo1.write(g_servo1Angle);
  }
  if (PS4.getButtonClick(DOWN)) {
    g_servo1Angle -= SERVO_STEP_DEG;
    if (g_servo1Angle < SERVO_MIN_DEG) g_servo1Angle = SERVO_MIN_DEG;
    g_servo1.write(g_servo1Angle);
  }
}

void handleServo2Clicks() {
  // 十字キー LEFT/RIGHT → サーボ#2 を ±60°
  if (PS4.getButtonClick(RIGHT)) {
    g_servo2Angle += SERVO_STEP_DEG;
    if (g_servo2Angle > SERVO_MAX_DEG) g_servo2Angle = SERVO_MAX_DEG;
    g_servo2.write(g_servo2Angle);
  }
  if (PS4.getButtonClick(LEFT)) {
    g_servo2Angle -= SERVO_STEP_DEG;
    if (g_servo2Angle < SERVO_MIN_DEG) g_servo2Angle = SERVO_MIN_DEG;
    g_servo2.write(g_servo2Angle);
  }
}

// ===================== Setup / Loop =======================
void setup() {
  // Serial.begin(115200);

  if (Usb.Init() == -1) {
    // USB Host Shield 初期化失敗
    while (1) { delay(1000); }
  }

  motorA.begin();
  motorB.begin();

  g_servo1.attach(SERVO1_PIN);
  g_servo2.attach(SERVO2_PIN);
  g_servo1.write(g_servo1Angle);
  g_servo2.write(g_servo2Angle);

  // NeoPixel 初期化
  strip.begin();
  strip.setBrightness(LED_BRIGHTNESS); // 0..255
  g_stripOn = false;
  stripApply();                        // 消灯反映
}

void loop() {
  Usb.Task();

  // 未接続時はフェイルセーフ
  if (!PS4.connected()) {
    if (millis() - lastRxMs > FAILSAFE_MS) {
      motorA.coast();
      motorB.coast();
    }
    return;
  }
  lastRxMs = millis();

  // 非常停止（PS）
  if (PS4.getButtonClick(PS)) {
    emergencyStopAll();
    return; // このフレームは終了
  }

  // L1押下中はブレーキ
  if (PS4.getButtonPress(L1)) {
    motorA.brake();
    motorB.brake();
  } else {
    // スティックから目標PWM生成
    uint8_t ly = PS4.getAnalogHat(LeftHatY);
    uint8_t ry = PS4.getAnalogHat(RightHatY);

    // PS4のY軸は 上=0 / 下=255 → 「上=前進」にしたいので符号反転
    int targetA = -mapStickToPWM(ly);
    int targetB = -mapStickToPWM(ry);

    motorA.setTargetPWM(targetA);
    motorB.setTargetPWM(targetB);
  }

  // サーボ：十字キーで段階回転
  handleServo1Clicks(); // UP/DOWN → サーボ#1
  handleServo2Clicks(); // LEFT/RIGHT → サーボ#2

  // 〇ボタンでテープLED トグル
  if (PS4.getButtonClick(CIRCLE)) {
    g_stripOn = !g_stripOn;
    stripApply();
  }

  // 一定周期でスロープ更新
  uint32_t now = millis();
  if (now - lastUpdateMs >= UPDATE_PERIOD) {
    lastUpdateMs = now;
    motorA.updateRamp(RAMP_STEP);
    motorB.updateRamp(RAMP_STEP);
  }
}
