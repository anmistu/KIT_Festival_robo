/*
 * PS4 Left/Right Stick (0..255, neutral band) -> Dual DC Motors via L298N
 * Board : Arduino UNO R3 + USB Host Shield 2.0 + BT dongle
 * A-CH  : D3=ENA(PWM), D4=IN1, D6=IN2  -> OUT1/OUT2 (Left stick Y)
 * B-CH  : D5=ENB(PWM), D7=IN3, D8=IN4  -> OUT3/OUT4 (Right stick Y)
 *
 * Controls:
 *   Left  Stick Y : Motor A（OUT1/2）  0..NEUTRAL_LO 逆転 / NEUTRAL帯 停止 / NEUTRAL_HI..255 正転
 *   Right Stick Y : Motor B（OUT3/4）  同上
 *   L1 (hold)     : 両モータ ブレーキ
 *   PS (click)    : 非常停止（両モータ停止）
 */

#include <SPI.h>
#include <usbhub.h>
#include <PS4BT.h>

// --- 通信は最初に学習した形を維持 ---
USB   Usb;
BTD   Btd(&Usb);
PS4BT PS4(&Btd, PAIR);

// ===== ドライバ：L298N =====
#define DRIVER_L298N 1

// ===== ピン割り当て（A=Left, B=Right）=====
const uint8_t PIN_PWM_A = 3;  // ENA  (PWM) ← D3
const uint8_t PIN_IN1_A = 4;  // IN1        ← D4
const uint8_t PIN_IN2_A = 6;  // IN2        ← D6

const uint8_t PIN_PWM_B = 5;  // ENB  (PWM) ← D5
const uint8_t PIN_IN3_B = 7;  // IN3        ← D7
const uint8_t PIN_IN4_B = 8;  // IN4        ← D8

// --- 中立バンド（この範囲は停止：必要に応じて調整）---
const uint8_t NEUTRAL_LO = 120;
const uint8_t NEUTRAL_HI = 135;

// デバッグ表示
const bool DEBUG_ECHO = false;

// ランプ（加減速をなめらかに）
const uint8_t  MAX_PWM_STEP  = 7;
const uint16_t UPDATE_PERIOD = 12;   // ms
const uint16_t FAILSAFE_MS   = 600;  // ms

// ランタイム（A/B 独立）
int16_t  targetPWM_A = 0, currentPWM_A = 0;
int16_t  targetPWM_B = 0, currentPWM_B = 0;
uint32_t lastUpdateMs_A = 0, lastUpdateMs_B = 0;
uint32_t lastRxMs = 0;

// ---- モータ制御（L298N）A側 ----
static inline void motorCoastA() {
  digitalWrite(PIN_IN1_A, LOW);
  digitalWrite(PIN_IN2_A, LOW);
  analogWrite(PIN_PWM_A, 0);
}
static inline void motorBrakeA() {
  digitalWrite(PIN_IN1_A, HIGH);
  digitalWrite(PIN_IN2_A, HIGH);
  analogWrite(PIN_PWM_A, 0);
}
static inline void motorDriveSignedA(int16_t pwmSigned) {
  int16_t mag = (pwmSigned >= 0) ? pwmSigned : -pwmSigned;
  if (mag == 0) { motorCoastA(); return; }
  if (pwmSigned > 0) { digitalWrite(PIN_IN1_A, HIGH); digitalWrite(PIN_IN2_A, LOW); }
  else               { digitalWrite(PIN_IN1_A, LOW);  digitalWrite(PIN_IN2_A, HIGH); }
  if (mag > 255) mag = 255;
  analogWrite(PIN_PWM_A, mag);
}

// ---- モータ制御（L298N）B側 ----
static inline void motorCoastB() {
  digitalWrite(PIN_IN3_B, LOW);
  digitalWrite(PIN_IN4_B, LOW);
  analogWrite(PIN_PWM_B, 0);
}
static inline void motorBrakeB() {
  digitalWrite(PIN_IN3_B, HIGH);
  digitalWrite(PIN_IN4_B, HIGH);
  analogWrite(PIN_PWM_B, 0);
}
static inline void motorDriveSignedB(int16_t pwmSigned) {
  int16_t mag = (pwmSigned >= 0) ? pwmSigned : -pwmSigned;
  if (mag == 0) { motorCoastB(); return; }
  if (pwmSigned > 0) { digitalWrite(PIN_IN3_B, HIGH); digitalWrite(PIN_IN4_B, LOW); }
  else               { digitalWrite(PIN_IN3_B, LOW);  digitalWrite(PIN_IN4_B, HIGH); }
  if (mag > 255) mag = 255;
  analogWrite(PIN_PWM_B, mag);
}

// ---- 0..255（NEUTRAL_LO..NEUTRAL_HI=停止）→ -255..+255 ----
static inline int16_t stickToSignedPWM_neutralBand(uint8_t v) {
  if (v <= NEUTRAL_LO) {
    int16_t mag = (int32_t)(NEUTRAL_LO - v) * 255 / NEUTRAL_LO;     // 0→最大逆転
    return -mag;
  } else if (v >= NEUTRAL_HI) {
    int16_t mag = (int32_t)(v - NEUTRAL_HI) * 255 / (255 - NEUTRAL_HI); // 255→最大正転
    return mag;
  } else {
    return 0; // 中立帯
  }
}

// ---- ランプ処理（A/B）----
static inline void applyRampA() {
  uint32_t now = millis();
  if ((uint32_t)(now - lastUpdateMs_A) < UPDATE_PERIOD) return;
  lastUpdateMs_A = now;
  if (currentPWM_A < targetPWM_A) {
    int16_t next = currentPWM_A + (int16_t)MAX_PWM_STEP;
    if (next > targetPWM_A) next = targetPWM_A;
    currentPWM_A = next;
  } else if (currentPWM_A > targetPWM_A) {
    int16_t next = currentPWM_A - (int16_t)MAX_PWM_STEP;
    if (next < targetPWM_A) next = targetPWM_A;
    currentPWM_A = next;
  }
}
static inline void applyRampB() {
  uint32_t now = millis();
  if ((uint32_t)(now - lastUpdateMs_B) < UPDATE_PERIOD) return;
  lastUpdateMs_B = now;
  if (currentPWM_B < targetPWM_B) {
    int16_t next = currentPWM_B + (int16_t)MAX_PWM_STEP;
    if (next > targetPWM_B) next = targetPWM_B;
    currentPWM_B = next;
  } else if (currentPWM_B > targetPWM_B) {
    int16_t next = currentPWM_B - (int16_t)MAX_PWM_STEP;
    if (next < targetPWM_B) next = targetPWM_B;
    currentPWM_B = next;
  }
}

// ---- フェイルセーフ（両モータ停止）----
static inline void failSafe(bool connected) {
  if (!connected) {
    targetPWM_A = currentPWM_A = 0;
    targetPWM_B = currentPWM_B = 0;
    motorCoastA();
    motorCoastB();
    return;
  }
  if ((uint32_t)(millis() - lastRxMs) > FAILSAFE_MS) {
    targetPWM_A = 0;
    targetPWM_B = 0;
    motorCoastA();
    motorCoastB();
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  if (Usb.Init() == -1) {
    Serial.println(F("USB Host Shield init failed."));
    while (1) { delay(1000); }
  }
  Serial.println(F("USB Host OK. Keep your PS4 pairing flow."));

  pinMode(PIN_PWM_A, OUTPUT);
  pinMode(PIN_IN1_A, OUTPUT);
  pinMode(PIN_IN2_A, OUTPUT);

  pinMode(PIN_PWM_B, OUTPUT);
  pinMode(PIN_IN3_B, OUTPUT);
  pinMode(PIN_IN4_B, OUTPUT);

  motorCoastA();
  motorCoastB();

  lastUpdateMs_A = lastUpdateMs_B = lastRxMs = millis();
}

void loop() {
  Usb.Task();                         // 通信維持
  const bool connected = PS4.connected();
  failSafe(connected);
  if (!connected) return;

  // 非常停止（PS）
  if (PS4.getButtonClick(PS)) {
    targetPWM_A = currentPWM_A = 0;
    targetPWM_B = currentPWM_B = 0;
    motorCoastA();
    motorCoastB();
    return;
  }

  // ブレーキ（L1）: 両モータ
  const bool braking = PS4.getButtonPress(L1);

  // 左スティックY → Motor A
  const uint8_t rawY_A = PS4.getAnalogHat(LeftHatY);
  const int16_t cmdA   = stickToSignedPWM_neutralBand(rawY_A);
  if (cmdA != targetPWM_A) { targetPWM_A = cmdA; lastRxMs = millis(); }

  // 右スティックY → Motor B
  const uint8_t rawY_B = PS4.getAnalogHat(RightHatY);
  const int16_t cmdB   = stickToSignedPWM_neutralBand(rawY_B);
  if (cmdB != targetPWM_B) { targetPWM_B = cmdB; lastRxMs = millis(); }

  if (DEBUG_ECHO) {
    Serial.print(F("LY=")); Serial.print(rawY_A);
    Serial.print(F(" pwmA=")); Serial.print(targetPWM_A);
    Serial.print(F(" | RY=")); Serial.print(rawY_B);
    Serial.print(F(" pwmB=")); Serial.println(targetPWM_B);
  }

  // 出力
  applyRampA();
  applyRampB();
  if (braking) {
    motorBrakeA();
    motorBrakeB();
  } else {
    motorDriveSignedA(currentPWM_A);
    motorDriveSignedB(currentPWM_B);
  }

  delay(1);
}
