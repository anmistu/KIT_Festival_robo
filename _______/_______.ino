/*  _______-servo.ino  (UNO/USB Host Shield/PS4/L298N/Servo)
 *  修正点:
 *   - Motor 構造体に 3引数コンストラクタを追加（pwmPin, in1Pin, in2Pin）
 *   - 十字キーUP/DOWNをクリック毎に±60°の段階回転
 *   - 既存: スティック中立帯, スロープ, フェイルセーフ, L1ブレーキ, PS非常停止
 */

#include <SPI.h>
#include <usbhub.h>
#include <PS4BT.h>
#include <Servo.h>

// ============== USB Host / PS4 ==============
USB     Usb;
BTD     Btd(&Usb);
PS4BT   PS4(&Btd);

// ============== Servo =======================
const uint8_t SERVO_PIN      = 2;
const int     SERVO_MIN_DEG  = 0;
const int     SERVO_MAX_DEG  = 180;
const int     SERVO_STEP_DEG = 60;   // 1クリックで動かす角度
const int     SERVO_INIT_DEG = 90;   // 非常停止時に戻す角度

Servo g_servo;
int   g_servoAngle = SERVO_INIT_DEG;


// ===== Servo #2 (LEFT/RIGHT クリックで±60°) =====
const uint8_t SERVO2_PIN      = 0;   // ←未使用のピンに変更可
const int     SERVO2_MIN_DEG  = 0;
const int     SERVO2_MAX_DEG  = 180;
const int     SERVO2_STEP_DEG = 60;  // 1クリックで動かす角度
const int     SERVO2_INIT_DEG = 90;  // 非常停止時などの基準角

Servo g_servo2;
int   g_servo2Angle = SERVO2_INIT_DEG;


// ============== Motor (L298N) ===============
struct Motor {
  uint8_t pwmPin;  // ENA/ENB (PWM)
  uint8_t in1Pin;
  uint8_t in2Pin;

  int currentPWM;  // -255..+255
  int targetPWM;

  // ★ 追加: 明示的コンストラクタ（UNOでの初期化エラー対策）
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

// ピン割り当て（L298N）: 左モータA / 右モータB
Motor motorA(3, 4, 6);  // ENA=3(PWM), IN1=4, IN2=6
Motor motorB(5, 7, 8);  // ENB=5(PWM), IN3=7, IN4=8

// ============== Control params ==============
const uint8_t  NEUTRAL_LO     = 120; // スティック中立帯 下限
const uint8_t  NEUTRAL_HI     = 135; // スティック中立帯 上限
const uint8_t  RAMP_STEP      = 7;   // スロープ1ステップのPWM変化量
const uint16_t UPDATE_PERIOD  = 12;  // スロープ更新周期[ms]
const uint16_t FAILSAFE_MS    = 600; // 入力喪失でCoastへ[ms]

uint32_t lastUpdateMs = 0;
uint32_t lastRxMs     = 0;

// ============== Helpers =====================
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
  
  g_servoAngle = SERVO_INIT_DEG;
  g_servo.write(g_servoAngle);
  
  g_servo2Angle = SERVO2_INIT_DEG;
  g_servo2.write(g_servo2Angle);
}

void handleServoClicks() {
  // 十字キーUP/DOWNの「クリック」で1回ずつ動かす
  if (PS4.getButtonClick(UP)) {
    g_servoAngle += SERVO_STEP_DEG;
    if (g_servoAngle > SERVO_MAX_DEG) g_servoAngle = SERVO_MAX_DEG;
    g_servo.write(g_servoAngle);
  }
  if (PS4.getButtonClick(DOWN)) {
    g_servoAngle -= SERVO_STEP_DEG;
    if (g_servoAngle < SERVO_MIN_DEG) g_servoAngle = SERVO_MIN_DEG;
    g_servo.write(g_servoAngle);
  }
}

void handleServo2Clicks() {
  // 右ボタン：+60°
  if (PS4.getButtonClick(RIGHT)) {
    g_servo2Angle += SERVO2_STEP_DEG;
    if (g_servo2Angle > SERVO2_MAX_DEG) g_servo2Angle = SERVO2_MAX_DEG;
    g_servo2.write(g_servo2Angle);
  }
  // 左ボタン：-60°
  if (PS4.getButtonClick(LEFT)) {
    g_servo2Angle -= SERVO2_STEP_DEG;
    if (g_servo2Angle < SERVO2_MIN_DEG) g_servo2Angle = SERVO2_MIN_DEG;
    g_servo2.write(g_servo2Angle);
  }
}


// ============== Setup/Loop ==================
void setup() {
  // Serial.begin(115200);

  if (Usb.Init() == -1) {
    // Serial.println(F("USB Host Shield init failed"));
    while (1) { delay(1000); }
  }

  motorA.begin();
  motorB.begin();

  g_servo.attach(SERVO_PIN);
  g_servo.write(g_servoAngle);

  g_servo2.attach(SERVO2_PIN);
  g_servo2.write(g_servo2Angle);
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
    return;  // このフレームはここで終了
  }

  // L1でブレーキ（押下中）
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

  // サーボ：UP/DOWNで±60°ずつ段階回転
  handleServoClicks();
  
  // サーボ：right/leftで±60°ずつ段階回転
  handleServo2Clicks();

  // 一定周期でスロープ更新
  uint32_t now = millis();
  if (now - lastUpdateMs >= UPDATE_PERIOD) {
    lastUpdateMs = now;
    motorA.updateRamp(RAMP_STEP);
    motorB.updateRamp(RAMP_STEP);
  }
}
