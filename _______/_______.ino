/*  _______.ino  (Arduino UNO + USB Host Shield 2.0 + PS4BT + L298N + 2xServo + NeoPixel)
 *
 *  変更点（今回）:
 *   - L2/R2 でサーボをトグル:  L2=0°↔65° / R2=0°↔10°
 *   - アナログ＋デジタルの両検出でR2無反応対策
 *   - 非常停止＆起動時にトグル状態をfalseへリセット
 *   - D-Pad操作は無効化（呼び出しを外しています）
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
#define LED_PIN    A0        // DIN（A0=D14）
#define LED_COUNT  108
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// LEDモード（2値のみ）
enum { LED_OFF=0, LED_RAINBOW=1 };
uint8_t g_ledMode = LED_OFF;

uint8_t LED_BRIGHTNESS = 64;

// RAINBOW用パラメータ（非ブロッキング）
uint8_t  g_rainbowOfs   = 0;
uint32_t g_rainbowLast  = 0;
const uint16_t RAINBOW_INTERVAL_MS = 20;

// ===================== Servo ==============================
const uint8_t SERVO1_PIN      = 2;    // UP/DOWN
const uint8_t SERVO2_PIN      = 0;    // LEFT/RIGHT（Serial使わない前提のD0）
const int     SERVO_MIN_DEG   = 0;
const int     SERVO_MAX_DEG   = 65;
const int     SERVO_STEP_DEG  = 65;   // D-Pad用に残置（未使用）
const int     SERVO1_INIT_DEG = 0;
const int     SERVO2_INIT_DEG = 0;

// === Added: distinct open angles for L2/R2 toggles ===
const int SERVO1_OPEN_DEG = 65;  // L2 toggル開角
const int SERVO2_OPEN_DEG = 10;  // R2 トグル開角（ご要望）
bool g_servo1Open = false;
bool g_servo2Open = false;
bool g_l2Prev = false;
bool g_r2Prev = false;

Servo g_servo1, g_servo2;
int   g_servo1Angle = SERVO1_INIT_DEG;
int   g_servo2Angle = SERVO2_INIT_DEG;

// ===================== Motors (L298N) =====================
struct Motor {
  uint8_t pwmPin, in1Pin, in2Pin;
  int currentPWM, targetPWM;

  Motor() : pwmPin(255), in1Pin(255), in2Pin(255), currentPWM(0), targetPWM(0) {}
  Motor(uint8_t pwm, uint8_t in1, uint8_t in2)
  : pwmPin(pwm), in1Pin(in1), in2Pin(in2), currentPWM(0), targetPWM(0) {}

  void begin() { pinMode(in1Pin,OUTPUT); pinMode(in2Pin,OUTPUT); pinMode(pwmPin,OUTPUT); coast(); }
  void setTargetPWM(int v){ if(v>255)v=255; if(v<-255)v=-255; targetPWM=v; }
  void coast(){ digitalWrite(in1Pin,LOW); digitalWrite(in2Pin,LOW); analogWrite(pwmPin,0); currentPWM=0; targetPWM=0; }
  void brake(){ digitalWrite(in1Pin,HIGH);digitalWrite(in2Pin,HIGH);analogWrite(pwmPin,0); currentPWM=0; targetPWM=0; }
  void apply(int pwm){
    if(pwm==0){ digitalWrite(in1Pin,LOW); digitalWrite(in2Pin,LOW); analogWrite(pwmPin,0); return; }
    if(pwm>0){  digitalWrite(in1Pin,HIGH); digitalWrite(in2Pin,LOW);  analogWrite(pwmPin,pwm); }
    else     {  digitalWrite(in1Pin,LOW);  digitalWrite(in2Pin,HIGH); analogWrite(pwmPin,-pwm); }
  }
  void updateRamp(int step){
    if(currentPWM<targetPWM){ currentPWM+=step; if(currentPWM>targetPWM) currentPWM=targetPWM; }
    else if(currentPWM>targetPWM){ currentPWM-=step; if(currentPWM<targetPWM) currentPWM=targetPWM; }
    apply(currentPWM);
  }
};

Motor motorA(3,4,6);
Motor motorB(5,7,8);

const uint8_t RAMP_STEP     = 7;
const uint16_t UPDATE_PERIOD= 12; // [ms]

uint32_t lastRxMs=0;
uint32_t lastUpdateMs=0;

// スティック中立帯
const int NEUTRAL_LO = 120;
const int NEUTRAL_HI = 135;

const uint16_t FAILSAFE_MS = 600;

// ===================== 色ユーティリティ ====================
uint32_t colorWheel(uint8_t pos){
  pos = 255 - pos;
  if(pos < 85){
    return strip.Color(255 - pos*3, 0, pos*3);
  }else if(pos < 170){
    pos -= 85;
    return strip.Color(0, pos*3, 255 - pos*3);
  }else{
    pos -= 170;
    return strip.Color(pos*3, 255 - pos*3, 0);
  }
}

void stripAllOff(){
  for(int i=0;i<LED_COUNT;++i) strip.setPixelColor(i, 0);
  strip.show();
}

void stripRainbowStep(){
  uint32_t now = millis();
  if(now - g_rainbowLast < RAINBOW_INTERVAL_MS) return;
  g_rainbowLast = now;

  for(int i=0;i<LED_COUNT;++i){
    uint8_t w = ((i * 256 / LED_COUNT) + g_rainbowOfs) & 0xFF;
    strip.setPixelColor(i, colorWheel(w));
  }
  strip.show();
  g_rainbowOfs++;  // 0..255で自然に周回
}

// ===================== Others =============================
int mapStickToPWM(uint8_t v){
  if(v>=NEUTRAL_LO && v<=NEUTRAL_HI) return 0;
  if(v>NEUTRAL_HI){
    float t=(float)(v-NEUTRAL_HI)/(255.0f-(float)NEUTRAL_HI);
    int pwm=(int)(t*255.0f+0.5f); if(pwm>255)pwm=255; return pwm;
  }else{
    float t=(float)(NEUTRAL_LO-v)/(float)NEUTRAL_LO;
    int pwm=(int)(t*255.0f+0.5f); if(pwm>255)pwm=255; return -pwm;
  }
}

void emergencyStopAll(){
  motorA.coast(); motorB.coast();
  g_servo1Angle = SERVO1_INIT_DEG; g_servo2Angle = SERVO2_INIT_DEG;
  g_servo1.write(g_servo1Angle);   g_servo2.write(g_servo2Angle);
  g_servo1Open = false; g_servo2Open = false;
  // LEDは状態維持（必要なら g_ledMode=LED_OFF; stripAllOff();）
}

// （D-Pad用：残置・未呼出）
void handleServo1Clicks(){
  if(PS4.getButtonClick(UP)){
    g_servo1Angle += SERVO_STEP_DEG; if(g_servo1Angle>SERVO_MAX_DEG) g_servo1Angle=SERVO_MAX_DEG;
    g_servo1.write(g_servo1Angle);
  }
  if(PS4.getButtonClick(DOWN)){
    g_servo1Angle -= SERVO_STEP_DEG; if(g_servo1Angle<SERVO_MIN_DEG) g_servo1Angle=SERVO_MIN_DEG;
    g_servo1.write(g_servo1Angle);
  }
}
void handleServo2Clicks(){
  if(PS4.getButtonClick(RIGHT)){
    g_servo2Angle += SERVO_STEP_DEG; if(g_servo2Angle == SERVO_MAX_DEG) g_servo2Angle=SERVO_MAX_DEG;
    g_servo2.write(g_servo2Angle);
  }
  if(PS4.getButtonClick(LEFT)){
    g_servo2Angle -= SERVO_STEP_DEG; if(g_servo2Angle == SERVO_MIN_DEG) g_servo2Angle=SERVO_MIN_DEG;
    g_servo2.write(g_servo2Angle);
  }
}

// === Added: L2/R2 toggle control for servos (L2->Servo1 0/65°, R2->Servo2 0/10°) ===
void updateServoToggle(){
  // 一部のPS4BTはトリガーをアナログのみ報告するため、両検出を合成
  const bool l2_now = (PS4.getAnalogButton(L2) > 20) || PS4.getButtonPress(L2);
  const bool r2_now = (PS4.getAnalogButton(R2) > 20) || PS4.getButtonPress(R2);

  if(l2_now && !g_l2Prev){
    g_servo1Open = !g_servo1Open;
    g_servo1Angle = g_servo1Open ? SERVO1_OPEN_DEG : SERVO_MIN_DEG;
    g_servo1.write(g_servo1Angle);
  }
  if(r2_now && !g_r2Prev){
    g_servo2Open = !g_servo2Open;
    g_servo2Angle = g_servo2Open ? SERVO2_OPEN_DEG : SERVO_MIN_DEG;
    g_servo2.write(g_servo2Angle);
  }
  g_l2Prev = l2_now;
  g_r2Prev = r2_now;
}

// ===================== Setup / Loop =======================
void setup(){
  // Serial.begin(115200);  // SERVO2_PIN=D0使用のため未使用
  if(Usb.Init()==-1){ while(1){ delay(1000);} }

  motorA.begin(); motorB.begin();

  g_servo1.attach(SERVO1_PIN); g_servo2.attach(SERVO2_PIN);
  g_servo1.write(g_servo1Angle); g_servo2.write(g_servo2Angle);
  g_servo1Open = false; g_servo2Open = false;

  strip.begin();
  strip.setBrightness(LED_BRIGHTNESS);
  stripAllOff();
  g_ledMode = LED_OFF;
}

void loop(){
  Usb.Task();

  if(!PS4.connected()){
    if(millis()-lastRxMs>FAILSAFE_MS){ motorA.coast(); motorB.coast(); }
    return;
  }
  lastRxMs = millis();

  if(PS4.getButtonClick(PS)){ emergencyStopAll(); return; }

  // L1ブレーキ
  if(PS4.getButtonPress(L1)){ motorA.brake(); motorB.brake(); }
  else{
    uint8_t ly=PS4.getAnalogHat(LeftHatY), ry=PS4.getAnalogHat(RightHatY);
    motorA.setTargetPWM(-mapStickToPWM(ly));
    motorB.setTargetPWM(-mapStickToPWM(ry));
  }

  // サーボ：L2/R2トグル（L2=65°, R2=10°）
  // handleServo1Clicks();  // disabled: now using L2 toggle
  updateServoToggle();     // added: L2/R2 servo toggles (R2=10°)

  // 〇ボタンで LEDモードを OFF ↔ RAINBOW にトグル
  if(PS4.getButtonClick(CIRCLE)){
    if(g_ledMode == LED_OFF){
      g_ledMode = LED_RAINBOW;
      g_rainbowLast = 0;
      stripRainbowStep();
    }else{
      g_ledMode = LED_OFF;
      stripAllOff();
    }
  }

  // RAINBOWモード更新（非ブロッキング）
  if(g_ledMode==LED_RAINBOW) stripRainbowStep();

  // モータのランプ更新
  uint32_t now=millis();
  if(now-lastUpdateMs>=UPDATE_PERIOD){
    lastUpdateMs=now;
    motorA.updateRamp(RAMP_STEP);
    motorB.updateRamp(RAMP_STEP);
  }
}
