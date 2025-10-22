/*  Dual-LED x2 + Button-LED (A0/A1/A2)
 *  - A0(strip0): ○ボタンで OFF ↔ RAINBOW
 *  - A1(strip1): 常時 赤色（定期上書き）
 *  - A2(strip2): 常時 白色（定期上書き）
 *
 *  既存仕様:
 *   - L2=Servo1: 0°↔65° トグル
 *   - R2=Servo2: 0°↔10° トグル
 *   - L1: ブレーキ / PS: 非常停止 (Coast & サーボ0°)
 *
 *  配線:
 *   - L298N モータA: ENA=3(PWM), IN1=4, IN2=6
 *   - L298N モータB: ENB=5(PWM), IN3=7, IN4=8
 *   - Servo1=D2, Servo2=D0（Serial未使用前提）
 *   - NeoPixel strip0 (A0=D14) : ボタン制御
 *   - NeoPixel strip1 (A1=D15) : 常時 赤
 *   - NeoPixel strip2 (A2=D16) : 常時 白
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

// ===================== NeoPixel (3本) =====================
// A0 側（strip0）: ○ボタンで OFF ↔ RAINBOW
#define LED0_PIN     A0
#define LED0_COUNT   108

// A1 側（strip1）: 常時 赤
#define LED1_PIN     A1
#define LED1_COUNT   4   // ←実機本数に合わせて変更

// A2 側（strip2）: 常時 白
#define LED2_PIN     A2
#define LED2_COUNT   4   // ←実機本数に合わせて変更

Adafruit_NeoPixel strip0(LED0_COUNT, LED0_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip1(LED1_COUNT, LED1_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip2(LED2_COUNT, LED2_PIN, NEO_GRB + NEO_KHZ800);

uint8_t LED_BRIGHTNESS = 64;

// strip0 用モード
enum { LED_OFF=0, LED_RAINBOW=1 };
uint8_t g_ledMode0 = LED_OFF;

// RAINBOW（strip0）用
uint8_t  g_rainbowOfs0   = 0;
uint32_t g_rainbowLast0  = 0;
const uint16_t RAINBOW_INTERVAL_MS = 20;

// 赤/白の定期上書き
uint32_t g_redLast1   = 0;
uint32_t g_whiteLast2 = 0;
const uint16_t SOLID_INTERVAL_MS = 50;  // 20〜100ms程度でOK

// ===================== Servo ==============================
const uint8_t SERVO1_PIN      = 2;
const uint8_t SERVO2_PIN      = 0;    // Serial未使用前提でD0
const int     SERVO_MIN_DEG   = 0;
const int     SERVO1_OPEN_DEG = 50;   // L2トグル時
const int     SERVO2_OPEN_DEG = 60;   // R2トグル時
const int     SERVO1_INIT_DEG = 0;
const int     SERVO2_INIT_DEG = 0;

Servo g_servo1, g_servo2;
int   g_servo1Angle = SERVO1_INIT_DEG;
int   g_servo2Angle = SERVO2_INIT_DEG;
bool  g_servo1Open  = false;
bool  g_servo2Open  = false;
bool  g_l2Prev      = false;
bool  g_r2Prev      = false;

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

const uint8_t  RAMP_STEP      = 7;
const uint16_t UPDATE_PERIOD  = 12; // [ms]
uint32_t lastRxMs=0, lastUpdateMs=0;

// スティック中立帯
const int NEUTRAL_LO = 120;
const int NEUTRAL_HI = 135;
const uint16_t FAILSAFE_MS = 600;

// ===================== LEDユーティリティ ====================
uint32_t colorWheel(Adafruit_NeoPixel &s, uint8_t pos){
  pos = 255 - pos;
  if(pos < 85)      return s.Color(255 - pos*3, 0, pos*3);
  else if(pos <170){ pos -= 85; return s.Color(0, pos*3, 255 - pos*3); }
  pos -= 170;       return s.Color(pos*3, 255 - pos*3, 0);
}

void setStrip0Off(){
  strip0.clear();
  strip0.show();
}

void updateRainbowStrip0(){
  uint32_t now = millis();
  if(now - g_rainbowLast0 < RAINBOW_INTERVAL_MS) return;
  g_rainbowLast0 = now;

  for(int i=0;i<LED0_COUNT;++i){
    uint8_t w = ((i * 256 / LED0_COUNT) + g_rainbowOfs0) & 0xFF;
    strip0.setPixelColor(i, colorWheel(strip0, w));
  }
  strip0.show();
  g_rainbowOfs0++;
}

// 常時 赤（strip1）
void forceRedStrip1(){
  uint32_t now = millis();
  if(now - g_redLast1 < SOLID_INTERVAL_MS) return;
  g_redLast1 = now;

  uint32_t col = strip1.Color(255, 0, 0);  // 赤
  for(int i=0;i<LED1_COUNT;++i) strip1.setPixelColor(i, col);
  strip1.show();
}

// 常時 白（strip2）
void forceWhiteStrip2(){
  uint32_t now = millis();
  if(now - g_whiteLast2 < SOLID_INTERVAL_MS) return;
  g_whiteLast2 = now;

  uint32_t col = strip2.Color(255, 255, 230);  // 白
  for(int i=0;i<LED2_COUNT;++i) strip2.setPixelColor(i, col);
  strip2.show();
}

// ===================== その他ユーティリティ =====================
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
  // strip0は状態維持（必要なら setStrip0Off();）
}

// === L2/R2 サーボトグル ===
void updateServoToggle(){
  const bool l2_now = (PS4.getAnalogButton(L2) > 20) || PS4.getButtonPress(L2);
  const bool r2_now = (PS4.getAnalogButton(R2) > 20) || PS4.getButtonPress(R2);

  if(l2_now && !g_l2Prev){
    g_servo1Open  = !g_servo1Open;
    g_servo1Angle = g_servo1Open ? SERVO1_OPEN_DEG : SERVO_MIN_DEG;
    g_servo1.write(g_servo1Angle);
  }
  if(r2_now && !g_r2Prev){
    g_servo2Open  = !g_servo2Open;
    g_servo2Angle = g_servo2Open ? SERVO2_OPEN_DEG : SERVO_MIN_DEG;
    g_servo2.write(g_servo2Angle);
  }
  g_l2Prev = l2_now;
  g_r2Prev = r2_now;
}

// ===================== Setup / Loop =======================
void setup(){
  // Serial.begin(115200);  // D0をServo2に使用するため未使用
  if(Usb.Init()==-1){ while(1){ delay(1000);} }

  motorA.begin(); motorB.begin();

  g_servo1.attach(SERVO1_PIN); g_servo2.attach(SERVO2_PIN);
  g_servo1.write(g_servo1Angle); g_servo2.write(g_servo2Angle);

  // LEDすべて初期化
  strip0.begin(); strip0.setBrightness(LED_BRIGHTNESS); setStrip0Off();
  strip1.begin(); strip1.setBrightness(LED_BRIGHTNESS);
  strip2.begin(); strip2.setBrightness(LED_BRIGHTNESS);

  // 起動直後から A1=赤 / A2=白 を点灯
  g_redLast1 = 0;  forceRedStrip1();
  g_whiteLast2 = 0; forceWhiteStrip2();
}

void loop(){
  Usb.Task();

  if(!PS4.connected()){
    if(millis()-lastRxMs>FAILSAFE_MS){ motorA.coast(); motorB.coast(); }
    // A1=赤 / A2=白 は常時維持
    forceRedStrip1();
    forceWhiteStrip2();
    return;
  }
  lastRxMs = millis();

  // 非常停止
  if(PS4.getButtonClick(PS)){ emergencyStopAll(); return; }

  // L1ブレーキ
  if(PS4.getButtonPress(L1)){ motorA.brake(); motorB.brake(); }
  else{
    uint8_t ly=PS4.getAnalogHat(LeftHatY), ry=PS4.getAnalogHat(RightHatY);
    motorA.setTargetPWM(-mapStickToPWM(ly));
    motorB.setTargetPWM(-mapStickToPWM(ry));
  }

  // サーボ：L2/R2トグル
  updateServoToggle();

  // ○ボタンで strip0: OFF ↔ RAINBOW
  if(PS4.getButtonClick(CIRCLE)){
    if(g_ledMode0 == LED_OFF){ g_ledMode0 = LED_RAINBOW; g_rainbowLast0 = 0; }
    else{ g_ledMode0 = LED_OFF; setStrip0Off(); }
  }

  // strip0 の表示更新
  if(g_ledMode0==LED_RAINBOW) updateRainbowStrip0();

  // A1=赤 / A2=白 は常時上書き
  forceRedStrip1();
  forceWhiteStrip2();

  // モータのランプ更新
  uint32_t now=millis();
  if(now-lastUpdateMs>=UPDATE_PERIOD){
    lastUpdateMs=now;
    motorA.updateRamp(RAMP_STEP);
    motorB.updateRamp(RAMP_STEP);
  }
}
