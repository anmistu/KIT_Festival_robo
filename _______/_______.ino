/*  _______.ino  (Arduino UNO + USB Host Shield 2.0 + PS4BT + L298N + 2xServo + NeoPixel)
 *
 *  変更点:
 *   - 〇ボタンのLEDモード切替を「OFF ↔ RAINBOW」の2モードみに簡略化
 *   - RAINBOWは非ブロッキングで20msごとにアニメ更新
 *
 *  既存機能:
 *   - 2モータ(ランプ/中立帯) + 2サーボ(UP/DOWN, LEFT/RIGHT 各±60°ステップ)
 *   - L1ブレーキ、PS非常停止（サーボ初期角に戻す／LED状態は維持）
 *
 *  配線:
 *   - L298N モータA: ENA=3(PWM), IN1=4, IN2=6
 *   - L298N モータB: ENB=5(PWM), IN3=7, IN4=8
 *   - サーボ1(DPad UP/DOWN)=D2, サーボ2(DPad LEFT/RIGHT)=D9
 *   - NeoPixel DIN=A0(D14)  ※USB HostのSPI(10–13)を避ける／GNDは必ず共通
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
#define LED_COUNT  108        // ★実機のLED本数に合わせて
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
const uint8_t SERVO2_PIN      = 0;    // LEFT/RIGHT
const int     SERVO_MIN_DEG   = 0;
const int     SERVO_MAX_DEG   = 180;
const int     SERVO_STEP_DEG  = 60;
const int     SERVO1_INIT_DEG = 90;
const int     SERVO2_INIT_DEG = 90;

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

Motor motorA(3,4,6), motorB(5,7,8);

// ===================== Control params =====================
const uint8_t  NEUTRAL_LO     = 120;
const uint8_t  NEUTRAL_HI     = 135;
const uint8_t  RAMP_STEP      = 7;
const uint16_t UPDATE_PERIOD  = 12;
const uint16_t FAILSAFE_MS    = 600;

uint32_t lastUpdateMs = 0, lastRxMs = 0;

// ===================== LED helpers ========================
uint32_t colorWheel(uint8_t pos){
  if(pos < 85){
    return strip.Color(pos*3, 255 - pos*3, 0);
  } else if(pos < 170){
    pos -= 85;
    return strip.Color(255 - pos*3, 0, pos*3);
  } else {
    pos -= 170;
    return strip.Color(0, pos*3, 255 - pos*3);
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
  // LEDは状態維持（必要なら g_ledMode=LED_OFF; stripAllOff();）
}

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
    g_servo2Angle += SERVO_STEP_DEG; if(g_servo2Angle>SERVO_MAX_DEG) g_servo2Angle=SERVO_MAX_DEG;
    g_servo2.write(g_servo2Angle);
  }
  if(PS4.getButtonClick(LEFT)){
    g_servo2Angle -= SERVO_STEP_DEG; if(g_servo2Angle<SERVO_MIN_DEG) g_servo2Angle=SERVO_MIN_DEG;
    g_servo2.write(g_servo2Angle);
  }
}

// ===================== Setup / Loop =======================
void setup(){
  // Serial.begin(115200);
  if(Usb.Init()==-1){ while(1){ delay(1000);} }

  motorA.begin(); motorB.begin();

  g_servo1.attach(SERVO1_PIN); g_servo2.attach(SERVO2_PIN);
  g_servo1.write(g_servo1Angle); g_servo2.write(g_servo2Angle);

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

  // サーボ（DPad）
  handleServo1Clicks();
  handleServo2Clicks();

  // 〇ボタンで LEDモードを OFF ↔ RAINBOW にトグル
  if(PS4.getButtonClick(CIRCLE)){
    if(g_ledMode == LED_OFF){
      g_ledMode = LED_RAINBOW;
      // 切り替え直後に即更新
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
