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
#include <LiquidCrystal.h>


// --- LCD pins (from LCD.ino, unchanged) ---
#define RS_PIN 2
#define E_PIN 4
#define DB4_PIN 3
#define DB5_PIN A3
#define DB6_PIN A4
#define DB7_PIN A5
// Follow the same constructor order as LCD.ino to match your wiring
LiquidCrystal lcd(RS_PIN, E_PIN, DB4_PIN, DB5_PIN, DB6_PIN, DB7_PIN);

// === 表示サイズ ===
const uint8_t LCD_COLS = 16;
const uint8_t LCD_ROWS = 2;

// === テキストセット ===
const char TXT_DEF_0[] = "SAFETY DRIVING!";
const char TXT_DEF_1[] = "  THANK YOU!   ";
const char TXT_HAL_0[] = "HAPPY HALLOWEEN!";
const char TXT_HAL_1[] = "TRICK OR TREAT! ";

// 行ごとのスクロール状態
struct Marquee {
  const char* text;        // 流したい文字列
  uint8_t     row;         // 表示行(0 or 1)
  uint16_t    idx;         // スクロール位置
  unsigned long last;      // 前回更新時刻
  unsigned long interval;  // 更新間隔(ms) ここを大きくすると“ゆっくり”
};

// 表示内容と速度（好みで調整OK）
Marquee m0 = { TXT_DEF_0, 0, 0, 0, 500 };
Marquee m1 = { TXT_DEF_0, 1, 0, 0, 500 };

// 文字列と文字列の間の“空白のすき間”量
const uint8_t GAP_SPACES = 6;

// 現在のテーマ
bool is_halloween = false;

// === UIモード ===
enum UIMode { MODE_WAITING, MODE_CONNECTED_FLASH, MODE_SCROLL };
UIMode ui_mode = MODE_WAITING;

// === 「PLEASE WAIT …」のドットアニメ設定 ===
const char WAIT_BASE[] = "PLEASE WAIT";
const uint8_t DOT_START_COL = 11;    // "PLEASE WAIT" の直後=列11(0始まり)
const uint8_t DOT_SLOTS = 3;         // 「…」の3枠
const unsigned long DOT_INTERVAL = 350; // ドット更新間隔(ms)
unsigned long dot_last = 0;
uint8_t dot_pos = 0;  // 0,1,2 の位置にピコピコ表示

// === 接続演出（Connected! → フェード） ===
#define BACKLIGHT_PWM_PIN  (-1)              // ← PWMでフェードするなら 5/6/10/11 などに変更
const char CONNECTED_MSG[] = "Connected!";
const unsigned long CONNECT_SHOW_MS = 3000;   // 表示の見せ時間（ms）
const unsigned long FADE_TOTAL_MS   = 1500;   // フェード全体時間（ms）
const unsigned long FADE_STEP_MS    = 500;   // （疑似フェード時）ステップ間隔

unsigned long connected_phase_start = 0;
unsigned long last_fade_step_ts = 0;
uint8_t fade_step = 0;                       // 疑似フェード段階 0..N
const uint8_t FADE_STEPS = 5;                // 疑似フェード段階数

// === 接続フラグ（あなたのコードから true にする） ===
volatile bool controller_connected = false;

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
const uint8_t SERVO1_PIN      = 1;
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

Motor motorA(9,5,6);
Motor motorB(9,7,8);

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
  
// LCDの列と行を設定する
  lcd.begin(LCD_COLS, LCD_ROWS);
  applyMessageSet(false);
  enterWaiting();
  
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

static void setMarqueeText(Marquee &m, const char* newText) {
  m.text = newText;
  m.idx = 0;        // 先頭からスクロール開始
  m.last = 0;       // すぐに描画更新
}

static void applyDefaultMessages() {
  setMarqueeText(m0, TXT_DEF_0);
  setMarqueeText(m1, TXT_DEF_1);
}

static void applyMessageSet(bool halloween) {
  if (halloween) {
    setMarqueeText(m0, TXT_HAL_0);
    setMarqueeText(m1, TXT_HAL_1);
  } else {
    setMarqueeText(m0, TXT_DEF_0);
    setMarqueeText(m1, TXT_DEF_1);
  }
}

static void toggleHalloween() {
  is_halloween = !is_halloween;
  applyMessageSet(is_halloween);
}

static void updateMarquee(Marquee &m) {
  unsigned long now = millis();
  if (now - m.last < m.interval) return;  // 非ブロッキング更新
  m.last = now;

  const char* s = m.text;
  uint16_t len = 0;
  while (s[len] != '\0') len++;

  // 先頭に画面幅ぶんの空白 + 本文 + 余白GAP をリング状に流す
  uint16_t padLen = LCD_COLS + len + GAP_SPACES;

  char window[LCD_COLS + 1];
  for (uint8_t j = 0; j < LCD_COLS; ++j) {
    uint16_t pos = (m.idx + j) % padLen;
    char ch;
    if (pos < LCD_COLS) {
      ch = ' '; // 画面外(右)からにゅっと出てくるための空白
    } else if (pos < LCD_COLS + len) {
      ch = s[pos - LCD_COLS];
    } else {
      ch = ' '; // 文末～次周回までの余白
    }
    window[j] = ch;
  }
  window[LCD_COLS] = '\0';

  lcd.setCursor(0, m.row);
  lcd.print(window);

  m.idx = (m.idx + 1) % padLen;
}

// ---------- WAITモード描画 ----------
static void enterWaiting() {
  ui_mode = MODE_WAITING;
  lcd.clear();
  lcd.noCursor();      // カーソル非表示
  // 行0に固定文字列
  lcd.setCursor(0, 0);
  lcd.print(WAIT_BASE);
  // ドット領域を初期化
  lcd.setCursor(DOT_START_COL, 0);
  lcd.print("   ");    // 3つ分の空白
  // 下の行は空白に（好みでメッセージ可）
  lcd.setCursor(0, 1);
  lcd.print("                ");
  dot_pos = 0;
  dot_last = 0;
}

static void updateWaitingDots() {
  unsigned long now = millis();
  if (now - dot_last < DOT_INTERVAL) return;
  dot_last = now;

  // 3枠を一旦消す
  lcd.setCursor(DOT_START_COL, 0);
  lcd.print("   ");
  // 現在位置にドットを1つだけ出す
  lcd.setCursor(DOT_START_COL + dot_pos, 0);
  lcd.print(".");
  // 次の位置へ
  dot_pos = (dot_pos + 1) % DOT_SLOTS;
}

// ---------------- CONNECTED! → フェード ----------------
static void drawCentered(const char* msg, uint8_t row) {
  uint8_t len = 0; while (msg[len] != '\0') len++;
  uint8_t col = (LCD_COLS > len) ? (LCD_COLS - len) / 2 : 0;
  lcd.setCursor(0, row); lcd.print("                ");
  lcd.setCursor(col, row); lcd.print(msg);
}

// 疑似フェード（文字を徐々に消す）: 両端から縮める
static void drawConnectedPseudoFade(uint8_t stage) {
  // stage=0 → "Connected!" 全表示, stageが進むごとに両端から消える
  uint8_t full = 0; while (CONNECTED_MSG[full] != '\0') full++;
  int show_len = (int)full - 2 * (int)stage;
  if (show_len <= 0) {
    drawCentered("", 0);
    return;
  }
  uint8_t start = stage;
  // 一時バッファ
  char buf[17];
  for (int i = 0; i < show_len && i < 16; ++i) buf[i] = CONNECTED_MSG[start + i];
  buf[(show_len < 16 ? show_len : 16)] = '\0';
  drawCentered(buf, 0);
}

static void enterConnectedFlash() {
  ui_mode = MODE_CONNECTED_FLASH;
  lcd.clear();
  drawCentered(CONNECTED_MSG, 0);
  // 2行目はクリア
  lcd.setCursor(0, 1);
  lcd.print("                ");

#if BACKLIGHT_PWM_PIN >= 0
  pinMode(BACKLIGHT_PWM_PIN, OUTPUT);
  analogWrite(BACKLIGHT_PWM_PIN, 255); // フェード開始は明るく
#endif

  connected_phase_start = millis();
  last_fade_step_ts = connected_phase_start;
  fade_step = 0;
}

static void updateConnectedFlash() {
  unsigned long now = millis();
  unsigned long elapsed = now - connected_phase_start;

#if BACKLIGHT_PWM_PIN >= 0
  // 実フェード（バックライトPWM）
  if (elapsed < CONNECT_SHOW_MS) {
    // まだ見せ時間中 → 何もしない
  } else if (elapsed < CONNECT_SHOW_MS + FADE_TOTAL_MS) {
    float t = (float)(elapsed - CONNECT_SHOW_MS) / (float)FADE_TOTAL_MS; // 0..1
    int level = (int)(255.0f * (1.0f - t)); // 255→0
    if (level < 0) level = 0;
    analogWrite(BACKLIGHT_PWM_PIN, level);
  } else {
    // フェード完了 → 明るさ戻して次へ
    analogWrite(BACKLIGHT_PWM_PIN, 255);
    // スクロールへ
    // 1行目=SAFETY DRIVING, 2行目=THANK YOU
    lcd.clear();
    applyDefaultMessages();
    ui_mode = MODE_SCROLL;
  }
#else
  // 疑似フェード（文字を徐々に消す）
  if (elapsed < CONNECT_SHOW_MS) {
    // 見せ時間中はそのまま
  } else {
    if (now - last_fade_step_ts >= FADE_STEP_MS) {
      last_fade_step_ts = now;
      if (fade_step <= FADE_STEPS) {
        drawConnectedPseudoFade(fade_step);
        fade_step++;
      } else {
        // フェード完了 → スクロールへ
        lcd.clear();
        applyDefaultMessages();
        ui_mode = MODE_SCROLL;
      }
    }
  }
#endif
}

// ---------- SCROLLモードへ切替 ----------
static void enterScroll() {
  ui_mode = MODE_SCROLL;
  lcd.clear();
  applyDefaultMessages();  // 1行目=SAFETY DRIVING, 2行目=THANK YOU
#if BACKLIGHT_PWM_PIN >= 0
  analogWrite(BACKLIGHT_PWM_PIN, 255);  // 念のため明るさ復帰
#endif
}

// ---------- 外部イベント：接続完了で呼ぶ ----------
void onControllerConnected() {
  controller_connected = true;
  enterConnectedFlash();
}

void loop(){
  Usb.Task();

  static bool wasConnected = false;
  bool nowConnected = PS4.connected();

  if (nowConnected && !wasConnected) {
    wasConnected = true;
    onControllerConnected();   // ← ここでWAIT→SCROLLへ切替
  } else if (!nowConnected && wasConnected) {
    wasConnected = false;
    enterWaiting();            // ← 切断検出でWAITに戻す（任意）
  }  

  if (ui_mode == MODE_WAITING) {
    updateWaitingDots();              // 「…」だけピコピコ
    if (controller_connected) {
      enterScroll();                  // 接続できたらスクロール表示へ
    }
    }
    else if(ui_mode == MODE_CONNECTED_FLASH){
      updateConnectedFlash();
    } else {
    updateMarquee(m0);
    updateMarquee(m1);
    }
  
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
    toggleHalloween();
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
