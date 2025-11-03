# define RS_PIN 8
# define E_PIN 9
# define DB4_PIN 10
# define DB5_PIN 11
# define DB6_PIN 12
# define DB7_PIN 13
# include <LiquidCrystal.h>;

LiquidCrystal lcd(RS_PIN, E_PIN,  DB4_PIN, DB5_PIN, DB6_PIN, DB7_PIN);

void setup() {
  lcd.begin(16, 2);
  lcd.print("hello, world!");
}

void loop() {
  lcd.setCursor(0, 1);
  lcd.print(millis()/1000);
}
