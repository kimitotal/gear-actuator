#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

/**************** PIN'AI ****************/
// L298N (Variklis)
const uint8_t PIN_ENA = 3;  
const uint8_t PIN_IN1 = 8;  
const uint8_t PIN_IN2 = 9;  

// Padėties daviklis
const uint8_t PIN_POS = A0; 

// Mygtukai (D ir N bus naudojami tik judėjimui)
const uint8_t PIN_BTN_D = 7; // Pirmyn (Forward)
const uint8_t PIN_BTN_N = 6; // Atgal (Reverse)

// Variklio greitis (Lėtas, kad būtų lengva tiksliai nustatyti poziciją)
const int PWM_CALIB = 80;

/**************** LCD ****************/
// Naudojamas adresas 0x27. Jei nerodo, pakeiskite į 0x3F.
LiquidCrystal_I2C lcd(0x27, 16, 2); 

/**************** PAGALBINĖS FUNKCIJOS ****************/

// Konvertuoja ADC vertę (0-1023) į įtampą V (0-5V)
float adcToV(int adc){
  const float ADC_VREF = 5.00f; 
  const int ADC_MAX = 1023;
  return (float)adc * ADC_VREF / (float)ADC_MAX;
}
// Nuskaito ir grąžina įtampą V
float readPosV(){
  return adcToV(analogRead(PIN_POS));
}
// Nuskaito ir grąžina ADC vertę
int readPosAdc(){
  return analogRead(PIN_POS);
}

// Variklio sustabdymas
void motorStop(){
  analogWrite(PIN_ENA, 0);
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
}
// Variklio paleidimas nurodyta kryptimi
void motorDrive(int pwm, bool forward){
  pwm = constrain(pwm, 0, 255);
  // Pirmyn/Atgal nustatymai, priklauso nuo laidų sujungimo, bet darome prielaidą
  digitalWrite(PIN_IN1, forward ? HIGH : LOW);
  digitalWrite(PIN_IN2, forward ? LOW  : HIGH);
  analogWrite(PIN_ENA, pwm);
}
// Patikrina, ar mygtukas paspaustas (INPUT_PULLUP: LOW=paspaustas)
inline bool btn(uint8_t p){ return digitalRead(p) == LOW; }


/**************** SETUP ****************/
void setup(){
  // L298N
  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  motorStop();

  // Analog padėtis
  pinMode(PIN_POS, INPUT);

  // Mygtukai (tik D ir N)
  pinMode(PIN_BTN_D, INPUT_PULLUP);
  pinMode(PIN_BTN_N, INPUT_PULLUP);

  // LCD inicijavimas
  Wire.begin(); delay(100);
  lcd.init(); lcd.begin(16,2); lcd.backlight();
  lcd.clear(); 
  lcd.setCursor(0,0); lcd.print("--- CALIBRATION ---");
  lcd.setCursor(0,1); lcd.print("V:0.000");
  delay(1000);
}

/**************** LOOP ****************/
void loop(){
  float currentV = readPosV();
  int currentA = readPosAdc();
  
  // 1. Judėjimas (Valdymas)
  // D mygtukas - judėti viena kryptimi (Pirmyn)
  if (btn(PIN_BTN_D)){
    motorDrive(PWM_CALIB, true); 
    lcd.setCursor(15,0); lcd.print(">"); // Rodo judėjimą
  } 
  // N mygtukas - judėti kita kryptimi (Atgal)
  else if (btn(PIN_BTN_N)){
    motorDrive(PWM_CALIB, false);
    lcd.setCursor(15,0); lcd.print("<"); // Rodo judėjimą
  } 
  // Nė vienas nepaspaustas - STOP
  else {
    motorStop();
    lcd.setCursor(15,0); lcd.print(" "); // Rodo, kad stovi
  }

  // 2. LCD atnaujinimas: Rodo realaus laiko įtampą (V) ir ADC vertę.
  lcd.setCursor(0,0); 
  lcd.print("V:"); lcd.print(currentV, 3); // Įtampa su 3 skaitmenimis po kablelio
  lcd.print(" ADC:"); lcd.print(currentA);
  
  // 3. Apatinė eilutė (Tik instrukcija)
  lcd.setCursor(0,1);
  lcd.print("D> FWD, N< REV");

  // Nereikia jokios sudėtingos logikos, fiksavimo ar persijungimo.
  // Jūs tiesiog stebite V ir užsirašote.
  
  delay(50); // Trumpa pauzė, kad ekranas nemirgėtų per greitai
}