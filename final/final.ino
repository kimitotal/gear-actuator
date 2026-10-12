/************************************************************
 * Pavarų aktuatorius su ANALOG padėties davikliu (be HOME)
 * - Variklis per L298N: ENA=3 (PWM), IN1=8, IN2=9
 * - Padėties daviklis: A0 (0..5V)
 * - ECU relės (LOW=ON):
 * POS2=A3, P=12, R=A2, POS3=11
 * - Mygtukai (INPUT_PULLUP): P=4, R=5, N=6, D=7
 * - I²C LCD1602: SDA=A4, SCL=A5 (addr 0x27)
 *
 * STARTAS: automatiškai nuvažiuojama į P (TARGET_P_V).
 * ECU kombinacija aktyvuojama tik tiksliai pasiekus pavaros taikinį.
 *************************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

/**************** LCD ****************/
LiquidCrystal_I2C lcd(0x27, 16, 2); // jei nerodo – keisk į 0x3F

/**************** PINAI ****************/
// L298N (vienas kanalas)
const uint8_t PIN_ENA = 3;    // PWM greitis
const uint8_t PIN_IN1 = 8;    // kryptis
const uint8_t PIN_IN2 = 9;    // kryptis

// Analog padėties daviklis (0..5V)
const uint8_t PIN_POS = A0;  // OUT -> A0

// Mygtukai (INPUT_PULLUP: paspausta=LOW)
const uint8_t PIN_BTN_P = 4;
const uint8_t PIN_BTN_R = 5;
const uint8_t PIN_BTN_N = 6;
const uint8_t PIN_BTN_D = 7;

// ECU relės (LOW=ON moduliams)
const uint8_t REL_POS2 = A3; 
const uint8_t REL_P    = 12; 
const uint8_t REL_R    = A2; 
const uint8_t REL_POS3 = 11; 

const bool RELAY_ACTIVE_LOW = true;

/**************** LOGIKA / KALIBRACIJA ****************/
// ADC -> voltai
const float ADC_VREF = 5.00f; // Galima patikslinti
const int   ADC_MAX  = 1023;

// sveikas padėties diapazonas (apsauga)
const float POS_MIN_V_OK = 0.00f;
const float POS_MAX_V_OK = 4.95f;

// tikslinės pavarų įtampos (ĮVESKITE SAVO VERTES ČIA)
// ***************************************************************
float TARGET_P_V = 0.60f; // PASTE HERE
float TARGET_R_V = 1.20f; // PASTE HERE
float TARGET_N_V = 2.00f; // PASTE HERE
float TARGET_D_V = 2.80f; // PASTE HERE
// ***************************************************************

// tolerancija ±V ir „išbuvimo“ laikas zonoje
float         TOL_V    = 0.03f;  // pvz. ±30 mV
unsigned long DWELL_MS = 120;    // ms zonoje iki patvirtinimo

// važiavimo parametrai
int   PWM_FAST      = 220;
int   PWM_SLOW      = 140;
float SLOW_BAND_V   = 0.15f;     // lėtėjam priartėję iki šios ribos

/**************** BŪSENA ****************/
enum Gear { G_NONE, G_P, G_R, G_N, G_D };
Gear currentGear = G_NONE;

/**************** RELĖS – FUNKCIJOS ****************/
inline void relayOn(uint8_t pin)  { digitalWrite(pin, RELAY_ACTIVE_LOW ? LOW  : HIGH); }
inline void relayOff(uint8_t pin) { digitalWrite(pin, RELAY_ACTIVE_LOW ? HIGH : LOW ); }

// Saugus PIN inicializavimas
void safeInitRelayPin(uint8_t pin){
  // saugus OFF startas (neįsijungtų per reset)
  bool offLevel = RELAY_ACTIVE_LOW ? HIGH : LOW;
  digitalWrite(pin, offLevel);  // kol dar INPUT
  pinMode(pin, OUTPUT);
  digitalWrite(pin, offLevel);
}

inline void allRelaysOff(){
  relayOff(REL_POS2);
  relayOff(REL_P);
  relayOff(REL_R);
  relayOff(REL_POS3);
}

// ECU kombinacijos pagal tavo žemėlapį:
// P  = P
// R  = R
// N  = POS2 + P + R
// D  = POS2 + POS3 + P
void relaysShowGear(Gear g){
  allRelaysOff();
  switch (g){
    case G_P: // tik P
      relayOn(REL_P);
      break;
    case G_R: // tik R
      relayOn(REL_R);
      break;
    case G_N: // POS2 + P + R
      relayOn(REL_POS2);
      relayOn(REL_P);
      relayOn(REL_R);
      break;
    case G_D: // POS2 + POS3 + P
      relayOn(REL_POS2);
      relayOn(REL_POS3);
      relayOn(REL_P);
      break;
    default:
      // G_NONE -> viskas OFF
      break;
  }
}

/**************** VARIKLIS ****************/
void motorStop(){
  analogWrite(PIN_ENA, 0);
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
}
void motorDrive(int pwm, bool forward){
  pwm = constrain(pwm, 0, 255);
  digitalWrite(PIN_IN1, forward ? HIGH : LOW);
  digitalWrite(PIN_IN2, forward ? LOW  : HIGH);
  analogWrite(PIN_ENA, pwm);
}

/**************** PAGALBINĖS ****************/
inline bool btn(uint8_t p){ return digitalRead(p) == LOW; }

float adcToV(int a){ return (a * (ADC_VREF / (float)ADC_MAX)); }

const uint8_t AVG_N = 8; // slankus vidurkis analogui
int readPosAdc(){
  long sum = 0;
  for (uint8_t i=0;i<AVG_N;i++){
    sum += analogRead(PIN_POS);
    delayMicroseconds(200);
  }
  return (int)(sum / AVG_N);
}
float readPosV(){ return adcToV(readPosAdc()); }

char gearCh(Gear g){ return (g==G_P)?'P':(g==G_R)?'R':(g==G_N)?'N':(g==G_D)?'D':'-'; }

float targetVof(Gear g){
  switch(g){
    case G_P: return TARGET_P_V;
    case G_R: return TARGET_R_V;
    case G_N: return TARGET_N_V;
    case G_D: return TARGET_D_V;
    default:  return TARGET_P_V;
  }
}

/**************** LCD ****************/
void lcdShow(Gear g){
  float v = readPosV();
  int   a = (int)round((v/ADC_VREF)*ADC_MAX);
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("Gear: "); lcd.print(gearCh(g));
  lcd.setCursor(0,1); lcd.print(v,3); lcd.print("V ADC="); lcd.print(a);
}

/**************** VAŽIAVIMAS IKI TIKSLINĖS ĮTAMPOS ****************/
// Grąžina true, jei pasiekta tikslo zona (su TOL_V)
bool moveToV(float targetV){
  float v = readPosV();

  // apsauga nuo „nulūžusio“ daviklio/laidų
  if (v < POS_MIN_V_OK - 0.05f || v > POS_MAX_V_OK + 0.05f){
    motorStop();
    lcd.clear(); lcd.setCursor(0,0); lcd.print("POS OOR!"); // out-of-range
    // Serial.println(F("ERROR: position voltage out of range"));
    delay(200);
    return false;
  }

  float diff  = targetV - v;
  float adiff = fabs(diff);

  if (adiff <= TOL_V){
    motorStop();
    return true; // tikslo zona
  }

  bool forward = (diff > 0);                      // didesnė V -> „pirmyn“
  int pwm = (adiff < SLOW_BAND_V) ? PWM_SLOW : PWM_FAST;
  motorDrive(pwm, forward);
  return false;
}

/**************** AUTO-P STARTUOJANT (be HOME) ****************/
void autoParkAtBoot(){
  allRelaysOff();
  lcd.clear(); lcd.setCursor(0,0); lcd.print("Auto-P at boot");

  unsigned long t0 = millis();
  unsigned long inZoneSince = 0;
  const unsigned long TIMEOUT_MS = 12000;

  while (true){
    bool at = moveToV(TARGET_P_V);

    if (at){
      if (inZoneSince == 0) inZoneSince = millis();
      if (millis() - inZoneSince >= DWELL_MS) break;
    } else {
      inZoneSince = 0;
    }

    if (millis() - t0 > TIMEOUT_MS){
      motorStop();
      lcd.clear(); lcd.setCursor(0,0); lcd.print("P TIMEOUT");
      // Serial.println(F("ERROR: Auto-P TIMEOUT"));
      return;
    }
    delay(5);
  }

  motorStop();
  currentGear = G_P;
  relaysShowGear(G_P);
  lcdShow(currentGear);
  // Serial.print(F("Boot -> G=P, V=")); Serial.println(readPosV(),3);
}

/**************** PAVARŲ KEITIMAS ****************/
void shiftTo(Gear g){
  if (g == currentGear){
    lcdShow(currentGear);
    return;
  }

  allRelaysOff(); // kol juda – ECU kombinacijos nesiunčiam
  float tgt = targetVof(g);

  // LCD rodo, kur važiuojam
  lcd.clear(); lcd.setCursor(0,0); lcd.print("MOVE to "); lcd.print(gearCh(g));

  unsigned long t0 = millis();
  unsigned long inZoneSince = 0;

  while (true){
    bool at = moveToV(tgt);
    
    // LCD atnaujinimas judėjimo metu
    lcd.setCursor(0,1); lcd.print(readPosV(),3); lcd.print("V "); 
    lcd.print("T:"); lcd.print(tgt,3);

    if (at){
      if (inZoneSince == 0) inZoneSince = millis();
      if (millis() - inZoneSince >= DWELL_MS) break; // laikom pasiekta
    } else {
      inZoneSince = 0;
    }

    if (millis() - t0 > 8000UL){
      // Serial.println(F("ERROR: move timeout – STOP"));
      motorStop();
      lcd.clear(); lcd.setCursor(0,0); lcd.print("MOVE TIMEOUT");
      return;
    }
    delay(5);
  }

  motorStop();
  currentGear = g;
  relaysShowGear(g);
  lcdShow(currentGear);

  // Serial.print(F("Gear=")); Serial.print(gearCh(g));
  // Serial.print(F("  V="));  Serial.println(readPosV(),3);
}

/**************** DEBOUNCE MYGTUKAMS ****************/
bool debounceEdge(uint8_t pin, unsigned long dbMs=60){
  static uint8_t last[40] = {HIGH};
  static unsigned long t[40] = {0};
  uint8_t s = digitalRead(pin);
  if (s != last[pin] && (millis() - t[pin] > dbMs)){
    t[pin] = millis();
    last[pin] = s;
    return (s == LOW); // INPUT_PULLUP: paspausta=LOW
  }
  return false;
}

/**************** SETUP ****************/
void setup(){
  // Serial.begin(115200);

  // L298N
  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  motorStop();

  // Analog padėtis
  pinMode(PIN_POS, INPUT);

  // Mygtukai
  pinMode(PIN_BTN_P, INPUT_PULLUP);
  pinMode(PIN_BTN_R, INPUT_PULLUP);
  pinMode(PIN_BTN_N, INPUT_PULLUP);
  pinMode(PIN_BTN_D, INPUT_PULLUP);

  // Relės – SAUGUS OFF STARTAS
  safeInitRelayPin(REL_POS2);
  safeInitRelayPin(REL_P);
  safeInitRelayPin(REL_R);
  safeInitRelayPin(REL_POS3);

  // LCD
  Wire.begin(); delay(100);
  lcd.init(); lcd.begin(16,2); lcd.backlight();
  lcd.clear(); lcd.setCursor(0,0); lcd.print("Analog shifter");
  delay(200);

  // Startas -> P (Automatiškai nuvažiuoja į sukalibruotą P poziciją)
  autoParkAtBoot();
}

/**************** LOOP ****************/
void loop(){
  // Mygtukų logika: Dabar jie tik perjungia pavarą, nebevaldo variklio rankiniu būdu
  if (debounceEdge(PIN_BTN_P)) shiftTo(G_P);
  if (debounceEdge(PIN_BTN_R)) shiftTo(G_R);
  if (debounceEdge(PIN_BTN_N)) shiftTo(G_N);
  if (debounceEdge(PIN_BTN_D)) shiftTo(G_D);
}