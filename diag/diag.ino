/************************************************************
 * DIAGNOSTIC sketch for the gear actuator  (diag.ino)
 * BENCH / PARKED USE ONLY – do not drive with this sketch.
 *
 * Same wiring as final_safe.ino (D10 = BRAKE).
 * The motor NEVER moves on its own – only on a serial command.
 *
 * LCD pages (rotate every 3 s; any input change jumps to page 1 for 5 s)
 *   1  2.803V G:D OK        position, detected gear, sensor range
 *      P... BRK             buttons held, brake
 *   2  Vcc4.98 min4.71      5 V supply now / lowest seen
 *      Spk P0R0N0D0         noise spikes per button (<20 ms pulses)
 *   3  Noise12 pk40 mV      position sensor noise (peak-to-peak / 1 s)
 *      Rst:BROWNOUT         why the board last restarted
 *
 * Serial monitor 115200 baud, send one letter:
 *   h help        s status stream on/off   p print status once
 *   f jog forward  b jog backward
 *   c check calibration                    l reset log (from final_safe)
 *   x clear reset log                      z zero noise/Vcc counters
 *   w watchdog test (board must reboot once and show Rst:WATCHDOG)
 ************************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
#include <avr/wdt.h>

// Before any function: the builder puts its generated prototypes up here.
enum Gear : uint8_t { G_NONE, G_P, G_R, G_N, G_D };
enum { I_P, I_R, I_N, I_D, I_BRK, I_COUNT };

/**************** RESET CAUSE CAPTURE ****************/
uint8_t g_mcusr __attribute__((section(".noinit")));
uint8_t g_r2    __attribute__((section(".noinit")));
void captureResetFlags(void) __attribute__((naked, used, section(".init3")));
void captureResetFlags(void){
  __asm__ __volatile__("sts %0, r2\n" : "=m"(g_r2));
  g_mcusr = MCUSR;
  MCUSR = 0;
  wdt_disable();
}

/**************** PINS (same as final_safe.ino) ****************/
const uint8_t PIN_ENA = 3, PIN_IN1 = 8, PIN_IN2 = 9;
const uint8_t PIN_POS = A0;

// inputs: index order matters for the interrupt code below
const uint8_t inPins[I_COUNT]  = {4, 5, 6, 7, 10};
const char* const inNames[I_COUNT] = {"P", "R", "N", "D", "BRK"};

/**************** CALIBRATION – copy the SAME values as final_safe.ino ****************/
const float ADC_VREF     = 5.00f;
const float POS_MIN_V_OK = 0.20f;
const float POS_MAX_V_OK = 4.80f;
const float TARGET_P_V   = 0.60f;
const float TARGET_R_V   = 1.20f;
const float TARGET_N_V   = 2.00f;
const float TARGET_D_V   = 2.80f;
const float DETECT_WIN_V = 0.15f;

/**************** TEST SETTINGS ****************/
const int           JOG_PWM       = 120;
const unsigned long JOG_MS        = 250;
const uint32_t      SPIKE_US      = 20000;   // active pulse shorter than this = spike

LiquidCrystal_I2C lcd(0x27, 16, 2);   // if blank, try 0x3F


/**************** NOISE SPIKE COUNTING (pin-change interrupts) ****************/
// Catches pulses far too short for the main loop to see.
volatile uint16_t spikes[I_COUNT];
volatile uint32_t activeSince[I_COUNT];
volatile uint8_t  lastPIND, lastPINB;
const uint8_t pdBit[4] = {4, 5, 6, 7};   // PORTD bits for P R N D

ISR(PCINT2_vect){
  uint8_t now = PIND, ch = now ^ lastPIND;
  lastPIND = now;
  uint32_t t = micros();
  for (uint8_t i = 0; i < 4; i++){
    uint8_t m = _BV(pdBit[i]);
    if (!(ch & m)) continue;
    if (!(now & m)) activeSince[i] = t;                                   // went LOW (active)
    else if (t - activeSince[i] < SPIKE_US && spikes[i] < 65535) spikes[i]++;  // short pulse
  }
}
ISR(PCINT0_vect){                                                         // D10 = PB2 (BRAKE)
  uint8_t now = PINB, ch = now ^ lastPINB;
  lastPINB = now;
  if (!(ch & _BV(2))) return;
  uint32_t t = micros();
  if (!(now & _BV(2))) activeSince[I_BRK] = t;
  else if (t - activeSince[I_BRK] < SPIKE_US && spikes[I_BRK] < 65535) spikes[I_BRK]++;
}

void readSpikes(uint16_t out[I_COUNT]){
  noInterrupts();
  for (uint8_t i = 0; i < I_COUNT; i++) out[i] = spikes[i];
  interrupts();
}

bool act(uint8_t i){ return digitalRead(inPins[i]) == LOW; }

/**************** MOTOR ****************/
void motorBrake(){ digitalWrite(PIN_IN1, LOW); digitalWrite(PIN_IN2, LOW); analogWrite(PIN_ENA, 255); }
void motorDrive(int pwm, bool fwd){
  digitalWrite(PIN_IN1, fwd ? HIGH : LOW);
  digitalWrite(PIN_IN2, fwd ? LOW : HIGH);
  analogWrite(PIN_ENA, constrain(pwm, 0, 255));
}

/**************** POSITION / GEARS ****************/
float readPosV(){
  long s = 0;
  for (uint8_t i = 0; i < 8; i++){ s += analogRead(PIN_POS); delayMicroseconds(150); }
  return (s / 8.0f) * ADC_VREF / 1023.0f;
}
bool posInRange(float v){ return v >= POS_MIN_V_OK && v <= POS_MAX_V_OK; }

float targetVof(Gear g){
  switch (g){
    case G_P: return TARGET_P_V; case G_R: return TARGET_R_V;
    case G_N: return TARGET_N_V; case G_D: return TARGET_D_V;
    default:  return -10.0f;
  }
}
char gearCh(Gear g){ return (g==G_P)?'P':(g==G_R)?'R':(g==G_N)?'N':(g==G_D)?'D':'-'; }

// nearest gear and signed distance (V - target)
Gear nearestGear(float v, float &delta){
  const Gear gs[4] = {G_P, G_R, G_N, G_D};
  Gear best = G_P; delta = v - TARGET_P_V;
  for (uint8_t i = 1; i < 4; i++){
    float d = v - targetVof(gs[i]);
    if (fabs(d) < fabs(delta)){ delta = d; best = gs[i]; }
  }
  return best;
}
Gear detectGear(float v){
  float d; Gear g = nearestGear(v, d);
  return (fabs(d) <= DETECT_WIN_V) ? g : G_NONE;
}

/**************** SUPPLY VOLTAGE (internal 1.1 V reference) ****************/
// Accuracy about ±10 % absolute, but dips show up clearly.
float readVcc(){
  ADMUX = _BV(REFS0) | 0x0E;                 // AVcc reference, measure bandgap
  delay(2);
  ADCSRA |= _BV(ADSC);
  while (ADCSRA & _BV(ADSC)) {}
  uint16_t a = ADC;
  analogRead(PIN_POS);                       // restore mux, discard
  return a ? 1.1f * 1023.0f / a : 0.0f;
}

/**************** RESET LOG (same EEPROM layout as final_safe.ino) ****************/
const int     EE_MAGIC = 0, EE_IDX = 1, EE_LOG = 2, EE_LOG_N = 8;
const int     EE_CNT_PWR = 10, EE_CNT_EXT = 11, EE_CNT_BOR = 12, EE_CNT_WDT = 13;
const uint8_t EE_MAGIC_VAL = 0x5A;

uint8_t bootFlags = 0;

uint8_t resetFlags(){
  uint8_t f = g_mcusr & 0x0F;
  if (!f && !(g_r2 & 0xF0)) f = g_r2 & 0x0F;
  return f;
}
const char* resetName(uint8_t f){
  if (f & _BV(WDRF))  return "WATCHDOG";
  if (f & _BV(EXTRF)) return "RESET BTN";
  if (f & _BV(PORF))  return "POWER ON";
  if (f & _BV(BORF))  return "BROWNOUT";
  return "UNKNOWN";
}

/**************** STATS ****************/
float    vccNow = 0, vccMin = 99;
int      rawMin = 1023, rawMax = 0;
uint16_t noiseMv = 0, noisePeak = 0;
uint16_t oorCount = 0;
bool     wasInRange = true;
unsigned long lastNoise = 0, lastVcc = 0, lastLcd = 0, lastStream = 0, liveAt = 0;
uint8_t  lastInputMask = 0xFF;
bool     streaming = false;

void zeroCounters(){
  noInterrupts();
  for (uint8_t i = 0; i < I_COUNT; i++) spikes[i] = 0;
  interrupts();
  vccMin = 99; noisePeak = 0; oorCount = 0;
}

/**************** LCD ****************/
void lcdLine(uint8_t row, const char* s){
  char b[17]; uint8_t i = 0;
  for (; i < 16 && s[i]; i++) b[i] = s[i];
  for (; i < 16; i++) b[i] = ' ';
  b[16] = 0;
  lcd.setCursor(0, row); lcd.print(b);
}
void lcd2(const char* a, const char* b){ lcdLine(0, a); lcdLine(1, b); }

unsigned u99(uint16_t x){ return x > 99 ? 99 : x; }

void updateLcd(unsigned long now, float v){
  char l0[17], l1[17], a[8], b[8];
  uint8_t page = (now - liveAt < 5000) ? 0 : (now / 3000) % 3;

  if (page == 0){
    Gear g = detectGear(v);
    dtostrf(v, 5, 3, a);
    snprintf(l0, sizeof l0, "%sV G:%c %s", a, gearCh(g),
             v < POS_MIN_V_OK ? "LOW!" : v > POS_MAX_V_OK ? "HIGH" : "OK");
    snprintf(l1, sizeof l1, "%c%c%c%c %s",
             act(I_P) ? 'P' : '.', act(I_R) ? 'R' : '.', act(I_N) ? 'N' : '.', act(I_D) ? 'D' : '.',
             act(I_BRK) ? "BRK" : "---");
  } else if (page == 1){
    uint16_t s[I_COUNT]; readSpikes(s);
    dtostrf(vccNow, 4, 2, a); dtostrf(vccMin, 4, 2, b);
    snprintf(l0, sizeof l0, "Vcc%s min%s", a, b);
    snprintf(l1, sizeof l1, "Spk P%uR%uN%uD%u", u99(s[I_P]), u99(s[I_R]), u99(s[I_N]), u99(s[I_D]));
  } else {
    snprintf(l0, sizeof l0, "Noise%u pk%u mV", noiseMv, noisePeak);
    snprintf(l1, sizeof l1, "Rst:%s", resetName(bootFlags));
  }
  lcd2(l0, l1);
}

/**************** SERIAL OUTPUT ****************/
void printHelp(){
  Serial.println(F("\n=== GEAR ACTUATOR DIAGNOSTICS ==="));
  Serial.println(F(" s status stream on/off   p status once"));
  Serial.println(F(" f / b  jog motor forward / backward (short, slow)"));
  Serial.println(F(" c calibration check      l reset log"));
  Serial.println(F(" x clear reset log        z zero counters"));
  Serial.println(F(" w watchdog test (must reboot ONCE)"));
  Serial.println(F(" Noise test: send z, then do NOT touch buttons, start engine,"));
  Serial.println(F(" run fan/lights/wipers; spikes should stay 0.\n"));
}

void printStatus(){
  float v = readPosV(), d;
  Gear g = nearestGear(v, d);
  uint16_t s[I_COUNT]; readSpikes(s);

  Serial.print(F("V=")); Serial.print(v, 3);
  Serial.print(posInRange(v) ? F(" [OK]") : F(" [OUT OF RANGE]"));
  Serial.print(F("  nearest ")); Serial.print(gearCh(g));
  Serial.print(F(" ")); if (d >= 0) Serial.print('+'); Serial.print(d, 3);
  if (fabs(d) > DETECT_WIN_V) Serial.print(F(" (between gears)"));

  Serial.print(F(" | held:"));
  for (uint8_t i = 0; i < I_COUNT; i++) if (act(i)){ Serial.print(' '); Serial.print(inNames[i]); }

  Serial.print(F(" | Vcc=")); Serial.print(vccNow, 2);
  Serial.print(F(" min=")); Serial.print(vccMin, 2);
  Serial.print(F(" | noise=")); Serial.print(noiseMv); Serial.print(F("mV pk="));
  Serial.print(noisePeak); Serial.print(F("mV OOR=")); Serial.print(oorCount);

  Serial.print(F(" | spikes"));
  for (uint8_t i = 0; i < I_COUNT; i++){ Serial.print(' '); Serial.print(inNames[i]); Serial.print('='); Serial.print(s[i]); }
  Serial.println();
}

void printResetLog(){
  Serial.print(F("This boot: ")); Serial.println(resetName(bootFlags));
  if (EEPROM.read(EE_MAGIC) != EE_MAGIC_VAL){
    Serial.println(F("No stored log yet (final_safe.ino has not run since it was cleared)."));
    return;
  }
  Serial.print(F("Counts  power-on=")); Serial.print(EEPROM.read(EE_CNT_PWR));
  Serial.print(F("  reset-btn="));      Serial.print(EEPROM.read(EE_CNT_EXT));
  Serial.print(F("  BROWNOUT="));       Serial.print(EEPROM.read(EE_CNT_BOR));
  Serial.print(F("  WATCHDOG="));       Serial.println(EEPROM.read(EE_CNT_WDT));
  Serial.print(F("Last restarts, oldest -> newest:"));
  uint8_t idx = EEPROM.read(EE_IDX) % EE_LOG_N;
  for (uint8_t k = 0; k < EE_LOG_N; k++){
    uint8_t e = EEPROM.read(EE_LOG + (idx + k) % EE_LOG_N);
    if (e){ Serial.print(' '); Serial.print(resetName(e)); }
  }
  Serial.println();
  if (EEPROM.read(EE_CNT_BOR) || EEPROM.read(EE_CNT_WDT))
    Serial.println(F("!! Brownout/watchdog resets happened -> check power supply, wiring, heat."));
}

void clearResetLog(){
  for (int a = 0; a < 16; a++) EEPROM.update(a, 0);
  Serial.println(F("Reset log cleared."));
}

void checkCalibration(){
  const float t[4] = {TARGET_P_V, TARGET_R_V, TARGET_N_V, TARGET_D_V};
  const char  n[4] = {'P', 'R', 'N', 'D'};
  bool ok = true;
  Serial.println(F("Calibration check:"));
  for (uint8_t i = 0; i < 4; i++){
    Serial.print(F("  ")); Serial.print(n[i]); Serial.print(F(" = ")); Serial.print(t[i], 3);
    if (t[i] < POS_MIN_V_OK + DETECT_WIN_V || t[i] > POS_MAX_V_OK - DETECT_WIN_V){
      Serial.print(F("  TOO CLOSE TO SENSOR LIMIT")); ok = false;
    }
    Serial.println();
    for (uint8_t j = i + 1; j < 4; j++)
      if (fabs(t[i] - t[j]) < 2 * DETECT_WIN_V + 0.05f){
        Serial.print(F("  ")); Serial.print(n[i]); Serial.print(F(" and ")); Serial.print(n[j]);
        Serial.println(F(" are TOO CLOSE together")); ok = false;
      }
  }
  Serial.println(ok ? F("  -> OK, final_safe.ino will accept these.") : F("  -> FAIL, final_safe.ino will show BAD CALIBRATION."));
}

/**************** TESTS ****************/
void jog(bool fwd){
  float v0 = readPosV();
  if (!posInRange(v0)){ Serial.println(F("Jog refused: sensor out of range.")); return; }
  if ((fwd && v0 > POS_MAX_V_OK - 0.3f) || (!fwd && v0 < POS_MIN_V_OK + 0.3f)){
    Serial.println(F("Jog refused: too close to the end of travel.")); return;
  }
  lcd2(fwd ? "JOG FORWARD" : "JOG BACKWARD", "");
  motorDrive(JOG_PWM, fwd);
  unsigned long t0 = millis();
  while (millis() - t0 < JOG_MS){
    if (!posInRange(readPosV())) break;       // stop at once on sensor problem
  }
  motorBrake();
  delay(150);
  float v1 = readPosV(), dv = v1 - v0;

  Serial.print(fwd ? F("Jog forward: ") : F("Jog backward: "));
  Serial.print(v0, 3); Serial.print(F(" -> ")); Serial.print(v1, 3);
  Serial.print(F("  dV=")); Serial.print(dv, 3);
  const char* res;
  if (fabs(dv) < 0.02f){
    res = "NO MOVE";
    Serial.println(F("  NO MOVEMENT – check motor power, L298N, linkage."));
  } else if ((fwd && dv < 0) || (!fwd && dv > 0)){
    res = "WRONG DIR!";
    Serial.println(F("  DIRECTION INVERTED – swap motor wires (or IN1/IN2)."));
  } else {
    res = "OK";
    Serial.println(F("  OK (forward = voltage goes up)."));
  }
  char l1[17], a[8];
  dtostrf(dv, 6, 3, a);
  snprintf(l1, sizeof l1, "dV%s %s", a, res);
  lcd2(fwd ? "JOG FORWARD" : "JOG BACKWARD", l1);
  delay(1500);
}

void watchdogTest(){
  Serial.println(F("Watchdog test: freezing now. The board should reboot ONCE in ~1 s"));
  Serial.println(F("and show Rst:WATCHDOG. If it keeps rebooting, flash the new bootloader."));
  Serial.flush();
  lcd2("WATCHDOG TEST", "should reboot...");
  wdt_enable(WDTO_1S);
  while (true) {}
}

void handleSerial(){
  if (!Serial.available()) return;
  char c = tolower(Serial.read());
  switch (c){
    case 'h': printHelp(); break;
    case 's': streaming = !streaming; Serial.println(streaming ? F("Stream ON") : F("Stream OFF")); break;
    case 'p': printStatus(); break;
    case 'f': jog(true); break;
    case 'b': jog(false); break;
    case 'c': checkCalibration(); break;
    case 'l': printResetLog(); break;
    case 'x': clearResetLog(); break;
    case 'z': zeroCounters(); Serial.println(F("Counters zeroed.")); break;
    case 'w': watchdogTest(); break;
    default: break;   // ignore newlines etc.
  }
}

/**************** SETUP ****************/
void setup(){
  // safe outputs
  digitalWrite(PIN_ENA, LOW);
  pinMode(PIN_ENA, OUTPUT); pinMode(PIN_IN1, OUTPUT); pinMode(PIN_IN2, OUTPUT);
  motorBrake();

  // inputs + pin-change interrupts
  for (uint8_t i = 0; i < I_COUNT; i++) pinMode(inPins[i], INPUT_PULLUP);
  pinMode(PIN_POS, INPUT);
  delay(5);
  lastPIND = PIND; lastPINB = PINB;
  PCMSK2 = _BV(PCINT20) | _BV(PCINT21) | _BV(PCINT22) | _BV(PCINT23); // D4-D7
  PCMSK0 = _BV(PCINT2);                                                              // D10
  PCIFR  = _BV(PCIF2) | _BV(PCIF0);
  PCICR |= _BV(PCIE2) | _BV(PCIE0);

  bootFlags = resetFlags();

  Serial.begin(115200);
  printHelp();
  printResetLog();
  checkCalibration();

  Wire.begin();
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(3000, true);
#endif
  lcd.init(); lcd.backlight();
  char l1[17];
  snprintf(l1, sizeof l1, "Rst:%s", resetName(bootFlags));
  lcd2("DIAGNOSTIC MODE", l1);
  delay(1500);

  vccNow = readVcc(); vccMin = vccNow;
  liveAt = millis();
}

/**************** LOOP ****************/
void loop(){
  unsigned long now = millis();
  handleSerial();

  float v = readPosV();

  // sensor noise: raw min/max over 1 s
  int raw = analogRead(PIN_POS);
  if (raw < rawMin) rawMin = raw;
  if (raw > rawMax) rawMax = raw;
  if (now - lastNoise >= 1000){
    lastNoise = now;
    noiseMv = (uint16_t)((rawMax - rawMin) * 5000L / 1023);
    if (noiseMv > noisePeak) noisePeak = noiseMv;
    rawMin = 1023; rawMax = 0;
  }

  // out-of-range events
  bool inR = posInRange(v);
  if (wasInRange && !inR && oorCount < 65535) oorCount++;
  wasInRange = inR;

  // supply voltage every 100 ms
  if (now - lastVcc >= 100){
    lastVcc = now;
    vccNow = readVcc();
    if (vccNow < vccMin) vccMin = vccNow;
  }

  // any input change -> show live page
  uint8_t mask = 0;
  for (uint8_t i = 0; i < I_COUNT; i++) if (act(i)) mask |= 1 << i;
  if (mask != lastInputMask){ lastInputMask = mask; liveAt = now; }

  if (now - lastLcd >= 250){ lastLcd = now; updateLcd(now, v); }
  if (streaming && now - lastStream >= 500){ lastStream = now; printStatus(); }
}
