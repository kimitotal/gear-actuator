/************************************************************
 * Gear actuator with ANALOG position feedback – SAFE version
 *
 * What changed vs. final.ino
 *  - NO movement at power-up / reset. The sensor is read and the gear the
 *    actuator is ALREADY in is adopted. A reset while driving no longer
 *    sends the gearbox to P.
 *  - Interlock: brake needed to select P or R, to leave P and to leave an
 *    unknown position (except to N). No vehicle-speed input.
 *  - Real button debounce (must be stable LOW for 80 ms). Two buttons at
 *    the same time = ignored (wiring/noise fault).
 *  - Non-blocking state machine + 1 s watchdog. Reset cause (power-on,
 *    brownout, watchdog, reset button) logged to EEPROM and shown at boot.
 *  - Move timeout, stall and wrong-direction detection. After ANY failed
 *    move the gear is re-detected from the sensor (no stale currentGear).
 *  - Position sensor valid window 0.20–4.80 V (broken/shorted wire caught).
 *    Must be out of range for 100 ms; meanwhile the motor is braked.
 *  - Position hold: if the actuator drifts off the selected gear it is pulled
 *    back (max 3 times / 10 s, then hold is disabled and the shown gear
 *    follows the real position).
 *  - Limp mode: in a fault, hold P or D + brake to creep the actuator blind
 *    toward that end (so the car can still be parked with a dead sensor).
 *  - Motor rests in brake (shorted) instead of free-wheeling.
 *  - LCD refreshed every 250 ms, never while the motor is moving. If the
 *    LCD/I2C bus hangs, the watchdog resets and the sketch runs without it.
 *
 * Pins (unchanged)
 *   L298N: ENA=3 (PWM), IN1=8, IN2=9      Position sensor: A0 (0..5 V)
 *   Buttons (INPUT_PULLUP): P=4 R=5 N=6 D=7
 *   LCD1602 I2C: SDA=A4 SCL=A5 (0x27)
 * NEW pin
 *   D10 = BRAKE, LOW = pressed (via optocoupler from brake-light switch,
 *         NEVER 12 V directly). Open wire reads "not pressed".
 *   Fails safe: a broken wire blocks P/R and leaving P.
 *
 * IMPORTANT: on an Arduino Nano with the "Old Bootloader", a watchdog reset
 * can cause an endless reboot loop. Use an Uno, or a Nano with the new
 * (Optiboot) bootloader.
 ************************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
#include <avr/wdt.h>

// Declared before any function: the Arduino builder puts its generated
// prototypes above the first function, and they need these types.
enum Gear  : uint8_t { G_NONE, G_P, G_R, G_N, G_D };
enum State : uint8_t { ST_IDLE, ST_MOVING, ST_FAULT };

/**************** RESET CAUSE CAPTURE (runs before setup) ****************/
// Optiboot clears MCUSR and passes it in r2; other bootloaders leave MCUSR.
uint8_t g_mcusr __attribute__((section(".noinit")));
uint8_t g_r2    __attribute__((section(".noinit")));
void captureResetFlags(void) __attribute__((naked, used, section(".init3")));
void captureResetFlags(void){
  __asm__ __volatile__("sts %0, r2\n" : "=m"(g_r2));
  g_mcusr = MCUSR;
  MCUSR = 0;
  wdt_disable();
}

/**************** PINS ****************/
const uint8_t PIN_ENA = 3;
const uint8_t PIN_IN1 = 8;
const uint8_t PIN_IN2 = 9;
const uint8_t PIN_POS = A0;

const uint8_t PIN_BTN_P = 4;
const uint8_t PIN_BTN_R = 5;
const uint8_t PIN_BTN_N = 6;
const uint8_t PIN_BTN_D = 7;

const uint8_t PIN_BRAKE   = 10;  // LOW = brake pressed

/**************** SAFETY CONFIG ****************/
const bool REQUIRE_BRAKE_FOR_P         = true;
const bool REQUIRE_BRAKE_TO_LEAVE_P   = true;
const bool REQUIRE_BRAKE_FOR_R        = true;
const bool REQUIRE_BRAKE_FROM_UNKNOWN = true;  // leaving "-" (except to N)
const bool ENABLE_POSITION_HOLD       = true;

/**************** CALIBRATION ****************/
const float ADC_VREF     = 5.00f;
const float POS_MIN_V_OK = 0.20f;   // below = broken wire / short to GND
const float POS_MAX_V_OK = 4.80f;   // above = short to 5 V

// ***************************************************************
// Paste values from kalibracija.ino here
const float TARGET_P_V = 0.60f;
const float TARGET_R_V = 1.20f;
const float TARGET_N_V = 2.00f;
const float TARGET_D_V = 2.80f;
// ***************************************************************

const float TOL_V        = 0.03f;   // arrival tolerance
const float HOLD_TOL_V   = 0.10f;   // drift allowed before pulling back
const float DETECT_WIN_V = 0.15f;   // window to recognise a gear from position

const int   PWM_FAST    = 220;
const int   PWM_SLOW    = 140;
const float SLOW_BAND_V = 0.15f;

/**************** TIMING ****************/
const unsigned long DWELL_MS             = 120;
const unsigned long MOVE_TIMEOUT_MS      = 5000;
const unsigned long STALL_MS             = 500;    // no progress this long = stall
const float         STALL_MIN_DV         = 0.02f;
const float         WRONG_DIR_V          = 0.15f;  // moved this far the wrong way
const uint8_t       WRONG_DIR_READINGS   = 10;     // ...for this many readings in a row (~25 ms)
const unsigned long BTN_STABLE_MS        = 80;
const unsigned long INPUT_STABLE_MS      = 30;
const unsigned long DRIFT_CONFIRM_MS     = 200;
const uint8_t       MAX_CORRECTIONS      = 3;
const unsigned long CORRECTION_WINDOW_MS = 10000;
const unsigned long SENSOR_FAULT_MS      = 100;    // out of range this long = fault
const unsigned long SENSOR_RECOVER_MS    = 1000;
const unsigned long LCD_PERIOD_MS        = 250;
const unsigned long MSG_MS               = 1500;

/**************** TYPES / STATE ****************/

LiquidCrystal_I2C lcd(0x27, 16, 2);   // if blank, try 0x3F

State state       = ST_IDLE;
Gear  currentGear = G_NONE;
bool  calOk       = true;
bool  holdSuspended = true;           // hold starts after first driver shift
char  faultText[17] = "";

/**************** DEBOUNCED INPUT ****************/
struct DebouncedInput {
  uint8_t pin;
  unsigned long stableMs;
  bool lastRaw;
  bool stable;              // true = active (LOW)
  unsigned long changedAt;

  void begin(){
    pinMode(pin, INPUT_PULLUP);
    delay(2);
    lastRaw = stable = (digitalRead(pin) == LOW);  // held at boot != press
    changedAt = millis();
  }
  // true only on a new, stable press
  bool update(unsigned long now){
    bool raw = (digitalRead(pin) == LOW);
    if (raw != lastRaw){ lastRaw = raw; changedAt = now; }
    if (raw != stable && (now - changedAt) >= stableMs){
      stable = raw;
      return stable;
    }
    return false;
  }
};

DebouncedInput btnP      = {PIN_BTN_P,   BTN_STABLE_MS,   false, false, 0};
DebouncedInput btnR      = {PIN_BTN_R,   BTN_STABLE_MS,   false, false, 0};
DebouncedInput btnN      = {PIN_BTN_N,   BTN_STABLE_MS,   false, false, 0};
DebouncedInput btnD      = {PIN_BTN_D,   BTN_STABLE_MS,   false, false, 0};
DebouncedInput inBrake   = {PIN_BRAKE,   INPUT_STABLE_MS, false, false, 0};

/**************** MESSAGES ****************/
char msgText[17] = "";
unsigned long msgAt = 0;
void showMsg(const char* t){
  strncpy(msgText, t, 16); msgText[16] = 0;
  msgAt = millis();
}

/**************** MOTOR ****************/
bool motorOn = false;   // true while the motor is being driven
// Brake: both low-side switches on -> motor shorted, resists back-driving.
void motorBrake(){
  motorOn = false;
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
  analogWrite(PIN_ENA, 255);
}
void motorDrive(int pwm, bool forward){
  motorOn = true;
  pwm = constrain(pwm, 0, 255);
  digitalWrite(PIN_IN1, forward ? HIGH : LOW);
  digitalWrite(PIN_IN2, forward ? LOW  : HIGH);
  analogWrite(PIN_ENA, pwm);
}

/**************** POSITION ****************/
float readPosV(){
  long sum = 0;
  for (uint8_t i = 0; i < 8; i++){
    sum += analogRead(PIN_POS);
    delayMicroseconds(150);
  }
  return (sum / 8.0f) * ADC_VREF / 1023.0f;
}
bool posInRange(float v){ return v >= POS_MIN_V_OK && v <= POS_MAX_V_OK; }

float targetVof(Gear g){
  switch (g){
    case G_P: return TARGET_P_V;
    case G_R: return TARGET_R_V;
    case G_N: return TARGET_N_V;
    case G_D: return TARGET_D_V;
    default:  return -10.0f;
  }
}

Gear detectGear(float v){
  const Gear gs[4] = {G_P, G_R, G_N, G_D};
  for (uint8_t i = 0; i < 4; i++)
    if (fabs(v - targetVof(gs[i])) <= DETECT_WIN_V) return gs[i];
  return G_NONE;
}

char gearCh(Gear g){
  return (g==G_P)?'P':(g==G_R)?'R':(g==G_N)?'N':(g==G_D)?'D':'?';
}

bool checkCalibration(){
  const float t[4] = {TARGET_P_V, TARGET_R_V, TARGET_N_V, TARGET_D_V};
  for (uint8_t i = 0; i < 4; i++){
    if (t[i] < POS_MIN_V_OK + DETECT_WIN_V || t[i] > POS_MAX_V_OK - DETECT_WIN_V) return false;
    for (uint8_t j = i + 1; j < 4; j++)
      if (fabs(t[i] - t[j]) < 2 * DETECT_WIN_V + 0.05f) return false;   // windows must not overlap
  }
  return true;
}

struct Move {
  Gear  target;
  float targetV;
  bool  correction;          // true = hold correction
  unsigned long startT;
  bool  inZone;
  unsigned long inZoneSince;
  float progressAdiff;       // distance to target at last real progress
  bool  progressFwd;         // drive direction at that moment
  unsigned long progressT;
  float startAbsDiff;
  uint8_t wrongN;            // consecutive wrong-direction readings
} mv;

/**************** FAULTS ****************/
bool recovering = false;
unsigned long recoverSince = 0;

void enterFault(const char* t){
  motorBrake();
  // Mid-shift = gear unknown. A hold correction keeps its gear.
  if (state == ST_MOVING && !mv.correction) currentGear = G_NONE;
  strncpy(faultText, t, 16); faultText[16] = 0;
  recovering = false;
  state = ST_FAULT;
}

void faultRecover(float v, unsigned long now){
  motorBrake();
  if (!calOk) return;                         // calibration error is permanent
  if (!posInRange(v)){ recovering = false; return; }
  if (!recovering){ recovering = true; recoverSince = now; return; }
  if (now - recoverSince >= SENSOR_RECOVER_MS){
    recovering    = false;
    currentGear   = detectGear(v);
    holdSuspended = true;
    state = ST_IDLE;
    showMsg("SENSOR OK");
  }
}

// Limp mode. In a fault there is no position feedback, so no gear can be
// selected. To still be able to park: press and HOLD P or D with the brake
// pressed and the actuator creeps blind toward that end of travel, for as
// long as the button is held (max JOG_MAX_MS per press). Needs a fresh press,
// so a shorted button wire cannot start it.
const unsigned long JOG_MAX_MS = 3000;

bool faultJog(Gear req, unsigned long now){
  static Gear jog = G_NONE;
  static unsigned long jogStart = 0;
  if (req == G_P || req == G_D){
    if (inBrake.stable){
      jog = req; jogStart = now;
      currentGear = G_NONE;
      recovering = false;
      showMsg(req == G_P ? "MANUAL -> P" : "MANUAL -> D");
    } else showMsg("PRESS BRAKE");
  } else if (req != G_NONE) showMsg("HOLD P/D + BRAKE");

  // raw pin, not debounced: letting go must stop the motor at once
  bool held = (jog == G_P) ? btnP.lastRaw : (jog == G_D) ? btnD.lastRaw : false;
  if (!held || !inBrake.stable || now - jogStart >= JOG_MAX_MS){ jog = G_NONE; return false; }
  motorDrive(PWM_SLOW, (jog == G_D) == (TARGET_D_V > TARGET_P_V));
  return true;
}

/**************** MOVE (non-blocking) ****************/

bool drifting = false;
unsigned long driftSince = 0;
uint8_t correctionCount = 0;
unsigned long corrWindowStart = 0;

void startMove(Gear g, bool correction, float v){
  mv.target       = g;
  mv.targetV      = targetVof(g);
  mv.correction   = correction;
  mv.startT       = millis();
  mv.inZone       = false;
  mv.progressT    = mv.startT;
  mv.startAbsDiff = fabs(mv.targetV - v);
  mv.progressAdiff = mv.startAbsDiff;
  mv.progressFwd   = mv.targetV > v;
  mv.wrongN        = 0;
  state = ST_MOVING;
}

void endMove(bool success, const char* why, float v){
  motorBrake();
  if (success){
    currentGear = mv.target;
    if (!mv.correction){ holdSuspended = false; correctionCount = 0; }
  } else {
    currentGear = detectGear(v);              // trust the sensor, not memory
    holdSuspended = true;                     // no automatic moves after a failure
    showMsg(why);
  }
  drifting = false;
  state = ST_IDLE;
}

void runMove(float v, unsigned long now){
  float diff  = mv.targetV - v;
  float adiff = fabs(diff);
  bool  fwd   = diff > 0;                      // higher V = forward

  if (adiff <= TOL_V){
    motorBrake();
    if (!mv.inZone){ mv.inZone = true; mv.inZoneSince = now; }
    mv.progressAdiff = adiff; mv.progressFwd = fwd; mv.progressT = now;
    if (now - mv.inZoneSince >= DWELL_MS) endMove(true, "", v);
    return;
  }
  mv.inZone = false;

  if (now - mv.startT > MOVE_TIMEOUT_MS)       { endMove(false, "SHIFT TOO SLOW", v);    return; }
  // One wild reading (a sensor wire dropping out passes through every voltage)
  // must not abort the shift: it has to be wrong several readings in a row.
  if (adiff <= mv.startAbsDiff + WRONG_DIR_V) mv.wrongN = 0;
  else if (++mv.wrongN >= WRONG_DIR_READINGS)  { endMove(false, "MOTOR REVERSED?", v); return; }

  // Progress = getting CLOSER (or reversing after an overshoot). Sensor noise
  // wobbling back and forth must not keep a stalled motor powered. If the
  // distance is suddenly far above the best seen, that best was a bad reading.
  if (fwd != mv.progressFwd || mv.progressAdiff - adiff >= STALL_MIN_DV ||
      adiff - mv.progressAdiff > WRONG_DIR_V){
    mv.progressAdiff = adiff; mv.progressFwd = fwd; mv.progressT = now;
  }
  else if (now - mv.progressT > STALL_MS)      { endMove(false, "MOTOR BLOCKED", v);           return; }

  motorDrive(adiff < SLOW_BAND_V ? PWM_SLOW : PWM_FAST, fwd);
}

/**************** POSITION HOLD ****************/
void holdMonitor(float v, unsigned long now){
  if (currentGear == G_NONE){ drifting = false; return; }
  float err = fabs(v - targetVof(currentGear));
  if (err <= HOLD_TOL_V){ drifting = false; return; }
  if (!drifting){ drifting = true; driftSince = now; return; }
  if (now - driftSince < DRIFT_CONFIRM_MS) return;
  drifting = false;

  if (!ENABLE_POSITION_HOLD || holdSuspended){
    // not allowed to move: at least make the shown gear match reality
    if (err > DETECT_WIN_V){
      currentGear = detectGear(v);
      showMsg("OUT OF GEAR");
    }
    return;
  }

  if (now - corrWindowStart > CORRECTION_WINDOW_MS){ corrWindowStart = now; correctionCount = 0; }
  if (++correctionCount > MAX_CORRECTIONS){
    holdSuspended = true;
    showMsg("CANT HOLD GEAR");
    if (err > DETECT_WIN_V) currentGear = detectGear(v);
    return;
  }
  startMove(currentGear, true, v);
}

/**************** SHIFT REQUESTS ****************/
const char* shiftBlockReason(Gear from, Gear to){
  bool brake   = inBrake.stable;
  if (to == G_P && REQUIRE_BRAKE_FOR_P && !brake)                      return "PRESS BRAKE";
  if (to == G_R && REQUIRE_BRAKE_FOR_R && !brake)                      return "PRESS BRAKE";
  if (from == G_P && REQUIRE_BRAKE_TO_LEAVE_P && !brake)               return "PRESS BRAKE";
  if (from == G_NONE && to != G_N && REQUIRE_BRAKE_FROM_UNKNOWN && !brake) return "PRESS BRAKE";
  return nullptr;
}

void requestShift(Gear g, float v){
  // same gear again = re-seat it, if it sits off-centre (e.g. after CANT HOLD GEAR)
  if (g == currentGear && fabs(v - targetVof(g)) <= HOLD_TOL_V) return;
  const char* why = shiftBlockReason(currentGear, g);
  if (why){ showMsg(why); return; }
  startMove(g, false, v);
  updateLcd(v, millis());    // show "D -> N" now: no LCD refresh while the motor runs
  mv.startT = mv.progressT = millis();   // the LCD write must not eat into the timeouts
}

Gear readButtons(unsigned long now){
  bool eP = btnP.update(now);
  bool eR = btnR.update(now);
  bool eN = btnN.update(now);
  bool eD = btnD.update(now);
  if (!(eP || eR || eN || eD)) return G_NONE;
  // raw, not debounced: a second button that is only just going down counts too
  uint8_t held = btnP.lastRaw + btnR.lastRaw + btnN.lastRaw + btnD.lastRaw;
  if (held > 1){ showMsg("2 BUTTONS HELD"); return G_NONE; }
  return eP ? G_P : eR ? G_R : eN ? G_N : G_D;
}

/**************** RESET LOG (EEPROM) ****************/
const int     EE_MAGIC = 0, EE_IDX = 1, EE_LOG = 2, EE_LOG_N = 8;
const int     EE_CNT_PWR = 10, EE_CNT_EXT = 11, EE_CNT_BOR = 12, EE_CNT_WDT = 13;
const uint8_t EE_MAGIC_VAL = 0x5A;   // change to wipe the log

uint8_t resetFlags(){
  uint8_t f = g_mcusr & 0x0F;
  if (!f && !(g_r2 & 0xF0)) f = g_r2 & 0x0F;
  return f;
}

// Several flags can be set at once. Optiboot leaves its bootloader through the
// watchdog, so a reset button / upload arrives as EXTRF+WDRF: EXTRF must win.
const char* resetName(uint8_t f){
  if (f & _BV(PORF))  return "POWER ON";
  if (f & _BV(EXTRF)) return "RESET BTN";
  if (f & _BV(WDRF))  return "WATCHDOG";
  if (f & _BV(BORF))  return "BROWNOUT";
  return "UNKNOWN";
}

void bump(int addr){ uint8_t c = EEPROM.read(addr); if (c < 255) EEPROM.write(addr, c + 1); }

void logReset(uint8_t f){
  if (EEPROM.read(EE_MAGIC) != EE_MAGIC_VAL){
    for (int a = 0; a < 16; a++) EEPROM.update(a, 0);
    EEPROM.update(EE_MAGIC, EE_MAGIC_VAL);
  }
  uint8_t idx = EEPROM.read(EE_IDX) % EE_LOG_N;
  EEPROM.update(EE_LOG + idx, f);
  EEPROM.update(EE_IDX, (idx + 1) % EE_LOG_N);
  if      (f & _BV(PORF))  bump(EE_CNT_PWR);   // same priority as resetName()
  else if (f & _BV(EXTRF)) bump(EE_CNT_EXT);
  else if (f & _BV(WDRF))  bump(EE_CNT_WDT);
  else if (f & _BV(BORF))  bump(EE_CNT_BOR);
}

/**************** LCD ****************/
unsigned long lastLcd = 0;

// Set while talking to the LCD. If the watchdog fires in there (stuck I2C
// bus), the next boot sees it and runs without the display until power-off.
uint16_t g_lcdBusy __attribute__((section(".noinit")));
const uint16_t LCD_BUSY_MAGIC = 0xA55A;
bool lcdOk = true;

void lcdLine(uint8_t row, const char* s){
  char buf[17]; uint8_t i = 0;
  for (; i < 16 && s[i]; i++) buf[i] = s[i];
  for (; i < 16; i++) buf[i] = ' ';
  buf[16] = 0;
  lcd.setCursor(0, row);
  lcd.print(buf);
}

void updateLcd(float v, unsigned long now){
  if (!lcdOk) return;
  char l0[17], l1[17], vs[8];
  if (state == ST_MOVING)                      // drawn once, by requestShift()
    snprintf(l0, sizeof l0, "%c -> %c", gearCh(currentGear), gearCh(mv.target));
  else if (msgText[0] && now - msgAt < MSG_MS)
    snprintf(l0, sizeof l0, "%s", msgText);
  else if (state == ST_FAULT)
    snprintf(l0, sizeof l0, "%s", faultText);
  else
    snprintf(l0, sizeof l0, "Gear: %c", gearCh(currentGear));

  dtostrf(v, 4, 2, vs);
  snprintf(l1, sizeof l1, "%sV %s", vs, inBrake.stable ? "BRAKE" : "");
  g_lcdBusy = LCD_BUSY_MAGIC;
  lcdLine(0, l0);
  lcdLine(1, l1);
  g_lcdBusy = 0;
}

/**************** SETUP ****************/
void setup(){
  // 1) Outputs to a safe state FIRST
  digitalWrite(PIN_ENA, LOW);
  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  motorBrake();

  // 2) Inputs
  pinMode(PIN_POS, INPUT);
  btnP.begin(); btnR.begin(); btnN.begin(); btnD.begin();
  inBrake.begin();

  // 3) Adopt the gear the actuator is ALREADY in – never move at boot
  calOk = checkCalibration();
  float v = readPosV();
  if (!calOk)                enterFault("BAD CALIBRATION");
  else if (!posInRange(v))   enterFault("SENSOR FAULT");
  else {
    currentGear = detectGear(v);
  }
  holdSuspended = true;

  // 4) Reset diagnostics
  uint8_t rf = resetFlags();
  logReset(rf);
  Serial.begin(115200);
  Serial.print(F("Reset cause: ")); Serial.println(resetName(rf));
  Serial.print(F("Counts PWR/EXT/BOR/WDT: "));
  Serial.print(EEPROM.read(EE_CNT_PWR)); Serial.print('/');
  Serial.print(EEPROM.read(EE_CNT_EXT)); Serial.print('/');
  Serial.print(EEPROM.read(EE_CNT_BOR)); Serial.print('/');
  Serial.println(EEPROM.read(EE_CNT_WDT));
  Serial.print(F("Boot gear: ")); Serial.print(gearCh(currentGear));
  Serial.print(F("  V=")); Serial.println(v, 3);

  // 5) LCD. Watchdog is already on (4 s: lcd.init() alone blocks ~1.1 s), so
  //    a stuck I2C bus resets instead of hanging; the boot after that skips
  //    the LCD and the gearbox stays controllable.
  wdt_enable(WDTO_4S);
  lcdOk = (g_lcdBusy != LCD_BUSY_MAGIC);
  if (lcdOk){
    g_lcdBusy = LCD_BUSY_MAGIC;
    Wire.begin();
#if defined(WIRE_HAS_TIMEOUT)
    Wire.setWireTimeout(3000, true);
#endif
    lcd.init();
    lcd.backlight();
    char l0[17], l1[17];
    snprintf(l0, sizeof l0, "Start: %s", resetName(rf));
    // the two counts that mean trouble; all four are on Serial and in diag.ino
    snprintf(l1, sizeof l1, "WD:%u BO:%u", EEPROM.read(EE_CNT_WDT), EEPROM.read(EE_CNT_BOR));
    lcdLine(0, l0);
    lcdLine(1, l1);
    g_lcdBusy = 0;
    wdt_reset();
    delay(1500);
  } else Serial.println(F("LCD skipped: it hung last time (power-cycle to retry)"));

  wdt_enable(WDTO_1S);
}

/**************** LOOP ****************/
void loop(){
  wdt_reset();
  unsigned long now = millis();
  float v = readPosV();

  inBrake.update(now);
  Gear req = readButtons(now);

  static unsigned long posOkAt = 0;
  bool posOk = posInRange(v);
  if (posOk) posOkAt = now;
  else if (state != ST_FAULT && now - posOkAt >= SENSOR_FAULT_MS) enterFault("SENSOR FAULT");

  if (!posOk && state != ST_FAULT) motorBrake();   // unconfirmed glitch: don't act on a bad reading
  else switch (state){
    case ST_IDLE:
      if (req != G_NONE) requestShift(req, v);
      else               holdMonitor(v, now);
      break;
    case ST_MOVING:
      runMove(v, now);                 // presses while travelling are ignored
      break;
    case ST_FAULT:
      if (!faultJog(req, now)) faultRecover(v, now);   // faultRecover brakes the motor
      break;
  }

  // ~50 ms of blocking I2C: never while the motor is being driven
  if (state != ST_MOVING && !motorOn && now - lastLcd >= LCD_PERIOD_MS){
    lastLcd = now;
    updateLcd(v, now);
  }
}
