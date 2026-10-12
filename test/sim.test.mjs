// Logic test for final_safe.ino: runs the REAL compiled sketch in an ATmega328P
// simulator (avr8js) against a simple actuator model, and checks behaviour
// through what the hardware would show: motor pins, actuator position, LCD text.
//
//   cd test && npm install && npm test
//
// Needs arduino-cli (on PATH or in ARDUINO_CLI) to build the hex.
//
// The actuator model is simple: speed per PWM with inertia (spin-up, brake and
// coast time constants, which are guesses until measured). It checks the state
// machine and how much overshoot it tolerates, NOT real motor tuning, noise or heat.
import test from 'node:test';
import assert from 'node:assert/strict';
import { execFileSync } from 'node:child_process';
import { readFileSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { fileURLToPath } from 'node:url';
import {
  CPU, avrInstruction, AVRIOPort, portBConfig, portCConfig, portDConfig,
  AVRTimer, timer0Config, timer1Config, timer2Config, AVRADC, adcConfig,
  AVRUSART, usart0Config, AVRTWI, twiConfig, AVREEPROM, EEPROMMemoryBackend,
  AVRClock, AVRWatchdog, watchdogConfig,
} from 'avr8js';

const HERE = dirname(fileURLToPath(import.meta.url));
const F_CPU = 16e6, CYCLES_PER_MS = F_CPU / 1000;
const P = 0.6, R = 1.2, N = 2.0, D = 2.8;          // TARGET_*_V in the sketch
const BTN = { P: 4, R: 5, N: 6, D: 7 };            // port D pins
const PORF = 1, EXTRF = 2, WDRF = 8;

function build(sketch) {
  const out = join(HERE, 'build', sketch);
  execFileSync(process.env.ARDUINO_CLI || 'arduino-cli',
    ['compile', '-b', 'arduino:avr:uno', '--output-dir', out, join(HERE, '..', sketch)], { stdio: 'ignore' });
  return loadHex(readFileSync(join(out, sketch + '.ino.hex'), 'ascii'));
}
const program = build('final_safe');
const diagProgram = build('diag');

function loadHex(text) {
  const bytes = new Uint8Array(0x8000);
  for (const line of text.split('\n')) {
    if (line[0] !== ':' || line.substr(7, 2) !== '00') continue;
    const n = parseInt(line.substr(1, 2), 16), addr = parseInt(line.substr(3, 4), 16);
    for (let i = 0; i < n; i++) bytes[addr + i] = parseInt(line.substr(9 + i * 2, 2), 16);
  }
  return new Uint16Array(bytes.buffer);
}

// HD44780 behind a PCF8574 backpack, as LiquidCrystal_I2C drives it.
class Lcd {
  rows = [Array(16).fill(' '), Array(16).fill(' ')];
  addr = 0; prev = 0; hi = null; last4 = [];
  write(b) {
    if ((this.prev & 4) && !(b & 4)) this.nibble(this.prev >> 4, this.prev & 1);   // EN falling edge
    this.prev = b;
  }
  nibble(n, rs) {
    this.last4 = [...this.last4, rs ? -1 : n].slice(-4);
    if (this.last4.join() === '3,3,3,2') { this.hi = null; return; }               // 4-bit init = resync
    if (this.hi === null) { this.hi = n; return; }
    const v = (this.hi << 4) | n; this.hi = null;
    if (rs) {
      const row = this.addr >= 0x40 ? 1 : 0, col = this.addr & 0x3f;
      if (col < 16) this.rows[row][col] = String.fromCharCode(v);
      this.addr++;
    } else if (v & 0x80) this.addr = v & 0x7f;
    else if (v === 1) { this.rows.forEach(r => r.fill(' ')); this.addr = 0; }
  }
  line(r) { return this.rows[r].join('').trimEnd(); }
}

class Sim {
  // startV: where the actuator sits at power-up. mcusr: reset flags the chip reports.
  constructor(startV, { mcusr = PORF, lcdPresent = true, eeprom = new EEPROMMemoryBackend(1024), hex = program, motor = {} } = {}) {
    this.v = startV;
    this.sensor = v => v;        // override to fake a broken / glitching sensor wire
    this.blocked = false;        // actuator mechanically jammed
    this.reversed = false;       // motor wired backwards
    this.drivenMs = 0;           // how long the motor has been powered
    this.reversals = 0;          // times the motor changed direction (hunting shows up here)
    this.lastDir = 0;
    this.vel = 0;                // actuator speed, V/s
    // Motor time constants in ms. Guesses for a geared actuator until measured on the real one.
    this.motor = { spinUp: 40, brake: 15, coast: 80, ...motor };
    this.serial = '';
    this.lcd = new Lcd();
    this.lcdLog = [];

    const cpu = this.cpu = new CPU(hex);
    cpu.data[0x54] = mcusr;
    this.portB = new AVRIOPort(cpu, portBConfig);
    new AVRIOPort(cpu, portCConfig);
    this.portD = new AVRIOPort(cpu, portDConfig);
    new AVRTimer(cpu, timer0Config); new AVRTimer(cpu, timer1Config); new AVRTimer(cpu, timer2Config);
    this.adc = new AVRADC(cpu, adcConfig);
    this.usart = new AVRUSART(cpu, usart0Config, F_CPU);
    this.usart.onByteTransmit = b => { this.serial += String.fromCharCode(b); };
    this.eeprom = eeprom;        // pass it to the next Sim to keep the EEPROM across a "power cycle"
    new AVREEPROM(cpu, eeprom);
    new AVRWatchdog(cpu, watchdogConfig, new AVRClock(cpu, F_CPU));
    const twi = new AVRTWI(cpu, twiConfig, F_CPU);
    twi.eventHandler = {
      start: () => twi.completeStart(),
      stop: () => twi.completeStop(),
      connectToSlave: addr => twi.completeConnect(lcdPresent && addr === 0x27),
      writeByte: b => { this.lcd.write(b); twi.completeWrite(true); },
      readByte: () => twi.completeRead(0xff),
    };
    for (const pin of Object.values(BTN)) this.portD.setPin(pin, true);   // released (pull-up)
    this.brake(false);
    this.adc.channelValues[0] = this.sensor(this.v);
    this.nextMs = CYCLES_PER_MS;
    this.run(3300);              // lcd.init() ~1.1 s + 1.5 s boot screen
  }

  // Signed motor drive -1..1 as the L298N would see it.
  drive() {
    const d = this.cpu.data;
    const pwm = d[0xb0] & 0x20;                                   // TCCR2A COM2B1: analogWrite active
    const ena = pwm ? d[0xb4] / 255 : ((d[0x2a] & d[0x2b] & 8) ? 1 : 0);
    const in1 = d[0x24] & d[0x25] & 1, in2 = d[0x24] & d[0x25] & 2;
    return in1 && !in2 ? ena : in2 && !in1 ? -ena : 0;
  }

  braked() {                                                      // ENA high, IN1 == IN2
    const d = this.cpu.data;
    const ena = (d[0xb0] & 0x20) ? d[0xb4] > 0 : !!(d[0x2a] & d[0x2b] & 8);
    return ena && !this.drive();
  }

  stepMs() {
    let drive = this.drive();
    if (drive) this.drivenMs++;
    if (drive && this.lastDir && Math.sign(drive) !== this.lastDir) this.reversals++;
    if (drive) this.lastDir = Math.sign(drive);
    if (this.reversed) drive = -drive;
    const duty = Math.abs(drive);
    // Free-running speed in V/s (PWM 220 ~1.7, PWM 140 ~0.7); below 35 % duty it does not turn.
    const target = duty <= 0.35 ? 0 : Math.sign(drive) * (duty - 0.35) / 0.65 * 2.2;
    // Inertia: speed follows with a time constant. Powered = spinUp, shorted by
    // the L298N brake = brake, pins floating / ENA low = coast (friction only).
    const tau = drive ? this.motor.spinUp : this.braked() ? this.motor.brake : this.motor.coast;
    this.vel += (target - this.vel) * Math.min(1, 1 / tau);
    if (this.blocked || (!target && Math.abs(this.vel) < 0.005)) this.vel = 0;   // friction holds it
    this.v = Math.min(3.2, Math.max(0.3, this.v + this.vel / 1000));
    if (this.v === 3.2 || this.v === 0.3) this.vel = 0;           // mechanical end stop
    this.adc.channelValues[0] = Math.min(5, Math.max(0, this.sensor(this.v)));
    const l = this.lcd.line(0) + '|' + this.lcd.line(1);
    if (l !== this.lcdLog.at(-1)) this.lcdLog.push(l);
  }

  run(ms) {
    const end = this.cpu.cycles + ms * CYCLES_PER_MS;
    while (this.cpu.cycles < end) {
      avrInstruction(this.cpu);
      this.cpu.tick();
      if (this.cpu.cycles >= this.nextMs) { this.nextMs += CYCLES_PER_MS; this.stepMs(); }
    }
  }
  runUntil(pred, maxMs) {
    for (let t = 0; t < maxMs; t += 5) { if (pred()) return true; this.run(5); }
    return pred();
  }

  // Type into the Serial monitor; returns what the sketch printed in reply.
  send(text, waitMs = 1500) {
    const from = this.serial.length;
    for (const ch of text) { this.usart.writeByte(ch.charCodeAt(0)); this.run(5); }
    this.run(waitMs);
    return this.serial.slice(from);
  }

  brake(on) { this.portB.setPin(2, !on); }                        // D10, LOW = pressed
  hold(...names) { for (const n of names) this.portD.setPin(BTN[n], false); }
  release(...names) { for (const n of names) this.portD.setPin(BTN[n], true); }
  press(name, ms = 200) { this.hold(name); this.run(ms); this.release(name); }

  // Let any move finish and the 1.5 s message expire, then return LCD line 0.
  settle() {
    this.run(300);
    this.runUntil(() => !this.drive(), 6000);
    this.run(300);                                   // LCD is not refreshed while moving
    this.runUntil(() => /^Gear: /.test(this.lcd.line(0)), 2500);
    this.run(60);                                    // let the LCD finish writing the line
    return this.lcd.line(0);
  }
  saw(msg) { return this.lcdLog.some(l => l.split('|').some(x => x.startsWith(msg))); }
}

const near = (v, target, tol = 0.035) => Math.abs(v - target) <= tol;

test('boot: adopts the gear it is in and never moves', () => {
  const s = new Sim(D);
  assert.equal(s.settle(), 'Gear: D');
  assert.equal(s.drivenMs, 0);
});

test('boot between gears: gear unknown, still no movement', () => {
  const s = new Sim(2.4);
  assert.equal(s.settle(), 'Gear: ?');
  assert.equal(s.drivenMs, 0);
});

test('D -> N needs no brake', () => {
  const s = new Sim(D);
  s.press('N');
  assert.equal(s.settle(), 'Gear: N');
  assert.ok(near(s.v, N), `v=${s.v}`);
});

test('P needs the brake', () => {
  const s = new Sim(N);
  s.press('P');
  s.settle();
  assert.ok(s.saw('PRESS BRAKE'));
  assert.equal(s.drivenMs, 0);
  s.brake(true);
  s.press('P');
  assert.equal(s.settle(), 'Gear: P');
  assert.ok(near(s.v, P), `v=${s.v}`);
});

test('leaving P needs the brake', () => {
  const s = new Sim(P);
  s.press('D');
  s.settle();
  assert.ok(s.saw('PRESS BRAKE'));
  assert.equal(s.drivenMs, 0);
  s.brake(true);
  s.press('D');
  assert.equal(s.settle(), 'Gear: D');
  assert.ok(near(s.v, D), `v=${s.v}`);
});

test('R needs the brake', () => {
  const s = new Sim(N);
  s.press('R');
  s.settle();
  assert.equal(s.drivenMs, 0);
  s.brake(true);
  s.press('R');
  assert.equal(s.settle(), 'Gear: R');
  assert.ok(near(s.v, R), `v=${s.v}`);
});

test('releasing the brake mid-shift does not abort the shift', () => {
  const s = new Sim(D);
  s.brake(true);
  s.hold('P'); s.run(200); s.release('P');
  s.runUntil(() => s.v < 2.0, 3000);
  s.brake(false);
  assert.equal(s.settle(), 'Gear: P');
  assert.ok(near(s.v, P), `v=${s.v}`);
});

test('two buttons at once are ignored', () => {
  const s = new Sim(D);
  s.brake(true);
  s.hold('N', 'P'); s.run(300); s.release('N', 'P');
  s.settle();
  assert.ok(s.saw('2 BUTTONS HELD'));
  assert.equal(s.drivenMs, 0);
});

test('a 40 ms noise pulse on a button wire is ignored', () => {
  const s = new Sim(D);
  for (let i = 0; i < 20; i++) { s.press('N', 40); s.run(40); }
  assert.equal(s.settle(), 'Gear: D');
  assert.equal(s.drivenMs, 0);
});

test('50 ms sensor glitch mid-shift: no fault, shift completes', () => {
  // several start times: the glitch edge lands differently in the ADC averaging
  for (const extraMs of [0, 0.4, 0.9, 1.3, 1.8, 2.2]) {
    const s = new Sim(D);
    s.press('N', 150);
    s.runUntil(() => s.v < 2.6, 2000);
    s.run(extraMs);
    s.sensor = () => 0; s.run(50); s.sensor = v => v;
    assert.equal(s.settle(), 'Gear: N', `glitch +${extraMs} ms: ${s.lcdLog.slice(-3)}`);
    assert.ok(!s.saw('SENSOR FAULT'));
    assert.ok(near(s.v, N), `v=${s.v}`);
  }
});

test('sensor wire off for 300 ms mid-shift: fault, motor off, then recovers', () => {
  const s = new Sim(D);
  s.press('N', 150);
  s.runUntil(() => s.v < 2.6, 2000);
  s.sensor = () => 0; s.run(150);
  const vAtFault = s.v, drivenAtFault = s.drivenMs;
  s.run(150);
  assert.ok(near(s.v, vAtFault, 0.002), 'actuator kept moving after the fault');
  assert.equal(s.drivenMs, drivenAtFault);
  s.run(300);
  assert.equal(s.lcd.line(1), 'SENSOR FAULT');
  s.sensor = v => v;
  s.run(1300);
  assert.ok(s.saw('SENSOR OK'));
  assert.equal(s.settle(), 'Gear: ?');
  assert.equal(s.drivenMs, drivenAtFault);
});

test('jammed actuator with a noisy sensor is given up on quickly (MOTOR BLOCKED)', () => {
  const s = new Sim(D);
  s.press('N', 150);
  s.runUntil(() => s.v < 2.5, 2000);
  s.blocked = true;
  let t = 0;
  s.sensor = v => v + (Math.floor(t++ / 20) % 2 ? 0.025 : -0.025);   // 50 mV p-p wobble
  const before = s.drivenMs;
  s.run(3000);
  assert.equal(s.drive(), 0);
  assert.ok(s.drivenMs - before < 1500, `motor powered ${s.drivenMs - before} ms while jammed`);
  assert.ok(s.saw('MOTOR BLOCKED'));
});

test('motor wired backwards is caught (MOTOR REVERSED?)', () => {
  const s = new Sim(N);
  s.reversed = true;
  s.press('D');
  s.settle();
  assert.ok(s.saw('MOTOR REVERSED?'));
  assert.ok(s.v > N - 0.3, `ran away to v=${s.v}`);
});

test('hold: after a shift, a pushed actuator is pulled back', () => {
  const s = new Sim(D);
  s.press('N');
  assert.equal(s.settle(), 'Gear: N');
  s.v += 0.2;
  s.run(1500);
  assert.ok(near(s.v, N), `v=${s.v}`);
  assert.equal(s.settle(), 'Gear: N');
});


test('no movement on its own: 10 s in gear with sensor noise', () => {
  const s = new Sim(D);
  s.press('N');
  s.settle();
  const before = s.drivenMs;
  let t = 0;
  s.sensor = v => v + (Math.floor(t++ / 7) % 2 ? 0.03 : -0.03);
  s.run(10000);
  assert.equal(s.drivenMs, before);
  assert.equal(s.settle(), 'Gear: N');
});


test('limp mode: dead sensor, hold P + brake creeps toward P, max 3 s', () => {
  const s = new Sim(D);
  s.sensor = () => 0;
  s.run(600);
  assert.equal(s.lcd.line(1), 'SENSOR FAULT');

  s.press('P', 500);                         // no brake
  s.run(200);
  assert.equal(s.drivenMs, 0);
  assert.ok(s.saw('PRESS BRAKE'));

  s.brake(true);
  s.hold('P'); s.run(1000);
  assert.ok(s.v < D - 0.3, `did not creep toward P, v=${s.v}`);
  s.release('P'); s.run(50);
  assert.equal(s.drive(), 0);

  const before = s.drivenMs;
  s.hold('D'); s.run(5000); s.release('D');
  const ms = s.drivenMs - before;
  assert.ok(ms > 2500 && ms < 3100, `jog ran ${ms} ms`);
  assert.equal(s.drive(), 0);
});

test('limp mode: a button already stuck down does not start a jog', () => {
  const s = new Sim(D);
  s.hold('P');
  s.run(500);
  s.sensor = () => 0;
  s.brake(true);
  s.run(2000);
  assert.equal(s.drivenMs, 0);
});

test('shifting works with the LCD unplugged', () => {
  const s = new Sim(D, { lcdPresent: false });
  s.press('N');
  s.run(3000);
  assert.ok(near(s.v, N), `v=${s.v}`);
});

test('reset cause: reset button via Optiboot (EXTRF+WDRF) is not a watchdog', () => {
  assert.match(new Sim(D, { mcusr: EXTRF | WDRF }).serial, /Reset cause: RESET BTN/);
  assert.match(new Sim(D, { mcusr: WDRF }).serial, /Reset cause: WATCHDOG/);
  assert.match(new Sim(D, { mcusr: PORF }).serial, /Reset cause: POWER ON/);
});

test('LCD: boot screen, voltage line, brake indicator and shift arrow', () => {
  const s = new Sim(D);
  assert.ok(s.lcdLog.includes('Gear: D|Start: POWER ON'), s.lcdLog.join('\n'));
  s.settle();
  assert.equal(s.lcd.line(1), '2.80V');
  s.brake(true); s.run(400);
  assert.equal(s.lcd.line(1), '2.80V BRAKE');
  s.brake(false); s.press('P'); s.run(400);          // blocked: message on line 1, gear stays
  assert.ok(s.lcdLog.includes('Gear: D|PRESS BRAKE'));
  s.run(1500);
  s.brake(false);
  s.hold('N'); s.runUntil(() => s.v < 2.6, 2000); s.release('N');
  assert.equal(s.lcd.line(0), 'D -> N');
  assert.equal(s.settle(), 'Gear: N');
});

test('hold is on after boot: a pushed actuator is pulled back', () => {
  const s = new Sim(D);
  assert.equal(s.settle(), 'Gear: D');
  assert.equal(s.drivenMs, 0);
  s.v -= 0.3;
  s.run(1500);
  assert.ok(near(s.v, D), `v=${s.v}`);
  assert.equal(s.settle(), 'Gear: D');
});

test('boot slightly off-centre: hold re-centres it within the gear', () => {
  const s = new Sim(D + 0.13);
  s.run(1000);
  assert.ok(near(s.v, D), `v=${s.v}`);
  assert.equal(s.settle(), 'Gear: D');
});

// After a sensor fault has cleared, hold stays off until the driver shifts.
function afterSensorFault() {
  const s = new Sim(D);
  s.sensor = () => 0; s.run(300); s.sensor = v => v;
  s.run(1500);
  assert.equal(s.settle(), 'Gear: D');
  return s;
}

test('hold suspended: a pushed actuator is NOT moved, gear follows reality', () => {
  const s = afterSensorFault();
  s.v -= 0.3;
  s.run(1000);
  assert.ok(s.saw('OUT OF GEAR'));
  assert.equal(s.settle(), 'Gear: ?');
  assert.equal(s.drivenMs, 0);
});

test('hold suspended: pressing the current gear re-seats an off-centre actuator', () => {
  const s = afterSensorFault();
  s.v += 0.13;
  s.run(1000);
  assert.equal(s.drivenMs, 0);
  s.press('D');
  assert.equal(s.settle(), 'Gear: D');
  assert.ok(near(s.v, D), `v=${s.v}`);
});

test('LCD: a WATCHDOG / BROWNOUT start stays on screen until a button is pressed', () => {
  const s = new Sim(D, { mcusr: WDRF });
  s.settle();
  s.run(5000);
  assert.equal(s.lcd.line(0), 'Gear: D');
  assert.equal(s.lcd.line(1), 'Start: WATCHDOG');
  s.press('N');
  assert.equal(s.settle(), 'Gear: N');
  assert.ok(s.lcd.line(1).endsWith('V'), s.lcd.line(1));   // back to the voltage

  const b = new Sim(D, { mcusr: 4 });          // BORF
  b.settle(); b.run(2000);
  assert.equal(b.lcd.line(1), 'Start: BROWNOUT');

  const ok = new Sim(D);                       // normal power-on: nothing sticks
  ok.settle(); ok.run(2000);
  assert.equal(ok.lcd.line(1), '2.80V');
});

// Decode the EEPROM event log the way diag.ino prints it: "NAME gear".
const EV = ['', 'START', 'SENSOR FAULT', 'BAD CALIBRATION', 'MOTOR BLOCKED', 'SHIFT TOO SLOW',
  'MOTOR REVERSED?', 'OUT OF GEAR', 'CANT HOLD GEAR', '2 BUTTONS HELD'];
function events(s) {
  const m = s.eeprom.memory, idx = m[16] % 16, out = [];
  for (let k = 0; k < 16; k++) {
    const a = 17 + 2 * ((idx + k) % 16), code = m[a] & 15;
    if (code) out.push(`${EV[code]} ${'?PRND'[m[a] >> 4]}`);
  }
  return out;
}

test('event log: a blocked shift is stored and survives a power cycle', () => {
  const s = new Sim(D);
  s.press('N', 150);
  s.runUntil(() => s.v < 2.5, 2000);
  s.blocked = true;
  s.run(2000);
  assert.deepEqual(events(s), ['START D', 'MOTOR BLOCKED N']);

  const next = new Sim(D, { eeprom: s.eeprom });
  assert.deepEqual(events(next), ['START D', 'MOTOR BLOCKED N', 'START D']);
});

test('event log: a flapping sensor fault is stored once, not on every flap', () => {
  const s = new Sim(D);
  for (let i = 0; i < 3; i++) {
    s.sensor = () => 0; s.run(300);
    s.sensor = v => v; s.run(1500);
  }
  assert.deepEqual(events(s), ['START D', 'SENSOR FAULT D']);
});

test('event log: two buttons and falling out of gear are stored with the gear', () => {
  const s = new Sim(N);
  s.hold('P', 'D'); s.run(300); s.release('P', 'D');
  s.sensor = () => 0; s.run(300); s.sensor = v => v; s.run(1500);   // hold now suspended
  s.v += 0.4;
  s.run(1000);
  assert.deepEqual(events(s), ['START N', '2 BUTTONS HELD N', 'SENSOR FAULT N', 'OUT OF GEAR N']);
});

test('calibration: final_safe.ino publishes it to EEPROM and diag.ino reads it from there', () => {
  const fresh = new Sim(D, { hex: diagProgram });                 // diag before final_safe ever ran
  assert.match(fresh.send('c'), /No calibration stored/);

  const s = new Sim(D);
  const cal = new Float32Array(s.eeprom.memory.slice(49, 49 + 32).buffer);
  assert.deepEqual([...cal].map(x => +x.toFixed(2)), [5, 0.2, 4.8, P, R, N, D, 0.15]);

  const diag = new Sim(D, { hex: diagProgram, eeprom: s.eeprom });
  const out = diag.send('c');
  for (const line of ['P = 0.600', 'R = 1.200', 'N = 2.000', 'D = 2.800', 'OK, final_safe.ino will accept these'])
    assert.ok(out.includes(line), out);
});

test('diag.ino prints the event log that final_safe.ino wrote', () => {
  const s = new Sim(D);
  s.press('N', 150);
  s.runUntil(() => s.v < 2.5, 2000);
  s.blocked = true;
  s.run(2000);

  const diag = new Sim(D, { hex: diagProgram, eeprom: s.eeprom });
  const out = diag.send('l');
  assert.ok(out.includes('START  gear D  (POWER ON)'), out);
  assert.ok(out.includes('MOTOR BLOCKED  gear N  +0 min'), out);

  assert.match(diag.send('x'), /logs cleared/);
  assert.doesNotMatch(diag.send('l'), /MOTOR BLOCKED/);
  assert.match(diag.send('c'), /OK, final_safe/);                  // clearing keeps the calibration
});

// Inertia: the actuator coasts on after the motor is braked, so it can overshoot
// the target. These check how much of that the controller copes with.
function shiftAll(motor, tol = 0.035) {
  const s = new Sim(P, { motor });
  s.brake(true);
  let worst = 0;
  for (const [name, target] of [['D', D], ['R', R], ['N', N], ['P', P], ['N', N], ['D', D]]) {
    const before = s.reversals;
    s.press(name);
    const line = s.settle();
    if (line !== 'Gear: ' + name || !near(s.v, target, tol)) return { ok: false, why: `${name}: ${line} at ${s.v.toFixed(3)} V` };
    worst = Math.max(worst, s.reversals - before);
  }
  return { ok: true, worst };
}

test('inertia: every shift lands without hunting, with the default motor', () => {
  const r = shiftAll({});
  assert.ok(r.ok, r.why);
  assert.ok(r.worst <= 1, `motor reversed ${r.worst} times in one shift`);
});

test('inertia: a slow-stopping actuator (brake time constant up to 60 ms) still lands', () => {
  for (const brake of [30, 60]) {
    const r = shiftAll({ brake });
    assert.ok(r.ok, `brake ${brake} ms: ${r.why}`);
  }
});

// The sketch calls a shift done after 120 ms inside +-0.03 V, moving or not. An
// actuator that coasts for a long time is therefore still rolling at that
// moment and comes to rest a little off-centre: right gear, inside the 0.10 V
// hold tolerance, but not within 0.03 V.
test('inertia: a very slow-stopping actuator (100-500 ms) ends in the right gear, slightly off-centre', () => {
  for (const brake of [100, 200, 500]) {
    const r = shiftAll({ brake }, 0.10);
    assert.ok(r.ok, `brake ${brake} ms: ${r.why}`);
    assert.ok(r.worst <= 3, `brake ${brake} ms: motor reversed ${r.worst} times in one shift`);
  }
});

test('Serial: one line per shift, message and fault', () => {
  const s = new Sim(D);
  s.press('N');
  s.settle();
  assert.match(s.serial, /\[\d+\.\ds\] D -> N ok \d\.\d\ds 2\.0\dV/);

  s.press('P');                                 // no brake
  s.settle();
  assert.match(s.serial, /\] PRESS BRAKE 2\.0\dV/);

  s.v += 0.2;                                   // pushed off: hold pulls it back
  s.run(1500);
  assert.match(s.serial, /\] hold N ok /);

  s.sensor = () => 0; s.run(300);
  assert.match(s.serial, /\] SENSOR FAULT 0\.00V/);

  const blocked = new Sim(D);
  blocked.press('N', 150);
  blocked.runUntil(() => blocked.v < 2.5, 2000);
  blocked.blocked = true;
  blocked.run(2000);
  assert.match(blocked.serial, /\] MOTOR BLOCKED 2\.\d\dV/);
  if (process.env.SHOW_SERIAL) console.log(s.serial + blocked.serial);
});

test('diag.ino calibration mode: buttons jog only while it is on, stored voltages are printed', () => {
  const s = new Sim(N, { hex: diagProgram });
  s.hold('D'); s.run(500); s.release('D');
  assert.equal(s.drivenMs, 0, 'buttons moved the motor outside calibration mode');
  assert.ok(s.send('3').includes('Send k first'));

  assert.ok(s.send('k').includes('CALIBRATION MODE ON'));
  s.hold('D'); s.run(1000); s.release('D'); s.run(200);
  assert.ok(s.v > N + 0.2, `D did not move it forward, v=${s.v}`);
  assert.equal(s.drive(), 0);
  const atD = s.v;
  const stored = s.send('4').match(/D = (\d\.\d+) V stored/);
  assert.ok(stored && near(+stored[1], atD, 0.01), `stored ${stored} at v=${atD}`);   // ADC rounds to ~5 mV

  s.hold('N'); s.run(1000); s.release('N'); s.run(200);
  assert.ok(s.v < atD - 0.2, `N did not move it back, v=${s.v}`);
  s.send('3');

  s.hold('N', 'D'); s.run(500); s.release('N', 'D');       // both at once: no movement
  const out = s.send('k');
  assert.ok(out.includes('// P not stored'), out);
  const pasted = out.match(/const float TARGET_D_V = (\d\.\d\d)f;/);
  assert.ok(pasted && near(+pasted[1], atD, 0.015), out);
  assert.ok(out.includes('const float TARGET_N_V = '), out);

  s.hold('D'); s.run(500); s.release('D');                 // mode is off again
  assert.equal(s.drive(), 0);
});

test('diag.ino calibration mode: jogging stops before the sensor limit', () => {
  const s = new Sim(P, { hex: diagProgram });
  s.send('k');
  s.hold('N'); s.run(4000);                                // backward, towards 0 V
  assert.equal(s.drive(), 0);
  assert.ok(s.v > 0.2 + 0.2, `ran to v=${s.v}`);           // POS_MIN_V_OK is 0.20 V
});

test('Serial: both sketches say when they were built, as their first line', () => {
  assert.match(new Sim(D).serial, /^final_safe\.ino built \w{3} [ \d]\d \d{4} \d\d:\d\d:\d\d\r\n/);
  assert.match(new Sim(D, { hex: diagProgram }).serial, /^diag\.ino built \w{3} [ \d]\d \d{4} /);
});
