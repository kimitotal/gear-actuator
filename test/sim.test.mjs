// Logic test for final_safe.ino: runs the REAL compiled sketch in an ATmega328P
// simulator (avr8js) against a simple actuator model, and checks behaviour
// through what the hardware would show: motor pins, actuator position, LCD text.
//
//   cd test && npm install && npm test
//
// Needs arduino-cli (on PATH or in ARDUINO_CLI) to build the hex.
//
// The actuator model is crude on purpose (constant speed per PWM, stops dead,
// no inertia). It checks the state machine, NOT motor tuning, noise or heat.
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

execFileSync(process.env.ARDUINO_CLI || 'arduino-cli',
  ['compile', '-b', 'arduino:avr:uno', '--output-dir', join(HERE, 'build'), join(HERE, '..', 'final_safe')],
  { stdio: 'ignore' });
const program = loadHex(readFileSync(join(HERE, 'build', 'final_safe.ino.hex'), 'ascii'));

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
  constructor(startV, { mcusr = PORF, lcdPresent = true } = {}) {
    this.v = startV;
    this.sensor = v => v;        // override to fake a broken / glitching sensor wire
    this.blocked = false;        // actuator mechanically jammed
    this.reversed = false;       // motor wired backwards
    this.drivenMs = 0;           // how long the motor has been powered
    this.serial = '';
    this.lcd = new Lcd();
    this.lcdLog = [];

    const cpu = this.cpu = new CPU(program);
    cpu.data[0x54] = mcusr;
    this.portB = new AVRIOPort(cpu, portBConfig);
    new AVRIOPort(cpu, portCConfig);
    this.portD = new AVRIOPort(cpu, portDConfig);
    new AVRTimer(cpu, timer0Config); new AVRTimer(cpu, timer1Config); new AVRTimer(cpu, timer2Config);
    this.adc = new AVRADC(cpu, adcConfig);
    new AVRUSART(cpu, usart0Config, F_CPU).onByteTransmit = b => { this.serial += String.fromCharCode(b); };
    new AVREEPROM(cpu, new EEPROMMemoryBackend(1024));
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

  stepMs() {
    let drive = this.drive();
    if (drive) this.drivenMs++;
    if (this.reversed) drive = -drive;
    const duty = Math.abs(drive);
    const speed = duty <= 0.35 ? 0 : (duty - 0.35) / 0.65 * 2.2;  // V/s; PWM 220 ~1.7, PWM 140 ~0.7
    if (!this.blocked) this.v = Math.min(3.2, Math.max(0.3, this.v + Math.sign(drive) * speed / 1000));
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
  assert.equal(s.v, vAtFault);
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
