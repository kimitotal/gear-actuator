# Gear actuator (P/R/N/D) – Arduino

Motorised gear selector: an L298N drives the actuator, an analog position
sensor on A0 closes the loop.

## Files

| File | Purpose |
|---|---|
| `kalibracija/kalibracija.ino` | Jog the actuator with D/N buttons and read the voltage for each gear |
| `final_safe/final_safe.ino` | Hardened version – use this one |
| `diag/diag.ino` | Bench / parked diagnostics (live inputs, noise, Vcc, reset log, motor tests) |

## Known problem with the original

The original sketch (`final/final.ino`) was removed so it cannot be flashed by mistake; it
is still in the git history (commit `454ae2a`).

While driving, the gearbox got hot and the system started changing gear on its own.
Likely causes: Arduino resets (heat / supply brownout) triggering the auto-park at
boot, or noise on the button wires being read as presses.

## What final_safe.ino changes

- No shift at power-up – adopts the gear the actuator is already in (position hold may re-centre it inside that gear)
- Brake needed to select P or R, leave P, leave unknown position (no vehicle-speed input)
- Real debounce, two-button press ignored
- Watchdog, reset cause logged to EEPROM
- Move timeout / stall / wrong-direction detection, gear re-detected after failures
- Sensor valid window 0.20–4.80 V
- Position hold with limited corrections
- Motor rests in brake
- Limp mode: in a fault, hold P or D + brake to creep the actuator blind toward that end (max 3 s per press)
- Pressing the current gear again re-seats it if it sits off-centre
- Event log in EEPROM: the last 16 faults / odd events (with gear and minutes since start) survive power-off; read them with `diag.ino`, command `l`
- A WATCHDOG or BROWNOUT start stays on the LCD until the first button press

## New wiring for final_safe.ino

- **D10** – brake, LOW = pressed (via optocoupler from brake-light switch, never 12 V directly)

Fails safe: a broken wire blocks P/R and leaving P.

## Hardware to-do

- Automotive 12→5 V buck converter + TVS diode (e.g. SMBJ24A) instead of the onboard regulator
- 10 kΩ pull-down on ENA
- External 2.2–4.7 kΩ pull-ups + 100 nF on button inputs
- Nano with "Old Bootloader": flash the new bootloader (watchdog reboot loop)
- Keep electronics away from gearbox / exhaust heat

## Bench tests (use diag.ino, serial 115200, send `h`)

1. `z`, then engine on, fans/lights/wipers on, don't touch buttons – spikes must stay 0, Vcc min ≳ 4.7 V
2. `f` / `b` – motor direction OK (forward = voltage up)
3. `w` – board reboots once and shows Rst:WATCHDOG
4. `c` – calibration values accepted

Then with final_safe.ino: pull sensor wire → `SENSOR FAULT`; reset in D → stays D;
P or R without brake → `PRESS BRAKE`; push actuator off position → pulls back.

## Build

Board: Arduino Uno / Nano (ATmega328P). Library: LiquidCrystal_I2C (0x27, try 0x3F).
Both new sketches compile for the Uno with arduino-cli 1.5.1 / arduino:avr 1.8.8, no
warnings from the sketches (final_safe ≈ 14 KB, diag ≈ 16.5 KB flash).
Not yet tested on hardware.

## Logic test (no hardware)

`test/sim.test.mjs` runs the compiled `final_safe.ino` in an ATmega328P simulator
(avr8js) against a simple actuator model and checks 28 scenarios: interlocks, button
noise, sensor glitch / fault / recovery, stall, wrong direction, hold, limp mode, reset
cause. Needs Node.js and `arduino-cli`.

```
cd test
npm install
npm test
```

The actuator model is crude (constant speed, no inertia, no electrical noise), so this
checks the state machine only – it does not replace the bench tests above.
