# Gear actuator (P/R/N/D) – Arduino

Motorised gear selector: an L298N drives the actuator, an analog position
sensor on A0 closes the loop, and relay combinations tell the ECU the gear
(P = P, R = R, N = POS2+P+R, D = POS2+POS3+P).

## Files

| File | Purpose |
|---|---|
| `kalibracija/kalibracija.ino` | Jog the actuator with D/N buttons and read the voltage for each gear |
| `final/final.ino` | Original version (known safety problems, kept for reference) |
| `final_safe/final_safe.ino` | Hardened version – use this one |
| `diag/diag.ino` | Bench / parked diagnostics (live inputs, noise, Vcc, reset log, relay and motor tests) |

## Known problem with the original

While driving, the gearbox got hot and the system started changing gear on its own.
Likely causes: Arduino resets (heat / supply brownout) triggering the auto-park at
boot, or noise on the button wires being read as presses.

## What final_safe.ino changes

- No movement at power-up – adopts the gear the actuator is already in
- P and R only when stopped; brake needed to leave P, select R, leave unknown position
- Real debounce, two-button press ignored
- Watchdog, reset cause logged to EEPROM
- Move timeout / stall / wrong-direction detection, gear re-detected after failures
- Sensor valid window 0.20–4.80 V
- Position hold with limited corrections
- Motor rests in brake

## New wiring for final_safe.ino

- **D2** – vehicle stopped, LOW = stopped (ECU output "speed < 3 km/h" or optocoupler)
- **D10** – brake, LOW = pressed (via optocoupler from brake-light switch, never 12 V directly)

Both fail safe: a broken wire blocks P/R and leaving P.

## Hardware to-do

- Automotive 12→5 V buck converter + TVS diode (e.g. SMBJ24A) instead of the onboard regulator
- 10 kΩ pull-down on ENA
- External 2.2–4.7 kΩ pull-ups + 100 nF on button inputs
- Nano with "Old Bootloader": flash the new bootloader (watchdog reboot loop)
- Keep electronics away from gearbox / exhaust heat

## Bench tests (use diag.ino, serial 115200, send `h`)

1. `z`, then engine on, fans/lights/wipers on, don't touch buttons – spikes must stay 0, Vcc min ≳ 4.7 V
2. `r` – ECU sees P, R, N, D, none in turn
3. `f` / `b` – motor direction OK (forward = voltage up)
4. `w` – board reboots once and shows Rst:WATCHDOG
5. `c` – calibration values accepted

Then with final_safe.ino: pull sensor wire → `FLT POS SENSOR`; reset in D → stays D;
R with D2 open → `NOT STOPPED`; push actuator off position → pulls back.

## Build

Board: Arduino Uno / Nano (ATmega328P). Library: LiquidCrystal_I2C (0x27, try 0x3F).
Both new sketches compile with no warnings (final_safe ≈ 16 KB, diag ≈ 19 KB flash).
Not yet tested on hardware.
