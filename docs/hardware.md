# Hardware

Step-by-step bring-up plan: [`hardware_bringup.md`](hardware_bringup.md).
Pin/contact map as characterised: [`transmitter_mapping.md`](transmitter_mapping.md).

## Bill of materials

| Item                     | Detail                                             |
| ------------------------ | -------------------------------------------------- |
| Toy drone + transmitter  | ~$10 used, undocumented RF, working stabilisation  |
| Arduino Nano             | ATmega328P, CH340 USB (see bootloader note below)  |
| Switch element           | IRF510 N-MOSFET on hand — works for a direct-to-GND contact but isn't logic-level and ties the grounds hard. Prefer a **74HC4066 / DG-series analog switch** (floatable → survives a scanned button matrix; also injects stick voltages) or an optocoupler/reed relay (isolated). See `hardware_bringup.md` H0. |
| Gate pulldowns           | 10 kΩ per channel, gate → GND (keeps a MOSFET off at boot / floating GPIO) |
| Electret mic module      | on A0, output biased ~Vcc/2 (for `acoustic_receiver`) |
| Laptop                   | NVIDIA RTX 5060, runs the host pipeline            |

## Wiring — MOSFET switch channel

```
transmitter signal pad ── Drain
Arduino GPIO ──┬────────── Gate
               └─ 10 kΩ ── GND
transmitter B- ─────────── Source ──┬── Arduino GND
                                    └── (common reference only)
```

GPIO HIGH → MOSFET on → signal pulled to GND → "button pressed".
GPIO LOW  → MOSFET off → contact open → "button released".

The Arduino never powers the transmitter. Shared ground is a reference only.

## Grounding

`Arduino GND ── transmitter B-`. One wire. Do not connect any positive rail
between the two boards.

## Arduino toolchain

Setup steps (one-time) are in [`notes/installation.MD`](../notes/installation.MD).
Quick build/upload:

```bash
arduino-cli compile -b arduino:avr:nano firmware/transmitter_controller
arduino-cli upload  -b arduino:avr:nano -p /dev/ttyUSB0 firmware/transmitter_controller
```

**Bootloader gotcha:** cheap CH340 Nanos ship with one of two bootloaders. If
upload times out (`stk500_recv(): programmer is not responding`) while compile
succeeds, add `:cpu=atmega328old` to the `-b` value.

## Safety

Props off during any electrical interface work. Flight testing only in an
open controlled area with an immediate kill method.
