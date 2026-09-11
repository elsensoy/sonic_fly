# Hardware Bring-Up

From bare transmitter PCB to a drone that flies a choreography. One moving
variable at a time — the policy is frozen (`policy-v1`), so every surprise from
here on is a hardware problem.

Interception layer: the transmitter's own **control contacts**. The TX MCU keeps
doing binding, channel hopping, packet framing and the failsafe heartbeat; we
only close its buttons. RF / SPI-bus RE is plan B, only if the input side turns
out to be an undriveable scan matrix (see H1).

**Props off for everything through H4.** Flight (H5) only in an open area with a
hardware kill that doesn't route through firmware.

---

## H0 — interface board

Goal: a reversible way to close each candidate contact from an Arduino pin.

- [ ] Identify `B+` / `B-` / `ANT` on the TX PCB (`docs/transmitter_mapping.md`).
- [ ] One channel per candidate contact: `GPIO → gate resistor → switch element → contact`,
      switch element source/common → `B-`. Everything brought out to a header.
- [ ] Switch element choice (contacts carry ~no current):
      - **analog switch** (74HC4066 / DG-series) — can be *floated*, needed if the
        buttons are matrixed; also lets you inject stick voltages later. Preferred.
      - **optocoupler / reed relay** — fully isolated, dead simple, ~1 ms is fine.
      - IRF510 (on hand) works for a direct-to-ground contact but isn't
        logic-level and ties the grounds hard — fine for a first latching-button
        test, reconsider before the sticks.
- [ ] 10 kΩ gate pulldown per channel (off at boot / floating GPIO).
- [ ] `Arduino GND → TX B-`, one wire. No positive rail between the boards.
- [ ] Solder to test points / tack onto pads. **Do not cut traces.**

## H1 — characterise (TX powered, no Arduino yet)

Goal: fill in the `docs/transmitter_mapping.md` table. Multimeter minimum; a
cheap logic analyzer pays for itself here.

For each control:

- [ ] **Off**: resistance from each contact to `B-`. ~0 Ω = ground side; its
      partner is the signal.
- [ ] **Powered idle**: DC voltage on the signal contact, not pressed.
- [ ] **Powered pressed**: DC voltage while held. (active-low to GND is typical.)
- [ ] **Pull-up**: idle voltage + a known load → the pull-up value, roughly.
- [ ] **Press type**: momentary / hold-to-repeat / latch (power button: short =
      wake, long = off).
- [ ] **Matrixed?** Count buttons vs MCU GPIO. Scope two "row" contacts while
      pressing one button — if idle contacts twitch in a scan pattern, it's a
      matrix and a hard short to GND will inject phantom presses. → analog
      switches only, or plan B.
- [ ] Part numbers: TX MCU, RF chip (often an 8051 clone + BK2423 / XN297).

Sticks — the crux:

- [ ] Pot or switches? 3 terminals with a wiper voltage that sweeps continuously
      = pot. 4 contacts to ground = switches (treat as buttons).
- [ ] If pot: idle voltage (≈ Vcc/2?), and both full-deflection voltages per
      axis. Find the thresholds the TX firmware acts on by sweeping slowly and
      watching the drone (props off) — you only need 3 levels (down / centre /
      up), the choreography is discrete.

## H2 — manual actuation (`firmware/bench_probe`)

Goal: confirm each Arduino channel reproduces a human press.

```bash
arduino-cli compile -b arduino:avr:nano firmware/bench_probe
arduino-cli upload  -b arduino:avr:nano -p /dev/ttyUSB0 firmware/bench_probe
arduino-cli monitor -p /dev/ttyUSB0 -c baudrate=115200
```

Edit `CANDIDATES[]` to your wired pins. Then, one contact at a time:

- [ ] `p <pin> 120` — drone reacts exactly like pressing that button by hand
      (LED codes, props-off motor twitch).
- [ ] No *other* behaviour changes (phantom-press / matrix check, in practice).
- [ ] Latching buttons: `h <pin>` … `r <pin>`, measure the hold time that wakes
      vs powers off.
- [ ] Record confirmed `pin ↔ contact ↔ behaviour` in `transmitter_mapping.md`.

## H3 — arming gesture

Goal: reproduce the flight-mode sequence (observed: left stick up → down →
right, then takeoff).

- [ ] `seq 6 150 5 150 7 150 2 150` (substitute your pins) — drone arms.
- [ ] Tune step duration / gap. Note the exact sequence for the firmware macro.

## H4 — graduate to `transmitter_controller.ino`

Goal: host-timed choreography, props off.

- [ ] Fill `CHANNELS[]` with the real pins, `conflictsWith`, and *measured*
      `cooldownMs`. Drop channels you don't have.
- [ ] Implement the arming sequence in `handleArm()` (replaces the gate flag).
- [ ] Host handshake: `HELLO` / `PING`-`PONG` clock sync, `ARM`.
- [ ] Run a canned choreography (`analysis/choreo_compare` primitive stream, or
      the visualizer with `--port`). Compare requested `at` vs `FIRE`
      timestamps — this is the fire-at-T latency number for real.
- [ ] Contacts on a scope during a run: requested edge vs actual contact edge.

## H5 — flight

- [ ] Props on, open controlled area, hardware kill in reach.
- [ ] Hover hold first (no primitives), then single primitives, then a full
      track.
- [ ] Record requested vs emitted vs observed for the demo.

---

## Firmware ladder

| Sketch | Phase | What it is |
| --- | --- | --- |
| `firmware/bench_probe` | H2–H3 | manual line-driven contact prodder, auto-release, no scheduler |
| `firmware/transmitter_controller` | H4–H5 | fire-at-T: host schedules `SCHED <id> <at> <act> <dur>`, 1 kHz ISR fires edges, full safety gate (`docs/fire_at_t_protocol.md`) |

`acoustic_receiver` is an older standalone prototype (on-board Goertzel tone
detection → 2 motors) — not part of this path.
