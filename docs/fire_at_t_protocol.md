# Fire-at-T Host ↔ Arduino Protocol (draft v1)

Status: **phase 2 implemented.** Supersedes the char-per-command scheme in
README > Host-to-Arduino Protocol, which survives as the "reflex" path
(`NOW`, below).

- Firmware: `firmware/transmitter_controller/transmitter_controller.ino`
- Host: `host/control/{protocol,link,clock_sync,scheduler}.py`
- Simulator + tests: `host/sim/fake_arduino.py`, `tests/` (run `pytest`)

**Done:** all messages below; duration clip, arm gate, horizon/late checks,
mutual-exclusion overlap, per-channel cooldown, concurrency cap, link watchdog
→ `SAFE`; optional checksum *enforcement* (`REQUIRE_CHECKSUM` firmware flag /
`require_checksum=` on `SerialLink`); linear drift model auto-selected once
sync probes span enough time (`ClockTracker`); `BOOT` / clock-backwards
detection with `DroneScheduler.service()` auto-resync.
**Deferred (phase 3):** wire into `motion_policy.py`, `--scheduled` mode in
`response_test.py`, latency measurement from `FIRE` timestamps, the real `ARM`
stick gesture (currently a gate flag), two-phase plan commit.

## 1. Why

The v0 protocol (`send 'U'` → `sleep` → `send 'S'`) triggers every motion edge
by a byte arriving and being processed. Beat-to-motion error is then the sum of
Python scheduling + USB microframe + `loop()` latency, resampled each beat, and
the pulse *width* is noisy because the release travels that path independently.

Fire-at-T moves timing onto the Arduino's own clock. The host computes *when*
an action should happen, sends it **early** with an absolute target timestamp,
and the Arduino schedules the edges locally. Host / USB / `loop()` jitter turns
from "adds directly to error" into "must beat the deadline with margin" — a far
weaker requirement. See [`architecture.md`](architecture.md) and README >
Temporal Alignment (`t_cmd = t_next − L`).

This is also the protocol the AI branch needs: GPU inference has high latency
variance, and learned choreography is by definition a *timed sequence* of
actions — expressible here, not in v0.

## 2. Transport

- USB CDC serial, **115200 baud, 8N1** (unchanged from v0 / telemetry).
- **Line-oriented ASCII.** One message per line, `\n`-terminated, fields
  separated by single spaces. Human-readable, greppable, works with
  `arduino-cli monitor`. Binary framing is a future optimisation (§11).
- Optional trailing checksum: `*HH`, where `HH` is the uppercase hex XOR of
  every byte before the `*` (NMEA-style). Receivers SHOULD validate when
  present; v1 accepts unchecksummed lines but logs them.
- Max line length **64 bytes** (fits the Arduino 64-byte serial RX ring).
- Unknown keyword, bad checksum, or over-length line → `ERR parse ...`,
  line discarded, no state change.

## 3. Timebase

- **Arduino clock** is `millis()` — `uint32`, ms since boot, wraps ≈ 49.7 d.
  All timestamps on the wire are in this timebase.
- All time comparisons use wrap-safe signed differences:
  `(int32_t)(now - target) >= 0`.
- Firing is checked from a **1 kHz timer ISR**, not `loop()`, so a busy main
  loop cannot delay an edge. `micros()` is an option for sub-ms edges (§11);
  v1 is millisecond-resolution, which is well below the RF + mechanical lag.
- The **host** works in its own monotonic seconds and maps to Arduino ms via a
  clock model estimated from the sync handshake (§5).

## 4. Message reference

### Host → Arduino

| Line | Meaning |
| --- | --- |
| `HELLO` | request capabilities |
| `PING <seq>` | clock-sync probe; `<seq>` is any uint16 echoed back |
| `ARM` / `DISARM` | run / exit the flight-mode stick gesture (Arduino owns the sequence) |
| `SCHED <id> <at> <act> <dur> [grp]` | schedule one action (see below) |
| `NOW <act> <dur>` | reflex path — assert immediately, release after `<dur>` ms |
| `CANCEL <id>` / `CANCEL grp <grp>` / `CANCEL all` | retract not-yet-fired events |
| `ABORT` | release every output now, clear the whole schedule, stay armed |

`SCHED` fields:

| Field | Type | Notes |
| --- | --- | --- |
| `id`   | uint16 | host-assigned, monotonic (wrap OK); used in `ACK`/`FIRE`/`CANCEL` |
| `at`   | uint32 | Arduino-ms target for the **assert** edge |
| `act`  | token  | see action table below |
| `dur`  | uint16 | ms; release edge fires at `at + dur`. `0` = latching (until `S`/`ABORT`/safety cap) |
| `grp`  | uint16 | optional plan id shared by a sequence, for one-shot `CANCEL grp` |

### Arduino → Host

| Line | Meaning |
| --- | --- |
| `BOOT <fw> <proto>` | sent once on reset — host MUST re-sync and drop its schedule model |
| `HELLO <fw> <proto> <slots> <maxpulse_ms> <horizon_ms> <tick_hz>` | capability reply (positional) |
| `PONG <seq> <millis>` | sync reply; `<millis>` sampled as late as possible before TX |
| `ACK <id> OK` | event accepted and scheduled |
| `NAK <id> <reason>` | event rejected (reasons §7) |
| `FIRE <id> <millis>` | assert edge executed, with the actual `millis()` — lets the host measure scheduling error |
| `REL <id> <millis>` | release edge executed / event retired |
| `SAFE <reason>` | entered safe-hold (§8) |
| `ERR <text>` | parse / range / protocol error, not tied to an `id` |

CSV telemetry from README > Telemetry continues unchanged on its own cadence;
these lines are interleaved with it (all distinguishable by first token).

### Action tokens

Wire tokens match v0. `dur` turns a level command into a pulse.

| Token | Action | Conflicts with |
| --- | --- | --- |
| `P` | power / wake | — |
| `T` | takeoff | — |
| `U` / `D` | throttle up / down | each other |
| `L` / `R` | yaw left / right | each other |
| `F` / `B` | pitch forward / back | each other |
| `S` | release all transient channels | — |

## 5. Clock synchronisation

SNTP-style, host-driven:

1. Host records `t0` (monotonic), sends `PING <seq>`.
2. Arduino replies `PONG <seq> <millis>` with `millis()` read just before TX.
3. Host records `t1`. Round trip `rtt = t1 − t0`; estimate
   `arduino_ms ≈ millis` at host time `t0 + rtt/2`.
4. Repeat **8×**, ~20 ms apart. Keep the sample with the **smallest `rtt`**
   (least queueing noise); its `offset = arduino_ms − host_ms`.

Two models, chosen automatically by `ClockTracker` from how much time the
retained probes span:

```
ClockModel        arduino_ms(t_host) = t_host*1000 + offset         (span < 5 s)
LinearClockModel  arduino_ms(t_host) = a + b*t_host                 (span ≥ 5 s, ≥ 3 probes)
```

The linear fit is least-squares over the lowest-RTT probes; `b − 1000` is the
oscillator drift (`.drift_ppm`). `DroneScheduler` re-syncs every **2 s**
(`maybe_resync`), keeps a **120 s** probe window, and calls it via `service()`.
1 ms `PONG` rounding limits a single burst to ~tens of ppm; drift only becomes
trustworthy after a minute or so of re-sync history.

On a `BOOT` line, or a `PONG millis` that jumps > 2 s backwards while host time
advances, `DroneScheduler._reset()` clears the tracker + event table, sets
`needs_resync`, fires `on_reset`, and `service()` re-runs the handshake.

## 6. Scheduling a beat (host side)

```
t_beat   = beat_detector.predict_next_beat()      # host monotonic seconds
L_act    = latency_model[act]                      # measured, per action (README Future Work)
at_ms    = arduino_ms(t_beat - L_act)              # via §5 clock model
send SCHED <id> <at_ms> <act> <dur> <grp>          # at least SEND_LEAD_MS before at_ms maps to now
```

- `SEND_LEAD_MS` default **50 ms**. If `at_ms` is already within `MIN_LEAD_MS`
  (**5 ms**) of the Arduino's now, the Arduino returns `NAK <id> late` and the
  host treats it as a skipped beat — never a late lurch.
- Events more than `MAX_HORIZON_MS` (**3000 ms**) ahead → `NAK <id> horizon`.
- A **sequence / choreography** is several `SCHED` lines sharing `grp`, sent
  back-to-back. v1 arms each on receipt (no two-phase commit); the host is
  responsible for sending complete plans with lead time. `CANCEL grp <grp>`
  retracts whatever hasn't fired. Two-phase commit (`HOLD`/`GO`) is a §11
  extension if partial-plan execution proves to be a problem.

## 7. Arduino scheduler

- Fixed event table, **`slots = 16`**, statically allocated — no heap.
  Slot: `{id, fire_at, release_at, ch, state}`, `state ∈ {free, armed, asserted}`.
- Timer2 1 kHz ISR: for each non-free slot, wrap-safe compare `now` against
  `fire_at` (→ assert output, record `lastFireMs[ch]`, queue `FIRE`) and
  `release_at` (→ release, queue `REL`, free slot). The ISR never touches
  `Serial`; `loop()` drains a note ring and prints.
- On `SCHED`/`NOW`, before arming, run §8 validation (`tryArm`). Table full →
  `NAK <id> full`.
- `CANCEL` frees matching armed slots (not ones already `asserted`); reports
  `REL <id>` for each.
- `ABORT`: drive all transient channels low, free all slots. `P`/`T` latched
  state is left as-is (power stays on).

`NAK` reasons: `late`, `full`, `horizon`, `range`, `cooldown`, `conflict`,
`disarmed`, `parse`. (`durclip` unused — over-long durations are clipped and
still `ACK`ed.)

## 8. Safety validation (Arduino, independent of the host)

Per README > Safety and Command State Machine. Checked before a `SCHED` is
armed:

| Check | Rule | On fail |
| --- | --- | --- |
| duration cap | `dur ≤ MAX_PULSE_MS` (**500**) | clip, still `ACK` |
| cooldown | `at` ≥ (last fire **or** latest queued fire on this channel) + `COOLDOWN_MS[act]`; skipped if neither exists | `NAK cooldown` |
| mutual exclusion | `[at, at+dur)` must not overlap an armed/asserted event on the same or the conflicting channel (table §4) | `NAK conflict` |
| concurrency | ≤ `MAX_CONCURRENT` (**3**) events with overlapping intervals | `NAK conflict` |
| arm gate | `U D L R F B` require `ARMED`; `P` `T` exempt | `NAK disarmed` |
| range | `act` is a known channel | `NAK range` |

`COOLDOWN_MS`: `P` 1000, `T` 2000, transient channels 60 (per-channel in the
`CHANNELS[]` table). These are guesses — tune against the real transmitter.

**Link watchdog:** if no valid line arrives for `LINK_TIMEOUT_MS` (**500**), the
Arduino releases all transient channels, keeps the schedule, and emits
`SAFE linkloss`. Normal operation resumes silently on the next valid line.

## 9. Wire example

```
→ HELLO
← HELLO 0.2.0 1 16 500 3000 1000
→ PING 1
← PONG 1 40232
  ... 7 more PING/PONG ...
→ ARM
← ACK 0 OK                      # arming gesture done
# host clock model locked; predicted downbeat at arduino_ms 41000, L=60ms
→ SCHED 17 40940 U 90 4
← ACK 17 OK
→ SCHED 18 41190 F 70 4         # choreography: pulse then nudge, same grp 4
← ACK 18 OK
← FIRE 17 40941                 # 1 ms scheduling error
← REL 17 41031
← FIRE 18 41190
← REL 18 41260
# music stops
→ ABORT
← REL 18 ...                    # (nothing pending) 
```

## 10. Host stack around the protocol

```
MotionCommand ─► ScheduledController.perform(cmd, at)      host/control/scheduled_controller.py
                    │  expand() -> channel ops
                    │  at - LatencyModel.get(primitive) + op.at_ms
                    ▼
                 DroneScheduler.schedule(act, t_host, dur) host/control/scheduler.py
                    │  ClockTracker: t_host -> arduino_ms
                    ▼
                 SerialLink ── SCHED ──►  Arduino  (or host/sim/fake_arduino.py)
```

- `ScheduledController` is the primitive → wire boundary. `perform(cmd, at)`
  places each channel op at `at - L`; `reflex(cmd)` fires ASAP for sub-`L`
  reactions; `hold()` / `stop()` map to `CANCEL all` / `ABORT`. A `Plan` groups
  one command's events and aggregates their status + schedule error.
- `LatencyModel` (`host/control/latency_model.py`) holds per-primitive `L`;
  `observe()` is where `analysis/latency_analysis.py` feeds measured values.
- v0 `DroneController` (`drone_controller.py`) keeps the imperative calls for
  bench work; `_pulse` maps onto `NOW`.

## 11. Working on it

```bash
# firmware — compile-check, then flash a connected Nano
arduino-cli compile -b arduino:avr:nano firmware/transmitter_controller
arduino-cli upload  -b arduino:avr:nano -p /dev/ttyUSB0 firmware/transmitter_controller

# host stack — unit + integration tests (no hardware; uses host/sim/fake_arduino.py)
.venv/bin/pytest -q

# drive the simulator by hand
python -c "
from host.sim.fake_arduino import FakeArduino
from host.control.link import SerialLink
from host.control.scheduler import DroneScheduler
fa = FakeArduino(); port = fa.start()
s = DroneScheduler(SerialLink(port)); s.link.open(settle=False)
print(s.hello()); s.sync(); s.arm()
ev = s.schedule_at('U', fa.millis()+150, dur_ms=90)
import time
for _ in range(60): s.pump(); time.sleep(0.01)
print(ev.status, ev.schedule_error_ms, 'ms')
s.link.close(); fa.stop()
"
```

`FakeArduino` mirrors the `.ino` logic in Python and talks over a pty, so the
real `pyserial` path is exercised. When hardware arrives, swap
`SerialLink(port)` for `SerialLink("/dev/ttyUSB0")` — nothing else changes.

## 12. Open questions

- **ASCII vs binary framing.** ASCII is ~2–3× the bytes; at a few `SCHED`/beat
  with lookahead it's still < 5 % of the 115200 link. Revisit only if it bites.
- **millis vs micros edges.** Is sub-ms scheduling worth the wrap/overflow care?
  Probably not until latency measurement says the Arduino edge is a real
  contributor.
- **Drift model.** Both are implemented; `ClockTracker` picks by probe span.
  Open: on hardware, does the linear fit actually beat offset-only, and what
  window / re-sync interval is best? Needs a measured drift log.
- **`grp` on the wire.** The host sends it and `ScheduledEvent` carries it, but
  the firmware doesn't store it yet, so `CANCEL grp <grp>` is a no-op. Wire it
  up when choreography sequences land (phase 3).
- **Two-phase plan commit** (`HOLD <grp>` … `GO <grp>`) — needed, or does
  "always send complete plans early" suffice?
- **Reflex-path budget.** What worst-case latency do we accept on `NOW` for
  true reflexes, and which events are allowed to use it?
- **`FIRE`/`REL` telemetry volume.** Fine at a few events/s; may need a
  summary mode if choreography gets dense.
