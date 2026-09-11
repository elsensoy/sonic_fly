# Command Translation Engine

Turns live audio into bounded motion primitives, rule-based for V0. Sits between
the perception front-end and any controller (2D sim or real drone via the
fire-at-T protocol).

```
AudioFrame ─► FeatureExtractor ─► AudioFeatures ──────────────┐  (timing: WHEN)
                                      │                        │
                                      ▼                        ▼
                              MusicStateEstimator          MotionPolicy
                              (DSP baseline)  ─► MusicState  .decide()  → candidate MotionCommand | None
              AudioModel (DALI mel → CLAP)   ─► MusicalState .select()  → PhysicalScheduler.submit()
              (semantic: WHAT — texture/energy/…)                 │        ├ SLAM/other min interval
                                                                 │        ├ per-primitive cooldown
                                                                 │        ├ conflict-with-running
                                                                 │        ├ max envelope / max run
                                                                 ▼        └ accepted MotionCommand
                              ScheduledController.perform(cmd, at)
                                          expand() → [ChannelOp(act, at_ms, dur_ms), …]
                                          schedule each at  at − L(primitive) + op.at_ms
                                               │
                                   ┌───────────┴────────────┐
                              2D simulator            DroneScheduler → SCHED  (fire-at-T)
                              (FakeArduino port)      → serial → Arduino
```

Everything downstream of `MotionCommand` is backend-agnostic — the same command
drives the sim and the drone.

## Modules

| File | What it produces |
| --- | --- |
| `host/audio/features.py` | `AudioFeatures` — one per frame. Timing (`beat`, `onset_strength`, `bpm`) from `BeatDetector`; a smoothed, percentile-normalised loudness envelope per band (`energy`, `bass`, `mid`, `high`, 0–1); `building` (−1..1 energy trend); `transient` (highs briefly dominate); `warmup` (True ~first 1.3 s while the normaliser calibrates). |
| `host/control/music_state.py` | `MusicState` ∈ {CALM, BUILDING, RHYTHMIC, ENERGETIC, TRANSIENT}. `MusicStateEstimator` — peak-decay energy follower + beat-rate, Schmitt bands + N-frame hysteresis so it doesn't flicker. TRANSIENT is a brief, rate-limited override, not a held state. Deterministic stand-in for the eventual GPU model (`host/audio/audio_model.py`, same output role). |
| `host/control/motion_primitives.py` | `MotionCommand(primitive, intensity, duration_ms)`; `ChannelOp(act, at_ms, dur_ms)`; `PRIMITIVES` registry (cooldown, nominal duration, conflict set); `ACT_NAMES` (channel → transmitter-contact name); `expand(cmd) -> (ops, envelope_ms)`. Primitives are **data**, not controller calls. |
| `host/control/motion_policy.py` | `MotionPolicy.decide(features, state[, musical])` — the rule table (`candidate` is an alias). `musical=None` → beat-only DSP baseline; `musical` present → `_decide_semantic`: `_family(texture, energy)` picks one of HOLD / DRIFT / SWAY / PULSE / SLAM, each with its own primitive palette. `.select(...)` = decide + `PhysicalScheduler`; exposes `last_candidate` / `last_decision`. Swap `decide()` for a learned policy later; nothing else changes. |
| `host/control/physical_scheduler.py` | `PhysicalScheduler.submit(cmd, now, *, strength, family)` → `Decision(emit, reason)`. Feasibility gate: SLAM vs other min interval (accents/section events relax it), per-primitive cooldown, conflict with a still-running primitive, max envelope (`too_long`), max consecutive emits on one axis (`repeat`). Tallies drops by reason (`stats`, `report()`). |
| `host/control/scheduled_controller.py` | `ScheduledController` — takes a `MotionCommand` + a host-clock target time, `expand()`s it, and schedules each channel op at `at − L(primitive) + op.at_ms` via `DroneScheduler` (fire-at-T). Same object drives the sim and the real drone. `LatencyModel` (`latency_model.py`) holds per-primitive `L`. |

## Primitives

`HOLD BOUNCE BOUNCE_HARD RISE DROP DIP SWAY_LEFT SWAY_RIGHT YAW_TWITCH` — each
expands to a short list of transmitter-contact pulses (`P T U D L R F B`).
`intensity` (0–1) scales the pulse widths. `BOUNCE` is a rounded up-down bob;
`BOUNCE_HARD` is a short punchy up-hit with no counter-pulse (SLAM/PULSE).

## Rule table (frozen V1)

Beat / onset is always the clock. `musical` present → the family branch;
`musical` absent → the beat-only DSP baseline (the comparison, `_decide_dsp`).

**Pre-family, both paths:**

| Situation | Command |
| --- | --- |
| `state == TRANSIENT` | `YAW_TWITCH`, intensity from `high` |
| armed high energy then sustained fall | `DROP`, once — intensity 0.85 if semantic energy is `high`, else 0.5 |
| `energy_smoothed < 0.12` | nothing (silence) |
| no beat this frame | nothing |
| first beat after a `section_change` | `RISE` (the section accent) |

**Family branch** (`_family(texture, energy)`, beat index `n`, `amp` from a blend
of frame energy and semantic energy class):

| Family (texture × energy) | Behaviour |
| --- | --- |
| HOLD (sparse · low) | a soft sway every 8th beat, else nothing |
| DRIFT (sparse · mid/high, melodic · low) | a gentle sway every 4th beat |
| SWAY (melodic · mid/high) | soft `RISE` every 8th, else alternate sway / rounded `BOUNCE` |
| PULSE (percussive · low/mid) | `DIP`-led with an occasional sway |
| SLAM (percussive · high) | `YAW_TWITCH` / `DIP` / `BOUNCE_HARD` every beat, high intensity |

`PhysicalScheduler` then paces the stream: SLAM gets `slam_interval_ms` (≈92
cmd/min), everything else `min_interval_ms` (≈150 cmd/min ceiling), with strong
accents and section events allowed inside the gap.

## Tuning knobs

- `features.py`: `_HISTORY_S` (normaliser window), envelope time constant
  (`_env_alpha`, ~0.4 s), `_WARMUP_FRAMES`.
- `music_state.py`: `energy_release`, `switch_frames`, `calm_energy`,
  `energetic_energy`, `leave_margin`.
- `motion_policy.py`: the `decide()` / `_decide_semantic()` tables; `_FAMILY_GRID`.
- `physical_scheduler.py`: `min_interval_ms`, `slam_interval_ms`, `priority_relief`,
  `max_run`, `max_duration_ms`; per-primitive `cooldown_ms` in `PRIMITIVES`.

These are guesses tuned against synthetic click tracks. The 2D visualizer
(`python -m host.sim.visualizer`, see [`visualizer.md`](visualizer.md)) is
where they get tuned against real music.

## Tests

`tests/test_translation.py` — primitive expansion, state classification +
hysteresis + transient override, each rule branch, and a `FeatureExtractor`
smoke test on a synthetic click track. `tests/test_physical_scheduler.py` —
the feasibility gate (intervals, cooldown, conflict, `too_long`, `repeat`).
`analysis/choreo_compare.py` — the cross-track behavioural check.
