# 2D Visualizer

`python -m host.sim.visualizer` runs the whole host stack against a synthetic
track, a mic, or a wav, and shows it live — a **hardware-behaviour rehearsal
tool**: it renders the exact transmitter-contact stream the Arduino would get
*after* the `PhysicalScheduler`, not just the choreography family.

```
┌──────────────────┬────────────────────────────────┐
│  2D drone         │  REHEARSAL READOUT             │
│  (driven by the   │   TIME                         │
│   exact channel   │   MODEL   texture/energy/      │
│   state           │           density/confidence   │
│   FakeArduino     │   TIMING  beat / BPM / novelty  │
│   receives)       │   POLICY  family/primitive/int  │
│                   │   HARDWARE COMMAND              │
│                   │     YAW_LEFT   120 ms   ...     │
│                   │     or  REJECTED  (cooldown)    │
│                   │   COOLDOWN  gate / cd / sticks  │
├──────────────────┴────────────────────────────────┤
│  timeline: energy · beat ticks · predicted-beat    │
│  lines · fire marks · physical-drop marks          │
├───────────────────────────────────────────────────┤
│  COMMAND STREAM  (after PhysicalScheduler)         │
│   12.4  SLAM/YAW_TWITCH -> YAW_RIGHT 54 YAW_LEFT 54 │
│   12.6  DIP  REJECTED  (min_interval)               │
└───────────────────────────────────────────────────┘
```

The drone reads `FakeArduino.pins` — the same channel state the real
transmitter would get — so it *is* a backend for the control stack, not a
separate mock. The pipeline is
`WAV → DALI+CLAP → beat/semantic state → MotionPolicy → PhysicalScheduler →
accepted contact stream → drone`; the readout is taken *after* the scheduler,
and rejected candidates are shown with their reason (`min_interval`, `cooldown`,
`conflict`, `too_long`, `repeat`).

## Reading three contrasting tracks

Run each with `--model --encoder clap`. They should look obviously different:

| track | model | family | typical stream | rate | look |
| --- | --- | --- | --- | --- | --- |
| metal | percussive / high | SLAM | DIP, YAW_TWITCH, BOUNCE_HARD @ ~0.85 | gate almost always busy, constant `min_interval` drops | frequent short aggressive accents |
| melodic buildup | melodic / (unknown) | SWAY | BOUNCE, RISE, occasional DROP @ ~0.5 | ~77/min, sticks mostly neutral | restrained, rises/drops on real energy change |
| ukulele | melodic / mid | SWAY | BOUNCE, SWAY_L/R, DROP @ ~0.6 | ~moderate | smooth sway, lower amplitude |

The beat-only baseline (`analysis/choreo_compare.py --baseline`) uses nearly the
same primitive mix for all three — the differentiation is the semantic branch.

## Run

```bash
python -m host.sim.visualizer --source synth --bpm 128
python -m host.sim.visualizer --source clip.wav
python -m host.sim.visualizer --source mic                 # needs a working mic
python -m host.sim.visualizer --source mic --port /dev/ttyUSB0   # + drive a real drone
```

Keys: `space` = emergency stop (`ABORT`), `esc` = quit.

Headless screenshot (CI / remote):

```bash
python -m host.sim.visualizer --source synth --screenshot out.png --screenshot-after 20
```

## Pieces

| File | Role |
| --- | --- |
| `host/audio/sources.py` | `open_source(spec)` → frame iterator. `mic` / `synth` (profiles `click`, `arc`) / `*.wav`. Shared with `response_test.py`. |
| `host/sim/drone_2d.py` | `Drone2D` — a damped-spring caricature (each axis springs back to hover; a channel pulse is a push). Not flight physics; just has to read coherently. |
| `host/sim/panels.py` | `draw_telemetry(...)` (rehearsal readout), `draw_command_log(...)` (scrolling scheduler outcomes) + `Timeline` (rolling energy + event marks, scrolls right-to-left). |
| `host/sim/visualizer.py` | wires audio → `FeatureExtractor` → `MusicStateEstimator` → `MotionPolicy` (→ `PhysicalScheduler`) → `ScheduledController` → `FakeArduino`, on a background thread; pygame renders at 60 fps off the latest state. `MotionPolicy.last_candidate` / `last_decision` expose the pre- and post-scheduler command so rejects show up. Aims each command at `predict_next_beat()` (falls back to `now + --lookahead`). |

## What it's for

Tuning. The feature normaliser, the state thresholds, and the `decide()` rule
table are all set against synthetic click tracks — this is where they get
checked against real music. The timeline is also the reactive-vs-predictive
latency picture: predicted-beat line vs. the actual fire mark.

Known: `synth`'s "peak" section sits near the RHYTHMIC/ENERGETIC boundary;
real tracks exercise ENERGETIC. Vertical drone travel is deliberately small
(short pulses, stiff spring) — watch the tilt and the lit arrows.
