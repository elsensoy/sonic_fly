# Latency experiment: reactive vs. predictive

`analysis/latency_analysis.py` runs the host stack against a synthetic track
whose beat times are known exactly, in two scheduling modes, and reports the
README > Temporal Alignment metric.

```bash
python -m analysis.latency_analysis --bpm 128 --seconds 30
python -m analysis.latency_analysis --bpm 128 --seconds 30 --calibrate
```

## What it measures

| Mode | When the command is issued | Fires at |
| --- | --- | --- |
| reactive | as soon as the beat is detected (`now + REACT_MARGIN`) | `true_beat + detection_lag + margin` |
| predictive | at `predict_next_beat(now, min_lead) - L(primitive)` | `predicted_beat - L` |

`min_lead` (80 ms) skips a predicted beat that's too soon to schedule for and
takes the next one — so predictive always has a full beat of lead and nothing
is NAK'd `late`.

Representative sim run (`--bpm 128 --seconds 18`, FakeArduino backend):

```
detection lag   +25 ms  (std 7)
reactive     |err| 69 ms   bias +69   std 10   p90 80     0 rejected
predictive   |err| 27 ms   bias +23   std 18   p90 39     0 rejected
```

Predictive cuts mean error ~2.6×. Its residual +23 ms bias here is the sim's
FIRE-path latency (pty + tick loop); on hardware that term is sub-ms ISR
jitter, and the bias is `report_latency_s` mistuning.

**motion proxy** = the Arduino's `FIRE` edge (when the MOSFET would close),
recovered from fire-at-T telemetry and converted back to host time via the
clock model. The physical tail (RF + stabilisation + motor spin-up) is *not*
in the sim; feed an external event series to
`estimate_L(run, motion_times=...)` when a mic-near-the-drone or camera pose
track exists — the comparison logic is unchanged.

**metric** — for every fire, signed `(fire - nearest true beat)`, then:
`mean |err|`, signed bias, std, p90.

## Why predictive should win

- *reactive* error ≈ `detection_lag + margin` — a fixed late bias, and its
  variance is the per-beat detection jitter (block quantisation).
- *predictive* schedules against a **future** time from the beat detector's
  phase-locked loop (`BeatDetector.predict_next_beat`), which has already
  integrated out per-beat jitter and rejected outliers. So its variance is
  much lower, and `report_latency_s` compensation removes most of the bias.

The headline number is the std reduction: predictive is *tighter*, which is
what "in time with the music" actually means.

## `--calibrate`

Measures `L` per primitive from the reactive run (`mean(fire - true_beat)`),
feeds it into a `LatencyModel`, and re-runs predictive. On the **sim** this
overshoots (subtracting `L` with no physical tail to absorb it fires early) —
that's expected. On **hardware** `L` is the real RF/mechanical delay and the
calibrated run is the one that lands motion on the beat. The per-primitive
numbers are what `LatencyModel.per_primitive` should hold in production.

## Knobs

- `BeatDetector.report_latency_s` — the detector's own reporting delay
  (~0.8 blocks); the experiment prints the measured detection lag to tune it.
- PLL gains in `BeatDetector._pll_update` (phase pull 0.12, tempo 0.02).
- `REACT_MARGIN_S`, `PRED_MIN_LEAD_S`, `MATCH_TOL_FRAC` in `analysis/latency_analysis.py`.

## Next

Run it on real hardware with a contact mic or accelerometer on the drone
frame feeding `motion_times`, to fill in the physical tail and get true `L`.
