# Architecture

Full narrative lives in the top-level [README](../README.md). This file is the
working reference for how the code is laid out against that design.

## Layers

| Layer                | Runs on            | Code                                   |
| -------------------- | ------------------ | -------------------------------------- |
| Perception + planning| RTX 5060 laptop    | `host/audio/`, `host/control/`         |
| Command timing       | Arduino Nano       | `firmware/transmitter_controller/`     |
| RF link              | original transmitter | (unmodified)                         |
| Flight control       | original drone PCB | (unmodified)                           |

## Host dataflow

```
capture.py ──> features.py ──(AudioFeatures)──> music_state.py ──(MusicState)──┐
               (wraps beat_detector.py + PLL)        │                          │
               audio_model.py ──(MusicalState: DALI GPU mel → encoder)──────────┤ (modulates)
                                                                                ▼
                                                                       motion_policy.py
                                                                       .select(f, state, musical) → MotionCommand
                                                                                │
                                                                       motion_primitives.py
                                                                       expand() → [ChannelOp…]
                                                                                │
                                                          ┌─────────────────────┴───────────┐
                                                     2D simulator          scheduler.py (fire-at-T)
                                                                           → SCHED → serial → Arduino
```

See [`translation_engine.md`](translation_engine.md) for the feature vector,
state machine, and rule table.

## Two firmware sketches

- `acoustic_receiver/` — the earlier standalone experiment: Arduino samples a
  mic at 8 kHz, Goertzel tone detection, drives motors directly. Kept as the
  embedded-perception comparison point.
- `transmitter_controller/` — the current direction: Arduino receives serial
  commands from the host and switches MOSFETs wired to the original
  transmitter's controls.

## Host ↔ Arduino protocol

v0 (README > Host-to-Arduino Protocol) is char-per-command, act-on-receipt.
The planned replacement — absolute-timestamp scheduling with host/Arduino
clock sync, so timing lives on the Arduino's clock and the AI branch can ship
timed sequences — is drafted in [`fire_at_t_protocol.md`](fire_at_t_protocol.md).

## Open design questions

Tracked in README > Experimental Questions. The big ones for the code:
what belongs on the GPU vs. the Nano, and how much latency prediction buys.
