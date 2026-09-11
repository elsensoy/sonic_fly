# Acoustic Physical AI Micro-Drone

> Reverse-engineering a low-cost toy drone into a programmable physical-AI platform for real-time audio perception, motion planning, and music-responsive flight.

![The 2D rehearsal visualizer, mid-track: drone state, perception telemetry, the decision policy, and the scheduled command stream, all driven by a live audio feed](docs/Screenshot.png)

*The 2D visualizer above is a rehearsal tool, not a mockup — it renders the exact
transmitter-contact stream the real Arduino would receive, driven by whatever
track you play it.*

## Highlights

![Final Drone Wiring Picture](docs/picture.jpg)

* **A $10 "broken" toy drone, made programmable** — no RF protocol reverse-engineering, no replacement flight controller. The original handheld transmitter is driven electronically instead, so the drone's existing radio link, stabilization, and motor control stay untouched.
* **Two-speed perception** — a classical beat/onset PLL running at ~31 Hz for tight timing, alongside a GPU semantic branch (DALI mel front-end + LAION CLAP zero-shot encoder) at ~2 Hz for *what kind* of musical moment is happening — on its own CUDA stream so a slow inference never stalls the timing path.
* **Predictive, not reactive, actuation** — commands are scheduled for the *next predicted* beat and fired by a 1 kHz timer ISR on the Arduino itself, so a busy host loop can't jitter the timing.
* **Safety lives on the hardware** — per-channel cooldowns, mutual exclusion, a concurrency cap, and a link watchdog are enforced on the Arduino, independent of whatever the perception layer asks for.
* **Watch it decide, live** — the 2D visualizer plays the track over your speakers while showing the same channel stream the physical drone would get, plus the telemetry and scheduler decisions behind each command.
* **Tested and pinned** — ~96 tests, a 3-track behavioural snapshot, and a frozen choreography policy (tag `policy-v1`) so hardware bring-up is the only moving variable left.

## Quick Start

```bash
cd ~/audio_drone
source .venv/bin/activate
python -m host.sim.visualizer --source media/track.wav
```

Opens the 2D visualizer window and plays `track.wav` through your speakers,
in sync. Useful variants:

```bash
# full choreography, driven by the CLAP semantic model (downloads ~2GB on first run)
python -m host.sim.visualizer --source media/track.wav --model --encoder clap

# a different track
python -m host.sim.visualizer --source media/track_2.wav

# silent (no speaker playback, just the visuals)
python -m host.sim.visualizer --source media/track.wav --no-play

# drive the real Arduino instead of the on-screen simulated drone
python -m host.sim.visualizer --source media/track.wav --port /dev/ttyUSB0
```

In the window: **Space** stops the drone, **Esc** or closing the window exits.
More runnable commands (analysis scripts, firmware) are under
[Running It](#running-it) further down.

## Contents

* [Overview](#overview)
* [Project Motivation](#project-motivation)
* [System Architecture](#system-architecture)
* [Reverse Engineering the Transmitter](#reverse-engineering-the-transmitter)
* [Analog Switch Control Interface](#analog-switch-control-interface)
* [Electronic Button Emulation](#electronic-button-emulation)
* [Host-to-Arduino Protocol](#host-to-arduino-protocol)
* [Physical AI Branch](#physical-ai-branch)
* [Audio Perception Pipeline](#audio-perception-pipeline)
* [Motion Primitives](#motion-primitives)
* [Safety and Command State Machine](#safety-and-command-state-machine)
* [Temporal Alignment](#temporal-alignment)
* [Current Development Status](#current-development-status)
* [Software Structure](#software-structure)
* [Running It](#running-it)
* [Earlier Acoustic Command Experiment](#earlier-acoustic-command-experiment) *(legacy, kept for comparison)*
* [Experimental Questions](#experimental-questions)
* [Future Work](#future-work)
* [Design Philosophy](#design-philosophy)
* [Disclaimer](#disclaimer)

## Overview

This project turns a used ~$10 toy drone into a programmable robotics platform without replacing its existing radio, flight controller, motor drivers, or stabilization electronics.

Instead of attempting to reverse-engineer the drone's proprietary RF protocol, the system electronically emulates the original handheld transmitter controls.

An Arduino Nano acts as the low-level hardware interface. CD4066 analog switches reproduce physical button and joystick switch closures on the transmitter PCB, allowing software commands to control the original transmitter.

A laptop equipped with an NVIDIA RTX 5060 provides the high-level compute layer. It listens to live music, performs real-time audio analysis and GPU-accelerated inference, interprets musical structure, selects motion primitives, and sends compact commands over USB serial to the Arduino.

The result is a heterogeneous robotics system:

```text
Audio
  │
  ▼
RTX 5060 Laptop
  │
  │  audio perception
  │  temporal inference
  │  motion selection
  │
  ▼
USB Serial
  │
  ▼
Arduino Nano
  │
  │  deterministic command timing
  │  analog switch (CD4066) actuation
  │
  ▼
Original Drone Transmitter
  │
  │  original RF protocol
  ▼
Drone Flight Controller
  │
  │  stabilization
  │  motor mixing
  ▼
Four Motors
```

The project is intentionally split between high-level and low-level computation:

* **RTX 5060:** perception, inference, musical interpretation, behavior selection
* **Arduino Nano:** command timing and hardware actuation
* **Original transmitter:** RF communication
* **Original drone PCB:** stabilization, motor mixing, and motor control

This allows a discarded consumer device to become a programmable Physical AI platform while preserving the engineering already present in the original aircraft.

---

# Project Motivation

A programmable drone normally provides an SDK, documented protocol, or accessible flight controller.

This drone provides none of those.

The platform began as a seller-described non-working toy purchased for approximately $10. Testing showed that the aircraft, transmitter, RF link, stabilization system, and motors were still functional.

![Seller Claiming Drone Wouldn't Work](docs/claim.png)

That created a different engineering problem:

> How can an undocumented consumer device be converted into a programmable robotic system without rebuilding its flight electronics from scratch?

The chosen solution is to intercept the system at the **human input boundary**.

Instead of recreating the RF protocol:

```text
Arduino -> custom RF implementation -> drone
```

the project retains the original radio link:

```text
Arduino
   │
   ▼
electronic switch emulation
   │
   ▼
original transmitter
   │
   ▼
original RF link
   │
   ▼
original drone electronics
```

This considerably reduces the reverse-engineering surface.

The project does **not** need to reconstruct:

* proprietary RF packet structure
* channel pairing
* stabilization firmware
* attitude-control loops
* motor mixing
* motor-driver electronics

It only needs to determine:

> What electrical event does each physical transmitter control generate?

---

# System Architecture

## High-Level Architecture

```text
                         ┌──────────────────────────────┐
                         │       AUDIO SOURCE           │
                         │ microphone / live music      │
                         └──────────────┬───────────────┘
                                        │
                                        ▼
                         ┌──────────────────────────────┐
                         │       RTX 5060 LAPTOP        │
                         │                              │
                         │  Audio acquisition           │
                         │  Beat / onset detection      │
                         │  Spectral analysis           │
                         │  GPU audio inference         │
                         │  Musical-state estimation    │
                         │  Motion-primitive selection  │
                         └──────────────┬───────────────┘
                                        │
                                   USB Serial
                                        │
                                        ▼
                         ┌──────────────────────────────┐
                         │       ARDUINO NANO           │
                         │                              │
                         │  Serial command parser       │
                         │  State machine               │
                         │  Command timing              │
                         │  GPIO output                 │
                         └──────────────┬───────────────┘
                                        │
                                        ▼
                         ┌──────────────────────────────┐
                         │  ANALOG SWITCH INTERFACE     │
                         │       (CD4066)               │
                         │ electronic button / stick    │
                         │ contact emulation            │
                         └──────────────┬───────────────┘
                                        │
                                        ▼
                         ┌──────────────────────────────┐
                         │  ORIGINAL RF TRANSMITTER     │
                         └──────────────┬───────────────┘
                                        │
                                       RF
                                        │
                                        ▼
                         ┌──────────────────────────────┐
                         │       ORIGINAL DRONE         │
                         │                              │
                         │ receiver                     │
                         │ flight controller            │
                         │ stabilization                │
                         │ motor drivers                │
                         └──────────────┬───────────────┘
                                        │
                                        ▼
                                   Four Motors
```

---

# Reverse Engineering the Transmitter

## Why the Transmitter Is the Interface

The original handheld transmitter already knows how to communicate with the drone.

Rather than replacing it, the Arduino behaves like an **electronic operator** of the existing controls.

The transmitter PCB exposes recognizable power markings:

```text
B+    battery positive
B-    battery negative / controller ground
ANT   RF antenna connection
```

The first control characterized was the takeoff/start button.

With the transmitter powered off, resistance was measured between each button contact and `B-`.

Example measurements:

```text
button pad A -> B- ≈ 0 Ω
button pad B -> B- ≈ 2.58 kΩ
```

This indicates that one side of the physical switch is connected to ground while the other is a controller signal.

The physical control therefore behaves approximately like:

```text
controller signal
       │
       │
    [ switch ]
       │
       ▼
      GND
```

Pressing the button connects the controller signal to ground.

This is useful because the same action can be reproduced electronically.

---

# Analog Switch Control Interface

A **CD4066** quad bilateral analog switch is used as the electronic switch, one
channel per transmitter control.

## Why an analog switch instead of a MOSFET

The initial characterization (resistance from one button pad to `B-`) only
confirmed a switch-to-ground topology for the single takeoff button that was
probed. An N-MOSFET is a natural fit for *that* case: its source sits at
ground and it only needs to pull one signal down to `B-`.

It's not yet established that every control on this PCB works the same way.
Joystick and multi-button transmitters wire their contacts as a
**scan matrix** — rows and columns shared across several buttons, with no
single side tied to ground. A MOSFET can't safely emulate a press between two
arbitrary, possibly-floating matrix nodes: it assumes a ground-referenced
source. A CD4066 channel is a bilateral pass switch — it connects two nodes
without caring which one (if either) is at ground — so it emulates a contact
closure correctly whether the underlying topology turns out to be
switch-to-`B-` or a row/column matrix. 

## Wiring (as built, `firmware/analog_switch/analog_switch.ino`)

```text
Nano 5V  ──────────────── 4066 pin 14 (Vdd)
Nano GND ──────────────── 4066 pin 7  (Vss)  ── controller battery − (common ref)
Nano D4  ──────────────── 4066 pin 13 (control A)     HIGH = switch closed
4066 pin 1 / pin 2 ─────── probe wires across the two button pads
```

Each additional channel uses one more of the CD4066's four switch pairs and
one more Nano GPIO as its control line. Unused control pins (4066 pins 5, 6,
12) must be tied to `Vss` (pin 7) — a floating CD4066 control input makes the
whole chip behave erratically, not just the unused channel.

The controller remains powered by its **own battery**. The Arduino does not
supply power to the transmitter. The shared ground only provides a common
electrical reference:

```text
Arduino GND ───────── Controller B-
```

---

# Electronic Button Emulation

From the Arduino's perspective, each transmitter control becomes a GPIO-controlled switch.

```text
GPIO LOW
   │
   ▼
switch OPEN
   │
   ▼
button pads disconnected from each other
   │
   ▼
button released
```

```text
GPIO HIGH
   │
   ▼
switch CLOSED
   │
   ▼
button pads connected together
   │
   ▼
button pressed
```

The eventual transmitter abstraction is intended to expose commands such as:

```text
POWER
TAKEOFF
THROTTLE_UP
THROTTLE_DOWN
YAW_LEFT
YAW_RIGHT
PITCH_FORWARD
PITCH_BACK
ROLL_LEFT
ROLL_RIGHT
```

The physical implementation may evolve as additional joystick contacts are characterized.

---

# Host-to-Arduino Protocol

The Arduino is intentionally unaware of music, neural networks, or semantic audio information.

It receives simple commands over serial.

An early protocol can be represented as:

```text
P   power / wake
T   takeoff
U   throttle up
D   throttle down
L   left
R   right
F   forward
B   backward
S   stop / release
```

Conceptually:

```text
Python
  │
  │ "U"
  ▼
USB Serial
  │
  ▼
Arduino Nano
  │
  ▼
THROTTLE_UP GPIO
  │
  ▼
analog switch (CD4066)
  │
  ▼
transmitter stick contact
```

This creates a clean hardware abstraction between the physical drone and the high-level software.

The host should eventually interact with an API such as:

```python
drone.takeoff()
drone.throttle_up(duration_ms=100)
drone.yaw_left(duration_ms=80)
drone.hover()
```

rather than manipulating serial bytes directly throughout the perception code.

---

# Physical AI Branch

## Goal

The second stage of the project uses the reverse-engineered drone as a physical output device for real-time music perception.

The goal is **not** simply:

```text
bass -> left
treble -> right
beat -> up
```

That produces a reactive audio visualizer.

Instead, the project treats music interpretation and physical expression as a small perception-planning-control problem.

```text
audio
  │
  ├─► DSP branch  (beat / onset / energy, ~31 Hz)  ──► predict_next_beat ─┐   WHEN
  │                                                                       │
  └─► semantic branch  (DALI mel -> CLAP, ~2 Hz, own CUDA stream)          │
                     │                                                    │
              musical state (texture · energy · section)                  │
                     │                                                    │
                     ▼                                                    │
              motion policy  (texture×energy -> family -> primitive) ◄──────┘   WHAT
                     │
                     ▼
              PhysicalScheduler  (min interval · cooldown · conflict · priority)
                     │
                     ▼
              ScheduledController ──► fire-at-T ──► transmitter ──► drone
```

As built: `docs/audio_model.md` (semantic branch), `docs/translation_engine.md`
(policy), `docs/fire_at_t_protocol.md` (scheduling).

---

# Audio Perception Pipeline

The audio system contains two complementary paths.

```text
                         microphone
                             │
                             ▼
                       audio window
                             │
                ┌────────────┴────────────┐
                │                         │
                ▼                         ▼
       deterministic DSP          GPU audio model
                │                         │
       beat / onset / BPM        semantic embedding
       energy / spectrum         mood / texture
       transient detection       musical context
                │                         │
                └────────────┬────────────┘
                             │
                             ▼
                     musical-state model
                             │
                             ▼
                       motion policy
```

The two branches answer different questions.

### Deterministic signal processing

Answers:

> **When should motion occur?**

Candidate features include:

* beat timing
* onset timing
* tempo
* broadband energy
* low/mid/high-band energy
* spectral changes
* transient strength

### GPU inference

Answers:

> **What kind of musical event is happening?**

Potential outputs include:

* learned audio embeddings
* musical texture
* energy/intensity class
* coarse genre characteristics
* section change
* mood or semantic state

The RTX 5060 therefore performs more than FFT acceleration. It provides the compute required for learned audio representation and inference while the deterministic signal-processing branch provides accurate timing information.

---

# Motion Primitives

The AI system does not directly command motors.

Instead, perception selects from a small library of safe, pre-defined physical behaviors.

Example mapping:

| Musical event            | Motion primitive                 |
| ------------------------ | -------------------------------- |
| Strong beat              | Vertical pulse                   |
| Bass onset               | Short downward accent            |
| High-frequency transient | Brief yaw twitch                 |
| Rising energy            | Increasing motion amplitude      |
| Section transition       | Change choreography              |
| Quiet passage            | Hover / restrained movement      |
| Sustained high energy    | Alternating lateral movement     |
| Musical drop             | Short climb followed by recovery |

The architecture becomes:

```text
audio perception
      │
      ▼
 musical state
      │
      ▼
motion primitive
      │
      ▼
trajectory / state machine
      │
      ▼
transmitter command sequence
```

This isolates perception from actuation.

It also allows motion primitives to be tested independently before an AI system is permitted to select them.

---

# Safety and Command State Machine

The low-level controller should enforce constraints independently of the perception layer.

The host may request a behavior, but a safety/state layer determines whether it is valid.

Conceptually:

```text
requested primitive
        │
        ▼
┌───────────────────┐
│ safety validation │
│                   │
│ command duration  │
│ cooldown          │
│ mutually-exclusive│
│ inputs            │
│ altitude limits*  │
└─────────┬─────────┘
          │
          ▼
Arduino command
```

`*` Closed-loop altitude constraints require external sensing and are a future extension.

This prevents the audio model from having unrestricted access to raw actuator commands.

---

# Temporal Alignment

A reactive system detects a beat and only then begins the actuation chain:

```text
beat detected
     │
     ▼
decision
     │
     ▼
USB serial
     │
     ▼
Arduino
     │
     ▼
analog switch
     │
     ▼
transmitter
     │
     ▼
RF
     │
     ▼
drone motion
```

Every stage introduces latency.

If the total measured delay between perception and observable motion is \(L\), and the estimated beat period is \(T\), then the next expected beat after beat \(k\) is

$$
t_{\text{next}} = t_k + T
$$

and the command can be issued at approximately

$$
t_{\text{cmd}} = t_{\text{next}} - L.
$$

This converts the problem from purely reactive beat detection into **predictive physical synchronization**.

A useful evaluation metric is therefore:

```text
Reactive controller
mean beat-to-motion timing error: _____ ms

Predictive controller
mean beat-to-motion timing error: _____ ms
```

This experiment is planned once the actuator interface and basic music-response pipeline are operational.

---

# Current Development Status

### Hardware / firmware
* [x] Verify drone + RF transmitter operation; identify transmitter power / ground; characterise initial button contacts; demonstrate switch-to-ground topology
* [x] MOSFET-based Arduino interface designed; shared reference ground; first GPIO-controlled transmitter channels
* [x] Switched to a CD4066 analog-switch interface (`firmware/analog_switch/analog_switch.ino`) — the full button/joystick network's topology isn't characterised yet and may turn out to be a scan matrix rather than switch-to-`B-`; a bilateral analog switch emulates a contact closure correctly either way, where a ground-referenced MOSFET would not
* [ ] Finish mapping joystick directions · complete the analog-switch channel bank
* [x] `transmitter_controller.ino` — fire-at-T protocol: line parser, 1 kHz timer-ISR event scheduler, clock sync, per-channel cooldown / mutual-exclusion / concurrency, link watchdog (`docs/fire_at_t_protocol.md`)
* [~] Validate PC -> Arduino -> transmitter -> drone — full stack runs against `host/sim/fake_arduino.py`; real hardware pending

### Host control stack
* [x] `DroneScheduler` + `ScheduledController` — host owns a clock model (offset -> linear drift), schedules `SCHED <id> <at> …`, tracks ACK/NAK -> FIRE -> REL
* [x] `predict_next_beat()` PLL + fire-at-T scheduling — predictive, not reactive
* [x] `PhysicalScheduler` feasibility layer — candidate -> min-interval / cooldown / conflict / priority -> emit-or-drop, with a per-reason tally
* [~] Actuation latency — reactive-vs-predictive + per-stage profile in sim (`docs/latency_experiment.md`, `analysis/pipeline_profile.py`); RF/mechanical tail needs hardware

### Perception
* [x] Mic / wav / synth acquisition (`host/audio/capture.py`, `sources.py`)
* [x] Beat / onset detector — spectral flux + phase-locked loop (`host/audio/beat_detector.py`)
* [x] GPU semantic branch — DALI GPU mel + `ThreadedAudioModel` on its own CUDA stream; pluggable `AudioEncoder` (`RandomProjection` default, `ClapEncoder` = LAION CLAP zero-shot, ~35 ms); smoothed, margin-gated `texture` / `energy`; windowed-novelty section detection (`docs/audio_model.md`)
* [x] Decoupling proven — 500 ms injected inference leaves `|scheduling error|` and DSP latency unmoved (`pipeline_profile.py --prove`); semantic-age-at-command telemetry

### Choreography
* [x] Command translation engine — `AudioFeatures` -> `MusicState` -> `MotionCommand` (`docs/translation_engine.md`)
* [x] `texture × energy` -> 5 motion families (HOLD / DRIFT / SWAY / PULSE / SLAM), each with its own primitive palette; beat/onset stays the clock
* [x] `PhysicalScheduler` feasibility gate — SLAM vs other min interval, per-primitive cooldown, still-running conflict, max envelope, max run on one axis; drop tally by reason
* [x] Semantic choreography differs by track — metal -> SLAM 92 %, DIP/BOUNCE_HARD/YAW @ ~90 cmd/min; melodic -> SWAY, soft BOUNCE/sway/RISE @ ~75 cmd/min — while the beat-only baseline uses the same mix for both (`analysis/choreo_compare.py --baseline`)
* [x] **Policy frozen** as of tag `policy-v1`. `decide()` / `_decide_semantic()` / `_FAMILY_GRID` / the scheduler constants / the CLAP prompts + pinned revision are fixed. Any change needs `pytest -m slow tests/test_policy_snapshot.py` (the 3-track behavioural band check) re-run and re-baselined. Hardware is the next moving variable, not the policy.

### Demo
* [x] 2D visualizer — drone sprite driven by the real channel state + telemetry + scrolling timeline (`docs/visualizer.md`)
* [ ] Record final demonstration · vision feedback loop

---

# Software Structure

```text
firmware/
  transmitter_controller/transmitter_controller.ino   fire-at-T protocol, event scheduler, transmitter channels
  analog_switch/analog_switch.ino                     single-channel CD4066 switch driver (c/o/t/? over serial) — characterisation tool
  bench_probe/bench_probe.ino                         multi-pin manual prodder for mapping transmitter contacts
  turnonoff/turnonoff.ino                             single power-button pulse, triggered over serial
  acoustic_receiver/acoustic_receiver.ino             earlier standalone Goertzel tone detector (kept for comparison)

host/
  audio/     capture · sources · beat_detector (flux + PLL) · features · dali_pipeline · encoders · audio_model
  control/   protocol · link · clock_sync · scheduler (DroneScheduler) · scheduled_controller
             latency_model · music_state · motion_primitives · motion_policy · physical_scheduler
  sim/       fake_arduino (protocol sim over a pty) · drone_2d · panels · visualizer
  response_test.py   mic -> beat -> response bring-up tool

analysis/
  detector_analysis.py   embedded-detector CSV characterisation
  latency_analysis.py    reactive vs predictive beat-to-motion
  pipeline_profile.py    per-stage latency, decoupling proof, feasibility tally
  semantic_trace.py      per-window CLAP labels + novelty stats for a track
  choreo_compare.py      command-timeline comparison across tracks

docs/       architecture · hardware · transmitter_mapping · host_setup · fire_at_t_protocol
            translation_engine · latency_experiment · audio_model · visualizer

tests/      ~96 tests (pytest); host-side logic + protocol sim, no hardware needed
```

Firmware compiles with `arduino-cli`; the host stack is Python 3.11+ (RTX 5060
for the GPU branch — falls back to CPU).

---

# Running It

```bash
python3 -m venv .venv && source .venv/bin/activate && pip install -r requirements.txt

# 2D visualizer — drone reacts to a track, CLAP driving the choreography family
python -m host.sim.visualizer --source media/track.wav --model --encoder clap
#   --encoder randproj (no CLAP)   --inject-delay-ms 400 (watch decoupling)
#   --screenshot out.png --screenshot-after 30 (headless)

# characterise a track's semantic labels + section novelty
python -m analysis.semantic_trace media/track.wav --raw

# compare the command timelines two tracks produce (+ the beat-only baseline)
python -m analysis.choreo_compare media/track.wav media/track_2.wav --baseline

# latency: reactive vs predictive, and the inference-decoupling proof
python -m analysis.latency_analysis --bpm 128 --seconds 30 --calibrate
python -m analysis.pipeline_profile --prove

# firmware
arduino-cli compile -b arduino:avr:nano firmware/transmitter_controller
```

---

# Earlier Acoustic Command Experiment

Before the GPU-based Physical AI branch, the project explored direct acoustic command decoding on a resource-constrained Arduino Nano.

The original question was:

> Can a small embedded system reliably distinguish intentional acoustic commands from music, speech, motor noise, and environmental interference?

The Arduino sampled an analog microphone and used the **Goertzel algorithm** to detect a small set of predefined command frequencies.

Example protocol:

```text
GUARD + SPIN   -> SPIN
GUARD + BOB    -> BOB
GUARD + WIGGLE -> WIGGLE
```

A guard tone acted as a simple start-of-frame delimiter to reduce accidental triggering.

This branch remains useful as a comparison between:

```text
embedded deterministic perception
```

and

```text
GPU-assisted learned perception
```

rather than being discarded.

## Embedded Audio Detector

The original detector uses:

* Arduino Nano
* analog microphone input
* 8 kHz ADC sampling
* 200-sample analysis windows
* four Goertzel frequency detectors
* adaptive energy threshold
* relative frequency-energy threshold
* guard-tone arming
* serial telemetry

Current target frequencies:

```text
1600 Hz   GUARD
2000 Hz   SPIN
2400 Hz   BOB
2800 Hz   WIGGLE
```

A block is accepted only when:

```text
energy >= adaptive_gate

AND

winning_frequency_ratio >= REL_THRESHOLD
```

The adaptive energy gate is

```text
gate = max(MIN_ENERGY, noise_floor × ENERGY_MARGIN)
```

where the ambient noise estimate is maintained with an exponential moving average:

```c
noiseEnergy += (energy - noiseEnergy) * alpha;
```

This lets the detector adapt to microphone gain and ambient noise without storing a long history of samples on the Nano's limited memory.

## Embedded Sampling Architecture

Audio acquisition uses a ping-pong buffering scheme.

```text
ADC ISR
  │
  ├──────── fills Buffer A
  │
  │            │
  │            └── main loop analyzes Buffer B
  │
  └──────── fills Buffer B
               │
               └── main loop analyzes Buffer A
```

The ADC is timer-driven at 8 kHz.

Each 200-sample block therefore represents:

$$
\frac{200}{8000} = 25\text{ ms}
$$

of audio.

This permits acquisition to continue while the previous block is analyzed.

A `dropped` counter records occasions when analysis and telemetry take too long and no free buffer is available.

## Telemetry

The embedded detector exposes CSV telemetry at 115200 baud:

```text
ms,energy,r1600,r2000,r2400,r2800,best,tone,armed,move,dropped,noise,gate
```

Important fields include:

| Field           | Meaning                              |
| --------------- | ------------------------------------ |
| `ms`            | block timestamp                      |
| `energy`        | total block energy                   |
| `r1600...r2800` | relative energy in each Goertzel bin |
| `best`          | strongest candidate frequency        |
| `tone`          | accepted tone or `-1`                |
| `armed`         | guard-tone state                     |
| `move`          | current decoded behavior             |
| `dropped`       | dropped acquisition blocks           |
| `noise`         | adaptive ambient-noise estimate      |
| `gate`          | current effective energy threshold   |

Example capture:

```bash
arduino-cli monitor \
  -p /dev/ttyUSB0 \
  -c baudrate=115200 > run.csv
```

Telemetry is primarily intended for detector characterization rather than normal field operation.

---

# Experimental Questions

The project is intended to answer several engineering questions.

### Reverse engineering

How little of an undocumented consumer robot needs to be replaced before it becomes programmable?

### Embedded perception

How robustly can a 2 KB-class microcontroller distinguish intentional acoustic commands from environmental audio?

### Heterogeneous robotics

What functionality belongs on a GPU host versus a microcontroller?

### Music-conditioned control

Can learned audio representations provide useful higher-level behavior information while deterministic DSP maintains precise timing?

### Temporal synchronization

How much does latency prediction improve alignment between musical events and physical motion?

---

# Future Work

Possible extensions include:

### Vision feedback

A laptop camera could track the drone and close the loop around actual motion.

```text
                    ┌──────── camera ◄────────┐
                    │                         │
audio -> RTX -> planner -> Arduino -> drone
                    ▲                         │
                    └──── pose estimate ──────┘
```

This would turn the current audio-conditioned command system into a true externally observed closed-loop controller.

### Learned choreography

Instead of selecting manually designed motion primitives, a model could generate short sequences conditioned on musical embeddings.

### Latency-aware motion planning

Different commands may have different RF, stabilization, and mechanical response delays. Each primitive could maintain its own measured timing model.

### Embedded migration

Parts of the host inference pipeline could eventually be moved to an NVIDIA Jetson platform for untethered operation.

---

# Design Philosophy

This project deliberately avoids replacing components simply because they are undocumented.

The original aircraft already contains:

* a working RF link
* a flight controller
* stabilization
* motor drivers
* mechanical integration

The engineering challenge is therefore not:

> How can I rebuild this drone?

It is:

> How can I expose the smallest possible programmable interface to an existing physical system, then build perception and intelligence above that interface?

The resulting stack combines reverse engineering, embedded systems, signal processing, GPU inference, robotics control, and physical experimentation on top of an extremely inexpensive consumer platform.

---

## Disclaimer

This project is an experimental robotics prototype intended for controlled indoor testing.

Propellers should be removed during electrical interface testing whenever flight is not required. Flight testing should be conducted in an open, controlled area with an immediate means of disabling the aircraft.
