# GPU / DALI audio-model branch

The **semantic** perception path — "what kind of musical event is happening" —
running on the RTX 5060, complementing the deterministic DSP timing path.

```
audio frames ──ring──┬─ FeatureExtractor  (CPU, ~31 Hz)                    → AudioFeatures  [timing]
                     │                                                       → predict_next_beat → scheduler
                     └─ ThreadedAudioModel  (worker thread, own CUDA stream, ~2 Hz)
                              │
                    DALI GPU mel  →  AudioEncoder  →  MusicalState  (semantic-state cache)
                                                          │  polled, stale-safe
                                                          ▼
                                                    motion_policy   (modulation only)
```

The model runs on a rolling **4 s** window at a **0.5 s** hop (overlapping) —
music "vibe" doesn't change every 25 ms, and CLAP needs context. It is on its
**own thread and CUDA stream**,
and the policy reads its output from a cache, so a slow forward pass can't
touch the timing path. See [profiling + the decoupling proof](#profiling--the-decoupling-proof).

## Pieces

| File | Role |
| --- | --- |
| `host/audio/dali_pipeline.py` | mel front-end. `DaliMel` = NVIDIA DALI GPU pipeline (spectrogram → mel_filter_bank), log + max-normalise in torch so it is bit-identical to `TorchMel` (`torch.stft` + a mel matrix, CUDA or CPU). `build_mel()` picks one. Output: `torch` tensor `[n_mels, frames]` on-device. |
| `host/audio/encoders.py` | `AudioEncoder` ABC — `encode(wave, sr) -> embedding`, `labels()` zero-shot, `analyze()` = both in one forward pass. `RandomProjectionEncoder` (default, no deps) and `ClapEncoder` (LAION `clap-htsat-unfused`, 512-d + zero-shot **texture / energy / density**, 3 concrete prompts each, `logit_scale` applied). `AudioModel(encoder=…)` swaps them. |
| `host/audio/audio_model.py` | `AudioModel` — mel → interpretable stats (`intensity`, `brightness`, `flux`, `density`) + `encoder.analyze()` → embedding-drift `novelty` / `section_change`. `MusicalState` is a **tiny 3-class-per-axis** record: `texture` (percussive/melodic/sparse), `energy` (low/mid/high), `density_class` (sparse/mid/dense, CLAP only), each also `"unknown"`; plus `section_change`, `confidence` (label margin). **Temporally smoothed** — label distributions are EMA'd (`semantic_alpha`) before the margin-gated argmax, so the "vibe" doesn't flip window-to-window while beat timing stays sharp. `embedding` rides along for novelty; the controller never touches it. `push()` = sync; `feed()` / `snapshot_window()` / `infer_window()` split it. `inference_delay_s` fakes a slow model. |
| ↳ `ThreadedAudioModel` | same model on a **worker thread + dedicated `torch.cuda.Stream`**. `push(frame)` only buffers (~60 µs, no CUDA); the worker runs inference on its own stream and publishes; `latest()` returns the most recent `MusicalState` (may lag by one inference). A slow forward pass never stalls or serialises behind the timing path. |
| `host/control/motion_policy.py` | `decide(features, state, musical=None)`. **`musical=None` → `_decide_dsp`** (beat-only baseline, the DSP `MusicState` table). **`musical` present → `_decide_semantic`**: `_family(texture, energy)` picks a primitive family — `PULSE` (DIP/BOUNCE/twitch), `FLOW` (sway/bounce), `CALM` (gentle sway / hold); `"unknown"` → neutral `FLOW`. Beat timing decides *when*. `density_class` nudges amplitude. `section_change` (deduped by `seq`) → variant flip + `RISE` accent. **CLAP = what kind of movement, DSP = when.** |

## Setup (RTX box)

```bash
pip install torch nvidia-dali-cuda120 nvidia-cufft-cu12
```

Plain `pip install torch` pulled `2.13.0+cu130` and it works on Blackwell
(sm_120). `dali_pipeline` preloads the cuFFT wheel's `.so` (DALI doesn't find
it on its own). No CUDA toolkit / `nvcc` needed — just the driver.

Verify:

```bash
python -c "from host.audio.dali_pipeline import build_mel; print(build_mel().backend)"   # -> dali
python -m host.sim.visualizer --source synth --model            # threaded model, panel shows [dali] Nms
python -m host.sim.visualizer --source synth --model --model-sync  # inline instead of threaded
```

Falls back to `TorchMel` (CUDA, then CPU) automatically if DALI is missing.

## Profiling + the decoupling proof

`analysis/pipeline_profile.py` runs the whole pipeline in real time over a
synthetic track with known beats and logs per-stage latency + timestamps.

```bash
python -m analysis.pipeline_profile --bpm 128 --seconds 30
python -m analysis.pipeline_profile --bpm 128 --seconds 24 --prove   # 0 vs 500 ms inference
```

`--prove` injects a 500 ms delay into the model worker. Representative run
(FakeArduino backend, `click` track):

| stage | no delay | +500 ms inference |
| --- | --- | --- |
| model inference | ~2 ms (200 ms first-call warmup) | **502 ms** |
| model-state staleness | 286 ms | 752 ms |
| DSP latency p95 | 0.56 ms | 0.42 ms |
| decision latency | ~0 ms | ~0 ms |
| serial-send latency | 0.1 ms | 0.1 ms |
| **`\|scheduling error\|` p95** (FIRE vs requested) | **0.3 ms** | **0.0 ms** |

The metric that isolates the claim is **`|scheduling error|`** — how far the
Arduino's FIRE edge lands from the time the host *asked* for, regardless of
*what* was asked. It doesn't move. Nor does DSP or decision latency. The
systems claim: *asynchronous GPU semantic inference on its own CUDA stream,
with a stale-safe semantic-state cache, is isolated from the latency-sensitive
physical control path.*

`beat-to-command` error *does* vary run-to-run once the model drives the
policy — because a staler semantic state makes the policy pick different
primitives, which fire on different beats. That's the *choreography* question
(below), not a timing-path leak.

**Semantic age at command** — the profiler logs, per motion command, how old
the semantic state it used was (`now − MusicalState.t`):

```
semantic age at command:  mean 285 ms  p95 512 ms  max 517 ms
100% of motion decisions used semantic context < 350 ms old   (no injected delay)
```

With `--inject-delay-ms 600` the age rises to ~800 ms and the fraction under
350 ms drops — but beat-to-command still doesn't move. That's the number to
quote: *N % of motion decisions used semantic context < X ms old while beat
timing was unaffected by model inference.*

Live demo: `python -m host.sim.visualizer --source synth --model --inject-delay-ms 400`
— the MODEL panel shows `infer 400 ms / stale ~900 ms` while the drone keeps
firing on the beat.

### With the real CLAP encoder

`python -m analysis.pipeline_profile --encoder clap` — same run, real model:

| stage | randproj | **CLAP** |
| --- | --- | --- |
| model inference | ~2 ms | **29 ms p50, 68 ms p95** (2.8 s first-call warmup) |
| DSP / decision / serial-send | 0.7 / 0 / 0.1 ms | 0.7 / 0 / 0.1 ms |
| **beat-to-command** | mean +34, std 66 ms | mean +34, **std 66 ms** |

Swapping a deterministic projection for a 190 M-param contrastive model
(30–70 ms/window) left the timing path **unchanged** — the "boring swap" the
async architecture was built for. CLAP inference lands squarely in the
predicted 20–80 ms.

## Encoders

| | `randproj` (default) | `clap` |
| --- | --- | --- |
| deps | none | `transformers`, `torchaudio`, ~600 MB download |
| embedding | 64-d random projection of z-scored log-mel | 512-d LAION CLAP contrastive |
| texture / energy / density | spectral heuristics (`_classify_texture`) | zero-shot: `logit_scale · cos(audio, prompts)`, softmax |
| inference | ~2 ms | ~35 ms (4 s window) |
| semantic? | no — stable feature space, drift ⇒ novelty only | yes |

**`laion/larger_clap_music` is broken under transformers ≥ 5.16** — text
embeddings collapse (mutual cosine 0.999), `logit_scale` fails to load, so
every softmax is uniform (`conf 0.33`). Default is `clap-htsat-unfused`;
`_ensure` raises if the loaded checkpoint's prompts collapse.

`_CLAP_PROMPTS` (in `encoders.py`) — 3 **concrete, contrastive** prompts per
axis ("music dominated by drums and percussion", not "percussive music").
Confidence is the **top1−top2 margin**; below `MIN_MARGIN` (0.10) the label is
`"unknown"` rather than a forced argmax.

Adding another model: subclass `AudioEncoder`, implement `encode()` (and
`analyze()` if it can do labels cheaply). `MusicalState`, the policy hook, the
threading, and the DALI front-end are untouched.

## Characterising real audio — `analysis/semantic_trace.py`

CLAP labels are near-uniform on **synthetic** signals (sine tones, clicks) —
out of distribution. On real music they work; run `semantic_trace` on 5–8 very
different clips before touching the mic:

```bash
python -m analysis.semantic_trace media/track.wav              # per-window trace + summary
python -m analysis.semantic_trace media/track.wav --raw        # + raw per-category cosines
python -m analysis.semantic_trace media/track.wav --alpha 0.25 # smoother EMA
```

Two real tracks, as a sanity check:

```
track.wav    texture percussive 78% (unknown 21%)   energy high 82%   density dense 89%   margin ~0.48
track_2.wav  texture melodic 49% (unknown 45%)       energy high 79%   density dense 99%   margin ~0.11
```

Distinct, stable within a track (texture runs ~16 s), honest about ambiguity
(`unknown`), and it catches the track.wav fade-out (`high → unknown → mid` in
the last 3 s). The question isn't "does CLAP recognise music" — it's **do the
labels evolve sensibly and stay stable enough to drive motion.** Tune
`_CLAP_PROMPTS`, `MIN_MARGIN`, and `semantic_alpha` from what you see.

Then, and only then, test the microphone (`--source mic`) — it adds room
acoustics, speaker colour, gain, and latency all at once, so a reproducible
wav baseline comes first.

## `section_change` — windowed novelty

Not adjacent 0.5 s windows. `AudioModel` compares the **mean embedding of the
previous ~`section_w_s` (4 s)** against the **mean of the current 4 s** by
cosine distance - a local self-similarity check that ignores per-beat jitter.
Fires when it clears `section_threshold` (0.10, CLAP-scaled) past a
`section_cooldown_s` (5 s) refractory window.

`semantic_trace` prints the calibration you need:

```
novelty  p50 0.017  p75 0.028  p90 0.063  p95 0.091  p99 0.120  max 0.198
novelty peaks:  147s 0.198   93s 0.110   82s 0.101  ...
```

Set `section_threshold` around p95-p99, then check the peak timestamps land on
real transitions in the track (listen).

## texture × energy → motion family

`_family()` maps the two trusted axes onto 5 families; beat/onset stays the
execution clock. **`density` is not trusted** (it read `mid` for both a metal
track and an acoustic one) - computed and traced, but not wired into motion.

|            | low   | mid   | high  |
| ---------- | ----- | ----- | ----- |
| percussive | PULSE | PULSE | SLAM  |
| melodic    | DRIFT | SWAY  | SWAY  |
| sparse     | HOLD  | DRIFT | DRIFT |

`unknown` on an axis → the middle row/column. Each family has a **distinct
primitive palette** so the choreography reads differently, not just "BOUNCE
everywhere":

| family | palette | cadence |
| --- | --- | --- |
| HOLD  | tiny SWAY | every 8th beat |
| DRIFT | soft SWAY | every 4th beat |
| SWAY  | SWAY_L/R · soft BOUNCE · RISE (8th) | ~every beat |
| PULSE | DIP-led · SWAY (3rd) | every beat |
| SLAM  | BOUNCE_HARD (short punch) · DIP · YAW_TWITCH (4th) | every beat |

Then the **physical feasibility layer** — `PhysicalScheduler`
(`host/control/physical_scheduler.py`), owned by `MotionPolicy`. `select()`
calls `decide()` (the candidate) then `scheduler.submit()`, which applies, and
tallies drop reasons for:

- **min inter-command interval** (`350 ms` ≈ 171/min ceiling); a strong onset
  (`≥ accent_strength`) fires at 40 % of that, a section RISE/DROP at an 80 ms
  hard floor
- **per-primitive cooldown**
- **mutually-exclusive / still-running** conflict

`scheduler.report()` → `candidate 312  emitted 190 (61%)  dropped 122  (min_interval 88  cooldown 24  conflict 10)`.
Musical events ≠ physical command opportunities; this is where the difference
is made explicit and countable. The firmware's `tryArm` is a second, stricter
line — `pipeline_profile` reports both (`host scheduler` + `arduino NAKs`).

## Does it actually differentiate music? — `analysis/choreo_compare.py`

Runs perception → policy over several tracks and compares the command streams:

```
media/track.wav   (metal)      family SLAM 76%   DIP 39% · BOUNCE_HARD 19% · YAW_TWITCH 18%   79 cmd/min · int 0.81
                               timeline ~~~~####...####~~~~####
media/track_2.wav (ambient-ish) family SWAY 93%   BOUNCE 36% · SWAY 31% · RISE 15%             62 cmd/min · int 0.60
                               timeline ~~~~~~~~~~~~~~~#++#~~~~~~~
```

The palettes barely overlap: metal is DIP / short BOUNCE_HARD punches /
twitches; ambient is soft rounded BOUNCE / lateral sway / rises. The
**beat-only baseline uses the same BOUNCE/SWAY mix for both tracks**
(`--baseline`) — the semantic path is what differentiates them, before flying.

## Tuning knobs

`window_seconds`, `hop_seconds`, `semantic_alpha`, `section_w_s`,
`section_threshold`, `section_cooldown_s` (`AudioModel.__init__`);
`_CLAP_PROMPTS`, `MIN_MARGIN`; `_FAMILY_GRID` and the per-family rules in
`motion_policy._decide_semantic`.

## Not done

- `density` axis needs re-prompting or dropping (`mid` everywhere).
- Section peaks not yet ear-verified against a known buildup/drop track.
- Embeddings aren't persisted or used for retrieval / learned choreography yet.
- The worker uses one dedicated stream; a real pipelined model might want
  double-buffered streams + `cuda.Event` for overlap. Not needed at V0 rates.
- `pipeline_profile` uses the FakeArduino backend, so `serial-send` and the
  FIRE edge are sim numbers; on hardware `send_ms` grows and beat-to-command
  picks up the RF/mechanical tail — but the *decoupling* result is unchanged.
