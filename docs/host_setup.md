# Host setup + capture/response test

The laptop side of the pipeline. Everything below runs from the repo root.

## 1. Environment

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

`sounddevice` needs the PortAudio system library for low-latency capture:

```bash
sudo apt install libportaudio2
```

Without it, `AudioCapture` automatically falls back to ALSA's `arecord`
(higher latency, but no install and no root needed).

## 2. What's implemented

| File                          | Role                                                  |
| ----------------------------- | ----------------------------------------------------- |
| `host/audio/capture.py`       | `AudioCapture` - mono float32 frames, sounddevice **or** `arecord` backend |
| `host/audio/beat_detector.py` | `BeatDetector` - spectral-flux onset detection, adaptive threshold, median-IOI tempo |
| `host/response_test.py`       | ties capture -> detector -> response (terminal meter + `ONSET`/`BEAT`, optional serial) |
| `host/control/drone_controller.py` | serial command abstraction (`P T U D L R F B S`) |

## 3. Run the response test

Dry run on the default mic - prints the command it *would* send:

```bash
python -m host.response_test
```

No mic / headless? Feed a synthetic click track and watch tempo lock:

```bash
python -m host.response_test --source synth --bpm 120
```

Run a recorded clip through the detector:

```bash
python -m host.response_test --source path/to/clip.wav
```

Drive the real drone - pulse throttle on every beat:

```bash
python -m host.response_test --port /dev/ttyUSB0 --respond throttle
```

Useful flags: `--backend {auto,sounddevice,arecord}`, `--device <idx|name>`,
`--respond {throttle,yaw,none}`, `--min-gap <s>`, `--list-devices`.

## 4. Reading the output

```
[##############--------------------------]  -13.8 dBFS   117.2 BPM      <- live level + tempo
  ♪ BEAT   str=1.00  117.2 BPM  -> U                                   <- event + response
```

`str` is normalised onset strength (0..1). `kind` is `onset` until ~4
consistent intervals have been seen, then `beat`.

## 5. Known limits (bring-up)

- Tempo resolution is quantised by the block length: 32 ms blocks -> roughly
  +/-7 BPM jitter around 120 BPM. Sub-block onset interpolation is the fix.
- The threshold constants (`THRESHOLD_K`, `REFRACTORY_S`, `FLUX_HISTORY_S` in
  `beat_detector.py`) are untuned - adjust against real music.
- ~1.5 s warm-up: the adaptive threshold only arms once its rolling flux
  window (`FLUX_HISTORY_S`) is full, so the first couple of onsets are missed.
- No predictive timing yet: responses fire on detection. Latency-compensated
  scheduling (`t_cmd = t_next - L`) comes with `analysis/latency_analysis.py`.
