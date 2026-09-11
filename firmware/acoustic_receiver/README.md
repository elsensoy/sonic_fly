# acoustic_receiver

Audio-controlled dancing drone — tone command receiver. Listens to an
electret mic on A0, detects one of four fixed tones via Goertzel, and drives
2 coreless motors on D9/D10 into a move pattern. No serial *input* protocol —
it's autonomous once running; serial is used only to stream telemetry.
Target: Arduino Nano clone (CH340 USB-serial), **new** bootloader —
`atmega328old` fails to sync on this board.

## Find the port

```
arduino-cli board list
```

Expect something like `/dev/ttyUSB0`.

## Compile

```
arduino-cli compile --fqbn arduino:avr:nano:cpu=atmega328 firmware/acoustic_receiver/acoustic_receiver.ino
```

## Upload

```
arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:nano:cpu=atmega328 firmware/acoustic_receiver/acoustic_receiver.ino
```

If you get `not in sync: resp=0x0c` errors, you're on the wrong bootloader
variant — use `cpu=atmega328` (new), not `cpu=atmega328old`.

## How it behaves

Tones (Hz): `1600` guard, `2000` spin, `2400` bob, `2800` wiggle. Play the
guard tone first, then a move tone within 1500ms (`ARM_WINDOW`) — the guard
requirement can be disabled by building with `REQUIRE_GUARD 0`. Each accepted
move plays for 1200ms (`MOVE_MS`) before the rig idles again.

## Watch telemetry (115200 baud)

`LOG_TELEMETRY` is on by default and streams one CSV row per ~25ms block:

```
ms,energy,r1600,r2000,r2400,r2800,best,tone,armed,move,dropped,noise,gate
```

Interactive, via arduino-cli's built-in monitor:

```
arduino-cli monitor -p /dev/ttyUSB0 -c baudrate=115200
```

Capture to a file for tuning `REL_THRESHOLD` / `MIN_ENERGY` in a spreadsheet
or pandas (via pyserial, `pip install pyserial`):

```
python3 - <<'EOF'
import serial
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=5)
with open('telemetry.csv', 'w') as f:
    for _ in range(500):        # ~12.5s at one row/25ms
        line = ser.readline().decode(errors='replace')
        f.write(line)
ser.close()
EOF
```

Set `LOG_TELEMETRY 0` and reflash for field runs — logging costs ~5ms/block
at 115200 baud.
