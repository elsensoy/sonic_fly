# turnonoff

Pulses pin 2 HIGH for 150ms (simulates a power-button press) on receiving `p`
over serial. Target: Arduino Nano clone (CH340 USB-serial), **new**
bootloader — `atmega328old` fails to sync on this board.

## Find the port

```
arduino-cli board list
```

Expect something like `/dev/ttyUSB0`.

## Compile

```
arduino-cli compile --fqbn arduino:avr:nano:cpu=atmega328 firmware/turnonoff/turnonoff.ino
```

## Upload

```
arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:nano:cpu=atmega328 firmware/turnonoff/turnonoff.ino
```

If you get `not in sync: resp=0x0c` errors, you're on the wrong bootloader
variant — use `cpu=atmega328` (new), not `cpu=atmega328old`.

## Trigger a pulse and watch the reply

Interactive, via arduino-cli's built-in monitor:

```
arduino-cli monitor -p /dev/ttyUSB0 -c baudrate=115200
```

Type `p` and press Enter. Expect:

```
turnonoff ready
PULSE POWER 150ms
```

(Ctrl+C to exit the monitor.)

Scripted, via `test_pulse.py` in this folder (requires `pip install pyserial`):

```
python3 firmware/turnonoff/test_pulse.py
```

Pass a different port as an argument if it's not `/dev/ttyUSB0`:

```
python3 firmware/turnonoff/test_pulse.py /dev/ttyACM0
```

Expect `b'PULSE POWER 150ms\r\n'`.
