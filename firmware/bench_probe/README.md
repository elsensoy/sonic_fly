# bench_probe

Manual transmitter-contact prodder for the characterisation phase. Assert /
release / pulse / sequence any of the candidate GPIO pins (`2 3 4 5 6 7 8 9`)
over serial to map which contact does what. Target: Arduino Nano clone
(CH340 USB-serial), **new** bootloader — `atmega328old` fails to sync on
this board.

SAFETY: drone props OFF for everything this sketch is for.

## Find the port

```
arduino-cli board list
```

Expect something like `/dev/ttyUSB0`.

## Compile

```
arduino-cli compile --fqbn arduino:avr:nano:cpu=atmega328 firmware/bench_probe/bench_probe.ino
```

## Upload

```
arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:nano:cpu=atmega328 firmware/bench_probe/bench_probe.ino
```

If you get `not in sync: resp=0x0c` errors, you're on the wrong bootloader
variant — use `cpu=atmega328` (new), not `cpu=atmega328old`.

## Commands (one per line, 115200 baud)

```
?                 list every candidate pin and its state
a <pin>           assert (HIGH); auto-releases after 3000ms
r <pin>           release (LOW)
h <pin>           assert and HOLD - no auto-release (latch / long-press tests)
p <pin> <ms>      pulse: assert, wait <ms> (<= 1500), release
seq <pin> <ms> [<pin> <ms> ...]   run pulses back to back, 120ms gap between
x                 release everything now (panic)
help              this list
```

Example — map the takeoff button, then try the arming gesture:

```
p 2 120
seq 6 150 5 150 7 150 2 150
```

## Talk to it

Interactive, via arduino-cli's built-in monitor:

```
arduino-cli monitor -p /dev/ttyUSB0 -c baudrate=115200
```

Type a command and press Enter, e.g. `p 2 120`. Expect a reply like:

```
+ 2
- 2
```

Scripted, via pyserial (`pip install pyserial`) — e.g. to list pin state:

```
python3 - <<'EOF'
import serial, time
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=2)
time.sleep(2.5)  # let the Nano finish its reset-on-connect
ser.reset_input_buffer()
ser.write(b'?\n')
time.sleep(0.3)
print(ser.read(500).decode())
ser.close()
EOF
```
