# transmitter_controller

fire-at-T host <-> Arduino protocol (draft v1, `docs/fire_at_t_protocol.md`).
Electronic operator of the drone's original handheld transmitter :  each
MOSFET channel shorts one transmitter control to B- when its GPIO is HIGH.
Target: Arduino Nano clone (CH340 USB-serial), **new** bootloader : 
`atmega328old` fails to sync on this board.

## Find the port

```
arduino-cli board list
```

Expect something like `/dev/ttyUSB0`.

## Compile

```
arduino-cli compile --fqbn arduino:avr:nano:cpu=atmega328 firmware/transmitter_controller/transmitter_controller.ino
```

## Upload

```
arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:nano:cpu=atmega328 firmware/transmitter_controller/transmitter_controller.ino
```

If you get `not in sync: resp=0x0c` errors, you're on the wrong bootloader
variant :  use `cpu=atmega328` (new), not `cpu=atmega328old`.

## Channel map (provisional :  only TAKEOFF/D2 is characterised so far)

```
P  pin 3   power / wake       cooldown 1000ms
T  pin 2   takeoff            cooldown 2000ms
U  pin 4   throttle up        conflicts D, cooldown 60ms
D  pin 5   throttle down      conflicts U, cooldown 60ms
L  pin 6   yaw left           conflicts R, cooldown 60ms
R  pin 7   yaw right          conflicts L, cooldown 60ms
F  pin 8   pitch forward      conflicts B, cooldown 60ms
B  pin 9   pitch back         conflicts F, cooldown 60ms
```

## Protocol (one line per command, 115200 baud)

```
HELLO                         -> HELLO <fw> <proto> <slots> <maxpulse> <horizon> <tick>
PING <seq>                    -> PONG <seq> <millis>
ARM                           -> ACK 0 OK   (gate must be open for U/D/L/R/F/B)
DISARM                        -> ACK 0 OK   (also releases all transient channels)
SCHED <id> <at> <act> <dur> [grp]
                               -> ACK <id> OK   or   NAK <id> <reason>
                               reason: late | horizon | cooldown | conflict | full | disarmed | range
NOW <act> <dur>                reflex path, fires ~now; silent on success, NAK 0 <reason> on failure
CANCEL <id> | CANCEL grp <grp> | CANCEL all
                               -> REL <id> <millis> for each event cancelled (armed only, won't yank an in-flight pulse)
ABORT                          -> SAFE abort  (frees every slot + releases all transient channels)
```

Unsolicited lines you'll also see:

```
BOOT <fw> <proto>              on power-up
FIRE <id> <millis>             a scheduled event asserted its channel
REL <id> <millis>              a scheduled event released its channel
SAFE linkloss                  no valid line received for 500ms -> transient channels released
```

`at` is an absolute timestamp in *this board's* `millis()` :  read the current
value from a `PING` reply first, then schedule relative to that.
`dur` is clipped to `MAX_PULSE_MS` (500ms); `at` must be 5–3000ms ahead of now
(`MIN_LEAD_MS`/`MAX_HORIZON_MS`).

Example :  read the clock, arm, then take off 200ms later for a 300ms pulse:

```
PING 1
ARM
SCHED 1 <ping_reply_millis+200> T 300
```

## Talk to it

Interactive, via arduino-cli's built-in monitor:

```
arduino-cli monitor -p /dev/ttyUSB0 -c baudrate=115200
```

Type a command and press Enter, e.g. `HELLO`.

Scripted, via pyserial (`pip install pyserial`) :  e.g. round-trip a PING:

```
python3 - <<'EOF'
import serial, time
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=2)
time.sleep(2.5)  # let the Nano finish its reset-on-connect
ser.reset_input_buffer()
ser.write(b'PING 1\n')
time.sleep(0.3)
print(ser.read(200).decode())
ser.close()
EOF
```
