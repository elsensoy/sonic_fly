# Transmitter Mapping

Record of what each physical control on the transmitter PCB does electrically,
and which Arduino pin / switch channel emulates it. Procedure: `docs/hardware_bringup.md`.

## Power rails (from PCB markings)

| Marking | Meaning                          |
| ------- | -------------------------------- |
| `B+`    | battery positive                 |
| `B-`    | battery negative / controller ground (Arduino GND ties here) |
| `ANT`   | RF antenna                       |

The transmitter runs on its **own** battery. The Arduino shares only ground.

## Control characterisation

Procedure per control: see `docs/hardware_bringup.md` H1. Powered-off resistance
to `B-` finds the ground side; its partner is the signal. Then powered idle /
pressed voltage, pull-up, press type, and the matrix check.

Columns: **R→B-** off-state resistance of the signal contact · **idle/press V**
powered · **type** momentary (M) / hold-repeat (H) / latch (L) · **pin** Arduino
GPIO · **✓** confirmed in H2 to match a hand press.

| Control              | R→B-    | idle V | press V | type | pin | ✓ | Notes |
| -------------------- | ------- | ------ | ------- | ---- | --- | - | ----- |
| Takeoff / start      | 2.58 kΩ | ?      | ?       | M    | D2  |   | first control mapped |
| Power / wake         | ?       | ?      | ?       | L    | ?   |   | short press = wake, long = power off; measure both hold times |
| Left stick — up      | ?       | ?      | ?       | ?    | ?   |   | arming gesture step 1 |
| Left stick — down    | ?       | ?      | ?       | ?    | ?   |   | arming gesture step 2 |
| Left stick — right   | ?       | ?      | ?       | ?    | ?   |   | arming gesture step 3 |
| Left stick — left    | ?       | ?      | ?       | ?    | ?   |   | |
| Right stick — up     | ?       | ?      | ?       | ?    | ?   |   | throttle up (`U`) |
| Right stick — down   | ?       | ?      | ?       | ?    | ?   |   | throttle down (`D`) |
| Right stick — left   | ?       | ?      | ?       | ?    | ?   |   | yaw left (`L`) |
| Right stick — right  | ?       | ?      | ?       | ?    | ?   |   | yaw right (`R`) |

**Sticks — pot or switches?**  (fill before wiring the stick channels)

- topology: ______ (3-terminal pot / 4 contacts to GND / resistor ladder)
- if pot — idle wiper V: ______ ; full-deflection V per axis: ______ / ______
- ADC levels the TX firmware acts on (down / centre / up): ______

**Matrixed?**  ______  (if yes: analog switches only, no hard short to GND)

**Parts:**  TX MCU ______ · RF chip ______

## Arming sequence (observed)

After every controller power-cycle the drone must be put into flight mode:

1. left stick **up**, then **down**, then **right**
2. press the takeoff button (the D2 / MOSFET channel above)

The serial command parser on the Arduino should own this sequence so the host
can just say "arm".

## Command → pin table (target)

Protocol chars from README > Host-to-Arduino Protocol:

| Char | Meaning        | Pin | MOSFET ch | Status |
| ---- | -------------- | --- | --------- | ------ |
| `P`  | power / wake   | ?   | ?         | TODO   |
| `T`  | takeoff        | D2  | 1         | wired  |
| `U`  | throttle up    | ?   | ?         | TODO   |
| `D`  | throttle down  | ?   | ?         | TODO   |
| `L`  | yaw / roll left| ?   | ?         | TODO   |
| `R`  | right          | ?   | ?         | TODO   |
| `F`  | forward        | ?   | ?         | TODO   |
| `B`  | backward       | ?   | ?         | TODO   |
| `S`  | stop / release all | — | —       | TODO   |
