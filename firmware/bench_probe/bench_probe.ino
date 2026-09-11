/*
 * bench_probe - manual transmitter-contact prodder for the characterisation phase
 * --
 * Board: Arduino Nano / Uno (ATmega328P)
 * Role : the simplest possible thing that can close one transmitter contact on
 *        command, so you can map "which contact does what" and confirm each
 *        Arduino channel reproduces a human button press *before* the real
 *        fire-at-T firmware (transmitter_controller.ino) goes on.
 *
 * No scheduler, no clock sync, no arm gate. Type a line in the serial monitor
 * (115200, newline ending), a GPIO goes HIGH (MOSFET / analog switch closes the
 * contact), and it auto-releases so nothing can stick down while you're heads-
 * down with a probe.
 *
 * Wiring: docs/hardware.md . GPIO HIGH -> switch element on -> contact shorted
 * to transmitter B- -> "button pressed". Shared ground only; the Arduino never
 * powers the transmitter.
 *
 * SAFETY: drone props OFF for everything this sketch is for. It only actuates
 * pins listed in CANDIDATES[]; every other pin is left untouched.
 *
 * Commands (one per line):
 *   ?                 list every candidate pin and its state
 *   a <pin>           assert (HIGH); auto-releases after AUTO_RELEASE_MS
 *   r <pin>           release (LOW)
 *   h <pin>           assert and HOLD - no auto-release (latch / long-press tests)
 *   p <pin> <ms>      pulse: assert, wait <ms> (<= MAX_PULSE_MS), release
 *   seq <pin> <ms> [<pin> <ms> ...]   run pulses back to back with GAP_MS between
 *                     (up to SEQ_MAX steps) - for the flight-mode arming gesture
 *   x                 release everything now (panic)
 *   help              this list
 *
 * Example - map the takeoff button, then try the arming gesture:
 *   p 2 120
 *   seq 6 150 5 150 7 150 2 150      (left up / down / right, then takeoff)
 */

// ---- edit to match your interface board -----------------------------------
// Raw Arduino pin numbers. Start with the provisional map from
// docs/transmitter_mapping.md; add/remove as you probe. A0..A5 are 14..19.
const uint8_t CANDIDATES[] = { 2, 3, 4, 5, 6, 7, 8, 9 };
const uint8_t N_CAND = sizeof(CANDIDATES) / sizeof(CANDIDATES[0]);

const uint16_t MAX_PULSE_MS     = 1500;   // longest single pulse this sketch will drive
const uint32_t AUTO_RELEASE_MS  = 3000;   // `a` asserts drop after this unless `h`eld
const uint16_t GAP_MS           = 120;    // spacing between `seq` steps
const uint8_t  SEQ_MAX          = 8;      // steps per `seq`
const uint8_t  LINE_MAX         = 80;

// ---- per-pin state -------------------------------------------------------
struct PinState { bool on; bool hold; uint32_t since; };
PinState st[N_CAND];

char    line[LINE_MAX + 1];
uint8_t lineLen = 0;

// pending single pulse (non-blocking so `x` / new input still work)
int8_t   pulseIdx = -1;
uint32_t pulseUntil = 0;

// pending seq
uint8_t  seqPin[SEQ_MAX];
uint16_t seqDur[SEQ_MAX];
uint8_t  seqLen = 0, seqAt = 0;
uint32_t seqNext = 0;
int8_t   seqActiveIdx = -1;
uint32_t seqActiveUntil = 0;

// ---- helpers ------------------------------------------------------------
int8_t idxOf(int pin) {
  for (uint8_t i = 0; i < N_CAND; i++) if (CANDIDATES[i] == pin) return i;
  return -1;
}

void setPin(uint8_t i, bool on, bool hold) {
  digitalWrite(CANDIDATES[i], on ? HIGH : LOW);
  st[i].on = on;
  st[i].hold = on && hold;
  st[i].since = millis();
  Serial.print(on ? F("+ ") : F("- "));
  Serial.print(CANDIDATES[i]);
  if (on && hold) Serial.print(F("  (hold)"));
  Serial.println();
}

void releaseAll(const __FlashStringHelper *why) {
  for (uint8_t i = 0; i < N_CAND; i++)
    if (st[i].on) { digitalWrite(CANDIDATES[i], LOW); st[i].on = st[i].hold = false; }
  pulseIdx = -1;
  seqLen = seqAt = 0;
  seqActiveIdx = -1;
  Serial.print(F("ALL LOW - ")); Serial.println(why);
}

void listState() {
  Serial.println(F("pin  state   held  ms-on"));
  for (uint8_t i = 0; i < N_CAND; i++) {
    Serial.print(' '); Serial.print(CANDIDATES[i]);
    Serial.print(CANDIDATES[i] < 10 ? F("    ") : F("   "));
    Serial.print(st[i].on ? F("HIGH  ") : F("low   "));
    Serial.print(st[i].hold ? F("yes  ") : F("no   "));
    Serial.println(st[i].on ? (millis() - st[i].since) : 0);
  }
}

void printHelp() {
  Serial.println(F("?  a<pin>  r<pin>  h<pin>  p<pin> <ms>  seq<pin> <ms>...  x  help"));
  Serial.print(F("candidates:"));
  for (uint8_t i = 0; i < N_CAND; i++) { Serial.print(' '); Serial.print(CANDIDATES[i]); }
  Serial.println();
}

// ---- command parsing --------------------------------------------------
void startPulse(int8_t i, uint16_t ms) {
  if (ms > MAX_PULSE_MS) ms = MAX_PULSE_MS;
  setPin(i, true, false);
  pulseIdx = i;
  pulseUntil = millis() + ms;
}

void handle(char *s) {
  while (*s == ' ') s++;
  if (*s == '\0') return;
  char c = *s;

  if (c == '?')            { listState(); return; }
  if (c == 'x' || c == 'X'){ releaseAll(F("panic")); return; }
  if (!strncmp(s, "help", 4)) { printHelp(); return; }

  if (!strncmp(s, "seq", 3)) {
    char *p = s + 3;
    seqLen = 0;
    while (seqLen < SEQ_MAX) {
      int pin = strtol(p, &p, 10);
      long ms  = strtol(p, &p, 10);
      if (pin == 0 && ms == 0) break;
      int8_t i = idxOf(pin);
      if (i < 0) { Serial.print(F("ERR not a candidate: ")); Serial.println(pin); return; }
      seqPin[seqLen] = i;
      seqDur[seqLen] = ms <= 0 ? 100 : (ms > MAX_PULSE_MS ? MAX_PULSE_MS : ms);
      seqLen++;
    }
    if (seqLen == 0) { Serial.println(F("ERR seq needs <pin> <ms> pairs")); return; }
    seqAt = 0; seqNext = millis(); seqActiveIdx = -1;
    Serial.print(F("seq ")); Serial.print(seqLen); Serial.println(F(" steps"));
    return;
  }

  // single-pin commands: <c> <pin> [<ms>]
  char *p = s + 1;
  int pin = strtol(p, &p, 10);
  int8_t i = idxOf(pin);
  if (i < 0) { Serial.print(F("ERR not a candidate: ")); Serial.println(pin); return; }

  switch (c) {
    case 'a': setPin(i, true, false); break;
    case 'h': setPin(i, true, true);  break;
    case 'r': setPin(i, false, false); break;
    case 'p': {
      long ms = strtol(p, NULL, 10);
      startPulse(i, ms <= 0 ? 100 : ms);
      break;
    }
    default: Serial.print(F("ERR unknown: ")); Serial.println(c);
  }
}

void readSerial() {
  while (Serial.available()) {
    char ch = (char) Serial.read();
    if (ch == '\r') continue;
    if (ch == '\n') { line[lineLen] = '\0'; if (lineLen) handle(line); lineLen = 0; }
    else if (lineLen < LINE_MAX) line[lineLen++] = ch;
    else { lineLen = 0; Serial.println(F("ERR line too long")); }
  }
}

// ---- timed releases ---------------------------------------------------
void serviceTimers() {
  uint32_t now = millis();

  if (pulseIdx >= 0 && (int32_t)(now - pulseUntil) >= 0) {
    setPin(pulseIdx, false, false);
    pulseIdx = -1;
  }

  if (seqActiveIdx >= 0 && (int32_t)(now - seqActiveUntil) >= 0) {
    setPin(seqActiveIdx, false, false);
    seqActiveIdx = -1;
    seqNext = now + GAP_MS;
  }
  if (seqLen && seqActiveIdx < 0 && seqAt < seqLen && (int32_t)(now - seqNext) >= 0) {
    seqActiveIdx = seqPin[seqAt];
    seqActiveUntil = now + seqDur[seqAt];
    setPin(seqActiveIdx, true, false);
    if (++seqAt >= seqLen) { Serial.println(F("seq done")); seqLen = 0; }
  }

  // backstop: an `a` assert (not `h`eld) can't stay HIGH forever
  for (uint8_t i = 0; i < N_CAND; i++)
    if (st[i].on && !st[i].hold && i != pulseIdx && i != seqActiveIdx
        && (now - st[i].since) > AUTO_RELEASE_MS) {
      setPin(i, false, false);
      Serial.print(F("auto-release ")); Serial.println(CANDIDATES[i]);
    }
}

// ---- setup / loop ---------------------------------------------------
void setup() {
  Serial.begin(115200);
  for (uint8_t i = 0; i < N_CAND; i++) {
    pinMode(CANDIDATES[i], OUTPUT);
    digitalWrite(CANDIDATES[i], LOW);
    st[i] = { false, false, 0 };
  }
  Serial.println(F("bench_probe ready - props OFF. `help` for commands."));
  printHelp();
}

void loop() {
  readSerial();
  serviceTimers();
}
