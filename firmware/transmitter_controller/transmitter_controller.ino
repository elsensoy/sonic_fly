/*
 * transmitter_controller - fire-at-T host <-> Arduino protocol (draft v1)
 * --
 * Board: Arduino Nano / Uno (ATmega328P)
 * Role : electronic operator of the drone's original handheld transmitter.
 *        Each MOSFET channel shorts one transmitter control to B- when its
 *        GPIO is HIGH (see README > MOSFET Control Interface).
 *
 * Spec : docs/fire_at_t_protocol.md . The host sends actions with an absolute
 *        target timestamp in *this board's* millis() timebase; a 1 kHz timer
 *        ISR fires the assert/release edges so a busy loop() can't jitter them.
 *
 * Timers: Timer0 = millis() (untouched). Timer2 = 1 kHz scheduler tick.
 *         Timer1 is free.
 *
 * Scope: HELLO PING ARM DISARM SCHED NOW CANCEL ABORT. Safety: duration clip,
 * arm gate, horizon/late checks, mutual-exclusion overlap, per-channel
 * cooldown, concurrency cap, link watchdog. Optional checksum enforcement
 * (REQUIRE_CHECKSUM). Deferred: real ARM stick gesture, two-phase plan commit.
 */

#include <util/atomic.h>

//  protocol constants (keep in sync with docs/fire_at_t_protocol.md) 
#define FW_VERSION   "0.3.0"
#define PROTO_VERSION 1
#define REQUIRE_CHECKSUM 0        // 1 = reject any line without a valid *HH

const uint8_t  SLOTS          = 16;
const uint16_t MAX_PULSE_MS   = 500;
const uint16_t MAX_HORIZON_MS = 3000;
const uint8_t  MIN_LEAD_MS    = 5;
const uint8_t  MAX_CONCURRENT = 3;      // channels asserted at once
const uint16_t LINK_TIMEOUT_MS = 500;
const uint8_t  LINE_MAX       = 64;
const uint16_t SCHEDULER_HZ   = 1000;

// channel map (PROVISIONAL - edit to match your wiring) 
// Only TAKEOFF/D2 is characterised so far (docs/transmitter_mapping.md).
struct Channel { char act; uint8_t pin; char conflictsWith; uint16_t cooldownMs; };
const Channel CHANNELS[] = {
  { 'P', 3, 0,   1000 },   // power / wake
  { 'T', 2, 0,   2000 },   // takeoff
  { 'U', 4, 'D',   60 },   // throttle up
  { 'D', 5, 'U',   60 },   // throttle down
  { 'L', 6, 'R',   60 },   // yaw left
  { 'R', 7, 'L',   60 },   // yaw right
  { 'F', 8, 'B',   60 },   // pitch forward
  { 'B', 9, 'F',   60 },   // pitch back
};
const uint8_t N_CHANNELS = sizeof(CHANNELS) / sizeof(CHANNELS[0]);

// ---- event table ---------------------------------------------------------
enum : uint8_t { SLOT_FREE = 0, SLOT_ARMED = 1, SLOT_ASSERTED = 2 };
struct Event {
  uint16_t id;
  uint32_t fireAt;
  uint32_t releaseAt;
  uint8_t  ch;                     // index into CHANNELS[]
  volatile uint8_t state;
};
Event events[SLOTS];

uint32_t lastFireMs[N_CHANNELS];   // when each channel was last asserted (ISR-written)

// ISR -> loop() notifications (so Serial never runs in the ISR)
enum : uint8_t { NOTE_FIRE = 1, NOTE_REL = 2 };
struct Note { uint16_t id; uint32_t t; uint8_t kind; };
Note noteBuf[16];                 // written only by the ISR, read by loop() under ATOMIC_BLOCK
volatile uint8_t noteHead = 0, noteTail = 0;

// ---- state --------------------------------------------------------------
bool     armed = false;
uint32_t lastValidLineMs = 0;
bool     inSafeHold = false;

char    line[LINE_MAX + 1];
uint8_t lineLen = 0;

// ===================== helpers ====================

int8_t channelIndex(char act) {
  for (uint8_t i = 0; i < N_CHANNELS; i++)
    if (CHANNELS[i].act == act) return i;
  return -1;
}

// wrap-safe "has time `t` arrived, given now?"
static inline bool reached(uint32_t now, uint32_t t) {
  return (int32_t)(now - t) >= 0;
}

static inline void writeChannel(uint8_t ch, bool on) {
  digitalWrite(CHANNELS[ch].pin, on ? HIGH : LOW);
}

static inline bool latching(uint8_t ch) {
  return CHANNELS[ch].act == 'P' || CHANNELS[ch].act == 'T';
}

void releaseAllTransient() {
  for (uint8_t i = 0; i < N_CHANNELS; i++)
    if (!latching(i)) digitalWrite(CHANNELS[i].pin, LOW);
}

void emitLine(const __FlashStringHelper *s) { Serial.println(s); }

//======================= scheduler ISR =====================

static inline void pushNote(uint16_t id, uint32_t t, uint8_t kind) {
  uint8_t nxt = (noteHead + 1) & 15;
  if (nxt != noteTail) {
    noteBuf[noteHead].id = id; noteBuf[noteHead].t = t; noteBuf[noteHead].kind = kind;
    noteHead = nxt;
  }
}

ISR(TIMER2_COMPA_vect) {
  uint32_t now = millis();
  for (uint8_t i = 0; i < SLOTS; i++) {
    Event &e = events[i];
    if (e.state == SLOT_ARMED && reached(now, e.fireAt)) {
      writeChannel(e.ch, true);
      lastFireMs[e.ch] = now;
      e.state = SLOT_ASSERTED;
      pushNote(e.id, now, NOTE_FIRE);
    }
    if (e.state == SLOT_ASSERTED && reached(now, e.releaseAt)) {
      writeChannel(e.ch, false);
      e.state = SLOT_FREE;
      pushNote(e.id, now, NOTE_REL);
    }
  }
}

void setupSchedulerTimer() {
  // Timer2, CTC, prescaler 64: 16 MHz / 64 / 250 = 1000 Hz
  TCCR2A = _BV(WGM21);
  TCCR2B = _BV(CS22);
  OCR2A  = 249;
  TIMSK2 = _BV(OCIE2A);
}

//  ======= command handlers ====== 

void sendAck(uint16_t id)                    { Serial.print(F("ACK ")); Serial.print(id); Serial.println(F(" OK")); }
void sendNak(uint16_t id, const __FlashStringHelper *reason) {
  Serial.print(F("NAK ")); Serial.print(id); Serial.print(' '); Serial.println(reason);
}

int8_t findFreeSlot() {
  for (uint8_t i = 0; i < SLOTS; i++) if (events[i].state == SLOT_FREE) return i;
  return -1;
}

// Validate and arm one event. Returns true on success; on failure sends
// `NAK <id> <reason>`. `id` 0 is used for NOW (host ignores NAK 0, but it
// still shows up on the wire for debugging).
bool tryArm(uint8_t ch, uint32_t at, uint32_t dur, uint16_t id) {
  uint32_t now = millis();
  int32_t lead = (int32_t)(at - now);
  if (lead < (int32_t)MIN_LEAD_MS)    { sendNak(id, F("late"));    return false; }
  if (lead > (int32_t)MAX_HORIZON_MS) { sendNak(id, F("horizon")); return false; }
  uint32_t releaseAt = at + dur;

  // cooldown: measure from the later of the last real fire and any armed
  // event already queued on this channel. Skipped when neither exists
  // (e.g. TAKEOFF right after boot).
  uint32_t ref = lastFireMs[ch];
  bool hasRef = (ref != 0);
  for (uint8_t i = 0; i < SLOTS; i++) {
    Event &e = events[i];
    if (e.state != SLOT_FREE && e.ch == ch && (!hasRef || (int32_t)(e.fireAt - ref) > 0)) {
      ref = e.fireAt; hasRef = true;
    }
  }
  if (hasRef && (int32_t)(at - ref) < (int32_t)CHANNELS[ch].cooldownMs) {
    sendNak(id, F("cooldown")); return false;
  }

  // mutual exclusion + concurrency over [at, releaseAt)
  char conflict = CHANNELS[ch].conflictsWith;
  uint8_t overlap = 0;
  for (uint8_t i = 0; i < SLOTS; i++) {
    Event &e = events[i];
    if (e.state == SLOT_FREE) continue;
    if (!((int32_t)(at - e.releaseAt) < 0 && (int32_t)(e.fireAt - releaseAt) < 0)) continue;
    if (e.ch == ch || CHANNELS[e.ch].act == conflict) { sendNak(id, F("conflict")); return false; }
    overlap++;
  }
  if (overlap + 1 > MAX_CONCURRENT) { sendNak(id, F("conflict")); return false; }

  int8_t s = findFreeSlot();
  if (s < 0) { sendNak(id, F("full")); return false; }
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    events[s].id = id; events[s].fireAt = at; events[s].releaseAt = releaseAt;
    events[s].ch = ch; events[s].state = SLOT_ARMED;
  }
  return true;
}

bool armGate(char act, uint16_t id) {
  if (armed || act == 'P' || act == 'T') return true;
  sendNak(id, F("disarmed"));
  return false;
}

void handleSched(char *args) {
  // SCHED <id> <at> <act> <dur> [grp]
  char *tId  = strtok(args, " ");
  char *tAt  = strtok(NULL, " ");
  char *tAct = strtok(NULL, " ");
  char *tDur = strtok(NULL, " ");
  if (!tId || !tAt || !tAct || !tDur) { emitLine(F("ERR parse SCHED")); return; }

  uint16_t id  = (uint16_t) strtoul(tId, NULL, 10);
  uint32_t at  = strtoul(tAt, NULL, 10);
  char     act = tAct[0];
  uint32_t dur = strtoul(tDur, NULL, 10);

  int8_t ci = channelIndex(act);
  if (ci < 0)            { sendNak(id, F("range")); return; }
  if (!armGate(act, id)) return;
  if (dur > MAX_PULSE_MS) dur = MAX_PULSE_MS;          // clip, still accept

  if (tryArm((uint8_t)ci, at, dur, id)) sendAck(id);
}

void handleNow(char *args) {
  // NOW <act> <dur>  - reflex path, schedule ~now (still safety-checked)
  char *tAct = strtok(args, " ");
  char *tDur = strtok(NULL, " ");
  if (!tAct || !tDur) { emitLine(F("ERR parse NOW")); return; }
  char     act = tAct[0];
  uint32_t dur = strtoul(tDur, NULL, 10);

  int8_t ci = channelIndex(act);
  if (ci < 0)          { emitLine(F("ERR range NOW")); return; }
  if (!armGate(act, 0)) return;
  if (dur > MAX_PULSE_MS) dur = MAX_PULSE_MS;

  tryArm((uint8_t)ci, millis() + MIN_LEAD_MS, dur, 0);
}

void handleCancel(char *args) {
  // CANCEL <id> | CANCEL grp <grp> | CANCEL all
  char *a = strtok(args, " ");
  if (!a) { emitLine(F("ERR parse CANCEL")); return; }
  bool all = (strcmp(a, "all") == 0);
  bool grp = (strcmp(a, "grp") == 0);   // grp id kept in `id` field is not stored in phase 1
  uint16_t want = all || grp ? 0 : (uint16_t) strtoul(a, NULL, 10);

  for (uint8_t i = 0; i < SLOTS; i++) {
    Event &e = events[i];
    if (e.state != SLOT_ARMED) continue;          // don't yank an in-flight pulse
    if (all || (!grp && e.id == want)) {
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { e.state = SLOT_FREE; }
      Serial.print(F("REL ")); Serial.print(e.id); Serial.print(' '); Serial.println(millis());
    }
  }
}

static inline void pulsePin(uint8_t pin, uint16_t ms) {
  digitalWrite(pin, HIGH);
  delay(ms);
  digitalWrite(pin, LOW);
}

void handleArm(bool on) {
  if (!on) {
    armed = false;
    releaseAllTransient();
    emitLine(F("ACK 0 OK"));
    return;
  }

  // Real flight-mode arming gesture: left stick up -> down -> right -> takeoff.
  // This reproduces the hardware sequence used during bench-probe validation.
  const uint8_t armPins[] = { 6, 5, 7, 2 };
  const uint16_t armMs = 150;

  releaseAllTransient();
  for (uint8_t i = 0; i < sizeof(armPins) / sizeof(armPins[0]); i++) {
    pulsePin(armPins[i], armMs);
  }

  armed = true;
  emitLine(F("ACK 0 OK"));
}

void handleAbort() {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    for (uint8_t i = 0; i < SLOTS; i++) events[i].state = SLOT_FREE;
  }
  releaseAllTransient();
  emitLine(F("SAFE abort"));
}

void handleHello() {
  // HELLO <fw> <proto> <slots> <maxpulse> <horizon> <tick>
  Serial.print(F("HELLO " FW_VERSION " "));
  Serial.print(PROTO_VERSION);   Serial.print(' ');
  Serial.print(SLOTS);           Serial.print(' ');
  Serial.print(MAX_PULSE_MS);    Serial.print(' ');
  Serial.print(MAX_HORIZON_MS);  Serial.print(' ');
  Serial.println(SCHEDULER_HZ);
}

void handlePing(char *args) {
  char *seq = strtok(args, " ");
  Serial.print(F("PONG "));
  Serial.print(seq ? seq : "0");
  Serial.print(' ');
  Serial.println(millis());       // sampled as late as possible before TX
}

// ======================= line dispatch =======================

// NMEA-style checksum "...*HH". Validated when present; a missing checksum is
// accepted unless REQUIRE_CHECKSUM.
bool checksumOK(char *s) {
  char *star = strrchr(s, '*');
  if (!star) return !REQUIRE_CHECKSUM;
  uint8_t x = 0;
  for (char *p = s; p < star; p++) x ^= (uint8_t)*p;
  uint8_t given = (uint8_t) strtoul(star + 1, NULL, 16);
  *star = '\0';                    // strip checksum for the parser
  return x == given;
}

void dispatch(char *s) {
  if (!checksumOK(s)) { emitLine(F("ERR checksum")); return; }
  lastValidLineMs = millis();
  if (inSafeHold) { inSafeHold = false; }

  char *kw = s;
  char *sp = strchr(s, ' ');
  char *rest = NULL;
  if (sp) { *sp = '\0'; rest = sp + 1; }   // split keyword / remainder

  if      (!strcmp(kw, "SCHED"))  handleSched(rest ? rest : (char*)"");
  else if (!strcmp(kw, "NOW"))    handleNow(rest ? rest : (char*)"");
  else if (!strcmp(kw, "PING"))   handlePing(rest ? rest : (char*)"");
  else if (!strcmp(kw, "CANCEL")) handleCancel(rest ? rest : (char*)"");
  else if (!strcmp(kw, "HELLO"))  handleHello();
  else if (!strcmp(kw, "ARM"))    handleArm(true);
  else if (!strcmp(kw, "DISARM")) handleArm(false);
  else if (!strcmp(kw, "ABORT"))  handleAbort();
  else { Serial.print(F("ERR unknown ")); Serial.println(kw); }
}

void readSerial() {
  while (Serial.available()) {
    char c = (char) Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      line[lineLen] = '\0';
      if (lineLen > 0) dispatch(line);
      lineLen = 0;
    } else if (lineLen < LINE_MAX) {
      line[lineLen++] = c;
    } else {
      lineLen = 0;                 // overflow: drop the line
      emitLine(F("ERR parse overflow"));
    }
  }
}

void drainNotes() {
  while (noteTail != noteHead) {
    Note n;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      n.id = noteBuf[noteTail].id;
      n.t = noteBuf[noteTail].t;
      n.kind = noteBuf[noteTail].kind;
      noteTail = (noteTail + 1) & 15;
    }
    Serial.print(n.kind == NOTE_FIRE ? F("FIRE ") : F("REL "));
    Serial.print(n.id); Serial.print(' '); Serial.println(n.t);
  }
}

void checkWatchdog() {
  if (inSafeHold) return;
  if (millis() - lastValidLineMs > LINK_TIMEOUT_MS) {
    releaseAllTransient();
    inSafeHold = true;
    emitLine(F("SAFE linkloss"));
  }
}

// ======================= setup / loop =======================

void setup() {
  Serial.begin(115200);
  for (uint8_t i = 0; i < N_CHANNELS; i++) {
    pinMode(CHANNELS[i].pin, OUTPUT);
    digitalWrite(CHANNELS[i].pin, LOW);
  }
  for (uint8_t i = 0; i < SLOTS; i++) events[i].state = SLOT_FREE;
  for (uint8_t i = 0; i < N_CHANNELS; i++) lastFireMs[i] = 0;

  setupSchedulerTimer();
  lastValidLineMs = millis();

  Serial.print(F("BOOT " FW_VERSION " "));
  Serial.println(PROTO_VERSION);
}

void loop() {
  readSerial();
  drainNotes();
  checkWatchdog();
}
