/*
 * Audio-controlled dancing drone -- tone command receiver
 * --
 * Board:  Arduino Nano / Uno (ATmega328P)
 * Mic:    electret mic module on A0 (output biased ~Vcc/2)
 * Motors: 2 coreless motors via MOSFET or DRV8833 on D9 & D10.
 * Timers: Timer1 = motor PWM *and* the sample clock -- Fast PWM mode 14 with
 *         TOP = ICR1, overflowing at exactly SAMPLE_RATE Hz. Every overflow
 *         hardware-triggers the ADC, whose conversion-complete ISR fills a
 *         ping-pong buffer. Timer0 (millis/micros) and Timer2 are untouched.
 *
 * Protocol v0 -- single-tone commands, volume-independent detection.
 *   - Each command = one distinct frequency.
 *   - A "guard" tone arms the receiver; a move tone must follow within a
 *     short window. This is start-of-frame delimiter and it kills most
 *     false triggers from music/noise. Set REQUIRE_GUARD 0 to bypass it.
 *
 * This is deliberately a scaffold to iterate on. The comments flag every
 * spot worth experimenting with (marked  <-- TUNE  or  <-- ITERATE).
 */

//  toggles 
#define REQUIRE_GUARD 1        // 0 = move tones fire directly (easiest first test)
#define LOG_TELEMETRY 1        // 1 = stream one CSV row per block on Serial (115200) <-- TUNE
                              //     Costs ~5 ms/block at 115200 baud; set 0 for field runs.

//  audio / detection 
const int      MIC_PIN     = A0;
const uint16_t SAMPLE_RATE = 8000;   // Hz. Nyquist = 4000, so tones must stay < 4000
const uint16_t N           = 200;    // samples/block -> 25 ms window, 40 Hz bins

// Target tones (Hz). Kept high & well-spaced to dodge music energy (which
// piles up below ~1 kHz). Each is a multiple of the 40 Hz bin size so it
// lands exactly on a Goertzel bin.                                  <-ITERATE
const float   FREQS[]   = { 1600, 2000, 2400, 2800 };
const uint8_t NUM_FREQS = 4;
enum { TONE_GUARD = 0, TONE_SPIN = 1, TONE_BOB = 2, TONE_WIGGLE = 3 };

// Detection thresholds: this is where most of the tuning lives.
const float REL_THRESHOLD = 0.20;   // share of block energy the tone must own <-TUNE
const float MIN_ENERGY    = 200;    // absolute floor under the adaptive gate:
                                    // stops the detector going hypersensitive
                                    // in dead silence                        <-TUNE

//  adaptive noise floor 
// The energy gate is not a fixed number: it tracks a running estimate of the
// ambient noise so the detector self-calibrates to the room, the mic gain and
// the supply. The estimate is an exponential moving average (EMA) updated only
// on blocks that don't look like a command, and never while the motors run.
//
//   gate = max(MIN_ENERGY, noiseEnergy * ENERGY_MARGIN)
//
const float    ENERGY_MARGIN = 6.0;    // a tone must be this many x the noise floor <-TUNE
const float    NOISE_ALPHA   = 0.02;   // steady-state EMA weight (~1.5 s settle)     <--TUNE
const float    NOISE_OUTLIER = 8.0;    // blocks above noise*this are not learned     <-TUNE
const uint16_t PRIME_BLOCKS  = 40;     // startup window (~1.3 s): learn the floor
                                       //   fast, accept no commands yet             <-TUNE

float    noiseEnergy = 0;              // current ambient-noise energy estimate
uint16_t primeLeft   = PRIME_BLOCKS;   // blocks remaining in the startup window

//  motors 
const uint8_t MOTOR_A = 9;
const uint8_t MOTOR_B = 10;

//  move state machine 
enum Move { IDLE, SPIN, BOB, WIGGLE };
Move          currentMove = IDLE;
unsigned long moveStart   = 0;
const unsigned long MOVE_MS = 1200;  // how long a move plays before idling  <-- TUNE

//  Goertzel 
float   coeff[NUM_FREQS];
int16_t samples[N];

void computeCoeffs() {
  for (uint8_t i = 0; i < NUM_FREQS; i++) {
    int   k = (int)(0.5 + (N * FREQS[i]) / SAMPLE_RATE);   // nearest bin
    float w = 2.0 * PI * k / N;
    coeff[i] = 2.0 * cos(w);
  }
}

//  ISR sampling (Timer1-paced, ping-pong) 
// The ADC free-runs, hardware-triggered by Timer1 overflow at SAMPLE_RATE Hz.
// ADC_vect drops one sample into the buffer it is currently filling; when that
// buffer is full it hands the index to loop() via `readyBuf` and switches to
// the other buffer. If loop() has not released the previous block yet, the ISR
// discards the fresh one and bumps `dropped` -- it never writes the buffer
// loop() is reading, so a slow loop() costs whole blocks, never a torn window.
//
// Why no lock is needed (2 buffers):
//   - ISR writes only adcBuf[fillBuf]; loop() reads only adcBuf[readyBuf].
//   - While readyBuf >= 0 the ISR keeps fillBuf != readyBuf.
//   - readyBuf goes -1 -> 0/1 only in the ISR, 0/1 -> -1 only in loop();
//     it is one byte, so each write is atomic on AVR.
const uint16_t T1_TOP = (F_CPU / SAMPLE_RATE) - 1;   // 16e6/8000 - 1 = 1999

volatile int16_t  adcBuf[2][N];
volatile uint16_t fillPos  = 0;
volatile uint8_t  fillBuf  = 0;
volatile int8_t   readyBuf = -1;
volatile uint16_t dropped  = 0;    // running total of blocks loop() was too slow for

ISR(ADC_vect) {
  TIFR1 = _BV(TOV1);                        // re-arm the Timer1-overflow trigger
  adcBuf[fillBuf][fillPos++] = ADC;         // 10-bit result, 0..1023
  if (fillPos >= N) {
    fillPos = 0;
    if (readyBuf < 0) {
      readyBuf = fillBuf;                   // hand this block to loop()...
      fillBuf ^= 1;                         // ...and fill the other one next
    } else {
      dropped++;                            // loop() still busy -> refill in place
    }
  }
}

// Mic sits at mid-rail; subtract the block mean so we measure the AC signal.
void removeDC() {
  long sum = 0;
  for (uint16_t n = 0; n < N; n++) sum += samples[n];
  int16_t mean = sum / N;
  for (uint16_t n = 0; n < N; n++) samples[n] -= mean;
}

// One Goertzel pass -> |X_k|^2 (magnitude squared at that bin).
float goertzel(float c) {
  float s0, s1 = 0, s2 = 0;
  for (uint16_t n = 0; n < N; n++) {
    s0 = samples[n] + c * s1 - s2;
    s2 = s1;
    s1 = s0;
  }
  return s1 * s1 + s2 * s2 - c * s1 * s2;
}

// Total block energy (sum of squares) for volume-independent scoring.
float blockEnergy() {
  float e = 0;
  for (uint16_t n = 0; n < N; n++) e += (float)samples[n] * samples[n];
  return e;
}

// Per-block detector snapshot, kept around so loop() can log it after deciding.
struct Telemetry {
  float energy;              // sum of squares of the DC-removed block
  float ratio[NUM_FREQS];    // each bin's share of block energy (~0..0.5)
  int   best;                // bin holding the most power, before thresholding
  float bestRatio;           // ratio[best]
  int   tone;                // accepted tone index, or -1 if nothing confident
  uint16_t dropped;          // cumulative blocks lost because loop() fell behind
  float noise;               // ambient-noise energy estimate at decision time
  float gate;                // effective energy threshold this block
};
Telemetry tlm;

// Returns the detected tone index, or -1 if nothing confident, and fills `tlm`.
// Confidence = share of total energy sitting in the winning tone's bin.
// (Parseval: sum of all |X_k|^2 = N * energy, so this ratio is ~0..0.5.)
int detectTone() {
  tlm.energy    = blockEnergy();
  tlm.best      = -1;
  tlm.bestRatio = 0;

  float bestPow = 0;
  for (uint8_t i = 0; i < NUM_FREQS; i++) {
    float p = goertzel(coeff[i]);
    tlm.ratio[i] = (tlm.energy > 0) ? p / (N * tlm.energy) : 0;
    if (p > bestPow) { bestPow = p; tlm.best = i; }
  }
  if (tlm.best >= 0) tlm.bestRatio = tlm.ratio[tlm.best];

  float gate = noiseEnergy * ENERGY_MARGIN;
  if (gate < MIN_ENERGY) gate = MIN_ENERGY;
  tlm.noise = noiseEnergy;
  tlm.gate  = gate;

  tlm.tone = -1;
  if (primeLeft == 0 && tlm.best >= 0 &&
      tlm.energy >= gate && tlm.bestRatio > REL_THRESHOLD)
    tlm.tone = tlm.best;

  updateNoise();          // fold this block into the estimate (see below)
  return tlm.tone;
}

// EMA update for the ambient-noise estimate. Seeded from the first block so it
// starts in the right ballpark; adapts fast during the startup window, then
// slowly. After startup it learns only from blocks that are (a) not an accepted
// command, (b) not taken while the motors run, and (c) not a wild outlier -- so
// neither a sustained tone nor a one-off clap/slam can inflate the floor and
// blind the detector for seconds. Steady ambient changes (a fan switching on)
// stay under NOISE_OUTLIER and are tracked.
void updateNoise() {
  if (primeLeft == PRIME_BLOCKS) noiseEnergy = tlm.energy;   // first block: seed

  if (primeLeft > 0) {                              // fast learn at power-on
    noiseEnergy += (tlm.energy - noiseEnergy) * 0.25;
    primeLeft--;
    return;
  }
  if (tlm.tone >= 0 || currentMove != IDLE) return;          // command / motion
  if (tlm.energy > noiseEnergy * NOISE_OUTLIER) return;      // impulsive transient
  noiseEnergy += (tlm.energy - noiseEnergy) * NOISE_ALPHA;   // slow steady tracking
}

//  motors / moves 
// Timer1 is in Fast PWM mode 14 (see setupTimer1), so the motor duty lives in
// OCR1A (D9) / OCR1B (D10) directly -- analogWrite() would fight that config.
// Map the familiar 0..255 range onto 0..T1_TOP; called rarely, so the divide
// is free. Fast PWM can't emit a true 0% (OCR=0 still spikes one clock per
// period), so a zero channel is disconnected from the timer and driven low --
// exactly what the core's analogWrite(pin, 0) does.
void setMotors(uint8_t a, uint8_t b) {
  if (a == 0) { TCCR1A &= ~_BV(COM1A1); digitalWrite(MOTOR_A, LOW); }
  else        { TCCR1A |=  _BV(COM1A1); OCR1A = ((uint32_t)a * T1_TOP + 127) / 255; }
  if (b == 0) { TCCR1A &= ~_BV(COM1B1); digitalWrite(MOTOR_B, LOW); }
  else        { TCCR1A |=  _BV(COM1B1); OCR1B = ((uint32_t)b * T1_TOP + 127) / 255; }
}

void startMove(Move m) {
  currentMove = m;
  moveStart   = millis();
}

// Non-blocking: recomputed every loop so detection keeps running underneath.
// These patterns assume a pivoted/tethered 2-motor rig.             <-- ITERATE
void updateMotors() {
  unsigned long t = millis() - moveStart;
  if (currentMove != IDLE && t > MOVE_MS) currentMove = IDLE;

  switch (currentMove) {
    case SPIN:                       // one motor high -> rig spins on its pivot
      setMotors(220, 60);
      break;
    case BOB: {                      // both pulse together -> bobs up & down
      uint8_t p = 120 + 100 * sin(t / 120.0);
      setMotors(p, p);
      break;
    }
    case WIGGLE: {                   // motors alternate -> tilts side to side
      uint8_t p = 120 + 100 * sin(t / 100.0);
      setMotors(p, 240 - p);
      break;
    }
    default:                         // IDLE
      setMotors(0, 0);
      break;
  }
}

//  arming / framing 
bool          armed   = false;
unsigned long armedAt = 0;
const unsigned long ARM_WINDOW = 1500;   // ms to send a move after the guard <-- TUNE

// One CSV row per block: raw energy, every bin ratio, the winner, what was
// accepted, and the current framing/motion state. Import straight into a
// spreadsheet or pandas to tune REL_THRESHOLD / MIN_ENERGY and to see how
// music, speech and motor noise actually score.                      <-- ITERATE
#if LOG_TELEMETRY
void logTelemetry() {
  Serial.print(millis());       Serial.print(',');
  Serial.print(tlm.energy, 0);  Serial.print(',');
  for (uint8_t i = 0; i < NUM_FREQS; i++) { Serial.print(tlm.ratio[i], 3); Serial.print(','); }
  Serial.print(tlm.best);       Serial.print(',');
  Serial.print(tlm.tone);       Serial.print(',');
  Serial.print(armed);          Serial.print(',');
  Serial.print((int)currentMove); Serial.print(',');
  Serial.print(tlm.dropped);      Serial.print(',');
  Serial.print(tlm.noise, 0);     Serial.print(',');
  Serial.println(tlm.gate, 0);
}
#endif

// Timer1: Fast PWM mode 14 (TOP = ICR1), non-inverting PWM on OC1A/OC1B,
// prescaler /1. Overflow rate = F_CPU / (T1_TOP + 1) = SAMPLE_RATE, and that
// same overflow is the ADC trigger. The PWM carrier also lands at SAMPLE_RATE
// (8 kHz) -- above the motors' mechanical bandwidth, and trivial for a MOSFET
// or the DRV8833 to switch.
void setupTimer1() {
  pinMode(MOTOR_A, OUTPUT);           // OC1A / D9
  pinMode(MOTOR_B, OUTPUT);           // OC1B / D10
  uint8_t s = SREG;
  cli();                              // 16-bit register writes -> keep them atomic
  TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11);
  TCCR1B = _BV(WGM13)  | _BV(WGM12)  | _BV(CS10);
  ICR1   = T1_TOP;
  OCR1A  = 0;
  OCR1B  = 0;
  TCNT1  = 0;
  SREG   = s;
}

// ADC: AVcc reference, right-adjusted, channel = MIC_PIN, prescaler /16
// (~13 us/conversion -- tiny next to the 125 us sample period). Auto-triggered
// by Timer1 overflow; ADC_vect clears TOV1 to re-arm the edge. Bump to /32
// ( _BV(ADPS2) | _BV(ADPS0) ) for cleaner low bits if the noise floor is high --
// still ~5x margin.                                                  <-- TUNE
void setupADC() {
  ADMUX  = _BV(REFS0) | ((MIC_PIN - A0) & 0x07);
  ADCSRB = _BV(ADTS2) | _BV(ADTS1);        // trigger source: Timer1 overflow (0b110)
  TIFR1  = _BV(TOV1);                       // drop any stale overflow flag first
  ADCSRA = _BV(ADEN) | _BV(ADSC) | _BV(ADATE) | _BV(ADIE) | _BV(ADPS2);
}

void setup() {
  Serial.begin(115200);
  computeCoeffs();
  setupTimer1();
  setupADC();

#if LOG_TELEMETRY
  Serial.println(F("ms,energy,r1600,r2000,r2400,r2800,best,tone,armed,move,dropped,noise,gate"));
#endif
}

void loop() {
  // Wait for the ISR to hand over a full block, keeping the motors updated so
  // motion stays smooth during the wait.
  while (readyBuf < 0) updateMotors();
  uint8_t b = readyBuf;

  // No lock: the ISR is filling the other buffer and won't touch adcBuf[b]
  // until we set readyBuf back to -1 below.
  for (uint16_t n = 0; n < N; n++) samples[n] = adcBuf[b][n];

  noInterrupts();
  tlm.dropped = dropped;      // atomic 16-bit read
  interrupts();

  readyBuf = -1;              // release the buffer back to the ISR

  removeDC();
  int tone = detectTone();

  if (tone == TONE_GUARD) {
    armed   = true;
    armedAt = millis();
  } else if (tone >= 1) {
    bool ok;
#if REQUIRE_GUARD
    ok = armed && (millis() - armedAt) < ARM_WINDOW;
#else
    ok = true;
#endif
    if (ok) {
      switch (tone) {
        case TONE_SPIN:   startMove(SPIN);   break;
        case TONE_BOB:    startMove(BOB);    break;
        case TONE_WIGGLE: startMove(WIGGLE); break;
      }
      armed = false;   // consume the arm
    }
  }

  updateMotors();

#if LOG_TELEMETRY
  logTelemetry();
#endif
}
