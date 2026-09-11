// analog_switch - drive four CD4066 channels as controllable button contacts
//
// Wiring (as built, first CD4066):
//   Nano 5V  -> 4066 pin 14 (Vdd)
//   Nano GND -> 4066 pin 7  (Vss)  and  -> controller battery -   (common ref)
//   10k pulldown on every control pin below (control -> 10k -> GND), so a
//   channel defaults OFF at boot / while the Nano is resetting.
//
//   Nano D2 -> 4066 pin 13 (control A)  -> THROTTLE_UP
//   Nano D3 -> 4066 pin 5  (control B)  -> THROTTLE_DOWN
//   Nano D4 -> 4066 pin 6  (control C)  -> YAW_LEFT
//   Nano D5 -> 4066 pin 12 (control D)  -> YAW_RIGHT
//   HIGH = switch closed. Each control pin's chip pin / pad pair is the
//   in/out for that switch (13->1/2, 5->3/4, 6->8/9, 12->10/11).
//
// Serial @ 115200, one char per command:
//   1   pulse THROTTLE_UP    (D2 / pin 13)
//   2   pulse THROTTLE_DOWN  (D3 / pin 5)
//   3   pulse YAW_LEFT       (D4 / pin 6)
//   4   pulse YAW_RIGHT      (D5 / pin 12)
//   ?   report resting state of all four control pins
//
// HOLD_MS below is the pulse length. Bring-up (Arduino/control-side wiring
// check, see docs/hardware_bringup.md): set it to 3000 so a multimeter can
// catch the 0V -> 5V -> 0V transition on each CD4066 control pin with no
// command active in between. Once all four pass, drop it to 150 (a real
// button tap) before testing actual controller responses.
//
// SAFETY: drone props OFF for any of this.

const unsigned long HOLD_MS = 3000;   // bring-up value; -> 150 for real use

const int THROTTLE_UP    = 2;
const int THROTTLE_DOWN  = 3;
const int YAW_LEFT       = 4;
const int YAW_RIGHT      = 5;

struct Chan { const char *name; int pin; };
const Chan CHANNELS[] = {
    { "THROTTLE_UP",   THROTTLE_UP },
    { "THROTTLE_DOWN", THROTTLE_DOWN },
    { "YAW_LEFT",      YAW_LEFT },
    { "YAW_RIGHT",     YAW_RIGHT },
};
const uint8_t N_CHANNELS = sizeof(CHANNELS) / sizeof(CHANNELS[0]);

void pulse(uint8_t i) {
    Serial.print(CHANNELS[i].name);
    Serial.println(" HIGH");
    digitalWrite(CHANNELS[i].pin, HIGH);
    delay(HOLD_MS);
    digitalWrite(CHANNELS[i].pin, LOW);
    Serial.print(CHANNELS[i].name);
    Serial.println(" LOW");
}

void reportState() {
    for (uint8_t i = 0; i < N_CHANNELS; i++) {
        Serial.print(CHANNELS[i].name);
        Serial.print(": ");
        Serial.println(digitalRead(CHANNELS[i].pin) ? "HIGH" : "LOW");
    }
}

void setup() {
    for (uint8_t i = 0; i < N_CHANNELS; i++) {
        pinMode(CHANNELS[i].pin, OUTPUT);
        digitalWrite(CHANNELS[i].pin, LOW);
    }
    Serial.begin(115200);
    Serial.println("ready: 1=THROTTLE_UP 2=THROTTLE_DOWN 3=YAW_LEFT 4=YAW_RIGHT ?=state");
    Serial.print("HOLD_MS=");
    Serial.println(HOLD_MS);
}

void loop() {
    if (!Serial.available()) return;
    char c = Serial.read();

    switch (c) {
        case '1': pulse(0); break;
        case '2': pulse(1); break;
        case '3': pulse(2); break;
        case '4': pulse(3); break;
        case '?': reportState(); break;
        case '\r': case '\n': case ' ':
            break;
        default:
            Serial.print("? unknown: ");
            Serial.println(c);
    }
}
