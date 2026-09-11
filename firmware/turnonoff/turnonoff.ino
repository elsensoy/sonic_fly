// Commands (one per line, 115200 baud):
//   p    press the power button (150ms pulse)

const int POWER = 2;

void setup() {
    pinMode(POWER, OUTPUT);
    digitalWrite(POWER, LOW);
    Serial.begin(115200);
    Serial.println(F("turnonoff ready"));
}

void loop() {
    if (Serial.available() && Serial.read() == 'p') {
        pressPower();
    }
}

void pressPower() {
    digitalWrite(POWER, HIGH);
    delay(150);
    digitalWrite(POWER, LOW);
    Serial.println(F("PULSE POWER 150ms"));
}