// ============================================================
//  G.U.I.D.E. — Status Signals (startup / honk / arrival)   [Faisal]
//  The one-shot LED + buzzer patterns the rover uses:
//   - armedBeep():  confirms warm-up finished / armed
//   - honkBurst():  rapid triple horn to warn a person in the path
//   - arrivalSignal(): celebration flash when the exit is reached
// ============================================================

#define BUZZER  A4        // active-LOW
#define LED_RED A5

void setup() {
  Serial.begin(9600);
  pinMode(BUZZER, OUTPUT);  digitalWrite(BUZZER, HIGH);
  pinMode(LED_RED, OUTPUT); digitalWrite(LED_RED, LOW);

  Serial.println("ARMED signal:");  armedBeep();
  delay(1000);
  Serial.println("HONK signal:");   honkBurst();
  delay(1000);
  Serial.println("ARRIVAL signal:"); arrivalSignal();
}

void loop() {
  // demo: replay the honk every few seconds
  honkBurst();
  delay(3000);
}

void armedBeep() {
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_RED, HIGH); digitalWrite(BUZZER, LOW);  delay(150);
    digitalWrite(LED_RED, LOW);  digitalWrite(BUZZER, HIGH); delay(150);
  }
}

void honkBurst() {
  for (int i = 0; i < 3; i++) {
    digitalWrite(BUZZER, LOW);  digitalWrite(LED_RED, HIGH); delay(60);
    digitalWrite(BUZZER, HIGH); digitalWrite(LED_RED, LOW);  delay(60);
  }
  delay(250);
}

void arrivalSignal() {
  for (int r = 0; r < 6; r++) {
    digitalWrite(LED_RED, HIGH); digitalWrite(BUZZER, LOW);  delay(80);
    digitalWrite(LED_RED, LOW);  digitalWrite(BUZZER, HIGH); delay(60);
  }
  digitalWrite(LED_RED, HIGH);   // solid = arrived
}
