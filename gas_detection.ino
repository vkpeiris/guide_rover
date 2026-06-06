

#define MQ2_PIN   8        
#define BUZZER    A4        
#define LED_RED   A5
#define WARMUP_SEC 60     
bool armed = false;

void setup() {
  Serial.begin(9600);
  pinMode(MQ2_PIN, INPUT);
  pinMode(BUZZER, OUTPUT);  digitalWrite(BUZZER, HIGH);
  pinMode(LED_RED, OUTPUT); digitalWrite(LED_RED, LOW);

  Serial.println("MQ-2 warming up");
  for (int i = WARMUP_SEC; i > 0; i--) {
    Serial.print("armed in "); Serial.print(i); Serial.println("s");
    delay(1000);
  }
  armed = true;
  Serial.println(">>> ARMED — watching for gas <<<");
}

void loop() {
  bool gas = (digitalRead(MQ2_PIN) == LOW);
  if (armed && gas) {
    Serial.println("GAS DETECTED");
    digitalWrite(LED_RED, HIGH); digitalWrite(BUZZER, LOW);   
  } else {
    Serial.println("clean");
    digitalWrite(LED_RED, LOW);  digitalWrite(BUZZER, HIGH);  
  }
  delay(500);
}
