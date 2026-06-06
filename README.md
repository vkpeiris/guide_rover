

#define BUZZER  A4        
#define LED_RED A5
#define ALARM_INTERVAL 250

unsigned long alarmTimer = 0;
bool alarmOn = false;

void setup() {
  Serial.begin(9600);
  pinMode(BUZZER, OUTPUT);  digitalWrite(BUZZER, HIGH);
  pinMode(LED_RED, OUTPUT); digitalWrite(LED_RED, LOW);
  alarmTimer = millis();
  Serial.println("Evacuation alarm running (non-blocking)...");
}

void updateAlarm() {
  if (millis() - alarmTimer >= ALARM_INTERVAL) {
    alarmTimer = millis();
    alarmOn = !alarmOn;
    digitalWrite(LED_RED, alarmOn ? HIGH : LOW);
    digitalWrite(BUZZER,  alarmOn ? LOW  : HIGH);
  }
}

void loop() {
  updateAlarm();
  
}
