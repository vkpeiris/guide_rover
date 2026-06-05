// ============================================================
//  G.U.I.D.E. — Temperature Detection (DHT11)   [Yitchen]
//  Standalone: reads temperature and flags when it exceeds a
//  threshold (the "heat / fire" trigger for the rover).
//  Needs libraries: "DHT sensor library" + "Adafruit Unified Sensor".
// ============================================================

#include <DHT.h>

#define DHT_PIN   A3
#define DHT_TYPE  DHT11
#define TEMP_THRESHOLD 40.0   // deg C that counts as high/fire. CHANGE ME.
#define BUZZER    A4          // active-LOW
#define LED_RED   A5

DHT dht(DHT_PIN, DHT_TYPE);

void setup() {
  Serial.begin(9600);
  dht.begin();
  pinMode(BUZZER, OUTPUT);  digitalWrite(BUZZER, HIGH);
  pinMode(LED_RED, OUTPUT); digitalWrite(LED_RED, LOW);
  Serial.print("Temp threshold: "); Serial.print(TEMP_THRESHOLD); Serial.println(" C");
  delay(2000);   // DHT11 settle
}

void loop() {
  float temp = dht.readTemperature();   // deg C
  if (isnan(temp)) {
    Serial.println("Read failed — check wiring");
    delay(1000);
    return;
  }

  Serial.print("Temp: "); Serial.print(temp, 1); Serial.print(" C");

  if (temp >= TEMP_THRESHOLD) {
    Serial.println("   >>> HIGH TEMP DETECTED <<<");
    digitalWrite(LED_RED, HIGH); digitalWrite(BUZZER, LOW);
  } else {
    Serial.println("   ...normal");
    digitalWrite(LED_RED, LOW);  digitalWrite(BUZZER, HIGH);
  }
  delay(2000);   // DHT11 min sample rate ~1/sec
}
