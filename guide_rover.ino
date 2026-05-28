
//  G.U.I.D.E.-Obstacle Avoidance (v10)
//
//  All fixes:
//
//  STALL DETECTION (3 layers):
//  1. EEPROM count-survives Arduino resets/brownouts
//  2. lastFreeTime timer-triggers escape if sensors have
//     been blocked for too long
//  3. Obstacle mode timer-Triggers escape if rover
//     has been in obstacle handling for too long even if
//     sensors read clear (rover stuck but sensors not covered)
//
//  BLIND SPOT FIX (better than v9):
//  Only substitutes last reading if it was < 25cm AND < 500ms
//  ago. Stops ghost obstacles from sensor noise.
//
//  REVERSE FIX:
//  Shorter reverse bursts so rover can't travel far enough
//  to hit back walls.
//
//  NUDGE FIX (from v9):
//  Requires 60cm clear before nudging forward after a pivot.
//  Stops the stop move stop loop.


#include <EEPROM.h>

#define IN1 2
#define IN2 3
#define IN3 4
#define IN4 7
#define ENA 5
#define ENB 6

#define TRIG_F  9
#define ECHO_F 10
#define TRIG_L 11
#define ECHO_L 12
#define TRIG_R 13
#define ECHO_R A0

#define BUZZER  A4
#define LED_RED A5

#define FRONT_DIST      35
#define SIDE_DIST       30
#define SIDE_CLOSE      15
#define CONTACT_DIST     5
#define BLIND_DIST      25
#define BLIND_WINDOW   500
#define MOTOR_SPEED    120
#define TURN_SPEED     255
#define CRAWL_SPEED     80
#define STEER_SLOW      35
#define REVERSE_SHORT  300
#define REVERSE_LONG   550
#define MAX_TURN_MS   3000
#define NUDGE_TIME     180
#define READINGS         5
#define STUCK_LIMIT      3
#define EEPROM_ADDR      0
#define FREE_TIMEOUT  4000
#define OBS_TIMEOUT   7000

int  obstacleCount    = 0;
bool wasEverFree      = false;
bool inObstacleMode   = false;
unsigned long lastFreeTime      = 0;
unsigned long obstacleModeStart = 0;

long lastF = 999, lastL = 999, lastR = 999;
unsigned long lastFTime = 0, lastLTime = 0, lastRTime = 0;

void setup() {
  delay(2000);
  Serial.begin(9600);

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);

  pinMode(TRIG_F, OUTPUT); pinMode(ECHO_F, INPUT);
  pinMode(TRIG_L, OUTPUT); pinMode(ECHO_L, INPUT);
  pinMode(TRIG_R, OUTPUT); pinMode(ECHO_R, INPUT);

  pinMode(BUZZER,  OUTPUT); digitalWrite(BUZZER, HIGH);
  pinMode(LED_RED, OUTPUT); digitalWrite(LED_RED, LOW);

  stopMotors();

  obstacleCount = EEPROM.read(EEPROM_ADDR);
  if (obstacleCount == 255) obstacleCount = 0;
  Serial.print("EEPROM count: "); Serial.println(obstacleCount);

  startupBeep();

  Serial.println("Clean start check...");
  unsigned long bootCheck = millis();
  bool cleanStart = true;
  while (millis() - bootCheck < 2000) {
    long F = getMinDist(TRIG_F, ECHO_F); delay(10);
    long L = getMinDist(TRIG_L, ECHO_L); delay(10);
    long R = getMinDist(TRIG_R, ECHO_R);
    if (F <= FRONT_DIST || L <= SIDE_DIST || R <= SIDE_DIST) {
      cleanStart = false; break;
    }
  }

  if (cleanStart) {
    obstacleCount = 0;
    EEPROM.update(EEPROM_ADDR, 0);
    Serial.println("Clean start — count cleared.");
  } else {
    Serial.println("Obstacle at boot — keeping count.");
  }

  lastFreeTime = millis();
  Serial.println("G.U.I.D.E. v10 ready.");
}

void saveCount(int val) {
  obstacleCount = val;
  EEPROM.update(EEPROM_ADDR, val);
}

void loop() {
  long F = getSmartDist(TRIG_F, ECHO_F, lastF, lastFTime); delay(10);
  long L = getSmartDist(TRIG_L, ECHO_L, lastL, lastLTime); delay(10);
  long R = getSmartDist(TRIG_R, ECHO_R, lastR, lastRTime);

  Serial.print("F:"); Serial.print(F);
  Serial.print("  L:"); Serial.print(L);
  Serial.print("  R:"); Serial.println(R);

  if (F <= CONTACT_DIST || L <= CONTACT_DIST || R <= CONTACT_DIST) {
    Serial.println("!! CONTACT — emergency reverse");
    stopMotors(); delay(50);
    reverse(); delay(REVERSE_SHORT);
    stopMotors(); delay(150);
    saveCount(STUCK_LIMIT);
    inObstacleMode = false;
    return;
  }

  if (wasEverFree && millis() - lastFreeTime > FREE_TIMEOUT) {
    Serial.println("!! FREE TIMEOUT — escape");
    stopMotors();
    escapeManoeuvre();
    lastFreeTime = millis();
    saveCount(0);
    inObstacleMode = false;
    return;
  }

  if (inObstacleMode && millis() - obstacleModeStart > OBS_TIMEOUT) {
    Serial.println("!! OBS TIMEOUT — rover likely stuck, escape");
    stopMotors();
    escapeManoeuvre();
    saveCount(0);
    inObstacleMode = false;
    lastFreeTime = millis();
    return;
  }

  if (F <= FRONT_DIST) {
    stopMotors();
    alertBeep();
    saveCount(obstacleCount + 1);

    if (!inObstacleMode) {
      inObstacleMode = true;
      obstacleModeStart = millis();
    }

    Serial.print(">> Front blocked #"); Serial.println(obstacleCount);

    if (obstacleCount >= STUCK_LIMIT) {
      Serial.println(">> Hit limit — escape");
      escapeManoeuvre();
      saveCount(0);
      inObstacleMode = false;
      lastFreeTime = millis();
      return;
    }

    delay(80);
    F = getMinDist(TRIG_F, ECHO_F); delay(10);
    L = getMinDist(TRIG_L, ECHO_L); delay(10);
    R = getMinDist(TRIG_R, ECHO_R);

    int revTime = (obstacleCount >= 2) ? REVERSE_LONG : REVERSE_SHORT;

    if ((L <= SIDE_DIST && R <= SIDE_DIST) || obstacleCount >= 2) {
      Serial.println(">> Reversing");
      reverse(); delay(revTime); stopMotors(); delay(100);
      L = getMinDist(TRIG_L, ECHO_L); delay(10);
      R = getMinDist(TRIG_R, ECHO_R);
    }

    bool goRight = (R >= L);
    bool cleared = pivotUntilClear(goRight);

    if (!cleared) {
      Serial.println(">> Pivot failed");
      saveCount(STUCK_LIMIT - 1);
      return;
    }

    delay(50);
    if (getMinDist(TRIG_F, ECHO_F) > FRONT_DIST + 25) {
      moveForward(); delay(NUDGE_TIME); stopMotors();
    }

    alertOff();
    return;
  }

  if (L <= SIDE_DIST && R <= SIDE_DIST) {
    if (!inObstacleMode) { inObstacleMode = true; obstacleModeStart = millis(); }
    alertOn();
    setMotors(CRAWL_SPEED, CRAWL_SPEED);
    delay(30);
    return;
  }

  if (L <= SIDE_DIST) {
    if (!inObstacleMode) { inObstacleMode = true; obstacleModeStart = millis(); }
    alertOn();
    if (L <= SIDE_CLOSE) {
      doTurnRight(); delay(180); stopMotors(); delay(40);
    } else {
      setMotors(STEER_SLOW, MOTOR_SPEED); delay(30);
    }
    return;
  }

  if (R <= SIDE_DIST) {
    if (!inObstacleMode) { inObstacleMode = true; obstacleModeStart = millis(); }
    alertOn();
    if (R <= SIDE_CLOSE) {
      doTurnLeft(); delay(180); stopMotors(); delay(40);
    } else {
      setMotors(MOTOR_SPEED, STEER_SLOW); delay(30);
    }
    return;
  }

  lastFreeTime   = millis();
  wasEverFree    = true;
  inObstacleMode = false;
  if (obstacleCount > 0) saveCount(0);

  alertOff();
  setMotors(MOTOR_SPEED, MOTOR_SPEED);
  delay(30);
}

bool pivotUntilClear(bool right) {
  unsigned long t = millis();
  while (millis() - t < MAX_TURN_MS) {
    if (allClear()) { stopMotors(); return true; }
    right ? doTurnRight() : doTurnLeft();
    delay(20);
  }
  stopMotors(); delay(150);
  t = millis();
  while (millis() - t < MAX_TURN_MS) {
    if (allClear()) { stopMotors(); return true; }
    right ? doTurnLeft() : doTurnRight();
    delay(20);
  }
  stopMotors();
  return false;
}

bool allClear() {
  long F = getMinDist(TRIG_F, ECHO_F); delay(10);
  long L = getMinDist(TRIG_L, ECHO_L); delay(10);
  long R = getMinDist(TRIG_R, ECHO_R);
  return F > FRONT_DIST + 10 && L > SIDE_DIST + 8 && R > SIDE_DIST + 8;
}

void escapeManoeuvre() {
  Serial.println(">> ESCAPE");
  digitalWrite(LED_RED, HIGH);
  reverse(); delay(700); stopMotors(); delay(200);

  unsigned long t = millis();
  while (millis() - t < 3000) {
    doTurnRight(); delay(20);
    long F = getMinDist(TRIG_F, ECHO_F); delay(10);
    long L = getMinDist(TRIG_L, ECHO_L); delay(10);
    long R = getMinDist(TRIG_R, ECHO_R);
    if (F > FRONT_DIST + 20 && L > SIDE_DIST && R > SIDE_DIST) {
      stopMotors(); digitalWrite(LED_RED, LOW); return;
    }
  }

  stopMotors(); delay(150);
  t = millis();
  while (millis() - t < 3000) {
    doTurnLeft(); delay(20);
    long F = getMinDist(TRIG_F, ECHO_F); delay(10);
    long L = getMinDist(TRIG_L, ECHO_L); delay(10);
    long R = getMinDist(TRIG_R, ECHO_R);
    if (F > FRONT_DIST + 20 && L > SIDE_DIST && R > SIDE_DIST) {
      stopMotors(); digitalWrite(LED_RED, LOW); return;
    }
  }

  stopMotors();
  for (int i = 0; i < 10; i++) {
    digitalWrite(LED_RED, !digitalRead(LED_RED));
    delay(150);
  }
}

long getSmartDist(int trig, int echo, long &lastVal, unsigned long &lastTime) {
  long d = getMinDist(trig, echo);
  unsigned long now = millis();
  if (d == 999 && lastVal < BLIND_DIST && (now - lastTime) < BLIND_WINDOW) {
    return lastVal;
  }
  if (d != 999) {
    lastVal  = d;
    lastTime = now;
  }
  return d;
}

long getMinDist(int trig, int echo) {
  long minVal = 999;
  for (int i = 0; i < READINGS; i++) {
    long d = readOnce(trig, echo);
    if (d < minVal) minVal = d;
    delay(2);
  }
  return minVal;
}

long readOnce(int trig, int echo) {
  digitalWrite(trig, LOW);  delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long dur = pulseIn(echo, HIGH, 15000UL);
  if (dur == 0) return 999;
  return dur / 58;
}

void setMotors(int l, int r) {
  analogWrite(ENA, l); analogWrite(ENB, r);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
}

void moveForward() { setMotors(MOTOR_SPEED, MOTOR_SPEED); }

void reverse() {
  analogWrite(ENA, MOTOR_SPEED); analogWrite(ENB, MOTOR_SPEED);
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

void doTurnRight() {
  analogWrite(ENA, TURN_SPEED); analogWrite(ENB, TURN_SPEED);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

void doTurnLeft() {
  analogWrite(ENA, TURN_SPEED); analogWrite(ENB, TURN_SPEED);
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
}

void stopMotors() {
  analogWrite(ENA, 0); analogWrite(ENB, 0);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

void alertOn()  { digitalWrite(LED_RED, HIGH); digitalWrite(BUZZER, LOW);  }
void alertOff() { digitalWrite(LED_RED, LOW);  digitalWrite(BUZZER, HIGH); }

void alertBeep() {
  digitalWrite(LED_RED, HIGH);
  digitalWrite(BUZZER, LOW);  delay(100);
  digitalWrite(BUZZER, HIGH); delay(60);
  digitalWrite(LED_RED, LOW);
}

void startupBeep() {
  for (int i = 0; i < 2; i++) {
    digitalWrite(LED_RED, HIGH);
    digitalWrite(BUZZER, LOW);  delay(150);
    digitalWrite(BUZZER, HIGH); delay(150);
    digitalWrite(LED_RED, LOW); delay(100);
  }
}
