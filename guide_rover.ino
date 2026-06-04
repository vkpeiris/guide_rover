#include <EEPROM.h>
#include <DHT.h>

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

#define MQ2_PIN   8
#define DHT_PIN   A3
#define DHT_TYPE  DHT11
#define TEMP_THRESHOLD 40.0
#define WARMUP_SEC     60
DHT dht(DHT_PIN, DHT_TYPE);

#define STATE_ROAM 0
#define STATE_EVAC 1
int state = STATE_ROAM;

#define MOTOR_SPEED    120
#define TURN_SPEED     255
#define STEER_SLOW      35
#define REVERSE_SHORT  300
#define REVERSE_LONG   550
#define READINGS         3
#define CONTACT_DIST     5
#define OBS_TIMEOUT   7000

#define FRONT_DIST      35
#define ROAM_SIDE_DIST  30
#define ROAM_SIDE_CLOSE 15
#define BLIND_DIST      25
#define BLIND_WINDOW   500
#define ROAM_CRAWL_SPEED 80
#define MAX_TURN_MS   3000
#define NUDGE_TIME     180
#define STUCK_LIMIT      3
#define EEPROM_ADDR      0
#define FREE_TIMEOUT  4000
#define HANG_TIMEOUT  4000
#define HANG_DELTA       3

#define LINE_LEFT  A1
#define LINE_RIGHT A2
#define LEFT_THRESHOLD  448
#define RIGHT_THRESHOLD 457
#define BASE_SPEED    110
#define TURN_OUTER    140
#define TURN_INNER    110
#define PIVOT_SPEED   200
#define SEARCH_SPEED  120
#define CRAWL_SPEED    95
#define SEARCH_FRONT  30
#define FOLLOW_FRONT  20
#define SIDE_DIST     25
#define SIDE_CLOSE    12
#define ALARM_INTERVAL 250
#define LOST_TIME      120
#define HONK_TIME     6000
#define SIDE_ESC_MS   2500
#define KICK_SPEED    200
#define KICK_TIME     150
#define WANDER_BIAS    30
#define WANDER_MIN    700
#define WANDER_MAX   1600
#define EXIT_COUNT      10
#define CREEP_KICK     200
#define CREEP_KICK_MS   60
#define CREEP_SPEED     85
#define CREEP_SAMPLE    15
#define CREEP_TIMEOUT 3000
#define PHASE_SEARCH 0
#define PHASE_FOLLOW 1

int  roam_obstacleCount   = 0;
bool roam_wasEverFree     = false;
bool roam_inObstacleMode  = false;
unsigned long roam_lastFreeTime      = 0;
unsigned long roam_obstacleModeStart = 0;
long roam_lastF = 999, roam_lastL = 999, roam_lastR = 999;
unsigned long roam_lastFTime = 0, roam_lastLTime = 0, roam_lastRTime = 0;
long roam_hangRefDist = 999;
unsigned long roam_hangTimer = 0;

int phase = PHASE_SEARCH;
unsigned long alarmTimer = 0;
bool alarmOn = false;
unsigned long curveStart   = 0;
unsigned long bothOffStart = 0;
int lastDir = 0;
bool inObstacleMode = false;
unsigned long obstacleModeStart = 0;
int wanderState = 0;
unsigned long wanderTimer = 0;
unsigned long wanderInterval = 1000;

void setup() {
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(TRIG_F, OUTPUT); pinMode(ECHO_F, INPUT);
  pinMode(TRIG_L, OUTPUT); pinMode(ECHO_L, INPUT);
  pinMode(TRIG_R, OUTPUT); pinMode(ECHO_R, INPUT);
  pinMode(BUZZER,  OUTPUT); digitalWrite(BUZZER, HIGH);
  pinMode(LED_RED, OUTPUT); digitalWrite(LED_RED, LOW);
  pinMode(MQ2_PIN, INPUT);
  stopMotors();
  dht.begin();

  roam_obstacleCount = EEPROM.read(EEPROM_ADDR);
  if (roam_obstacleCount == 255) roam_obstacleCount = 0;

  unsigned long wt = millis();
  while (millis() - wt < (unsigned long)WARMUP_SEC * 1000UL) {
    for (int i = 0; i < 3; i++) {
      digitalWrite(LED_RED, HIGH); delay(120);
      digitalWrite(LED_RED, LOW);  delay(120);
    }
    delay(500);
    long left = ((long)WARMUP_SEC * 1000L - (long)(millis() - wt)) / 1000L;
  }
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_RED, HIGH); digitalWrite(BUZZER, LOW);  delay(150);
    digitalWrite(LED_RED, LOW);  digitalWrite(BUZZER, HIGH); delay(150);
  }

  unsigned long bootCheck = millis();
  bool cleanStart = true;
  while (millis() - bootCheck < 2000) {
    long F = roam_getMinDist(TRIG_F, ECHO_F); delay(10);
    long L = roam_getMinDist(TRIG_L, ECHO_L); delay(10);
    long R = roam_getMinDist(TRIG_R, ECHO_R);
    if (F <= FRONT_DIST || L <= ROAM_SIDE_DIST || R <= ROAM_SIDE_DIST) { cleanStart = false; break; }
  }
  if (cleanStart) { roam_obstacleCount = 0; EEPROM.update(EEPROM_ADDR, 0); }

  roam_lastFreeTime = millis();
  roam_hangTimer    = millis();
  state = STATE_ROAM;
}

void loop() {
  if (state == STATE_ROAM) {
    if (dangerDetected()) { startEvac(); return; }
    roam_step();
    return;
  }
  updateAlarm();
  if (phase == PHASE_SEARCH) phaseA_search();
  else                       phaseB_follow();
}

bool dangerDetected() {
  if (digitalRead(MQ2_PIN) == LOW) { return true; }
  static unsigned long lastDht = 0;
  static float lastTemp = 0;
  if (millis() - lastDht > 1200) {
    lastDht = millis();
    float t = dht.readTemperature();
    if (!isnan(t)) lastTemp = t;
  }
  if (lastTemp >= TEMP_THRESHOLD) { return true; }
  return false;
}

void startEvac() {
  stopMotors();
  phase = PHASE_SEARCH;
  randomSeed(micros());
  unsigned long now = millis();
  alarmTimer = now; curveStart = now; wanderTimer = now;
  alarmOn = false; bothOffStart = 0; lastDir = 0; inObstacleMode = false;
  setMotors(KICK_SPEED, KICK_SPEED);
  delayAlarm(KICK_TIME);
  state = STATE_EVAC;
}

void roam_saveCount(int val) {
  roam_obstacleCount = val;
  EEPROM.update(EEPROM_ADDR, val);
}

void roam_step() {
  long F = roam_getSmartDist(TRIG_F, ECHO_F, roam_lastF, roam_lastFTime); delay(10);
  long L = roam_getSmartDist(TRIG_L, ECHO_L, roam_lastL, roam_lastLTime); delay(10);
  long R = roam_getSmartDist(TRIG_R, ECHO_R, roam_lastR, roam_lastRTime);

  if (F <= CONTACT_DIST || L <= CONTACT_DIST || R <= CONTACT_DIST) {
    stopMotors(); delay(50);
    reverse(); delay(REVERSE_SHORT);
    stopMotors(); delay(150);
    roam_saveCount(STUCK_LIMIT);
    roam_inObstacleMode = false;
    return;
  }

  if (roam_wasEverFree && millis() - roam_lastFreeTime > FREE_TIMEOUT) {
    stopMotors();
    roam_escapeManoeuvre();
    roam_lastFreeTime = millis();
    roam_saveCount(0);
    roam_inObstacleMode = false;
    return;
  }

  if (roam_inObstacleMode && millis() - roam_obstacleModeStart > OBS_TIMEOUT) {
    stopMotors();
    roam_escapeManoeuvre();
    roam_saveCount(0);
    roam_inObstacleMode = false;
    roam_lastFreeTime = millis();
    return;
  }

  if (F <= FRONT_DIST) {
    stopMotors();
    roam_alertBeep();
    roam_saveCount(roam_obstacleCount + 1);

    if (!roam_inObstacleMode) {
      roam_inObstacleMode = true;
      roam_obstacleModeStart = millis();
    }

    if (roam_obstacleCount >= STUCK_LIMIT) {
      roam_escapeManoeuvre();
      roam_saveCount(0);
      roam_inObstacleMode = false;
      roam_lastFreeTime = millis();
      return;
    }

    delay(80);
    F = roam_getMinDist(TRIG_F, ECHO_F); delay(10);
    L = roam_getMinDist(TRIG_L, ECHO_L); delay(10);
    R = roam_getMinDist(TRIG_R, ECHO_R);

    int revTime = (roam_obstacleCount >= 2) ? REVERSE_LONG : REVERSE_SHORT;

    if ((L <= ROAM_SIDE_DIST && R <= ROAM_SIDE_DIST) || roam_obstacleCount >= 2) {
      reverse(); delay(revTime); stopMotors(); delay(100);
      L = roam_getMinDist(TRIG_L, ECHO_L); delay(10);
      R = roam_getMinDist(TRIG_R, ECHO_R);
    }

    bool goRight = (R >= L);
    bool cleared = roam_pivotUntilClear(goRight);

    if (!cleared) {
      roam_saveCount(STUCK_LIMIT - 1);
      return;
    }

    delay(50);
    if (roam_getMinDist(TRIG_F, ECHO_F) > FRONT_DIST + 25) {
      moveForward(); delay(NUDGE_TIME); stopMotors();
    }

    roam_alertOff();
    return;
  }

  if (L <= ROAM_SIDE_DIST && R <= ROAM_SIDE_DIST) {
    if (!roam_inObstacleMode) { roam_inObstacleMode = true; roam_obstacleModeStart = millis(); }
    roam_alertOn();
    setMotors(ROAM_CRAWL_SPEED, ROAM_CRAWL_SPEED);
    delay(30);
    return;
  }

  if (L <= ROAM_SIDE_DIST) {
    if (!roam_inObstacleMode) { roam_inObstacleMode = true; roam_obstacleModeStart = millis(); }
    roam_alertOn();
    if (L <= ROAM_SIDE_CLOSE) {
      doTurnRight(); delay(180); stopMotors(); delay(40);
    } else {
      setMotors(STEER_SLOW, MOTOR_SPEED); delay(30);
    }
    return;
  }

  if (R <= ROAM_SIDE_DIST) {
    if (!roam_inObstacleMode) { roam_inObstacleMode = true; roam_obstacleModeStart = millis(); }
    roam_alertOn();
    if (R <= ROAM_SIDE_CLOSE) {
      doTurnLeft(); delay(180); stopMotors(); delay(40);
    } else {
      setMotors(MOTOR_SPEED, STEER_SLOW); delay(30);
    }
    return;
  }

  roam_lastFreeTime  = millis();
  roam_wasEverFree   = true;
  roam_inObstacleMode = false;
  if (roam_obstacleCount > 0) roam_saveCount(0);

  if (abs(F - roam_hangRefDist) > HANG_DELTA) {
    roam_hangRefDist = F;
    roam_hangTimer   = millis();
  } else if (millis() - roam_hangTimer > HANG_TIMEOUT) {
    stopMotors();
    reverse(); delay(REVERSE_LONG); stopMotors(); delay(100);
    long rr = roam_getMinDist(TRIG_R, ECHO_R);
    long ll = roam_getMinDist(TRIG_L, ECHO_L);
    roam_pivotUntilClear(rr >= ll);
    roam_hangRefDist = 999;
    roam_hangTimer   = millis();
    return;
  }

  roam_alertOff();
  setMotors(MOTOR_SPEED, MOTOR_SPEED);
  delay(30);
}

bool roam_pivotUntilClear(bool right) {
  unsigned long t = millis();
  while (millis() - t < MAX_TURN_MS) {
    if (roam_allClear()) { stopMotors(); return true; }
    right ? doTurnRight() : doTurnLeft();
    delay(20);
  }
  stopMotors(); delay(150);
  t = millis();
  while (millis() - t < MAX_TURN_MS) {
    if (roam_allClear()) { stopMotors(); return true; }
    right ? doTurnLeft() : doTurnRight();
    delay(20);
  }
  stopMotors();
  return false;
}

bool roam_allClear() {
  long F = roam_getMinDist(TRIG_F, ECHO_F); delay(10);
  long L = roam_getMinDist(TRIG_L, ECHO_L); delay(10);
  long R = roam_getMinDist(TRIG_R, ECHO_R);
  return F > FRONT_DIST + 10 && L > ROAM_SIDE_DIST + 8 && R > ROAM_SIDE_DIST + 8;
}

void roam_escapeManoeuvre() {
  reverse(); delay(700); stopMotors(); delay(200);

  unsigned long t = millis();
  while (millis() - t < 3000) {
    doTurnRight(); delay(20);
    long F = roam_getMinDist(TRIG_F, ECHO_F); delay(10);
    long L = roam_getMinDist(TRIG_L, ECHO_L); delay(10);
    long R = roam_getMinDist(TRIG_R, ECHO_R);
    if (F > FRONT_DIST + 20 && L > ROAM_SIDE_DIST && R > ROAM_SIDE_DIST) {
      stopMotors();
      return;
    }
  }

  stopMotors(); delay(150);
  t = millis();
  while (millis() - t < 3000) {
    doTurnLeft(); delay(20);
    long F = roam_getMinDist(TRIG_F, ECHO_F); delay(10);
    long L = roam_getMinDist(TRIG_L, ECHO_L); delay(10);
    long R = roam_getMinDist(TRIG_R, ECHO_R);
    if (F > FRONT_DIST + 20 && L > ROAM_SIDE_DIST && R > ROAM_SIDE_DIST) {
      stopMotors();
      return;
    }
  }

  stopMotors();
}

long roam_getSmartDist(int trig, int echo, long &lastVal, unsigned long &lastTime) {
  long d = roam_getMinDist(trig, echo);
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

long roam_getMinDist(int trig, int echo) {
  long minVal = 999;
  for (int i = 0; i < READINGS; i++) {
    long d = roam_readOnce(trig, echo);
    if (d < minVal) minVal = d;
    delay(10);
  }
  return minVal;
}

long roam_readOnce(int trig, int echo) {
  digitalWrite(trig, LOW);  delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long dur = pulseIn(echo, HIGH, 25000UL);
  if (dur == 0) return 999;
  return dur / 58;
}

void moveForward() { setMotors(MOTOR_SPEED, MOTOR_SPEED); }

void roam_alertOn()   { }
void roam_alertOff()  { }
void roam_alertBeep() { }

void roam_startupBeep() {
  for (int i = 0; i < 2; i++) {
    digitalWrite(LED_RED, HIGH);
    digitalWrite(BUZZER, LOW);  delay(150);
    digitalWrite(BUZZER, HIGH); delay(150);
    digitalWrite(LED_RED, LOW); delay(100);
  }
}

void updateAlarm() {
  if (millis() - alarmTimer >= ALARM_INTERVAL) {
    alarmTimer = millis();
    alarmOn = !alarmOn;
    digitalWrite(LED_RED, alarmOn ? HIGH : LOW);
    digitalWrite(BUZZER,  alarmOn ? LOW  : HIGH);
  }
}

void phaseA_search() {
  if (tryLockOn()) return;
  long F = readOnce(TRIG_F, ECHO_F);
  if (tryLockOn()) return;
  long L = readOnce(TRIG_L, ECHO_L);
  if (tryLockOn()) return;
  long R = readOnce(TRIG_R, ECHO_R);
  if (tryLockOn()) return;
  updateAlarm();

  if (F <= CONTACT_DIST || L <= CONTACT_DIST || R <= CONTACT_DIST) {
    stopMotors();
    reverseShort();
    if (L <= CONTACT_DIST && L <= R)      escapeSide(true);
    else if (R <= CONTACT_DIST && R < L)  escapeSide(false);
    else {
      long L2 = getMinDist(TRIG_L, ECHO_L);
      long R2 = getMinDist(TRIG_R, ECHO_R);
      pivotUntilClear(R2 >= L2);
    }
    curveStart = millis();
    return;
  }

  if (inObstacleMode && millis() - obstacleModeStart > OBS_TIMEOUT) {
    escapeSpin();
    inObstacleMode = false;
    curveStart = millis();
    return;
  }

  if (F <= SEARCH_FRONT) {
    long Fc = getMinDist(TRIG_F, ECHO_F);
    if (Fc <= SEARCH_FRONT) {
      if (!inObstacleMode) { inObstacleMode = true; obstacleModeStart = millis(); }
      long Lc = getMinDist(TRIG_L, ECHO_L);
      long Rc = getMinDist(TRIG_R, ECHO_R);
      avoidObstacle(Lc, Rc);
      curveStart = millis();
      return;
    }
  }

  if (L <= SIDE_CLOSE) { escapeSide(true);  curveStart = millis(); return; }
  if (R <= SIDE_CLOSE) { escapeSide(false); curveStart = millis(); return; }
  if (L <= SIDE_DIST)  { setMotors(SEARCH_SPEED, CRAWL_SPEED); return; }
  if (R <= SIDE_DIST)  { setMotors(CRAWL_SPEED, SEARCH_SPEED); return; }

  inObstacleMode = false;
  cruiseWander();
}

void cruiseWander() {
  if (millis() - wanderTimer > wanderInterval) {
    wanderTimer = millis();
    wanderInterval = random(WANDER_MIN, WANDER_MAX);
    int r = random(0, 3);
    wanderState = (r == 0) ? -1 : (r == 1) ? 0 : 1;
  }
  int l = SEARCH_SPEED, rr = SEARCH_SPEED;
  if      (wanderState < 0) l  = SEARCH_SPEED - WANDER_BIAS;
  else if (wanderState > 0) rr = SEARCH_SPEED - WANDER_BIAS;
  setMotors(l, rr);
}

bool tryLockOn() {
  bool L = lineLeft(), R = lineRight();
  if (L && R) return false;
  if (L || R) { lockOntoLine(L); return true; }
  return false;
}

void lockOntoLine(bool leftDetected) {
  stopMotors();
  phase = PHASE_FOLLOW;
  lastDir = leftDetected ? -1 : +1;
  bothOffStart = 0;
  inObstacleMode = false;
}

void phaseB_follow() {
  long F = getMinDist(TRIG_F, ECHO_F);
  if (F <= FOLLOW_FRONT) {
    stopMotors();
    bool cleared = honkAtObstacle();
    if (!cleared) {
      bool detouredRight = detourObstacle();
      if (!reacquireLine(detouredRight)) {
        phase = PHASE_SEARCH;
        curveStart = millis();
      }
    }
    return;
  }

  bool L = lineLeft();
  bool R = lineRight();

  if (L && R) {
    if (isExitByCreep()) exitAnimation();
    bothOffStart = 0;
    return;
  }

  if (L && !R) {
    lastDir = -1; bothOffStart = 0;
    setMotors(TURN_INNER, TURN_OUTER);
    return;
  }
  if (R && !L) {
    lastDir = +1; bothOffStart = 0;
    setMotors(TURN_OUTER, TURN_INNER);
    return;
  }

  if (bothOffStart == 0) bothOffStart = millis();
  if (millis() - bothOffStart < LOST_TIME) {
    setMotors(BASE_SPEED, BASE_SPEED);
  } else {
    if (lastDir < 0)      pivotLeft();
    else if (lastDir > 0) pivotRight();
    else                  setMotors(BASE_SPEED, BASE_SPEED);
  }
}

bool isExitByCreep() {
  stopMotors();

  setMotors(CREEP_KICK, CREEP_KICK);
  delayAlarm(CREEP_KICK_MS);
  setMotors(CREEP_SPEED, CREEP_SPEED);

  int both = 0, off = 0;
  unsigned long t = millis();
  while (millis() - t < CREEP_TIMEOUT) {
    updateAlarm();
    if (lineLeft() && lineRight()) {
      both++; off = 0;
      if (both >= EXIT_COUNT) { stopMotors(); return true; }
    } else {
      off++;
      if (off >= 2) break;
    }
    delay(CREEP_SAMPLE);
  }

  unsigned long moved = millis() - t;
  stopMotors();
  analogWrite(ENA, CREEP_SPEED); analogWrite(ENB, CREEP_SPEED);
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  delayAlarm(moved);
  stopMotors();
  return false;
}

bool reacquireLine(bool detouredRight) {
  unsigned long t = millis();
  while (millis() - t < 2500) {
    updateAlarm();
    if (lineLeft() || lineRight()) {
      stopMotors();
      lastDir = lineLeft() ? -1 : +1;
      bothOffStart = 0;
      return true;
    }
    if (detouredRight) setMotors(60, 120);
    else               setMotors(120, 60);
  }
  stopMotors();
  return false;
}

bool honkAtObstacle() {
  unsigned long honkStart = millis();
  while (millis() - honkStart < HONK_TIME) {
    for (int i = 0; i < 3; i++) {
      digitalWrite(LED_RED, HIGH); digitalWrite(BUZZER, LOW);  delay(90);
      digitalWrite(LED_RED, LOW);  digitalWrite(BUZZER, HIGH); delay(70);
    }
    delay(250);
    if (getMinDist(TRIG_F, ECHO_F) > FOLLOW_FRONT + 5) {
      return true;
    }
  }
  return false;
}

bool detourObstacle() {
  long L = getMinDist(TRIG_L, ECHO_L);
  long R = getMinDist(TRIG_R, ECHO_R);
  bool goRight = (R >= L);
  if (goRight) {
    doTurnRight(); delayAlarm(450); stopMotors();
    forwardAlarm(700);
    doTurnLeft();  delayAlarm(450); stopMotors();
    forwardAlarm(900);
  } else {
    doTurnLeft();  delayAlarm(450); stopMotors();
    forwardAlarm(700);
    doTurnRight(); delayAlarm(450); stopMotors();
    forwardAlarm(900);
  }
  stopMotors();
  return goRight;
}

void avoidObstacle(long L, long R) {
  stopMotors();
  updateAlarm();
  delay(60);
  if (L <= SIDE_DIST && R <= SIDE_DIST) {
    reverseShort();
    L = getMinDist(TRIG_L, ECHO_L);
    R = getMinDist(TRIG_R, ECHO_R);
  }
  bool goRight = (R >= L);
  pivotUntilClear(goRight);
}

void escapeSide(bool objectOnLeft) {
  unsigned long t = millis();
  while (millis() - t < SIDE_ESC_MS) {
    updateAlarm();
    if (tryLockOn()) return;
    objectOnLeft ? doTurnRight() : doTurnLeft();
    delay(20);
    long side = objectOnLeft ? getMinDist(TRIG_L, ECHO_L)
                             : getMinDist(TRIG_R, ECHO_R);
    long F = getMinDist(TRIG_F, ECHO_F);
    if (side > SIDE_DIST + 8 && F > SEARCH_FRONT) { stopMotors(); return; }
  }
  stopMotors();
}

void pivotUntilClear(bool right) {
  unsigned long t = millis();
  while (millis() - t < 3000) {
    updateAlarm();
    if (tryLockOn()) return;
    if (getMinDist(TRIG_F, ECHO_F) > SEARCH_FRONT + 10) { stopMotors(); return; }
    right ? doTurnRight() : doTurnLeft();
    delay(20);
  }
  t = millis();
  while (millis() - t < 2000) {
    updateAlarm();
    if (tryLockOn()) return;
    if (getMinDist(TRIG_F, ECHO_F) > SEARCH_FRONT + 10) { stopMotors(); return; }
    right ? doTurnLeft() : doTurnRight();
    delay(20);
  }
  stopMotors();
}

void escapeSpin() {
  reverseShort();
  unsigned long t = millis();
  doTurnRight();
  while (millis() - t < 2500) {
    updateAlarm();
    if (tryLockOn()) return;
    long F = readOnce(TRIG_F, ECHO_F); delay(5);
    long L = readOnce(TRIG_L, ECHO_L); delay(5);
    long R = readOnce(TRIG_R, ECHO_R);
    if (F > SEARCH_FRONT + 15 && L > SIDE_DIST && R > SIDE_DIST) { stopMotors(); return; }
  }
  stopMotors();
}

void exitAnimation() {
  stopMotors();
  for (int round = 0; round < 6; round++) {
    for (int i = 0; i < 3; i++) {
      digitalWrite(LED_RED, HIGH); digitalWrite(BUZZER, LOW);  delay(80);
      digitalWrite(LED_RED, LOW);  digitalWrite(BUZZER, HIGH); delay(60);
    }
    delay(150);
  }
  digitalWrite(LED_RED, HIGH); digitalWrite(BUZZER, LOW);
  delay(800);
  digitalWrite(BUZZER, HIGH);
  digitalWrite(LED_RED, HIGH);
  while (true) { }
}

bool lineLeft()  { return analogRead(LINE_LEFT)  > LEFT_THRESHOLD;  }
bool lineRight() { return analogRead(LINE_RIGHT) > RIGHT_THRESHOLD; }

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

void reverse() {
  analogWrite(ENA, BASE_SPEED); analogWrite(ENB, BASE_SPEED);
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}
void reverseShort() { reverse(); delayAlarm(350); stopMotors(); delay(80); }

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

void pivotLeft() {
  analogWrite(ENA, PIVOT_SPEED); analogWrite(ENB, PIVOT_SPEED);
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
}
void pivotRight() {
  analogWrite(ENA, PIVOT_SPEED); analogWrite(ENB, PIVOT_SPEED);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

void stopMotors() {
  analogWrite(ENA, 0); analogWrite(ENB, 0);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

void forwardAlarm(int ms) {
  unsigned long t = millis();
  setMotors(BASE_SPEED, BASE_SPEED);
  while (millis() - t < (unsigned long)ms) { updateAlarm(); delay(10); }
  stopMotors();
}

void delayAlarm(int ms) {
  unsigned long t = millis();
  while (millis() - t < (unsigned long)ms) { updateAlarm(); delay(5); }
}