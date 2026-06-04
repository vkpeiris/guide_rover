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

#define LINE_LEFT  A1
#define LINE_RIGHT A2
#define LEFT_THRESHOLD  448
#define RIGHT_THRESHOLD 457

#define BUZZER  A4
#define LED_RED A5

#define BASE_SPEED    110
#define TURN_OUTER    140
#define TURN_INNER     90  
#define PIVOT_SPEED   150
#define SEARCH_SPEED  120
#define TURN_SPEED    255
#define CRAWL_SPEED    95

#define SEARCH_FRONT  30
#define FOLLOW_FRONT  20
#define SIDE_DIST     25
#define SIDE_CLOSE    12
#define CONTACT_DIST   5

#define READINGS         3
#define ALARM_INTERVAL 250
#define LOST_TIME      120
#define HONK_TIME     6000
#define OBS_TIMEOUT   7000
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
  delay(2000);

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);

  pinMode(TRIG_F, OUTPUT); pinMode(ECHO_F, INPUT);
  pinMode(TRIG_L, OUTPUT); pinMode(ECHO_L, INPUT);
  pinMode(TRIG_R, OUTPUT); pinMode(ECHO_R, INPUT);

  pinMode(BUZZER,  OUTPUT); digitalWrite(BUZZER, HIGH);
  pinMode(LED_RED, OUTPUT); digitalWrite(LED_RED, LOW);

  stopMotors();
  delay(1000);

  randomSeed(micros());
  curveStart  = millis();
  wanderTimer = millis();

  setMotors(KICK_SPEED, KICK_SPEED);
  delayAlarm(KICK_TIME);
}

void loop() {
  updateAlarm();
  if (phase == PHASE_SEARCH) phaseA_search();
  else                       phaseB_follow();
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
