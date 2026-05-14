// =======================
// MOTOR PINS
// =======================

#define IN1 2
#define IN2 3
#define IN3 4
#define IN4 7

#define ENA 5
#define ENB 6

// =======================
// ULTRASONIC SENSOR PINS
// =======================

// FRONT
#define TRIG_F 9
#define ECHO_F 10

// LEFT
#define TRIG_L 11
#define ECHO_L 12

// RIGHT
#define TRIG_R 13
#define ECHO_R A0


// =======================
// SETUP
// =======================

void setup() {

  // Motor pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Sensor pins
  pinMode(TRIG_F, OUTPUT);
  pinMode(ECHO_F, INPUT);

  pinMode(TRIG_L, OUTPUT);
  pinMode(ECHO_L, INPUT);

  pinMode(TRIG_R, OUTPUT);
  pinMode(ECHO_R, INPUT);

  Serial.begin(9600);
}


// =======================
// DISTANCE FUNCTION
// =======================

float getDistance(int trigPin, int echoPin) {

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);

  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);

  float distance = duration * 0.034 / 2;

  return distance;
}


// =======================
// MOTOR FUNCTIONS
// =======================

void moveForward() {

  analogWrite(ENA, 180);
  analogWrite(ENB, 180);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void stopMotors() {

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void turnLeft() {

  analogWrite(ENA, 180);
  analogWrite(ENB, 180);

  // Left motor backward
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  // Right motor forward
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  delay(500);
}

void turnRight() {

  analogWrite(ENA, 180);
  analogWrite(ENB, 180);

  // Left motor forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  // Right motor backward
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  delay(500);
}


// =======================
// MAIN LOOP
// =======================

void loop() {

  float front = getDistance(TRIG_F, ECHO_F);
  float left  = getDistance(TRIG_L, ECHO_L);
  float right = getDistance(TRIG_R, ECHO_R);

  Serial.print("Front: ");
  Serial.print(front);

  Serial.print(" | Left: ");
  Serial.print(left);

  Serial.print(" | Right: ");
  Serial.println(right);


  // =======================
  // OBSTACLE LOGIC
  // =======================

  if (front < 20) {

    stopMotors();

    delay(300);

    // Choose clearer side
    if (left > right) {

      turnLeft();

    } else {

      turnRight();
    }

  } else {

    moveForward();
  }

  delay(100);
}
