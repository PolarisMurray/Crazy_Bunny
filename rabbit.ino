#include <Servo.h>
#include <Adafruit_MotorShield.h>  // "Adafruit Motor Shield V2 Library"

// rabbit code - reacts to loud sounds + stuff coming up behind it,
// and turns around if something is in front

// ---------- mic ----------
// mic = adafruit 1063 (MAX4466), OUT -> A1
// sits at VCC/2 when quiet, swings around that with sound
const int MIC_PIN = A1;
const unsigned long MIC_WINDOW_MS = 50; // 50ms from the adafruit example (down to 20Hz)
const bool MIC_ENABLED = false; // mic seems broken rn, sound = 0 when off

const int SOUND_THRESHOLD = 100; // TODO calibrate
const int SOUND_MAX = 400;       // TODO calibrate

// ---------- ears servo (VS-5M) ----------
Servo ears;
const int EARS_PIN = 9;
const int EARS_UP_ANGLE = 90;
const int EARS_DOWN_ANGLE = 30;
const unsigned long WAIT_TIME_MS = 2000; // ears up this long -> run

// ---------- back distance sensor ----------
// on the back; range: 20-150cm
const int DIST_PIN = A0;
const int DIST_SAMPLES = 15;

const int DIST_THRESHOLD_CM = 40;
const int DIST_MIN_CM = 20;

// ---------- front distance sensor ----------
// only for obstacles; Vo -> A2
// (don't use A4/A5, the motor shield needs them)
const int FRONT_DIST_PIN = A2;
const int FRONT_DIST_SAMPLES = 15;
const int FRONT_THRESHOLD_CM = 30; // closer than this -> obstacle

// ---------- motors (adafruit shield v2.3) ----------
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *leftMotor = AFMS.getMotor(1);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(2);
// motors face opposite ways - flip one of these if a wheel goes backwards
const uint8_t LEFT_FORWARD = FORWARD;
const uint8_t RIGHT_FORWARD = BACKWARD;
const uint8_t LEFT_BACKWARD = (LEFT_FORWARD == FORWARD) ? BACKWARD : FORWARD;

const int MOTOR_MIN_SPEED = 8;
const int MOTOR_MAX_SPEED = 25;

// left turn on the spot - no encoders so it's by time
const int TURN_SPEED = 150;
const unsigned long TURN_180_MS = 1200; // TODO calibrate on the floor until it's ~180 deg

// ---------- state ----------
bool earsUp = true;
unsigned long earsUpSince = 0;
int lastSpeed = 0;


// ---------- sensors ----------

int readSoundLevel() {
  int lo = 1023, hi = 0;
  unsigned long start = millis();
  while (millis() - start < MIC_WINDOW_MS) {
    int v = analogRead(MIC_PIN);
    if (v < lo) lo = v;
    if (v > hi) hi = v;
  }
  return hi - lo;
}

// median of `samples` reads from `pin` (max 15)
int readMedianRaw(int pin, int samples) {
  int vals[15];
  samples = constrain(samples, 1, 15);
  for (int i = 0; i < samples; i++) { vals[i] = analogRead(pin); delay(3); }
  // insertion sort
  for (int i = 1; i < samples; i++) {
    int key = vals[i];
    int j = i - 1;
    while (j >= 0 && vals[j] > key) { vals[j + 1] = vals[j]; j--; }
    vals[j + 1] = key;
  }
  return vals[samples / 2];
}

// raw ADC -> cm (sharp 20-150cm curve), 999 = nothing there
float rawToCm(int raw) {
  if (raw < 10) return 999.0;
  float volts = raw / 1023.0 * 5.0;
  return 63.46 * pow(volts, -1.193);
}

float readDistanceCm() {
  return rawToCm(readMedianRaw(DIST_PIN, DIST_SAMPLES));
}

float readFrontDistanceCm() {
  return rawToCm(readMedianRaw(FRONT_DIST_PIN, FRONT_DIST_SAMPLES));
}


// ---------- motors ----------

// 0 = stop, 255 = full
void setMotorSpeed(int speed) {
  speed = constrain(speed, 0, 255);
  leftMotor->setSpeed(speed);
  rightMotor->setSpeed(speed);
  if (speed == 0) {
    leftMotor->run(RELEASE);
    rightMotor->run(RELEASE);
  } else {
    leftMotor->run(LEFT_FORWARD);
    rightMotor->run(RIGHT_FORWARD);
  }
}

// spin left ~180 deg on the spot, then stop
void turnLeft180() {
  leftMotor->setSpeed(TURN_SPEED);
  rightMotor->setSpeed(TURN_SPEED);
  leftMotor->run(LEFT_BACKWARD);  // left wheel back
  rightMotor->run(RIGHT_FORWARD); // right wheel forward
  delay(TURN_180_MS);
  setMotorSpeed(0);
}


// ---------- main ----------

void setup() {
  Serial.begin(9600);
  Serial.println("# rabbit starting");
  ears.attach(EARS_PIN);
  ears.write(EARS_DOWN_ANGLE);
  if (!AFMS.begin()) {
    Serial.println("Something wrong with the motor shield");
    while (1);
  }
  Serial.println("# motor shield ok");
  if (!MIC_ENABLED) Serial.println("# mic disabled");
  setMotorSpeed(0);
  Serial.println("# sound,dist,frontDist,earsUp,soundSpd,distSpd,speed");
}

void loop() {
  // something in front -> turn left 180
  float frontDistance = readFrontDistanceCm();
  if (frontDistance < FRONT_THRESHOLD_CM) {
    Serial.println("# obstacle in front! turning left 180");
    turnLeft180();
    lastSpeed = 0;
    return;
  }

  int sound = 0;
  if (MIC_ENABLED) sound = readSoundLevel();
  float distance = readDistanceCm();
  unsigned long now = millis();

  // loud -> ears up, otherwise ears down
  if (sound > SOUND_THRESHOLD) {
    if (!earsUp) {
      earsUp = true;
      earsUpSince = now;
      ears.write(EARS_UP_ANGLE);
      Serial.println("# ears up");
    }
  } else {
    if (earsUp) Serial.println("# ears down");
    earsUp = false;
    ears.write(EARS_DOWN_ANGLE);
  }

  // ears up long enough -> run. louder = faster
  int soundSpeed = 0;
  if (earsUp && now - earsUpSince >= WAIT_TIME_MS) {
    int s = constrain(sound, SOUND_THRESHOLD, SOUND_MAX);
    soundSpeed = map(s, SOUND_THRESHOLD, SOUND_MAX, MOTOR_MIN_SPEED, MOTOR_MAX_SPEED);
  }

  // something close behind -> run. closer = faster
  int distSpeed = 0;
  if (distance < DIST_THRESHOLD_CM) {
    int d = constrain((int)distance, DIST_MIN_CM, DIST_THRESHOLD_CM);
    distSpeed = map(d, DIST_THRESHOLD_CM, DIST_MIN_CM, MOTOR_MIN_SPEED, MOTOR_MAX_SPEED);
  }

  int speed = max(soundSpeed, distSpeed); // whichever is faster
  setMotorSpeed(speed);
  if (speed > 0 && lastSpeed == 0) Serial.println("# running!");
  if (speed == 0 && lastSpeed > 0) Serial.println("# stopped");
  lastSpeed = speed;

  Serial.println(String(sound) + "," + distance + "," + frontDistance + "," + earsUp + "," +
                 soundSpeed + "," + distSpeed + "," + speed);
}
