#include <ZumoMotors.h>
//==============================================
// Cameron Yeoman Zumo Bot
// 06/10/2025
// Obstacle Course (adaptive single/dual ultrasonic)
// 2 Light Sensors (A2, A3)
// 2 Ultrasonic 'eyes' (trig/echo pairs)
// Auto-detects missing ultrasonic and switches to zigzag scan.
// Non-blocking collision & panic recovery.
//==============================================
//
// Possible improvements (short list)
// - Add motor speed clamping to protect motors (safety: prevents out-of-range commands).
// - Change ultrasonic "no reading" sentinel to -1 and map it to a large NO_READING_FAR to avoid blind cruising.
// - Replace exact-equality light stuck test with a small delta window (e.g. DELTA = 6) to avoid false positives.
// - Add a CSV serial logger (timestamp, state, distL, distR, leftLight, rightLight, cmds) for marking evidence. (Not in requirments)
// - Add simple bumper switches or a contact sensor as a hardware fallback for non-reflective obstacles. (Assumiong obstacles are of simple readabile distances)
// - Add a calibration routine to pick LIGHT_THRESHOLD at the start of each run. (light should only matter at threshold obstructed)
// - Use wheel encoders or motor-current watchdog to strengthen stuck detection (optional but robust if advancing project).
// - Tune and document timing values (reverse/turn durations) and record battery voltage during runs. (Out of scope)
//
// End of header notes
//==============================================
 
 
ZumoMotors motors;
 
// === CONFIG / FLAGS ===
bool SINGLE_ULTRASONIC_MODE = false; // auto-set at runtime if one eye fails
 
// === Pins ===
const int trigLeft = 13;
const int echoLeft = 12;
const int trigRight = 2;
const int echoRight = 6;
const int leftLightPin = A2;
const int rightLightPin = A3;
 
// === Distance / light ===
float distLeft = 200;
float distRight = 200;
int leftLight = 0;
int rightLight = 0;
 
// === Timing / serial ===
unsigned long lastPrintTime = 0;
const unsigned long printInterval = 400;
unsigned long loopDelay = 30;
 
// === Dist thresholds (cm) ===
const float SAFE_DISTANCE   = 40.0;
const float DANGER_DISTANCE = 20.0;
const float STOP_DISTANCE   = 10.0;
 
// === Light threshold ===
const int LIGHT_THRESHOLD = 890;
 
// === Speeds ===
const int CRUISE_SPEED    = 150;
const int SLOW_SPEED      = 120;
const int REVERSE_SPEED   = -150;
const int TURN_SPEED      = 150;
const int FAST_SPEED      = 250;
 
// === Panic/backoff tuning ===
const int PANIC_REVERSE_MS_DEFAULT = 480;
const int PANIC_TURN_MS_DEFAULT    = 600;
const int COLLISION_REVERSE_MS     = 420;
const int COLLISION_TURN_MS        = 600;
const int COLLISION_TURN_BOOST     = 60;
const int PANIC_TURN_BOOST         = 80;
const unsigned long STUCK_WINDOW_MS_ULTRA = 700;
const float MIN_DISTANCE_CHANGE = 3.0;
const int STUCK_CONFIRMATIONS_ULTRA = 2;
 
// === Sensor health ===
const int HEALTH_WINDOW = 6;
int leftLightHistory[HEALTH_WINDOW];
int rightLightHistory[HEALTH_WINDOW];
int leftUltrasonicInvalidCount = 0;
int rightUltrasonicInvalidCount = 0;
const int ULTRASONIC_INVALID_LIMIT = 6;
 
// === Misc memory ===
unsigned long lastStuckCheck = 0;
int lastTurnDir = 1; // prefer right
const int CLEAR_REQUIRED = 3;
int clearCounter = 0;
int stuckCounter = 0;
 
// === Motor command tracking ===
int lastLeftCmd = 0;
int lastRightCmd = 0;
 
// === STOP timeout ===
unsigned long stopStartMillis = 0;
const unsigned long STOP_STUCK_TIMEOUT_MS = 1200;
 
// === Zigzag params (for single-eye sweeping) ===
unsigned long zigFlipMillis = 0;
const unsigned long ZIG_INTERVAL_MS = 1050;
int zigState = 1; // 1 -> slight right drift, -1 -> slight left drift
const int ZIG_TURN_MAG = 80; // amount to bias wheel speeds for zigzag
 
 
 
// === FSM ===
enum RobotState {
  STATE_CRUISE,
  STATE_SEEK_LIGHT,
  STATE_AVOID,
  STATE_COLLISION,
  STATE_STOP,
  STATE_PANIC
};
RobotState currentState = STATE_CRUISE;
 
 
 
// === Non-blocking action state for collision/panic ===
int collisionPhase = 0; // 0 idle, 1 backing, 2 turning, 3 forward/finish
unsigned long collisionActionStart = 0;
int collisionTurnDir = 1;
int collisionAttempts = 0;
const int MAX_COLLISION_ATTEMPTS = 3;
 
int panicPhase = 0; // 0 idle, 1 backing, 2 turning, 3 forward, 4 done
unsigned long panicActionStart = 0;
int panicTurnDir = 1;
int panicAttempts = 0;
const int MAX_PANIC_ATTEMPTS = 3;
 
// === Helpers ===
void setMotorSpeeds(int l, int r) {
  lastLeftCmd = l;
  lastRightCmd = r;
  motors.setSpeeds(l, r);
}
 
// === Ultrasonic smoothing ===
float readDistanceStable(int trigPin, int echoPin, int samples = 3) {
  float total = 0;
  int validCount = 0;
  for (int i = 0; i < samples; i++) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
 
    long duration = pulseIn(echoPin, HIGH, 20000); // 20ms timeout
    if (duration == 0) {
      // no echo for this sample -> skip
      continue;
    }
    float distance = (duration * 0.0343) / 2.0; // cm
    if (distance > 0 && distance <= 400) {
      if (distance < 5) distance = STOP_DISTANCE; // treat tiny readings as STOP
      total += distance;
      validCount++;
    }
    delay(2);
  }
  if (validCount == 0) return 200; // sentinel meaning "no valid reading / timeout"
  return total / validCount;
}
 
// === Read ultrasonics (with correct missing-sensor detection) ===
void readUltrasonics() {
  float l = readDistanceStable(trigLeft, echoLeft);
  float r = readDistanceStable(trigRight, echoRight);
 
  // detect disconnected / timeout sentinel (readDistanceStable returns 200)
  if (l == 200) leftUltrasonicInvalidCount++; else leftUltrasonicInvalidCount = 0;
  if (r == 200) rightUltrasonicInvalidCount++; else rightUltrasonicInvalidCount = 0;
 
  // Auto-detect single-eye mode when one sensor repeatedly times out
  // require several consecutive misses to avoid transient false positives
  if (rightUltrasonicInvalidCount > 6 && leftUltrasonicInvalidCount < 3) {
    SINGLE_ULTRASONIC_MODE = true;
  } else if (leftUltrasonicInvalidCount > 6 && rightUltrasonicInvalidCount < 3) {
    SINGLE_ULTRASONIC_MODE = true;
  } else if (leftUltrasonicInvalidCount < 3 && rightUltrasonicInvalidCount < 3) {
    SINGLE_ULTRASONIC_MODE = false;
  }
 
  // Mirror working sensor when in single-eye mode
  if (SINGLE_ULTRASONIC_MODE) {
    // pick the sensor with fewer invalid counts as the working eye
    if (leftUltrasonicInvalidCount > rightUltrasonicInvalidCount) {
      // right is healthier -> use right reading as front
      distLeft = r;
      distRight = r;
    } else {
      // left healthier (or equal) -> use left reading as front
      distLeft = l;
      distRight = l;
    }
  } else {
    // both healthy — use both
    distLeft = l;
    distRight = r;
  }
}
 
// === Light sensors ===
void readLightSensors() {
  int l = analogRead(leftLightPin);
  int r = analogRead(rightLightPin);
  for (int i = HEALTH_WINDOW - 1; i >= 1; i--) {
    leftLightHistory[i] = leftLightHistory[i - 1];
    rightLightHistory[i] = rightLightHistory[i - 1];
  }
  leftLightHistory[0] = l;
  rightLightHistory[0] = r;
  leftLight = l;
  rightLight = r;
}
 
// === Health helpers ===
bool isLightSensorStuck(int history[]) {
  int sameCount = 0;
  for (int i = 1; i < HEALTH_WINDOW; i++) if (history[i] == history[0]) sameCount++;
  if (sameCount >= HEALTH_WINDOW - 1) return true;
  if (history[0] <= 2) return true;
  if (history[0] >= 1020) return true;
  return false;
}
bool leftLightHealthy()  { return !isLightSensorStuck(leftLightHistory); }
bool rightLightHealthy() { return !isLightSensorStuck(rightLightHistory); }
bool leftUltraHealthy()  { return (leftUltrasonicInvalidCount < ULTRASONIC_INVALID_LIMIT); }
bool rightUltraHealthy() { return (rightUltrasonicInvalidCount < ULTRASONIC_INVALID_LIMIT); }
 
// === Non-blocking collision handler ===
void startCollisionAction() {
  collisionPhase = 1;
  collisionActionStart = millis();
  collisionAttempts++;
  // choose direction: if left closer => turn right (dir=1), else left
  if (!SINGLE_ULTRASONIC_MODE && distLeft < distRight) collisionTurnDir = 1;
  else if (!SINGLE_ULTRASONIC_MODE && distRight < distLeft) collisionTurnDir = -1;
  else collisionTurnDir = (zigState == 1) ? -1 : 1; // if single eye, prefer turning away from current zig
}
void processCollisionAction() {
  unsigned long now = millis();
  if (collisionPhase == 1) {
    // backing
    setMotorSpeeds(REVERSE_SPEED, REVERSE_SPEED);
    if (now - collisionActionStart >= COLLISION_REVERSE_MS || (distLeft > SAFE_DISTANCE && distRight > SAFE_DISTANCE)) {
      collisionPhase = 2;
      collisionActionStart = now;
      int turnPower = TURN_SPEED + COLLISION_TURN_BOOST;
      if (collisionTurnDir == 1) setMotorSpeeds(turnPower, -turnPower); else setMotorSpeeds(-turnPower, turnPower);
    }
  } else if (collisionPhase == 2) {
    // turning
    if (now - collisionActionStart >= COLLISION_TURN_MS) {
      collisionPhase = 3;
      collisionActionStart = now;
      setMotorSpeeds(SLOW_SPEED, SLOW_SPEED);
    } else {
      // if front clears early, move to forward phase
      if (distLeft > SAFE_DISTANCE && distRight > SAFE_DISTANCE) {
        collisionPhase = 3;
        collisionActionStart = now;
        setMotorSpeeds(SLOW_SPEED, SLOW_SPEED);
      }
    }
  } else if (collisionPhase == 3) {
    if (now - collisionActionStart >= 200) {
      collisionPhase = 0;
      // if still too close and attempts remain, flip direction and retry
      if ((distLeft < SAFE_DISTANCE || distRight < SAFE_DISTANCE) && collisionAttempts < MAX_COLLISION_ATTEMPTS) {
        collisionTurnDir = -collisionTurnDir;
        startCollisionAction();
      } else {
        collisionAttempts = 0;
        currentState = STATE_CRUISE;
      }
    }
  }
}
 
// === Non-blocking panic handler ===
void startPanicAction() {
  panicPhase = 1;
  panicActionStart = millis();
  panicAttempts++;
  panicTurnDir = (random(0,100) < 50) ? 1 : -1;
}
void processPanicAction() {
  unsigned long now = millis();
  int reverseMs = PANIC_REVERSE_MS_DEFAULT;
  int turnMs = PANIC_TURN_MS_DEFAULT;
  int turnPower = TURN_SPEED + PANIC_TURN_BOOST;
  if (panicPhase == 1) {
    setMotorSpeeds(REVERSE_SPEED, REVERSE_SPEED);
    if (now - panicActionStart >= reverseMs) {
      panicPhase = 2;
      panicActionStart = now;
      if (panicTurnDir == 1) setMotorSpeeds(turnPower, -turnPower); else setMotorSpeeds(-turnPower, turnPower);
    }
  } else if (panicPhase == 2) {
    if (now - panicActionStart >= turnMs) {
      panicPhase = 3;
      panicActionStart = now;
      setMotorSpeeds(SLOW_SPEED, SLOW_SPEED);
    }
  } else if (panicPhase == 3) {
    if (now - panicActionStart >= 200) {
      panicPhase = 0;
      if ((distLeft < SAFE_DISTANCE || distRight < SAFE_DISTANCE) && panicAttempts < MAX_PANIC_ATTEMPTS) {
        panicTurnDir = -panicTurnDir;
        startPanicAction();
      } else {
        panicAttempts = 0;
        currentState = STATE_CRUISE;
      }
    }
  }
}
 
// === Stuck detection (ultrasonic + commanded motion) ===
unsigned long stuckWindowStartUltra = 0;
float prevDistLeft = 999;
float prevDistRight = 999;
int stuckFailsUltra = 0;
 
void checkUltrasonicForStuck() {
  unsigned long now = millis();
 
  // STOP timeout
  if (currentState == STATE_STOP) {
    if (stopStartMillis == 0) stopStartMillis = now;
    else if (now - stopStartMillis > STOP_STUCK_TIMEOUT_MS) {
      Serial.println("🚨 STOP timeout — forcing PANIC");
      if (panicPhase == 0) startPanicAction();
      currentState = STATE_PANIC;
      stopStartMillis = 0;
      return;
    }
  } else stopStartMillis = 0;
 
  // movement-based stuck detection (only when commanded to move)
  if (stuckWindowStartUltra == 0) {
    stuckWindowStartUltra = now;
    prevDistLeft = distLeft; prevDistRight = distRight;
    return;
  }
  if (now - stuckWindowStartUltra < STUCK_WINDOW_MS_ULTRA) return;
 
  float dL = abs(distLeft - prevDistLeft);
  float dR = abs(distRight - prevDistRight);
  prevDistLeft = distLeft; prevDistRight = distRight;
  stuckWindowStartUltra = now;
 
  int cmdMag = max(abs(lastLeftCmd), abs(lastRightCmd));
  if (cmdMag < 70) { stuckFailsUltra = 0; return; }
 
  if (dL < MIN_DISTANCE_CHANGE && dR < MIN_DISTANCE_CHANGE) stuckFailsUltra++;
  else stuckFailsUltra = 0;
 
  if (stuckFailsUltra >= STUCK_CONFIRMATIONS_ULTRA) {
    Serial.println("🚨 STUCK DETECTED (ultrasonic + commanded motion) — starting PANIC");
    if (panicPhase == 0) startPanicAction();
    currentState = STATE_PANIC;
    stuckFailsUltra = 0;
  }
}
 
// === Zigging helper ===
void maybeFlipZig() {
  unsigned long now = millis();
  if (now - zigFlipMillis >= ZIG_INTERVAL_MS) {
    zigFlipMillis = now;
    zigState = -zigState;
  }
}
 
// === FSM update ===
void updateState() {
  // preserve active action
  if (collisionPhase != 0) { currentState = STATE_COLLISION; return; }
  if (panicPhase != 0)     { currentState = STATE_PANIC; return; }
 
  if (distLeft < STOP_DISTANCE || distRight < STOP_DISTANCE) { currentState = STATE_STOP; return; }
  if (distLeft < DANGER_DISTANCE || distRight < DANGER_DISTANCE) {
    currentState = STATE_COLLISION;
    startCollisionAction();
    return;
  }
  if (distLeft < SAFE_DISTANCE || distRight < SAFE_DISTANCE) { currentState = STATE_AVOID; return; }
 
  bool leftHealthy = leftLightHealthy();
  bool rightHealthy = rightLightHealthy();
  bool eitherLight = (leftLight >= LIGHT_THRESHOLD && leftHealthy) || (rightLight >= LIGHT_THRESHOLD && rightHealthy);
  if (eitherLight) { currentState = STATE_SEEK_LIGHT; return; }
 
  unsigned long now = millis();
  if (now - lastStuckCheck >= 1200) {
    lastStuckCheck = now;
    bool bothUltraBad = !leftUltraHealthy() && !rightUltraHealthy();
    bool bothLightBad = !leftHealthy && !rightHealthy;
    if (bothUltraBad || bothLightBad) {
      stuckCounter++;
      if (stuckCounter >= 2) { startPanicAction(); currentState = STATE_PANIC; return; }
    } else stuckCounter = 0;
  }
 
  currentState = STATE_CRUISE;
}
 
// === Actuation ===
void handleState() {
  switch (currentState) {
    case STATE_CRUISE:
      if (SINGLE_ULTRASONIC_MODE) {
        maybeFlipZig();
        int baseSpeed = 120; // forward speed
        int turn = ZIG_TURN_MAG;
        if (zigState == 1) setMotorSpeeds(baseSpeed + turn, baseSpeed - turn);
        else setMotorSpeeds(baseSpeed - turn, baseSpeed + turn);
      } else {
        setMotorSpeeds(CRUISE_SPEED, CRUISE_SPEED); // straight in dual-eye mode
      }
      break; // <<< IMPORTANT VAUGHN
 
    case STATE_SEEK_LIGHT: {
      bool lHealthy = leftLightHealthy();
      bool rHealthy = rightLightHealthy();
      if (leftLight >= LIGHT_THRESHOLD && rightLight >= LIGHT_THRESHOLD && lHealthy && rHealthy) {
        setMotorSpeeds(FAST_SPEED, FAST_SPEED);
      } else if (leftLight >= LIGHT_THRESHOLD && lHealthy) {
        // turn LEFT toward left light -> right motor faster
        setMotorSpeeds(SLOW_SPEED, FAST_SPEED);
      } else if (rightLight >= LIGHT_THRESHOLD && rHealthy) {
        // turn RIGHT toward right light -> left motor faster
        setMotorSpeeds(FAST_SPEED, SLOW_SPEED);
      } else {
        setMotorSpeeds(CRUISE_SPEED, SLOW_SPEED);
      }
      break;
    }
 
    case STATE_AVOID: {
      if (!SINGLE_ULTRASONIC_MODE) {
        if (distLeft < SAFE_DISTANCE && distRight < SAFE_DISTANCE) {
          if (random(0,100) < 50) lastTurnDir = 1; else lastTurnDir = -1;
        }
        if (distLeft < SAFE_DISTANCE) { setMotorSpeeds(CRUISE_SPEED + 50, CRUISE_SPEED / 3); lastTurnDir = 1; }
        else if (distRight < SAFE_DISTANCE) { setMotorSpeeds(CRUISE_SPEED / 3, CRUISE_SPEED + 50); lastTurnDir = -1; }
      } else {
        if (zigState == 1) setMotorSpeeds(SLOW_SPEED, CRUISE_SPEED + 50);
        else setMotorSpeeds(CRUISE_SPEED + 50, SLOW_SPEED);
      }
      break;
    }
 
    case STATE_COLLISION:
      processCollisionAction();
      break;
 
    case STATE_STOP:
      setMotorSpeeds(0, 0);
      if (distLeft > SAFE_DISTANCE && distRight > SAFE_DISTANCE) {
        clearCounter++;
        if (clearCounter >= CLEAR_REQUIRED) { currentState = STATE_CRUISE; clearCounter = 0; }
      } else clearCounter = 0;
      break;
 
    case STATE_PANIC:
      processPanicAction();
      break;
  }
}
 
// === Debug printing ===
void printReadings() {
  if (millis() - lastPrintTime < printInterval) return;
  Serial.print("Ldist: "); Serial.print(distLeft);
  Serial.print(" | Rdist: "); Serial.print(distRight);
  Serial.print(" | Light L: "); Serial.print(leftLight);
  Serial.print(" R: "); Serial.print(rightLight);
  Serial.print(" | Mode: "); Serial.print(SINGLE_ULTRASONIC_MODE ? "SINGLE_EYE" : "DUAL_EYE");
  Serial.print(" | State: ");
  switch (currentState) {
    case STATE_CRUISE: Serial.print("CRUISE"); break;
    case STATE_SEEK_LIGHT: Serial.print("SEEK_LIGHT"); break;
    case STATE_AVOID: Serial.print("AVOID"); break;
    case STATE_COLLISION: Serial.print("COLLISION"); break;
    case STATE_STOP: Serial.print("STOP"); break;
    case STATE_PANIC: Serial.print("PANIC"); break;
  }
  Serial.print(" | lastCmd: "); Serial.print(lastLeftCmd); Serial.print(","); Serial.print(lastRightCmd);
  Serial.print(" | collPhase: "); Serial.print(collisionPhase); Serial.print(" panicPhase: "); Serial.print(panicPhase);
  Serial.println();
  lastPrintTime = millis();
}
 
// === Setup ===
void setup() {
  pinMode(trigLeft, OUTPUT); pinMode(echoLeft, INPUT);
  pinMode(trigRight, OUTPUT); pinMode(echoRight, INPUT);
  pinMode(leftLightPin, INPUT); pinMode(rightLightPin, INPUT);
  digitalWrite(trigLeft, LOW); digitalWrite(trigRight, LOW);
  Serial.begin(9600);
  delay(30);
  randomSeed(analogRead(A0));
 
  Serial.println("Zumo FSM (adaptive single/dual ultrasonic) ready");
 
  // initialize light history so stuck checks have values
  for (int i = 0; i < HEALTH_WINDOW; i++) {
    leftLightHistory[i] = analogRead(leftLightPin);
    rightLightHistory[i] = analogRead(rightLightPin);
  }
  zigFlipMillis = millis();
  lastStuckCheck = millis();
}
 
// === Main loop ===
void loop() {
  readUltrasonics();
  readLightSensors();
  updateState();
  handleState();
  checkUltrasonicForStuck();
  printReadings();
  delay(loopDelay);
}