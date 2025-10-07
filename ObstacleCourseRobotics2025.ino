//==============================================================
// Cameron Yeoman — Zumo Bot Obstacle Course Controller
// Date: 06/10/2025
//
// Description:
// Adaptive obstacle navigation using dual ultrasonic sensors and dual light sensors.
// Automatically switches to single-eye zigzag mode if one ultrasonic fails.
// Implements non-blocking collision and panic recovery routines.
// Finite State Machine (FSM) governs behavior: cruise, seek light, avoid, stop, recover.
// Designed for robustness, auditability, and performance.
//==============================================================


#include <ZumoMotors.h>
ZumoMotors motors;


// === CONFIG / FLAGS ===
bool SINGLE_ULTRASONIC_MODE = false;  // auto-set at runtime if one eye fails

// === Pins ===
const int trigLeft = 13;
const int echoLeft = 12;
const int trigRight = 2;
const int echoRight = 6;
const int leftLightPin = A2;
const int rightLightPin = A3;


// === Sensor Readings ===
float distLeft = 200;
float distRight = 200;
int leftLight = 0;
int rightLight = 0;

// === Timing / serial ===
unsigned long lastPrintTime = 0;
const unsigned long printInterval = 400;
unsigned long loopDelay = 30;

// === Dist thresholds (cm) ===
const float SAFE_DISTANCE = 40.0;
const float DANGER_DISTANCE = 20.0;
const float STOP_DISTANCE = 10.0;

// === Light threshold ===
// 750 is high enough to find light strongly and get around noise
const int LIGHT_THRESHOLD = 750;

// === Motor Speeds ===
const int CRUISE_SPEED = 250;
const int SLOW_SPEED = 120;
const int ZIGZAG = 225;
const int REVERSE_SPEED = -150;
const int TURN_SPEED = 175;
const int FAST_SPEED = 350;

// === Panic / Collision Timing ===
const int PANIC_REVERSE_MS_DEFAULT = 480;
const int PANIC_TURN_MS_DEFAULT = 600;
const int COLLISION_REVERSE_MS = 420;
const int COLLISION_TURN_MS = 600;
const int COLLISION_TURN_BOOST = 60;
const int PANIC_TURN_BOOST = 80;
const unsigned long STUCK_WINDOW_MS_ULTRA = 700;
const float MIN_DISTANCE_CHANGE = 3.0;
const int STUCK_CONFIRMATIONS_ULTRA = 4;

// === Sensor Health Tracking ===
const int HEALTH_WINDOW = 6;
int leftLightHistory[HEALTH_WINDOW];
int rightLightHistory[HEALTH_WINDOW];
int leftUltrasonicInvalidCount = 0;
int rightUltrasonicInvalidCount = 0;
const int ULTRASONIC_INVALID_LIMIT = 6;

// === Misc State Memory ===
unsigned long lastStuckCheck = 0;
int lastTurnDir = 1;  // 1 = right, -1 = left (perferd right)
const int CLEAR_REQUIRED = 3;
int clearCounter = 0;
int stuckCounter = 0;

// === Motor command tracking ===
int lastLeftCmd = 0;
int lastRightCmd = 0;

// === STOP timeout ===
unsigned long stopStartMillis = 0;
const unsigned long STOP_STUCK_TIMEOUT_MS = 1200;

// === Zigzag Parameters (Single-Eye Mode) ===
unsigned long zigFlipMillis = 0;
const unsigned long ZIG_INTERVAL_MS = 1050;
int zigState = 1;              // 1 = drift right, -1 = drift left
const int ZIG_TURN_MAG = 80;  // amount bias wheel speed for zigzag

// === Finite State Machine (FSM) ===
enum RobotState {
  STATE_CRUISE,
  STATE_SEEK_LIGHT,
  STATE_AVOID,
  STATE_COLLISION,
  STATE_STOP,
  STATE_PANIC
};
RobotState currentState = STATE_CRUISE;

// === Non-blocking Collision Recovery ===
int collisionPhase = 0;  // 0 idle, 1 backing, 2 turning, 3 forward/finish
unsigned long collisionActionStart = 0;
int collisionTurnDir = 1;
int collisionAttempts = 0;
const int MAX_COLLISION_ATTEMPTS = 3;

// === Non-blocking Panic Recovery ===
int panicPhase = 0;  // 0 idle, 1 backing, 2 turning, 3 forward, 4 done
unsigned long panicActionStart = 0;
int panicTurnDir = 1;
int panicAttempts = 0;
const int MAX_PANIC_ATTEMPTS = 3;

// === Motor Control Helper ===
// Sets motor speeds and tracks last command
void setMotorSpeeds(int l, int r) {
  lastLeftCmd = l;
  lastRightCmd = r;
  motors.setSpeeds(l, r);
}

// === Ultrasonic smoothing ===
// Reads and averages multiple samples with filtering and timeout
float readDistanceStable(int trigPin, int echoPin, int samples = 3) {
  float total = 0;
  int validCount = 0;
  for (int i = 0; i < samples; i++) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    long duration = pulseIn(echoPin, HIGH, 20000);  // 20ms timeout
    if (duration == 0) continue;                    

    float distance = (duration * 0.0343) / 2.0;  
    if (distance > 0 && distance <= 150) {
      if (distance < 5) distance = STOP_DISTANCE;  // treat tiny readings as STOP
      total += distance;
      validCount++;
    }
    delay(2);
  }
  if (validCount == 0) return 200;  // sentinel for no reading pass
  return total / validCount;
}

// === Ultrasonic Sensor Manager ===
// Reads both sensors, tracks invalid counts, and switches to single-eye mode if needed
void readUltrasonics() {
  float l = readDistanceStable(trigLeft, echoLeft);
  float r = readDistanceStable(trigRight, echoRight);

  if (l == 200) leftUltrasonicInvalidCount++;
  else leftUltrasonicInvalidCount = 0;
  if (r == 200) rightUltrasonicInvalidCount++;
  else rightUltrasonicInvalidCount = 0;

  const int SINGLE_EYE_ENTER_THRESHOLD = 6;
  const int SINGLE_EYE_EXIT_THRESHOLD = 4;
  static int singleEyeValidCount = 0;

  if (!SINGLE_ULTRASONIC_MODE) {
    if ((leftUltrasonicInvalidCount >= SINGLE_EYE_ENTER_THRESHOLD && rightUltrasonicInvalidCount < 3) || (rightUltrasonicInvalidCount >= SINGLE_EYE_ENTER_THRESHOLD && leftUltrasonicInvalidCount < 3)) {
      SINGLE_ULTRASONIC_MODE = true;
      singleEyeValidCount = 0;
    }
  } else {
    if (leftUltrasonicInvalidCount < 3 && rightUltrasonicInvalidCount < 3) {
      singleEyeValidCount++;
      if (singleEyeValidCount >= SINGLE_EYE_EXIT_THRESHOLD) SINGLE_ULTRASONIC_MODE = false;
    } else singleEyeValidCount = 0;
  }

  // Mirror working sensor in single-eye mode (prevents single-eye stepin brackage)
  if (SINGLE_ULTRASONIC_MODE) {
    if (leftUltrasonicInvalidCount > rightUltrasonicInvalidCount) {
      distLeft = r;
      distRight = r;
    } else {
      distLeft = l;
      distRight = l;
    }
  } else {
    distLeft = l;
    distRight = r;
  }
}

// === Light Sensor Reader ===
// Reads both light sensors and updates history buffer for health checks
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

// === Light Sensor Health Check ===
// Returns true if sensor is not stuck or saturated (risk of invalid response)
bool isLightSensorStuck(int history[]) {
  int sameCount = 0;
  const int tolerance = 5;  // Allow small variation
  for (int i = 1; i < HEALTH_WINDOW; i++)
    if (abs(history[i] - history[0]) <= tolerance) sameCount++;
  if (sameCount >= HEALTH_WINDOW - 1) return true;
  if (history[0] <= 2 || history[0] >= 1020) return true;
  return false;
}
bool leftLightHealthy() {
  return !isLightSensorStuck(leftLightHistory);
}
bool rightLightHealthy() {
  return !isLightSensorStuck(rightLightHistory);
}
bool leftUltraHealthy() {
  return (leftUltrasonicInvalidCount < ULTRASONIC_INVALID_LIMIT);
}
bool rightUltraHealthy() {
  return (rightUltrasonicInvalidCount < ULTRASONIC_INVALID_LIMIT);
}

// === startCollisionAction ===
// Initializes the non-blocking collision recovery sequence.
// Chooses turn direction based on sensor readings or zigzag state.
void startCollisionAction() {
  collisionPhase = 1;
  collisionActionStart = millis();
  collisionAttempts++;
  if (!SINGLE_ULTRASONIC_MODE && distLeft < distRight) collisionTurnDir = 1;
  else if (!SINGLE_ULTRASONIC_MODE && distRight < distLeft) collisionTurnDir = -1;
  else collisionTurnDir = (zigState == 1) ? -1 : 1;
}

// === processPanicAction ===
// Executes multi-phase panic recovery:
// Phase 1: Reverse
// Phase 2: Turn
// Phase 3: Resume forward briefly, then exit
void processCollisionAction() {
  unsigned long now = millis();
  if (collisionPhase == 1) {
    setMotorSpeeds(REVERSE_SPEED, REVERSE_SPEED);
    if (now - collisionActionStart >= COLLISION_REVERSE_MS || (distLeft > SAFE_DISTANCE && distRight > SAFE_DISTANCE)) {
      collisionPhase = 2;
      collisionActionStart = now;
      int turnPower = TURN_SPEED + COLLISION_TURN_BOOST;
      if (collisionTurnDir == 1) setMotorSpeeds(turnPower, -turnPower);
      else setMotorSpeeds(-turnPower, turnPower);
    }
  } else if (collisionPhase == 2) {
    if (now - collisionActionStart >= COLLISION_TURN_MS) {
      collisionPhase = 3;
      collisionActionStart = now;
      setMotorSpeeds(SLOW_SPEED, SLOW_SPEED);
    } else {
      if (distLeft > SAFE_DISTANCE && distRight > SAFE_DISTANCE) {
        collisionPhase = 3;
        collisionActionStart = now;
        setMotorSpeeds(SLOW_SPEED, SLOW_SPEED);
      }
    }
  } else if (collisionPhase == 3) {
    if (now - collisionActionStart >= 200) {
      collisionPhase = 0;
      if ((distLeft < SAFE_DISTANCE || distRight < SAFE_DISTANCE) && collisionAttempts < MAX_COLLISION_ATTEMPTS) {
        collisionTurnDir = -collisionTurnDir;
        startCollisionAction();  // Retry with opposite turn
      } else {
        collisionAttempts = 0;
        currentState = STATE_CRUISE;
      }
    }
  }
}

// === startPanicAction ===
// Begins panic recovery sequence when stuck or STOP timeout occurs.
// Chooses random turn direction.
void startPanicAction() {
  panicPhase = 1;
  panicActionStart = millis();
  panicAttempts++;
  panicTurnDir = (random(0, 100) < 50) ? 1 : -1;
}

// === processPanicAction ===
// Executes multi-phase panic recovery:
// Phase 1: Reverse
// Phase 2: Turn
// Phase 3: Resume forward briefly, then exit
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
      if (panicTurnDir == 1) setMotorSpeeds(turnPower, -turnPower);
      else setMotorSpeeds(-turnPower, turnPower);
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
        startPanicAction();  // Retry with opposite turn
      } else {
        panicAttempts = 0;
        currentState = STATE_CRUISE;
      }
    }
  }
}


// === Stuck Detection State ===
// Tracks ultrasonic readings over time to detect lack of progress.
// If the robot is issuing strong motor commands but distance readings don't change,
// it is considered stuck and will trigger panic recovery.
unsigned long stuckWindowStartUltra = 0;
float prevDistLeft = 999;
float prevDistRight = 999;
int stuckFailsUltra = 0;


// === checkUltrasonicForStuck ===
// Monitors ultrasonic readings for lack of change while motors are active.
// If stuck confirmed, triggers panic recovery

void checkUltrasonicForStuck() {
  unsigned long now = millis();

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

 // Stuck detection window
  if (stuckWindowStartUltra == 0) {
    stuckWindowStartUltra = now;
    prevDistLeft = distLeft;
    prevDistRight = distRight;
    return;
  }
  if (now - stuckWindowStartUltra < STUCK_WINDOW_MS_ULTRA) return;

  float dL = abs(distLeft - prevDistLeft);
  float dR = abs(distRight - prevDistRight);
  prevDistLeft = distLeft;
  prevDistRight = distRight;
  stuckWindowStartUltra = now;

  int cmdMag = max(abs(lastLeftCmd), abs(lastRightCmd));
  if (cmdMag < 70) {
    stuckFailsUltra = 0;
    return;
  }

  if (dL < MIN_DISTANCE_CHANGE && dR < MIN_DISTANCE_CHANGE) stuckFailsUltra++;
  else stuckFailsUltra = 0;

  if (stuckFailsUltra >= STUCK_CONFIRMATIONS_ULTRA) {
    Serial.println("🚨 STUCK DETECTED — starting PANIC");
    if (panicPhase == 0) startPanicAction();
    currentState = STATE_PANIC;
    stuckFailsUltra = 0;
  }
}

// === maybeFlipZig ===
// Toggles zigzag drift direction at regular intervals (single-eye mode only)
void maybeFlipZig() {
  unsigned long now = millis();
  if (now - zigFlipMillis >= ZIG_INTERVAL_MS) {
    zigFlipMillis = now;
    zigState = -zigState;
  }
}

// === updateState ===
// Finite State Machine (FSM) decision logic.
// Evaluates sensor inputs and internal flags to determine the robot's next behavior state.
// Priority order:
//   1. Collision or panic recovery (non-blocking phases)
//   2. Light-seeking if healthy light detected
//   3. Obstacle avoidance based on ultrasonic distance thresholds
//   4. Stuck detection fallback to panic if sensors are unhealthy
//   5. Default to cruise if no other condition is met
void updateState() {
  if (collisionPhase != 0) {
    currentState = STATE_COLLISION;
    return;
  }
  if (panicPhase != 0) {
    currentState = STATE_PANIC;
    return;
  }
 // Light-seeking takes priority if sensors are healthy
  bool leftHealthy = leftLightHealthy();
  bool rightHealthy = rightLightHealthy();
  bool eitherLight = (leftLight >= LIGHT_THRESHOLD && leftHealthy) || (rightLight >= LIGHT_THRESHOLD && rightHealthy);
  if (eitherLight) {
    currentState = STATE_SEEK_LIGHT;
    return;
  }


// Skip obstacle logic if both sensors report far distances
  if (distLeft > 150 && distRight > 150) {
    return; // Continue cruising or light-seeking
  }

// Obstacle proximity triggers STOP, COLLISION, or AVOID states
  if (distLeft < STOP_DISTANCE || distRight < STOP_DISTANCE) {
    currentState = STATE_STOP;
    return;
  }
  if (distLeft < DANGER_DISTANCE || distRight < DANGER_DISTANCE) {
    currentState = STATE_COLLISION;
    startCollisionAction();
    return;
  }
  if (distLeft < SAFE_DISTANCE || distRight < SAFE_DISTANCE) {
    currentState = STATE_AVOID;
    return;
  }

 // Stuck detection based on sensor health and lack of progress
  unsigned long now = millis();
  if (now - lastStuckCheck >= 1200) {
    lastStuckCheck = now;
    bool bothUltraBad = !leftUltraHealthy() && !rightUltraHealthy();
    bool bothLightBad = !leftHealthy && !rightHealthy;
    if (bothUltraBad || bothLightBad) {
      stuckCounter++;
      if (stuckCounter >= 2) {
        startPanicAction();
        currentState = STATE_PANIC;
        return;
      }
    } else stuckCounter = 0;
  }

  // Default state if no other condition is met
  currentState = STATE_CRUISE;
}

// === handleState ===
// Executes motor commands based on current FSM state.
// Each state corresponds to a specific behavior pattern.
void handleState() {
  switch (currentState) {
    case STATE_CRUISE:
      // Normal forward motion — zigzag if in single-eye mode
      if (SINGLE_ULTRASONIC_MODE) {
        maybeFlipZig();
        int baseSpeed = ZIGZAG;  // safer forward speed for single-eye
        if (zigState == 1) setMotorSpeeds(baseSpeed + ZIG_TURN_MAG, baseSpeed - ZIG_TURN_MAG);
        else setMotorSpeeds(baseSpeed - ZIG_TURN_MAG, baseSpeed + ZIG_TURN_MAG);
      } else {
        setMotorSpeeds(CRUISE_SPEED, CRUISE_SPEED);
      }
      break;

    case STATE_SEEK_LIGHT:
     // Accelerate toward brighter light if sensors are healthy
      {
        bool lHealthy = leftLightHealthy();
        bool rHealthy = rightLightHealthy();
        if (leftLight >= LIGHT_THRESHOLD && rightLight >= LIGHT_THRESHOLD && lHealthy && rHealthy) {
          setMotorSpeeds(FAST_SPEED, FAST_SPEED);
        } else if (leftLight >= LIGHT_THRESHOLD && lHealthy) {
          setMotorSpeeds(SLOW_SPEED, FAST_SPEED);
        } else if (rightLight >= LIGHT_THRESHOLD && rHealthy) {
          setMotorSpeeds(FAST_SPEED, SLOW_SPEED);
        } else {
          setMotorSpeeds(CRUISE_SPEED, SLOW_SPEED);
        }
        break;
      }

    case STATE_AVOID:
     // Steer away from nearby obstacles
      {
        if (!SINGLE_ULTRASONIC_MODE) {
          if (distLeft < SAFE_DISTANCE && distRight < SAFE_DISTANCE) {
            lastTurnDir = (random(0, 100) < 50) ? 1 : -1;
          }
          if (distLeft < SAFE_DISTANCE) {
            setMotorSpeeds(CRUISE_SPEED + 50, CRUISE_SPEED / 3);
            lastTurnDir = 1;
          } else if (distRight < SAFE_DISTANCE) {
            setMotorSpeeds(CRUISE_SPEED / 3, CRUISE_SPEED + 50);
            lastTurnDir = -1;
          }
        } else {
          // single-eye: slow down + zigzag
          int baseSpeed = SLOW_SPEED;
          if (zigState == 1) setMotorSpeeds(baseSpeed, baseSpeed + ZIG_TURN_MAG);
          else setMotorSpeeds(baseSpeed + ZIG_TURN_MAG, baseSpeed);
        }
        break;
      }

    case STATE_COLLISION:
      // Halt motion and wait for clearance
      // Execute non-blocking collision recovery
      processCollisionAction();
      break;

    case STATE_STOP:
      setMotorSpeeds(0, 0);
      if (distLeft > SAFE_DISTANCE && distRight > SAFE_DISTANCE) {
        clearCounter++;
        if (clearCounter >= CLEAR_REQUIRED) {
          currentState = STATE_CRUISE;
          clearCounter = 0;
        }
      } else clearCounter = 0;
      break;

    case STATE_PANIC:
      processPanicAction();
      break;
  }
}

// === printReadings ===
// Outputs current sensor readings and FSM state to serial monitor.
// Useful for debugging and runtime telemetry.
void printReadings() {
  if (millis() - lastPrintTime < printInterval) return;
  Serial.print("Ldist: ");
  Serial.print(distLeft);
  Serial.print(" | Rdist: ");
  Serial.print(distRight);
  Serial.print(" | Light L: ");
  Serial.print(leftLight);
  Serial.print(" R: ");
  Serial.print(rightLight);
  Serial.print(" | Mode: ");
  Serial.print(SINGLE_ULTRASONIC_MODE ? "SINGLE_EYE" : "DUAL_EYE");
  Serial.print(" | State: ");
  switch (currentState) {
    case STATE_CRUISE: Serial.print("CRUISE"); break;
    case STATE_SEEK_LIGHT: Serial.print("SEEK_LIGHT"); break;
    case STATE_AVOID: Serial.print("AVOID"); break;
    case STATE_COLLISION: Serial.print("COLLISION"); break;
    case STATE_STOP: Serial.print("STOP"); break;
    case STATE_PANIC: Serial.print("PANIC"); break;
  }
  Serial.print(" | lastCmd: ");
  Serial.print(lastLeftCmd);
  Serial.print(",");
  Serial.print(lastRightCmd);
  Serial.print(" | collPhase: ");
  Serial.print(collisionPhase);
  Serial.print(" panicPhase: ");
  Serial.print(panicPhase);
  Serial.println();
  lastPrintTime = millis();
}

// === setup ===
// Initializes pins, serial output, and sensor history buffers.
// Seeds random generator and prepares FSM for runtime.
void setup() {
  pinMode(trigLeft, OUTPUT);
  pinMode(echoLeft, INPUT);
  pinMode(trigRight, OUTPUT);
  pinMode(echoRight, INPUT);
  pinMode(leftLightPin, INPUT);
  pinMode(rightLightPin, INPUT);
  digitalWrite(trigLeft, LOW);
  digitalWrite(trigRight, LOW);
  Serial.begin(9600);
  delay(30);
  randomSeed(analogRead(A0)); // Entropy source for random turn direction

  Serial.println("Zumo FSM (adaptive single/dual ultrasonic) ready");

 // Initialize light sensor history for health checks
  for (int i = 0; i < HEALTH_WINDOW; i++) {
    leftLightHistory[i] = analogRead(leftLightPin);
    rightLightHistory[i] = analogRead(rightLightPin);
  }
  zigFlipMillis = millis();
  lastStuckCheck = millis();
}

// === loop ===
// Main runtime loop — reads sensors, updates FSM, executes behavior, and prints debug info.
void loop() {
  readUltrasonics();             // Update ultrasonic readings
  readLightSensors();            // Update light sensor readings
  updateState();                 // Decide next FSM state
  handleState();                 // Actuate motors based on state
  checkUltrasonicForStuck();    // Detect lack of progress
  printReadings();               // Output telemetry
  delay(loopDelay);             // Loop pacing
}