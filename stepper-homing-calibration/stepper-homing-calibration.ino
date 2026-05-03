// DRV8825 + limit switch homing calibration sketch for ESP32.
// Open Serial Monitor at 115200 baud and set line ending to "Newline".
//
// Stepper wiring:
// GPIO26 -> DRV8825 STEP
// GPIO25 -> DRV8825 DIR
// GPIO27 -> DRV8825 EN
//
// Limit switch wiring, default:
// GPIO32 -> limit switch signal
// Uses INPUT_PULLUP, but this switch/module is inverted: pressed = HIGH by default.
//
// DRV8825 requirements:
// EN is active LOW: 0V enabled, HIGH disabled.
// RST and SLP must be tied HIGH on the driver/PCB.
// ESP32 GND and DRV8825 GND must be connected together.

const int STEP_PIN = 26;
const int DIR_PIN = 25;
const int EN_PIN = 27;
const int LIMIT_PIN = 32;

const int STEP_HIGH_US = 10;
const float MIN_SPEED_SPS = 0.0f;
const float MAX_SPEED_SPS = 2500.0f;
const uint32_t LIMIT_DEBOUNCE_MS = 25;

String inputLine = "";

float speedSps = 500.0f;
float homeSpeedSps = 500.0f;
float backoffSpeedSps = 500.0f;
int directionSign = 1;
int limitDirectionSign = -1;
const int downDirectionSign = 1;
const int upDirectionSign = -1;
long homeBackoffSteps = 100;

bool enabled = false;
bool continuousRun = false;
bool moveActive = false;
long moveRemainingSteps = 0;
int moveDirectionSign = 1;
float moveSpeedSps = 500.0f;

bool homingActive = false;
uint8_t homingPhase = 0;  // 0 idle, 1 seek switch, 2 back off, 3 done.

long currentPositionSteps = 0;
long lastMoveStartSteps = 0;
long lastMoveDeltaSteps = 0;
long calibrationStartSteps = 0;
bool calibrationActive = false;
float stepsPerMm = 0.0f;

bool limitActiveLow = false;
bool limitStablePressed = false;
bool limitLastRawPressed = false;
uint32_t limitLastChangeMs = 0;

uint32_t lastStepUs = 0;
uint32_t lastStatusMs = 0;

void printHelp() {
  Serial.println();
  Serial.println("Manual movement:");
  Serial.println("  start              continuous run using current direction/speed");
  Serial.println("  stop               stop motor");
  Serial.println("  d 1                direction one way");
  Serial.println("  d -1               direction other way");
  Serial.println("  s 200              speed in steps/second");
  Serial.println("  up                 continuous run upward, same as d -1 + start");
  Serial.println("  down               continuous run downward, same as d 1 + start");
  Serial.println("  jogup 500          move upward by 500 steps");
  Serial.println("  jogdown 500        move downward by 500 steps");
  Serial.println("  jog 500            move +500 steps");
  Serial.println("  jog -500           move -500 steps");
  Serial.println("  goto 1000          move to absolute step position");
  Serial.println("  zero               set current position to 0");
  Serial.println();
  Serial.println("Limit switch:");
  Serial.println("  ls                 print limit switch state");
  Serial.println("  lsinv 0            pressed = LOW, normal switch to GND");
  Serial.println("  lsinv 1            pressed = HIGH, current default");
  Serial.println();
  Serial.println("Homing calibration:");
  Serial.println("  hdir -1            direction toward limit switch");
  Serial.println("  hspeed 120         homing seek speed in steps/second");
  Serial.println("  backoff 100        steps to move away from switch after hit");
  Serial.println("  home               seek switch, back off, set position to 0");
  Serial.println();
  Serial.println("Height calibration:");
  Serial.println("  calstart           remember current step position");
  Serial.println("  caldone 25.0       enter measured movement in mm");
  Serial.println("  mm 10              move by +10 mm after caldone");
  Serial.println("  mm -10             move by -10 mm after caldone");
  Serial.println();
  Serial.println("Info:");
  Serial.println("  status             print current settings");
  Serial.println("  help               print this menu");
  Serial.println();
}

bool readLimitRawPressed() {
  int raw = digitalRead(LIMIT_PIN);
  return limitActiveLow ? (raw == LOW) : (raw == HIGH);
}

void updateLimitDebounce() {
  bool rawPressed = readLimitRawPressed();
  uint32_t now = millis();

  if (rawPressed != limitLastRawPressed) {
    limitLastRawPressed = rawPressed;
    limitLastChangeMs = now;
  }

  if ((uint32_t)(now - limitLastChangeMs) >= LIMIT_DEBOUNCE_MS) {
    limitStablePressed = rawPressed;
  }
}

void applyOutputs() {
  digitalWrite(DIR_PIN, directionSign > 0 ? HIGH : LOW);
  digitalWrite(EN_PIN, enabled ? LOW : HIGH);
}

void setEnabled(bool on) {
  enabled = on;
  applyOutputs();
  if (!on) {
    digitalWrite(STEP_PIN, LOW);
    continuousRun = false;
    moveActive = false;
    homingActive = false;
    homingPhase = 0;
  }
}

void printStatus() {
  Serial.println();
  Serial.print("Enabled=");
  Serial.print(enabled ? "YES" : "NO");
  Serial.print(" | PosSteps=");
  Serial.print(currentPositionSteps);
  Serial.print(" | Dir=");
  Serial.print(directionSign);
  Serial.print(" | UpDir=");
  Serial.print(upDirectionSign);
  Serial.print(" | DownDir=");
  Serial.print(downDirectionSign);
  Serial.print(" | Speed=");
  Serial.print(speedSps, 1);
  Serial.print(" steps/s");
  Serial.print(" | Limit=");
  Serial.print(limitStablePressed ? "PRESSED" : "OPEN");
  Serial.print(" | LimitDir=");
  Serial.print(limitDirectionSign);
  Serial.print(" | Backoff=");
  Serial.print(homeBackoffSteps);
  Serial.print(" steps");
  if (stepsPerMm > 0.0f) {
    Serial.print(" | Steps/mm=");
    Serial.print(stepsPerMm, 3);
    Serial.print(" | PosMm=");
    Serial.print((float)currentPositionSteps / stepsPerMm, 3);
  }
  Serial.println();
}

void startRelativeMove(long steps, float sps) {
  if (steps == 0) {
    return;
  }

  moveDirectionSign = (steps >= 0) ? 1 : -1;
  moveRemainingSteps = labs(steps);
  moveSpeedSps = constrain(sps, 1.0f, MAX_SPEED_SPS);
  directionSign = moveDirectionSign;
  continuousRun = false;
  moveActive = true;
  enabled = true;
  lastMoveStartSteps = currentPositionSteps;
  applyOutputs();
}

void finishMove() {
  moveActive = false;
  lastMoveDeltaSteps = currentPositionSteps - lastMoveStartSteps;
  Serial.print("Move done. Delta steps=");
  Serial.print(lastMoveDeltaSteps);
  Serial.print(" | Position=");
  Serial.println(currentPositionSteps);
}

bool doOneStep(float sps, int sign) {
  if (!enabled || sps <= 0.01f) {
    digitalWrite(STEP_PIN, LOW);
    return false;
  }

  uint32_t now = micros();
  uint32_t intervalUs = (uint32_t)(1000000.0f / sps);
  intervalUs = max(intervalUs, (uint32_t)(STEP_HIGH_US + 50));

  if ((uint32_t)(now - lastStepUs) < intervalUs) {
    return false;
  }

  lastStepUs = now;
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(STEP_HIGH_US);
  digitalWrite(STEP_PIN, LOW);
  currentPositionSteps += (sign >= 0) ? 1 : -1;
  return true;
}

void runMotion() {
  if (homingActive) {
    if (homingPhase == 1) {
      directionSign = limitDirectionSign;
      applyOutputs();

      if (limitStablePressed) {
        Serial.println("Limit switch hit. Backing off...");
        startRelativeMove(-limitDirectionSign * homeBackoffSteps, backoffSpeedSps);
        homingPhase = 2;
        return;
      }

      doOneStep(homeSpeedSps, limitDirectionSign);
      return;
    }

    if (homingPhase == 2) {
      if (moveActive) {
        if (doOneStep(moveSpeedSps, moveDirectionSign)) {
          moveRemainingSteps--;
          if (moveRemainingSteps <= 0) {
            finishMove();
            currentPositionSteps = 0;
            homingActive = false;
            homingPhase = 0;
            enabled = false;
            applyOutputs();
            Serial.println("Homing complete. Current position set to 0.");
            printStatus();
          }
        }
      }
      return;
    }
  }

  if (moveActive) {
    if (doOneStep(moveSpeedSps, moveDirectionSign)) {
      moveRemainingSteps--;
      if (moveRemainingSteps <= 0) {
        finishMove();
      }
    }
    return;
  }

  if (continuousRun) {
    doOneStep(speedSps, directionSign);
  }
}

void startHome() {
  if (limitStablePressed) {
    Serial.println("Limit already pressed. Backing off first...");
    homingActive = true;
    homingPhase = 2;
    startRelativeMove(-limitDirectionSign * homeBackoffSteps, backoffSpeedSps);
    return;
  }

  Serial.println("Homing started. Seeking limit switch...");
  continuousRun = false;
  moveActive = false;
  homingActive = true;
  homingPhase = 1;
  enabled = true;
  directionSign = limitDirectionSign;
  applyOutputs();
}

void handleCommand(String cmd) {
  cmd.trim();
  cmd.toLowerCase();
  if (cmd.length() == 0) {
    return;
  }

  if (cmd == "help" || cmd == "h") {
    printHelp();
  } else if (cmd == "status") {
    printStatus();
  } else if (cmd == "ls") {
    Serial.print("Limit raw=");
    Serial.print(readLimitRawPressed() ? "PRESSED" : "OPEN");
    Serial.print(" stable=");
    Serial.println(limitStablePressed ? "PRESSED" : "OPEN");
  } else if (cmd == "stop") {
    setEnabled(false);
    Serial.println("Stopped");
  } else if (cmd == "start") {
    continuousRun = true;
    moveActive = false;
    homingActive = false;
    enabled = true;
    applyOutputs();
    Serial.println("Continuous run started");
  } else if (cmd == "up") {
    directionSign = upDirectionSign;
    continuousRun = true;
    moveActive = false;
    homingActive = false;
    enabled = true;
    applyOutputs();
    Serial.println("Moving upward continuously");
  } else if (cmd == "down") {
    directionSign = downDirectionSign;
    continuousRun = true;
    moveActive = false;
    homingActive = false;
    enabled = true;
    applyOutputs();
    Serial.println("Moving downward continuously");
  } else if (cmd == "zero") {
    currentPositionSteps = 0;
    Serial.println("Current position set to 0");
  } else if (cmd == "home") {
    startHome();
  } else if (cmd == "calstart") {
    calibrationStartSteps = currentPositionSteps;
    calibrationActive = true;
    Serial.print("Calibration start position=");
    Serial.println(calibrationStartSteps);
  } else if (cmd.startsWith("caldone ")) {
    float mm = cmd.substring(8).toFloat();
    long delta = currentPositionSteps - calibrationStartSteps;
    if (!calibrationActive || fabsf(mm) < 0.001f || delta == 0) {
      Serial.println("Calibration failed. Use calstart, move, then caldone <measured_mm>.");
    } else {
      stepsPerMm = fabsf((float)delta / mm);
      calibrationActive = false;
      Serial.print("Calibration done. Delta steps=");
      Serial.print(delta);
      Serial.print(" | Measured mm=");
      Serial.print(mm, 3);
      Serial.print(" | Steps/mm=");
      Serial.println(stepsPerMm, 3);
    }
  } else if (cmd.startsWith("d ")) {
    int d = cmd.substring(2).toInt();
    directionSign = (d >= 0) ? 1 : -1;
    applyOutputs();
    Serial.println("Direction updated");
  } else if (cmd.startsWith("s ")) {
    speedSps = constrain(cmd.substring(2).toFloat(), MIN_SPEED_SPS, MAX_SPEED_SPS);
    Serial.println("Speed updated");
  } else if (cmd.startsWith("jog ")) {
    long steps = cmd.substring(4).toInt();
    startRelativeMove(steps, speedSps);
    Serial.print("Jog started: ");
    Serial.print(steps);
    Serial.println(" steps");
  } else if (cmd.startsWith("jogup ")) {
    long steps = labs(cmd.substring(6).toInt());
    startRelativeMove(upDirectionSign * steps, speedSps);
    Serial.print("Jog up started: ");
    Serial.print(steps);
    Serial.println(" steps");
  } else if (cmd.startsWith("jogdown ")) {
    long steps = labs(cmd.substring(8).toInt());
    startRelativeMove(downDirectionSign * steps, speedSps);
    Serial.print("Jog down started: ");
    Serial.print(steps);
    Serial.println(" steps");
  } else if (cmd.startsWith("goto ")) {
    long target = cmd.substring(5).toInt();
    startRelativeMove(target - currentPositionSteps, speedSps);
    Serial.print("Moving to absolute step position ");
    Serial.println(target);
  } else if (cmd.startsWith("mm ")) {
    float mm = cmd.substring(3).toFloat();
    if (stepsPerMm <= 0.0f) {
      Serial.println("No height calibration yet. Use calstart, move, then caldone <mm>.");
    } else {
      long steps = lroundf(mm * stepsPerMm);
      startRelativeMove(steps, speedSps);
      Serial.print("Moving ");
      Serial.print(mm, 3);
      Serial.print(" mm = ");
      Serial.print(steps);
      Serial.println(" steps");
    }
  } else if (cmd.startsWith("hdir ")) {
    int d = cmd.substring(5).toInt();
    limitDirectionSign = (d >= 0) ? 1 : -1;
    Serial.println("Limit direction updated");
  } else if (cmd.startsWith("hspeed ")) {
    homeSpeedSps = constrain(cmd.substring(7).toFloat(), 1.0f, MAX_SPEED_SPS);
    Serial.println("Home speed updated");
  } else if (cmd.startsWith("backoff ")) {
    homeBackoffSteps = max(0L, cmd.substring(8).toInt());
    Serial.println("Backoff steps updated");
  } else if (cmd.startsWith("lsinv ")) {
    int inv = cmd.substring(6).toInt();
    limitActiveLow = (inv == 0);
    Serial.println(limitActiveLow ? "Limit mode: pressed LOW" : "Limit mode: pressed HIGH");
  } else {
    Serial.println("Unknown command. Type: help");
  }
}

void readSerialCommands() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (inputLine.length() > 0) {
        handleCommand(inputLine);
        inputLine = "";
      }
    } else {
      inputLine += c;
      if (inputLine.length() > 60) {
        inputLine = "";
        Serial.println("Input too long. Type: help");
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  pinMode(LIMIT_PIN, INPUT_PULLUP);

  digitalWrite(STEP_PIN, LOW);
  setEnabled(false);

  limitLastRawPressed = readLimitRawPressed();
  limitStablePressed = limitLastRawPressed;
  limitLastChangeMs = millis();

  Serial.println("Stepper homing calibration started");
  printHelp();
  printStatus();
}

void loop() {
  updateLimitDebounce();
  readSerialCommands();
  runMotion();

  if (millis() - lastStatusMs >= 2000) {
    lastStatusMs = millis();
    if (enabled || homingActive || moveActive || continuousRun) {
      printStatus();
    }
  }
}
