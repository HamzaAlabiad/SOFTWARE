// DRV8825 + NEMA17 serial speed/direction test for ESP32.
// Open Serial Monitor at 115200 baud and set line ending to "Newline".
//
// Wiring:
// GPIO26 -> STEP
// GPIO25 -> DIR
// GPIO27 -> EN
//
// DRV8825 requirements:
// EN is active LOW: 0V enabled, HIGH disabled.
// RST and SLP must be tied HIGH on the driver/PCB.
// ESP32 GND and DRV8825 GND must be connected together.

const int STEP_PIN = 26;
const int DIR_PIN = 25;
const int EN_PIN = 27;

const int STEPS_PER_REV = 200;      // Full-step motor value. Microstepping changes effective steps/rev.
const int STEP_HIGH_US = 10;        // DRV8825 needs about 1.9 us minimum.
const float MIN_SPEED_SPS = 0.0f;
const float MAX_SPEED_SPS = 1200.0f;

String inputLine = "";
float speedSps = 100.0f;            // Steps per second.
int directionSign = 1;              // 1 or -1.
bool enabled = true;
uint32_t lastStepUs = 0;

void printHelp() {
  Serial.println();
  Serial.println("Commands:");
  Serial.println("  d 1        direction one way");
  Serial.println("  d -1       direction other way");
  Serial.println("  s 100      speed in steps/second");
  Serial.println("  r 30       speed in RPM, based on STEPS_PER_REV");
  Serial.println("  stop       stop stepping");
  Serial.println("  start      enable stepping");
  Serial.println("  status     print current settings");
  Serial.println();
}

void printStatus() {
  float rpm = (speedSps * 60.0f) / (float)STEPS_PER_REV;
  Serial.print("Enabled=");
  Serial.print(enabled ? "YES" : "NO");
  Serial.print(" | Direction=");
  Serial.print(directionSign);
  Serial.print(" | Speed=");
  Serial.print(speedSps, 1);
  Serial.print(" steps/s");
  Serial.print(" | RPM=");
  Serial.println(rpm, 2);
}

void applyOutputs() {
  digitalWrite(DIR_PIN, directionSign > 0 ? HIGH : LOW);
  digitalWrite(EN_PIN, enabled ? LOW : HIGH);
}

void setSpeedSps(float sps) {
  speedSps = constrain(sps, MIN_SPEED_SPS, MAX_SPEED_SPS);
  if (speedSps <= 0.01f) {
    enabled = false;
  } else {
    enabled = true;
  }
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
  } else if (cmd == "stop") {
    enabled = false;
    applyOutputs();
    Serial.println("Stopped");
  } else if (cmd == "start") {
    if (speedSps > 0.01f) {
      enabled = true;
      applyOutputs();
    }
    Serial.println("Started");
  } else if (cmd.startsWith("d ")) {
    int d = cmd.substring(2).toInt();
    directionSign = (d >= 0) ? 1 : -1;
    applyOutputs();
    Serial.println("Direction updated");
  } else if (cmd.startsWith("s ")) {
    float sps = cmd.substring(2).toFloat();
    setSpeedSps(sps);
    Serial.println("Speed updated in steps/s");
  } else if (cmd.startsWith("r ")) {
    float rpm = cmd.substring(2).toFloat();
    setSpeedSps((rpm * (float)STEPS_PER_REV) / 60.0f);
    Serial.println("Speed updated in RPM");
  } else {
    Serial.println("Unknown command. Type: help");
  }

  printStatus();
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
      if (inputLine.length() > 40) {
        inputLine = "";
        Serial.println("Input too long. Type: help");
      }
    }
  }
}

void runStepper() {
  if (!enabled || speedSps <= 0.01f) {
    digitalWrite(STEP_PIN, LOW);
    return;
  }

  uint32_t now = micros();
  uint32_t intervalUs = (uint32_t)(1000000.0f / speedSps);
  intervalUs = max(intervalUs, (uint32_t)(STEP_HIGH_US + 50));

  if ((uint32_t)(now - lastStepUs) >= intervalUs) {
    lastStepUs = now;
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(STEP_HIGH_US);
    digitalWrite(STEP_PIN, LOW);
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);

  digitalWrite(STEP_PIN, LOW);
  applyOutputs();

  Serial.println("DRV8825 serial stepper test started");
  printHelp();
  printStatus();
}

void loop() {
  readSerialCommands();
  runStepper();
}
