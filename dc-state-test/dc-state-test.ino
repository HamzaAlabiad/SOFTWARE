// Simple autonomous wheel-direction test for ESP32.
// No Serial Monitor input is required.
// Sequence per wheel: clockwise -> anticlockwise -> stop.
// Safety: keep robot lifted so wheels spin freely.

#include <Arduino.h>

struct MotorPins {
  uint8_t in1;
  uint8_t in2;
  const char* name;
};

// Motor map from your table
// MOTA -> Back Right -> GPIO27, GPIO14
// MOTB -> Front Right -> GPIO13, GPIO19
// MOTC -> Back Left -> GPIO4, GPIO16
// MOTD -> Front Left -> GPIO17, GPIO18
MotorPins motors[4] = {
    {27, 14, "MOTA BackRight"},
    {13, 19, "MOTB FrontRight"},
    {4, 16, "MOTC BackLeft"},
    {17, 18, "MOTD FrontLeft"},
};

const uint8_t LED_PIN = 2;  // Onboard LED on most ESP32 dev boards.

const uint32_t START_DELAY_MS = 6000;
const uint32_t CW_RUN_MS = 1700;
const uint32_t CCW_RUN_MS = 1700;
const uint32_t STOP_HOLD_MS = 1200;
const uint32_t BETWEEN_WHEELS_MS = 900;

bool testFinished = false;

void blink(uint8_t count, uint16_t onMs, uint16_t offMs) {
  for (uint8_t i = 0; i < count; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(onMs);
    digitalWrite(LED_PIN, LOW);
    delay(offMs);
  }
}

void setMotorState(const MotorPins& m, bool s1, bool s2) {
  digitalWrite(m.in1, s1 ? HIGH : LOW);
  digitalWrite(m.in2, s2 ? HIGH : LOW);
}

void stopAllMotors() {
  for (uint8_t i = 0; i < 4; i++) {
    setMotorState(motors[i], false, false);
  }
}

void runWheelSequence(uint8_t motorIdx) {
  const MotorPins& m = motors[motorIdx];

  // Marker: wheel number (1..4)
  blink(motorIdx + 1, 260, 180);
  delay(250);

  // 1) Clockwise assumption: IN1=1, IN2=0
  setMotorState(m, true, false);
  delay(CW_RUN_MS);

  // 2) Anticlockwise assumption: IN1=0, IN2=1
  setMotorState(m, false, true);
  delay(CCW_RUN_MS);

  // 3) Stop: IN1=0, IN2=0
  setMotorState(m, false, false);
  delay(STOP_HOLD_MS);

  // End-of-wheel marker
  blink(2, 200, 160);
  delay(BETWEEN_WHEELS_MS);
}

void setup() {
  Serial.begin(115200);  // Optional logs only

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  for (uint8_t i = 0; i < 4; i++) {
    pinMode(motors[i].in1, OUTPUT);
    pinMode(motors[i].in2, OUTPUT);
  }

  stopAllMotors();

  Serial.println();
  Serial.println("3-state wheel test starting.");
  Serial.println("Per wheel: clockwise -> anticlockwise -> stop.");

  // Countdown marker
  blink(5, 140, 140);
  delay(START_DELAY_MS);

  for (uint8_t i = 0; i < 4; i++) {
    runWheelSequence(i);
  }

  stopAllMotors();
  testFinished = true;

  Serial.println("3-state wheel test complete.");

  // Finished marker
  blink(4, 450, 220);
}

void loop() {
  // Idle heartbeat after test completion.
  if (testFinished) {
    digitalWrite(LED_PIN, HIGH);
    delay(80);
    digitalWrite(LED_PIN, LOW);
    delay(1200);
    return;
  }

  delay(50);
}
