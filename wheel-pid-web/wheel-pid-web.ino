// Wheel speed PID tuning sketch for ESP32 mecanum base.
// Display and tuning are done via Wi-Fi web page (no serial interaction required).
// Connect to AP: ESP32-PID-TUNE / pid12345, then open http://192.168.4.1

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <math.h>
#include <Wire.h>

struct EncoderPins {
  uint8_t a;
  uint8_t b;
  const char* name;
};

struct MotorPins {
  uint8_t in1;
  uint8_t in2;
  const char* name;
  uint8_t encIndex;
  uint8_t ch1;
  uint8_t ch2;
};

struct PIDState {
  float kp;
  float ki;
  float kd;
  float integral;
  float prevError;
  float out;
};

struct CascadePidState {
  float kp;
  float ki;
  float kd;
  float integral;
  float prevError;
  float out;
};

enum ControlMode : uint8_t {
  MODE_SINGLE_WHEEL_VELOCITY = 0,
  MODE_POSITION_HEADING = 1,
};

struct MinPwmSweepState {
  bool active;
  int8_t direction;
  uint8_t startPwm;
  uint8_t endPwm;
  uint8_t stepPwm;
  uint16_t holdMs;
  uint8_t currentPwm;
  uint8_t detectedPwm;
  uint8_t stableMoveTicks;
  float detectRpmThreshold;
  float avgAbsRpm;
  uint32_t stepStartMs;
};

EncoderPins encoders[4] = {
    {39, 36, "ENC1 BackLeft"},
    {34, 35, "ENC2 FrontLeft"},
    {33, 32, "ENC3 FrontRight"},
    {25, 26, "ENC4 BackRight"},
};

// Motor mapping validated in your testing:
// MOTA -> Back Right -> GPIO27, GPIO14 -> ENC4
// MOTB -> Front Right -> GPIO13, GPIO19 -> ENC3
// MOTC -> Back Left  -> GPIO4, GPIO16  -> ENC1
// MOTD -> Front Left -> GPIO17, GPIO18 -> ENC2
MotorPins motors[4] = {
    {27, 14, "MOTA BackRight", 3, 0, 1},
    {13, 19, "MOTB FrontRight", 2, 2, 3},
    {4, 16,  "MOTC BackLeft",   0, 4, 5},
    {17, 18, "MOTD FrontLeft",  1, 6, 7},
};

volatile int32_t encoderCount[4] = {0, 0, 0, 0};
volatile uint8_t encoderPrevState[4] = {0, 0, 0, 0};

int32_t lastEncoderCount[4] = {0, 0, 0, 0};
float measuredRpmRaw[4] = {0, 0, 0, 0};
float measuredRpm[4] = {0, 0, 0, 0};
float targetRpm[4] = {0, 0, 0, 0};
int16_t pwmCmd[4] = {0, 0, 0, 0};
int8_t rpmSign[4] = {1, 1, -1, -1};

PIDState pid[4] = {
  {50.0f, 6.0f, 0.0f, 0.0f, 0.0f, 0.0f},
  {50.0f, 6.0f, 0.0f, 0.0f, 0.0f, 0.0f},
  {50.0f, 6.0f, 0.0f, 0.0f, 0.0f, 0.0f},
  {50.0f, 6.0f, 0.0f, 0.0f, 0.0f, 0.0f},
};

  MinPwmSweepState minSweep = {
    false,
    1,
    20,
    90,
    2,
    700,
    0,
    0,
    0,
    2.5f,
    0.0f,
    0,
  };

WebServer server(80);

const char* AP_SSID = "ESP32-PID-TUNE";
const char* AP_PASS = "pid12345";

const float COUNTS_PER_WHEEL_REV = 4346.8f;  // From your calibration data
const uint32_t CONTROL_INTERVAL_MS = 20;      // 50 Hz speed loop
const uint16_t PWM_FREQ = 1000;
const uint8_t PWM_RES_BITS = 8;
const int PWM_MAX = 255;
const float TARGET_RPM_MAX = 60.0f;

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
void pwmAttachPinCompat(uint8_t pin, uint8_t channelHint) {
  (void)channelHint;
  // ESP32 Arduino Core 3.x API: bind PWM directly to pin.
  ledcAttach(pin, PWM_FREQ, PWM_RES_BITS);
}

void pwmWriteCompat(uint8_t pin, uint8_t channelHint, uint32_t duty) {
  (void)channelHint;
  ledcWrite(pin, duty);
}
#else
void pwmAttachPinCompat(uint8_t pin, uint8_t channel) {
  // ESP32 Arduino Core 2.x API: setup channel then attach pin.
  ledcSetup(channel, PWM_FREQ, PWM_RES_BITS);
  ledcAttachPin(pin, channel);
}

void pwmWriteCompat(uint8_t pin, uint8_t channel, uint32_t duty) {
  (void)pin;
  ledcWrite(channel, duty);
}
#endif

float pidTargetRpm = 50.0f;
uint8_t activeWheel = 0;
bool controllerEnabled = false;
uint8_t minPwm = 160;
bool rawModeEnabled = false;
int16_t rawPwmCmd = 0;
ControlMode controlMode = MODE_SINGLE_WHEEL_VELOCITY;

CascadePidState posPid = {0.05f, 0.0018f, 0.0f, 0.0f, 0.0f, 0.0f};
CascadePidState headingPid = {1.80f, 0.00f, 0.08f, 0.0f, 0.0f, 0.0f};

float positionTargetCounts = 0.0f;
float positionCurrentCounts = 0.0f;
float positionErrorCounts = 0.0f;
float positionTolCm = 1.5f;
float positionCmdRpm = 0.0f;
float positionStepCm = 30.0f;
float wheelDiameterCm = 9.7f;
bool positionDone = false;
uint8_t positionDoneTicks = 0;

float headingTargetDeg = 0.0f;
float headingErrorDeg = 0.0f;
float headingCorrRpm = 0.0f;
float headingCorrMaxRpm = 20.0f;
int8_t headingCorrSign = -1;
float headingDeadbandDeg = 1.5f;
float headingEnableBaseRpm = 0.8f;
bool autoLockHeadingOnStep = true;
bool holdHeadingAtStop = true;
bool yawHoldInPlaceMode = false;
float headingHoldMaxRpm = 10.0f;
float headingHoldDeadbandDeg = 2.0f;
CascadePidState headingHoldPid = {0.65f, 0.00f, 1.20f, 0.0f, 0.0f, 0.0f};
float headingHoldMinRpm = 1.2f;  // legacy, no longer used in control loop
uint8_t holdMinPwm = 0;          // kept for UI compatibility; intentionally not used in hold control
float headingHoldEnterDeg = 5.0f;
float headingHoldExitDeg = 1.0f;
float headingHoldExitRateDps = 2.5f;
float headingHoldKickErrDeg = 10.0f;
float headingHoldKickGyroDps = 1.0f;
float headingCorrSlewRpmPerSec = 70.0f;
bool headingHoldActive = false;
float headingHoldStableMs = 0.0f;
float headingCorrCmdPrev = 0.0f;

// Sequenced move command state: first rotate to heading, then drive distance while holding heading.
bool poseMoveActive = false;
uint8_t poseMovePhase = 0;  // 0=idle, 1=turning, 2=driving
float poseMoveDistanceCm = 0.0f;
float poseMoveStartCounts = 0.0f;
float poseMoveTargetCounts = 0.0f;

// Wheel-loop gain scheduling for gentle yaw hold behavior.
float yawHoldWheelKpScale = 0.78f;
float yawHoldWheelKdScale = 0.20f;
int16_t yawHoldWheelPwmMax = 255;
float yawHoldTargetDeadbandRpm = 1.0f;
uint8_t yawHoldBreakawayPwm = 12;
float yawHoldBreakawayErrDeg = 9.0f;

// In-place yaw speed profile: cruise fast, then transition to a low-speed final capture zone near target.
float yawHoldCruiseRpm = 50.0f;
float yawHoldMinApproachRpm = 30.0f;
float yawHoldSlowdownStartDeg = 30.0f;
float yawHoldFinalCaptureDeg = 10.0f;
float yawHoldFinalCaptureRpm = 5.5f;
float yawHoldFinalMinRpm = 2.0f;
float headingHoldSettleHoldMs = 320.0f;

bool imuOk = false;
float imuYawDeg = 0.0f;
float imuGyroDps = 0.0f;
float imuGyroDpsFilt = 0.0f;
float headingGyroLpfAlpha = 0.18f;
float imuBiasDps = 0.0f;
float imuAccXg = 0.0f;
float imuAccYg = 0.0f;
float imuAccXYg = 0.0f;
float imuAccAlpha = 0.20f;
float rpmFilterAlpha = 0.02f;

bool slipEnabled = true;
bool slipDetected = false;
float slipCandidateMs = 0.0f;
float slipWheelRpmThresh = 12.0f;
float slipAccelThreshG = 0.03f;
float slipDetectMs = 300.0f;
float slipReleaseMs = 200.0f;
float slipMaxRpm = 25.0f;
float slipAvgWheelRpm = 0.0f;

// Runtime debug telemetry for yaw-hold diagnostics.
bool debugSerialEnabled = false;
uint16_t debugSerialPeriodMs = 120;
uint32_t lastDebugSerialMs = 0;
uint8_t debugYawPhase = 0;  // 0=idle/off, 1=moving correction, 2=hold active, 3=hold settled
bool debugErrSettled = false;
bool debugRateSettled = false;
float debugRawHeadingErrDeg = 0.0f;
float debugUsedHeadingErrDeg = 0.0f;
float debugCorrTargetRpm = 0.0f;
float debugCorrAfterSlewRpm = 0.0f;
float debugWheelErr[4] = {0, 0, 0, 0};
float debugWheelPidOut[4] = {0, 0, 0, 0};
int16_t debugWheelCmdPreBreakaway[4] = {0, 0, 0, 0};

const uint8_t MPU6050_ADDR = 0x68;
const float MPU6050_GYRO_Z_SCALE = 65.5f;  // LSB/(deg/s) at +-500 dps
const float MPU6050_ACC_SCALE = 16384.0f;  // LSB/g at +-2g

const uint8_t MIN_SWEEP_MIN_PWM = 0;
const uint8_t MIN_SWEEP_MAX_PWM = 255;

uint32_t lastControlMs = 0;
uint32_t lastPrintMs = 0;

bool isInputOnlyPin(uint8_t pin) {
  return pin == 34 || pin == 35 || pin == 36 || pin == 39;
}

float wrapAngleDeg(float angle) {
  while (angle > 180.0f) angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}

float shortestAngleErrorDeg(float targetDeg, float currentDeg) {
  float d = (targetDeg - currentDeg) * DEG_TO_RAD;
  return atan2f(sinf(d), cosf(d)) * RAD_TO_DEG;
}

float countsToCm(float counts) {
  float d = constrain(wheelDiameterCm, 1.0f, 30.0f);
  float countsPerCm = COUNTS_PER_WHEEL_REV / (PI * d);
  return counts / countsPerCm;
}

float cmToCounts(float cm) {
  float d = constrain(wheelDiameterCm, 1.0f, 30.0f);
  float countsPerCm = COUNTS_PER_WHEEL_REV / (PI * d);
  return cm * countsPerCm;
}

void resetCascadePid(CascadePidState& s) {
  s.integral = 0.0f;
  s.prevError = 0.0f;
  s.out = 0.0f;
}

void resetHighLevelControllers() {
  resetCascadePid(posPid);
  resetCascadePid(headingPid);
  resetCascadePid(headingHoldPid);
  positionCmdRpm = 0.0f;
  headingCorrRpm = 0.0f;
  positionDone = false;
  positionDoneTicks = 0;
  headingHoldActive = false;
  headingHoldStableMs = 0.0f;
  headingCorrCmdPrev = 0.0f;
  yawHoldInPlaceMode = false;
  slipCandidateMs = 0.0f;
  slipDetected = false;
  debugYawPhase = 0;
  debugErrSettled = false;
  debugRateSettled = false;
  debugRawHeadingErrDeg = 0.0f;
  debugUsedHeadingErrDeg = 0.0f;
  debugCorrTargetRpm = 0.0f;
  debugCorrAfterSlewRpm = 0.0f;
}

void cancelPoseMove() {
  poseMoveActive = false;
  poseMovePhase = 0;
  poseMoveDistanceCm = 0.0f;
  poseMoveStartCounts = 0.0f;
  poseMoveTargetCounts = 0.0f;
}

bool imuWriteReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool imuReadRegs(uint8_t startReg, uint8_t* buf, uint8_t len) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(startReg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  uint8_t read = Wire.requestFrom((int)MPU6050_ADDR, (int)len, (int)true);
  if (read != len) {
    return false;
  }

  for (uint8_t i = 0; i < len; i++) {
    buf[i] = Wire.read();
  }
  return true;
}

bool imuReadGyroZRaw(int16_t& gzRaw) {
  uint8_t data[2];
  if (!imuReadRegs(0x47, data, 2)) {
    return false;
  }
  gzRaw = (int16_t)(((uint16_t)data[0] << 8) | data[1]);
  return true;
}

bool imuReadAccelXYRaw(int16_t& axRaw, int16_t& ayRaw) {
  uint8_t data[4];
  if (!imuReadRegs(0x3B, data, 4)) {
    return false;
  }
  axRaw = (int16_t)(((uint16_t)data[0] << 8) | data[1]);
  ayRaw = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
  return true;
}

void imuZeroYaw() {
  imuYawDeg = 0.0f;
}

bool imuCalibrateBias(uint16_t samples) {
  if (!imuOk || samples < 20) {
    return false;
  }

  float sum = 0.0f;
  uint16_t okCount = 0;
  for (uint16_t i = 0; i < samples; i++) {
    int16_t gzRaw = 0;
    if (imuReadGyroZRaw(gzRaw)) {
      sum += ((float)gzRaw) / MPU6050_GYRO_Z_SCALE;
      okCount++;
    }
    delay(2);
  }

  if (okCount < samples / 2) {
    return false;
  }

  imuBiasDps = sum / (float)okCount;
  return true;
}

bool imuInit() {
  Wire.begin();
  delay(10);

  bool ok = true;
  ok &= imuWriteReg(0x6B, 0x00);  // Wake up MPU6050
  ok &= imuWriteReg(0x1B, 0x08);  // Gyro FS_SEL=1 => +-500 dps
  ok &= imuWriteReg(0x1A, 0x03);  // DLPF for less noise
  delay(50);

  imuOk = ok;
  if (imuOk) {
    imuBiasDps = 0.0f;
    imuGyroDps = 0.0f;
    imuYawDeg = 0.0f;
    imuCalibrateBias(400);
  }
  return imuOk;
}

void imuUpdate(float dt) {
  if (!imuOk || dt <= 0.0001f) {
    imuGyroDps = 0.0f;
    imuGyroDpsFilt = 0.0f;
    imuAccXg = 0.0f;
    imuAccYg = 0.0f;
    imuAccXYg = 0.0f;
    return;
  }

  int16_t axRaw = 0;
  int16_t ayRaw = 0;
  if (imuReadAccelXYRaw(axRaw, ayRaw)) {
    float ax = ((float)axRaw) / MPU6050_ACC_SCALE;
    float ay = ((float)ayRaw) / MPU6050_ACC_SCALE;
    float a = constrain(imuAccAlpha, 0.02f, 1.0f);
    imuAccXg += a * (ax - imuAccXg);
    imuAccYg += a * (ay - imuAccYg);
    imuAccXYg = sqrtf(imuAccXg * imuAccXg + imuAccYg * imuAccYg);
  }

  int16_t gzRaw = 0;
  if (!imuReadGyroZRaw(gzRaw)) {
    imuGyroDps = 0.0f;
    return;
  }

  float gzDps = ((float)gzRaw) / MPU6050_GYRO_Z_SCALE;
  imuGyroDps = gzDps - imuBiasDps;
  float a = constrain(headingGyroLpfAlpha, 0.02f, 1.0f);
  imuGyroDpsFilt += a * (imuGyroDps - imuGyroDpsFilt);
  imuYawDeg = wrapAngleDeg(imuYawDeg + imuGyroDps * dt);
}

void updateSlipDetector(float dt) {
  bool yawHoldOnly = (controlMode == MODE_POSITION_HEADING) && (fabsf(positionCmdRpm) < headingEnableBaseRpm);

  if (dt <= 0.0001f || !controllerEnabled || rawModeEnabled || minSweep.active || !slipEnabled || !imuOk) {
    slipCandidateMs = 0.0f;
    slipDetected = false;
    slipAvgWheelRpm = 0.0f;
    return;
  }

  if (yawHoldOnly) {
    slipCandidateMs = 0.0f;
    slipDetected = false;
    slipAvgWheelRpm = 0.0f;
    return;
  }

  float avgAbsTarget = 0.0f;
  float avgAbsMeasured = 0.0f;
  for (uint8_t i = 0; i < 4; i++) {
    avgAbsTarget += fabsf(targetRpm[i]);
    avgAbsMeasured += fabsf(measuredRpm[i]);
  }
  avgAbsTarget /= 4.0f;
  avgAbsMeasured /= 4.0f;
  slipAvgWheelRpm = avgAbsMeasured;

  bool wheelsSpinning = avgAbsMeasured >= slipWheelRpmThresh;
  bool commandMoving = avgAbsTarget >= slipWheelRpmThresh;
  bool bodyNotMoving = imuAccXYg <= slipAccelThreshG;
  bool candidate = wheelsSpinning && commandMoving && bodyNotMoving;

  float dtMs = dt * 1000.0f;
  float detectMs = constrain(slipDetectMs, 80.0f, 3000.0f);
  float releaseMs = constrain(slipReleaseMs, 40.0f, 3000.0f);

  if (candidate) {
    slipCandidateMs += dtMs;
    if (slipCandidateMs > detectMs * 2.0f) {
      slipCandidateMs = detectMs * 2.0f;
    }
  } else {
    slipCandidateMs -= dtMs * (detectMs / releaseMs);
    if (slipCandidateMs < 0.0f) {
      slipCandidateMs = 0.0f;
    }
  }

  slipDetected = (slipCandidateMs >= detectMs);
}

float getAverageMotorEncoderCount() {
  int64_t sum = 0;
  for (uint8_t i = 0; i < 4; i++) {
    // Keep position sign convention aligned with measured RPM convention.
    int32_t raw = readEncoderCount(motors[i].encIndex);
    int32_t signedCount = rpmSign[i] * (-raw);
    sum += (int64_t)signedCount;
  }
  return (float)sum / 4.0f;
}

void updateHighLevelTargets(float dt) {
  for (uint8_t i = 0; i < 4; i++) {
    targetRpm[i] = 0.0f;
  }

  debugYawPhase = 0;
  debugErrSettled = false;
  debugRateSettled = false;
  debugRawHeadingErrDeg = 0.0f;
  debugUsedHeadingErrDeg = 0.0f;
  debugCorrTargetRpm = 0.0f;
  debugCorrAfterSlewRpm = 0.0f;

  if (!controllerEnabled) {
    return;
  }

  if (controlMode == MODE_SINGLE_WHEEL_VELOCITY) {
    targetRpm[activeWheel] = constrain(pidTargetRpm, 0.0f, TARGET_RPM_MAX);
    return;
  }

  if (poseMoveActive && poseMovePhase == 1) {
    // During turn phase, force in-place yaw hold behavior.
    yawHoldInPlaceMode = true;
    holdHeadingAtStop = true;
  }

  positionCurrentCounts = getAverageMotorEncoderCount();
  if (yawHoldInPlaceMode) {
    // Dedicated in-place yaw mode: freeze translation loop so hold phase is never interrupted.
    positionErrorCounts = 0.0f;
    positionDone = true;
    positionDoneTicks = 5;
    positionCmdRpm = 0.0f;
    posPid.out = 0.0f;
    resetCascadePid(posPid);
  } else {
    positionErrorCounts = positionTargetCounts - positionCurrentCounts;
    float positionErrorCm = countsToCm(positionErrorCounts);

    if (fabsf(positionErrorCm) <= positionTolCm) {
      if (positionDoneTicks < 255) positionDoneTicks++;
    } else {
      positionDoneTicks = 0;
    }
    positionDone = (positionDoneTicks >= 5);

    posPid.integral += positionErrorCounts * dt;
    posPid.integral = constrain(posPid.integral, -30000.0f, 30000.0f);
    float posDeriv = (positionErrorCounts - posPid.prevError) / dt;
    posPid.prevError = positionErrorCounts;

    float base = posPid.kp * positionErrorCounts + posPid.ki * posPid.integral + posPid.kd * posDeriv;
    if (positionDone) {
      base = 0.0f;
    }
    base = constrain(base, -TARGET_RPM_MAX, TARGET_RPM_MAX);
    posPid.out = base;
    positionCmdRpm = base;
  }

  float corr = 0.0f;
  headingErrorDeg = 0.0f;
  if (imuOk) {
    float rawHeadingError = shortestAngleErrorDeg(headingTargetDeg, imuYawDeg);
    debugRawHeadingErrDeg = rawHeadingError;
    bool movingPhase = !positionDone;
    bool holdPhase = positionDone && holdHeadingAtStop;
    float corrTarget = 0.0f;

    if (movingPhase) {
      debugYawPhase = 1;
      float errMove = rawHeadingError;
      if (fabsf(errMove) <= headingDeadbandDeg) {
        errMove = 0.0f;
      }
      debugUsedHeadingErrDeg = errMove;

      // Damped move correction: P on heading error, D on filtered yaw-rate.
      float uMove = headingPid.kp * errMove - headingPid.kd * imuGyroDpsFilt;
      if (fabsf(headingPid.ki) > 0.0001f && fabsf(errMove) < 12.0f) {
        headingPid.integral += errMove * dt;
        headingPid.integral = constrain(headingPid.integral, -40.0f, 40.0f);
        uMove += headingPid.ki * headingPid.integral;
      } else {
        headingPid.integral = 0.0f;
      }

      uMove *= (float)headingCorrSign;
      corrTarget = constrain(uMove, -headingCorrMaxRpm, headingCorrMaxRpm);
      debugCorrTargetRpm = corrTarget;

      headingPid.prevError = errMove;
      headingPid.out = corrTarget;
      headingErrorDeg = errMove;

      resetCascadePid(headingHoldPid);
      headingHoldActive = false;
      headingHoldStableMs = 0.0f;
    } else if (holdPhase) {
      bool errSettled = fabsf(rawHeadingError) <= headingHoldDeadbandDeg;
      bool rateSettled = fabsf(imuGyroDpsFilt) <= headingHoldExitRateDps;
      debugErrSettled = errSettled;
      debugRateSettled = rateSettled;

      if (errSettled && rateSettled) {
        debugYawPhase = 3;
        headingHoldStableMs += dt * 1000.0f;
        if (headingHoldStableMs >= headingHoldSettleHoldMs) {
          // Require a short dwell before declaring settled so the loop cannot chatter between states.
          corrTarget = 0.0f;
          debugUsedHeadingErrDeg = 0.0f;
          debugCorrTargetRpm = 0.0f;
          headingHoldActive = false;
          resetCascadePid(headingHoldPid);
        }
      } else {
        debugYawPhase = 2;
        headingHoldStableMs = 0.0f;
        headingHoldActive = true;

        float errHold = rawHeadingError;
        if (fabsf(errHold) <= headingHoldDeadbandDeg) {
          errHold = 0.0f;
        } else if (errHold > 0.0f) {
          errHold -= headingHoldDeadbandDeg;
        } else {
          errHold += headingHoldDeadbandDeg;
        }
        debugUsedHeadingErrDeg = errHold;

        // Stable hold correction: P on deadbanded error, D on filtered yaw-rate.
        float uHold = headingHoldPid.kp * errHold - headingHoldPid.kd * imuGyroDpsFilt;

        // Keep hold I-term disabled in dedicated in-place yaw mode to avoid large-angle limit cycles.
        float holdKiEff = yawHoldInPlaceMode ? 0.0f : headingHoldPid.ki;
        if (fabsf(holdKiEff) > 0.0001f && fabsf(errHold) < 6.0f) {
          headingHoldPid.integral += errHold * dt;
          headingHoldPid.integral = constrain(headingHoldPid.integral, -20.0f, 20.0f);
          uHold += holdKiEff * headingHoldPid.integral;
        } else {
          headingHoldPid.integral = 0.0f;
        }

        uHold *= (float)headingCorrSign;

        float holdMaxEff = headingHoldMaxRpm;
        corrTarget = constrain(uHold, -holdMaxEff, holdMaxEff);

        if (yawHoldInPlaceMode) {
          float absRawErr = fabsf(rawHeadingError);
          float db = max(headingHoldDeadbandDeg, 0.1f);
          float slowStart = max(yawHoldSlowdownStartDeg, db + 0.5f);
          float finalCapDeg = constrain(yawHoldFinalCaptureDeg, db + 0.3f, slowStart - 0.1f);

          float minCmd = 0.0f;
          if (absRawErr > db) {
            if (absRawErr >= slowStart) {
              minCmd = yawHoldCruiseRpm;
            } else if (absRawErr >= finalCapDeg) {
              float t = (absRawErr - finalCapDeg) / (slowStart - finalCapDeg);
              t = constrain(t, 0.0f, 1.0f);
              minCmd = yawHoldFinalCaptureRpm + t * (yawHoldCruiseRpm - yawHoldFinalCaptureRpm);
            } else {
              float t = (absRawErr - db) / (finalCapDeg - db);
              t = constrain(t, 0.0f, 1.0f);
              minCmd = yawHoldFinalMinRpm + t * (yawHoldFinalCaptureRpm - yawHoldFinalMinRpm);
            }
          }

          minCmd = constrain(minCmd, 0.0f, holdMaxEff);
          bool enforceMin = absRawErr > (db + 0.3f) && headingHoldStableMs < headingHoldSettleHoldMs;
          if (enforceMin && minCmd > 0.0f && fabsf(corrTarget) < minCmd) {
            int sign = 0;
            if (corrTarget > 0.0f) sign = 1;
            else if (corrTarget < 0.0f) sign = -1;
            else sign = (rawHeadingError >= 0.0f) ? 1 : -1;
            corrTarget = ((float)sign) * minCmd;
          }
        }
        debugCorrTargetRpm = corrTarget;

        headingHoldPid.prevError = errHold;
        headingHoldPid.out = corrTarget;
      }

      headingErrorDeg = rawHeadingError;
      resetCascadePid(headingPid);
    } else {
      // If heading hold is disabled at stop, freeze target to current yaw.
      if (positionDone && !holdHeadingAtStop) {
        headingTargetDeg = imuYawDeg;
        headingErrorDeg = 0.0f;
      }
      debugYawPhase = 0;
      yawHoldInPlaceMode = false;
      resetCascadePid(headingPid);
      resetCascadePid(headingHoldPid);
      headingHoldActive = false;
      headingHoldStableMs = 0.0f;
      corrTarget = 0.0f;
    }

    // Slew limiting suppresses command sign flapping and helps eliminate yaw chatter.
    float slew = max(0.0f, headingCorrSlewRpmPerSec) * dt;
    corr = headingCorrCmdPrev + constrain(corrTarget - headingCorrCmdPrev, -slew, slew);
    if (holdPhase && !headingHoldActive) {
      // In settled hold, drop correction immediately to avoid residual twitch.
      corr = 0.0f;
    }
    headingCorrCmdPrev = corr;
    debugCorrAfterSlewRpm = corr;
  }
  headingCorrRpm = corr;

  if (poseMoveActive) {
    if (poseMovePhase == 1) {
      bool headingSettledNow = debugErrSettled && debugRateSettled && (headingHoldStableMs >= headingHoldSettleHoldMs);
      if (headingSettledNow) {
        // Start drive phase once heading is stably aligned.
        poseMovePhase = 2;
        yawHoldInPlaceMode = false;
        positionTargetCounts = poseMoveTargetCounts;
        positionDone = false;
        positionDoneTicks = 0;
        positionCmdRpm = 0.0f;
        resetCascadePid(posPid);
      }
    } else if (poseMovePhase == 2) {
      if (positionDone) {
        cancelPoseMove();
      }
    }
  }

  // Right wheels: MOTA, MOTB. Left wheels: MOTC, MOTD.
  targetRpm[0] = constrain(positionCmdRpm - headingCorrRpm, -TARGET_RPM_MAX, TARGET_RPM_MAX);
  targetRpm[1] = constrain(positionCmdRpm - headingCorrRpm, -TARGET_RPM_MAX, TARGET_RPM_MAX);
  targetRpm[2] = constrain(positionCmdRpm + headingCorrRpm, -TARGET_RPM_MAX, TARGET_RPM_MAX);
  targetRpm[3] = constrain(positionCmdRpm + headingCorrRpm, -TARGET_RPM_MAX, TARGET_RPM_MAX);
}

void setupEncoderPin(uint8_t pin) {
  if (isInputOnlyPin(pin)) {
    pinMode(pin, INPUT);
  } else {
    pinMode(pin, INPUT_PULLUP);
  }
}

void IRAM_ATTR updateEncoder(uint8_t idx) {
  uint8_t state = (digitalRead(encoders[idx].a) << 1) | digitalRead(encoders[idx].b);
  uint8_t prev = encoderPrevState[idx];
  uint8_t trans = (prev << 2) | state;

  switch (trans) {
    case 0b0001:
    case 0b0111:
    case 0b1110:
    case 0b1000:
      encoderCount[idx]++;
      break;
    case 0b0010:
    case 0b0100:
    case 0b1101:
    case 0b1011:
      encoderCount[idx]--;
      break;
    default:
      break;
  }

  encoderPrevState[idx] = state;
}

void IRAM_ATTR isrEnc0A() { updateEncoder(0); }
void IRAM_ATTR isrEnc0B() { updateEncoder(0); }
void IRAM_ATTR isrEnc1A() { updateEncoder(1); }
void IRAM_ATTR isrEnc1B() { updateEncoder(1); }
void IRAM_ATTR isrEnc2A() { updateEncoder(2); }
void IRAM_ATTR isrEnc2B() { updateEncoder(2); }
void IRAM_ATTR isrEnc3A() { updateEncoder(3); }
void IRAM_ATTR isrEnc3B() { updateEncoder(3); }

int32_t readEncoderCount(uint8_t idx) {
  noInterrupts();
  int32_t c = encoderCount[idx];
  interrupts();
  return c;
}

void resetEncoders() {
  noInterrupts();
  for (uint8_t i = 0; i < 4; i++) {
    encoderCount[i] = 0;
  }
  interrupts();

  for (uint8_t i = 0; i < 4; i++) {
    lastEncoderCount[i] = 0;
    measuredRpmRaw[i] = 0.0f;
    measuredRpm[i] = 0.0f;
  }
}

void resetPidState(uint8_t i) {
  pid[i].integral = 0.0f;
  pid[i].prevError = 0.0f;
  pid[i].out = 0.0f;
}

void resetAllPidStates() {
  for (uint8_t i = 0; i < 4; i++) {
    resetPidState(i);
  }
}

void setMotorPwmSigned(uint8_t wheel, int cmd) {
  // Explicit signed mapping:
  // cmd in [-255, +255]
  // +cmd => IN1 PWM, IN2 = 0
  // -cmd => IN1 = 0, IN2 PWM
  // 0 => both 0
  cmd = constrain(cmd, -PWM_MAX, PWM_MAX);
  pwmCmd[wheel] = cmd;

  const MotorPins& m = motors[wheel];
  if (cmd > 0) {
    pwmWriteCompat(m.in1, m.ch1, (uint32_t)cmd);
    pwmWriteCompat(m.in2, m.ch2, 0);
  } else if (cmd < 0) {
    pwmWriteCompat(m.in1, m.ch1, 0);
    pwmWriteCompat(m.in2, m.ch2, (uint32_t)(-cmd));
  } else {
    pwmWriteCompat(m.in1, m.ch1, 0);
    pwmWriteCompat(m.in2, m.ch2, 0);
  }
}

void stopAllMotors() {
  for (uint8_t i = 0; i < 4; i++) {
    setMotorPwmSigned(i, 0);
  }
}

void stopRawMode() {
  rawModeEnabled = false;
  rawPwmCmd = 0;
  stopAllMotors();
}

void startRawMode(int16_t cmd) {
  if (minSweep.active) {
    stopMinPwmSweep();
  }

  controllerEnabled = false;
  updateTargets();
  resetAllPidStates();

  rawModeEnabled = true;
  rawPwmCmd = (int16_t)constrain((int)cmd, -PWM_MAX, PWM_MAX);

  stopAllMotors();
  setMotorPwmSigned(activeWheel, rawPwmCmd);
}

void setAllWheelsPwmSigned(int cmd) {
  for (uint8_t i = 0; i < 4; i++) {
    setMotorPwmSigned(i, cmd);
  }
}

void updateMeasuredRpm(float dt) {
  if (dt <= 0.0001f) {
    return;
  }

  const float rpmScale = (60.0f / COUNTS_PER_WHEEL_REV) / dt;
  float alpha = constrain(rpmFilterAlpha, 0.02f, 1.0f);
  for (uint8_t i = 0; i < 4; i++) {
    // Keep measuredRpm[] in motor order by reading each motor's mapped encoder.
    const uint8_t encIdx = motors[i].encIndex;
    int32_t countNow = readEncoderCount(encIdx);
    int32_t delta = countNow - lastEncoderCount[encIdx];
    lastEncoderCount[encIdx] = countNow;

    // Per-wheel RPM sign lets you align measured RPM with your chosen target convention.
    measuredRpmRaw[i] = ((float)rpmSign[i]) * (-((float)delta) * rpmScale);
    measuredRpm[i] += alpha * (measuredRpmRaw[i] - measuredRpm[i]);
  }
}

void stopMinPwmSweep() {
  minSweep.active = false;
  minSweep.stableMoveTicks = 0;
  minSweep.avgAbsRpm = 0.0f;
  stopAllMotors();
}

void startMinPwmSweep(int8_t direction, uint8_t startPwm, uint8_t endPwm, uint8_t stepPwm, uint16_t holdMs) {
  direction = (direction >= 0) ? 1 : -1;

  startPwm = constrain(startPwm, MIN_SWEEP_MIN_PWM, MIN_SWEEP_MAX_PWM);
  endPwm = constrain(endPwm, MIN_SWEEP_MIN_PWM, MIN_SWEEP_MAX_PWM);
  if (endPwm < startPwm) {
    uint8_t t = startPwm;
    startPwm = endPwm;
    endPwm = t;
  }

  stepPwm = constrain(stepPwm, (uint8_t)1, (uint8_t)20);
  holdMs = constrain((int)holdMs, 200, 3000);

  controllerEnabled = false;
  updateTargets();
  resetAllPidStates();
  resetEncoders();

  minSweep.active = true;
  minSweep.direction = direction;
  minSweep.startPwm = startPwm;
  minSweep.endPwm = endPwm;
  minSweep.stepPwm = stepPwm;
  minSweep.holdMs = holdMs;
  minSweep.currentPwm = startPwm;
  minSweep.detectedPwm = 0;
  minSweep.stableMoveTicks = 0;
  minSweep.avgAbsRpm = 0.0f;
  minSweep.stepStartMs = millis();

  int signedPwm = ((int)minSweep.currentPwm) * ((minSweep.direction > 0) ? 1 : -1);
  setAllWheelsPwmSigned(signedPwm);
}

void updateMinPwmSweep(uint32_t now) {
  if (!minSweep.active) {
    return;
  }

  float sumAbs = 0.0f;
  for (uint8_t i = 0; i < 4; i++) {
    sumAbs += fabsf(measuredRpm[i]);
  }
  minSweep.avgAbsRpm = sumAbs / 4.0f;

  if (minSweep.detectedPwm == 0) {
    if (minSweep.avgAbsRpm >= minSweep.detectRpmThreshold) {
      if (minSweep.stableMoveTicks < 255) {
        minSweep.stableMoveTicks++;
      }
    } else {
      minSweep.stableMoveTicks = 0;
    }

    if (minSweep.stableMoveTicks >= 3) {
      minSweep.detectedPwm = minSweep.currentPwm;
    }
  }

  if (now - minSweep.stepStartMs < minSweep.holdMs) {
    return;
  }

  uint16_t nextPwm = (uint16_t)minSweep.currentPwm + (uint16_t)minSweep.stepPwm;
  if (nextPwm > minSweep.endPwm) {
    stopMinPwmSweep();
    return;
  }

  minSweep.currentPwm = (uint8_t)nextPwm;
  minSweep.stableMoveTicks = 0;
  minSweep.stepStartMs = now;

  int signedPwm = ((int)minSweep.currentPwm) * ((minSweep.direction > 0) ? 1 : -1);
  setAllWheelsPwmSigned(signedPwm);
}

void updateTargets() {
  for (uint8_t i = 0; i < 4; i++) {
    targetRpm[i] = 0.0f;
  }
  if (controllerEnabled && controlMode == MODE_SINGLE_WHEEL_VELOCITY) {
    targetRpm[activeWheel] = pidTargetRpm;
  }
}

void runControlLoop(float dt) {
  updateMeasuredRpm(dt);
  imuUpdate(dt);
  updateHighLevelTargets(dt);
  updateSlipDetector(dt);

  if (slipEnabled && slipDetected) {
    float lim = constrain(slipMaxRpm, 0.0f, TARGET_RPM_MAX);
    for (uint8_t i = 0; i < 4; i++) {
      targetRpm[i] = constrain(targetRpm[i], -lim, lim);
    }
  }

  bool yawHoldWheelMode = (controlMode == MODE_POSITION_HEADING) && (fabsf(positionCmdRpm) < headingEnableBaseRpm) && (headingHoldActive || yawHoldInPlaceMode);

  for (uint8_t i = 0; i < 4; i++) {
    float target = targetRpm[i];
    if (yawHoldWheelMode && fabsf(target) < yawHoldTargetDeadbandRpm) {
      target = 0.0f;
    }

    float err = target - measuredRpm[i];
    debugWheelErr[i] = err;

    if (fabsf(target) < 0.01f) {
      resetPidState(i);
      debugWheelPidOut[i] = 0.0f;
      debugWheelCmdPreBreakaway[i] = 0;
      setMotorPwmSigned(i, 0);
      continue;
    }

    pid[i].integral += err * dt;
    pid[i].integral = constrain(pid[i].integral, -300.0f, 300.0f);

    float deriv = (err - pid[i].prevError) / dt;
    pid[i].prevError = err;

    float kpEff = pid[i].kp;
    float kiEff = pid[i].ki;
    float kdEff = pid[i].kd;
    if (yawHoldWheelMode) {
      kpEff *= yawHoldWheelKpScale;
      kiEff = 0.0f;
      kdEff *= yawHoldWheelKdScale;
      pid[i].integral = 0.0f;
    }

    float u = kpEff * err + kiEff * pid[i].integral + kdEff * deriv;
    pid[i].out = u;
    debugWheelPidOut[i] = u;

    int cmd = (int)lroundf(u);
    int maxPwm = yawHoldWheelMode ? constrain((int)yawHoldWheelPwmMax, 0, PWM_MAX) : PWM_MAX;
    cmd = constrain(cmd, -maxPwm, maxPwm);
    debugWheelCmdPreBreakaway[i] = (int16_t)cmd;

    // Breakaway MinPWM is applied only during translation.
    bool translationalMotion = !(controlMode == MODE_POSITION_HEADING && fabsf(positionCmdRpm) < headingEnableBaseRpm);
    bool wantsMotion = fabsf(target) > 0.8f;
    bool wheelNearlyStopped = fabsf(measuredRpm[i]) < 1.0f;
    int breakawayPwm = translationalMotion ? (int)minPwm : 0;
    if (wantsMotion && wheelNearlyStopped && breakawayPwm > 0 && abs(cmd) < breakawayPwm) {
      int sign = 0;
      if (cmd > 0) sign = 1;
      else if (cmd < 0) sign = -1;
      else sign = (err >= 0.0f) ? 1 : -1;
      cmd = sign * breakawayPwm;
    }

    // In yaw hold, allow a modest kick only when far from target and nearly stalled.
    if (yawHoldWheelMode && yawHoldInPlaceMode) {
      bool farFromTarget = fabsf(debugRawHeadingErrDeg) >= yawHoldBreakawayErrDeg;
      bool stalled = fabsf(measuredRpm[i]) < 0.8f;
      bool nonSettled = !debugErrSettled;
      bool withinSettleDwell = headingHoldStableMs < headingHoldSettleHoldMs;
      if (withinSettleDwell && farFromTarget && stalled && nonSettled && abs(cmd) < yawHoldBreakawayPwm) {
        int sign = 0;
        if (cmd > 0) sign = 1;
        else if (cmd < 0) sign = -1;
        else sign = (target >= 0.0f) ? 1 : -1;
        cmd = sign * (int)yawHoldBreakawayPwm;
      }
    }

    setMotorPwmSigned(i, cmd);
  }
}

String pageHtml() {
  return R"HTML(
<!doctype html>
<html>
<head>
<meta charset="utf-8" />
<meta name="viewport" content="width=device-width,initial-scale=1" />
<meta http-equiv="Cache-Control" content="no-cache, no-store, must-revalidate" />
<meta http-equiv="Pragma" content="no-cache" />
<meta http-equiv="Expires" content="0" />
<title>Wheel PID Tuner</title>
<style>
:root{--bg:#0b1220;--panel:#111827;--line:#293548;--txt:#e5e7eb;--muted:#9ca3af;--ok:#16a34a;--warn:#f59e0b;}
body{margin:0;font-family:Segoe UI,Tahoma,sans-serif;background:radial-gradient(circle at top,#101a30,#0b1220 60%);color:var(--txt)}
.wrap{max-width:980px;margin:18px auto;padding:0 12px}
.card{background:rgba(17,24,39,.96);border:1px solid #223049;border-radius:12px;padding:14px;margin-bottom:12px}
h2{margin:0 0 10px 0;font-size:20px}
.row{display:flex;gap:10px;flex-wrap:wrap;align-items:center}
label{font-size:13px;color:var(--muted)}
input,select,button{background:#0a1324;color:var(--txt);border:1px solid #334155;border-radius:8px;padding:8px 10px}
textarea{background:#0a1324;color:var(--txt);border:1px solid #334155;border-radius:8px;padding:8px 10px;font-family:Consolas,monospace}
button{cursor:pointer}
button.main{background:#0b5ed7;border-color:#0b5ed7}
button.start{background:#15803d;border-color:#15803d}
button.stop{background:#b91c1c;border-color:#b91c1c}
.grid{display:grid;grid-template-columns:repeat(4,minmax(170px,1fr));gap:8px}
.kpi{border:1px solid #334155;border-radius:10px;padding:8px;background:#0b1220}
.small{font-size:12px;color:var(--muted)}
.val{font-size:20px;font-weight:700}
.table{width:100%;border-collapse:collapse}
.table th,.table td{border-bottom:1px solid #2d3748;padding:8px;text-align:left;font-size:14px}
.code{font-family:Consolas,monospace}
.plotPanel{border:1px solid #334155;border-radius:10px;padding:8px;background:#0b1220;margin-top:8px}
.plotCanvas{width:100%;height:220px;display:block;border:1px solid #334155;border-radius:8px;background:linear-gradient(180deg,#0c1730,#091123)}
.legendRow{display:flex;gap:12px;flex-wrap:wrap;margin:6px 2px 10px 2px}
.legendItem{font-size:12px;color:var(--muted);display:flex;align-items:center;gap:6px}
.legendSwatch{width:12px;height:3px;border-radius:2px;display:inline-block}
</style>
</head>
<body>
<div class="wrap">
  <div class="card">
    <h2>Wheel Speed PID Tuner</h2>
    <div class="small">Connect to Wi-Fi <span class="code">ESP32-PID-TUNE</span> (pass <span class="code">pid12345</span>) and open <span class="code">192.168.4.1</span>.</div>
    <div class="small">Mode 0: single-wheel velocity tuning. Mode 1: full position + IMU heading control.</div>
  </div>

  <div class="card">
    <div class="row">
      <label>Mode</label>
      <select id="mode">
        <option value="0">0 - Single Wheel Velocity</option>
        <option value="1">1 - Position + Heading</option>
      </select>
      <button class="main" onclick="setMode()">Set Mode</button>

      <label>Wheel</label>
      <select id="wheel">
        <option value="0">MOTA BackRight (ENC4)</option>
        <option value="1">MOTB FrontRight (ENC3)</option>
        <option value="2">MOTC BackLeft (ENC1)</option>
        <option value="3">MOTD FrontLeft (ENC2)</option>
      </select>

      <label>Target RPM</label><input id="target" type="number" value="50" step="1" min="0" max="60" />
      <label>Kp</label><input id="kp" type="number" value="50" step="0.05" />
      <label>Ki</label><input id="ki" type="number" value="6" step="0.05" />
      <label>Kd</label><input id="kd" type="number" value="0.00" step="0.01" />
      <label>MinPWM</label><input id="minpwm" type="number" value="160" step="1" max="255" />
      <label>RPM LPF alpha</label><input id="alpha" type="number" value="0.02" step="0.01" min="0.02" max="1.00" />

      <button class="main" onclick="applyConfig()">Apply Velocity Config</button>
      <button class="start" onclick="startCtrl()">Start</button>
      <button class="stop" onclick="stopCtrl()">Stop</button>
      <button onclick="resetI()">Reset I</button>
      <button onclick="resetEnc()">Reset Encoders</button>
    </div>
    <div class="row" style="margin-top:8px">
      <label>RPM Sign BR</label>
      <select id="rs0" onchange="applyRpmSign()"><option value="1">+1</option><option value="-1">-1</option></select>
      <label>FR</label>
      <select id="rs1" onchange="applyRpmSign()"><option value="1">+1</option><option value="-1">-1</option></select>
      <label>BL</label>
      <select id="rs2" onchange="applyRpmSign()"><option value="1">+1</option><option value="-1">-1</option></select>
      <label>FL</label>
      <select id="rs3" onchange="applyRpmSign()"><option value="1">+1</option><option value="-1">-1</option></select>
      <button class="main" onclick="applyRpmSign()">Apply RPM Sign</button>
    </div>
    <div class="small" id="status">Status: -</div>
  </div>

  <div class="card">
    <h2>Position + IMU Heading Control (4 Wheels)</h2>
    <div class="row">
      <label>Move Step (cm)</label><input id="pos_stepcm" type="number" value="30" step="1" onchange="applyMotionConfig()" />
      <label>Wheel Diameter (cm)</label><input id="wheel_diam_cm" type="number" value="9.7" step="0.1" min="1" max="30" onchange="applyMotionConfig()" />
      <label>Position Tol (cm)</label><input id="pos_tol" type="number" value="1.5" step="0.1" min="0.1" oninput="scheduleMotionConfigApply()" onchange="applyMotionConfig()" />
      <label>Pos Kp</label><input id="pos_kp" type="number" value="0.05" step="0.005" onchange="applyMotionConfig()" />
      <label>Pos Ki</label><input id="pos_ki" type="number" value="0.0018" step="0.0001" onchange="applyMotionConfig()" />
      <label>Pos Kd</label><input id="pos_kd" type="number" value="0.0" step="0.001" onchange="applyMotionConfig()" />
      <button class="start" onclick="startPosStep(1)">Move Forward Step</button>
      <button class="start" onclick="startPosStep(-1)">Move Backward Step</button>
    </div>
    <div class="row" style="margin-top:8px">
      <label>Target Heading (deg)</label><input id="head_target" type="number" value="0" step="1" min="-180" max="180" onchange="applyMotionConfig()" />
      <label>Head Kp</label><input id="head_kp" type="number" value="1.80" step="0.05" onchange="applyMotionConfig()" />
      <label>Head Kd</label><input id="head_kd" type="number" value="0.08" step="0.01" onchange="applyMotionConfig()" />
      <label>Max Head Corr RPM</label><input id="head_max" type="number" value="20" step="1" min="0" max="60" onchange="applyMotionConfig()" />
      <label>Hold At Stop</label>
      <select id="head_hold" onchange="applyMotionConfig()"><option value="1">ON</option><option value="0">OFF</option></select>
      <label>Hold Max Corr RPM</label><input id="head_hold_max" type="number" value="10" step="0.5" min="0" max="60" onchange="applyMotionConfig()" />
      <label>Hold Deadband (deg)</label><input id="head_hold_db" type="number" value="2.0" step="0.1" min="0" max="30" onchange="applyMotionConfig()" />
      <label>Hold Kp</label><input id="head_hold_kp" type="number" value="0.65" step="0.05" onchange="applyMotionConfig()" />
      <label>Hold Kd</label><input id="head_hold_kd" type="number" value="1.20" step="0.01" onchange="applyMotionConfig()" />
      <button class="main" onclick="applyMotionConfig()">Apply Motion Config</button>
      <button onclick="imuZero()">Zero Yaw</button>
      <button onclick="imuCal()">Calibrate IMU Bias</button>
    </div>
    <div class="row" style="margin-top:8px">
      <label>Seq Distance (cm)</label><input id="pose_distcm" type="number" value="30" step="1" />
      <label>Seq Heading (deg)</label><input id="pose_yaw" type="number" value="0" step="1" min="-180" max="180" />
      <button class="start" onclick="startPoseMove()">TURN THEN MOVE</button>
      <div class="small">Sequence behavior: rotate in place to target heading, then move forward while continuously holding that heading with IMU correction.</div>
    </div>
    <div class="small">Move deadband works only while translating; hold deadband works only when position is done and yaw is being held in place.</div>
    <div class="row" style="margin-top:8px">
      <button class="start" onclick="startYawHold()">START YAW HOLD (IN PLACE)</button>
      <button class="stop" onclick="stopCtrl()">STOP YAW HOLD</button>
      <div class="small">Use this row for yaw-only bench tests: set target heading then press START.</div>
    </div>
    <div class="row" style="margin-top:8px">
      <label>Serial Debug</label>
      <select id="dbg_en"><option value="0">OFF</option><option value="1">ON</option></select>
      <label>Period ms</label><input id="dbg_ms" type="number" value="120" step="10" min="50" max="2000" />
      <button class="main" onclick="applyDebugConfig()">Apply Debug Config</button>
    </div>
    <div class="row" style="margin-top:8px">
      <label>Web Recorder</label>
      <button class="start" onclick="startWebRecorder()">Start Capture</button>
      <button class="stop" onclick="stopWebRecorder()">Stop Capture</button>
      <button onclick="clearWebRecorder()">Clear</button>
      <button class="main" onclick="copyWebRecorder()">Copy CSV</button>
    </div>
    <div class="small" id="motionStatus">Motion: -</div>
    <div class="small" id="imuStatus">IMU: -</div>
    <div class="small" id="debugStatus">Debug: -</div>
    <div class="small" id="rec_status">Recorder: idle</div>
    <textarea id="rec_out" rows="8" style="width:100%;margin-top:8px" readonly></textarea>
  </div>

  <div class="card">
    <h2>Slip/Stall Observer (IMU X/Y + Wheel RPM)</h2>
    <div class="row">
      <label>Enable</label>
      <select id="slip_en"><option value="1">ON</option><option value="0">OFF</option></select>
      <label>Wheel RPM Thresh</label><input id="slip_rpm" type="number" value="12" step="1" min="1" max="60" />
      <label>Accel Thresh g</label><input id="slip_accg" type="number" value="0.03" step="0.005" min="0.001" max="1.5" />
      <label>Detect ms</label><input id="slip_detect" type="number" value="300" step="20" min="80" max="3000" />
      <label>Release ms</label><input id="slip_release" type="number" value="200" step="20" min="40" max="3000" />
      <label>Slip Max RPM</label><input id="slip_maxrpm" type="number" value="25" step="1" min="0" max="60" />
      <label>IMU Acc LPF</label><input id="slip_accalpha" type="number" value="0.20" step="0.01" min="0.02" max="1.00" />
      <button class="main" onclick="applySlipConfig()">Apply Slip Config</button>
    </div>
    <div class="small" id="slipStatus">Slip: -</div>
  </div>

  <div class="card">
    <h2>Raw Single-Wheel Test (Bypass PID)</h2>
    <div class="row">
      <label>Raw PWM</label><input id="rawpwm" type="number" value="255" step="1" min="0" max="255" />
      <button class="start" onclick="startRaw(1)">Run +Raw</button>
      <button class="start" onclick="startRaw(-1)">Run -Raw</button>
      <button class="stop" onclick="stopRaw()">Stop Raw</button>
    </div>
    <div class="small" id="rawStatus">Raw mode: -</div>
    <div class="small">Signed mapping is strict: +Raw drives IN1, -Raw drives IN2, both with duty 0..255.</div>
  </div>

  <div class="card">
    <h2>4-Wheel MinPWM Auto Sweep (Ground Test)</h2>
    <div class="row">
      <label>Start PWM</label><input id="ms_start" type="number" value="20" step="1" />
      <label>End PWM</label><input id="ms_end" type="number" value="90" step="1" />
      <label>Step</label><input id="ms_step" type="number" value="2" step="1" />
      <label>Hold ms</label><input id="ms_hold" type="number" value="700" step="50" />
      <button class="start" onclick="startSweep(1)">Start FWD Sweep</button>
      <button class="start" onclick="startSweep(-1)">Start REV Sweep</button>
      <button class="stop" onclick="stopSweep()">Stop Sweep</button>
    </div>
    <div class="small" id="sweepStatus">Sweep: -</div>
    <div class="small">Detected PWM is the first sweep value where average absolute RPM stays above threshold for multiple control ticks.</div>
  </div>

  <div class="card">
    <h2>Live Wheel Data</h2>
    <div class="grid">
      <div class="kpi"><div class="small">BackRight RPM</div><div class="val" id="rpm0">0</div></div>
      <div class="kpi"><div class="small">FrontRight RPM</div><div class="val" id="rpm1">0</div></div>
      <div class="kpi"><div class="small">BackLeft RPM</div><div class="val" id="rpm2">0</div></div>
      <div class="kpi"><div class="small">FrontLeft RPM</div><div class="val" id="rpm3">0</div></div>
    </div>
  </div>

  <div class="card">
    <table class="table">
      <thead><tr><th>Wheel</th><th>Target RPM</th><th>Measured RPM</th><th>Error</th><th>PWM Cmd</th></tr></thead>
      <tbody>
        <tr><td>MOTA BackRight</td><td id="tr0">0</td><td id="mr0">0</td><td id="er0">0</td><td id="pw0">0</td></tr>
        <tr><td>MOTB FrontRight</td><td id="tr1">0</td><td id="mr1">0</td><td id="er1">0</td><td id="pw1">0</td></tr>
        <tr><td>MOTC BackLeft</td><td id="tr2">0</td><td id="mr2">0</td><td id="er2">0</td><td id="pw2">0</td></tr>
        <tr><td>MOTD FrontLeft</td><td id="tr3">0</td><td id="mr3">0</td><td id="er3">0</td><td id="pw3">0</td></tr>
      </tbody>
    </table>
  </div>

  <div class="card">
    <h2>Live Tuning Graphs</h2>
    <div class="small">Graphs track the active wheel selected in the control section and keep roughly the last 45-50 seconds.</div>

    <div class="plotPanel">
      <div class="small">Position Response (Mode 1)</div>
      <div class="legendRow">
        <span class="legendItem"><span class="legendSwatch" style="background:#22c55e"></span>Target Position (cm)</span>
        <span class="legendItem"><span class="legendSwatch" style="background:#38bdf8"></span>Measured Position (cm)</span>
        <span class="legendItem"><span class="legendSwatch" style="background:#ef4444"></span>Error (cm)</span>
        <span class="legendItem"><span class="legendSwatch" style="background:#f59e0b"></span>Pos Cmd (rpm)</span>
      </div>
      <canvas id="plot_pos" class="plotCanvas"></canvas>
    </div>

    <div class="plotPanel">
      <div class="small">Active Wheel RPM / Target / Error</div>
      <div class="legendRow">
        <span class="legendItem"><span class="legendSwatch" style="background:#38bdf8"></span>Target RPM</span>
        <span class="legendItem"><span class="legendSwatch" style="background:#22c55e"></span>Measured RPM</span>
        <span class="legendItem"><span class="legendSwatch" style="background:#ef4444"></span>Error</span>
      </div>
      <canvas id="plot_rpm" class="plotCanvas"></canvas>
    </div>

    <div class="plotPanel">
      <div class="small">Active Wheel PWM Command</div>
      <div class="legendRow">
        <span class="legendItem"><span class="legendSwatch" style="background:#f97316"></span>PWM Cmd</span>
        <span class="legendItem"><span class="legendSwatch" style="background:#a855f7"></span>Slip Marker</span>
      </div>
      <canvas id="plot_pwm" class="plotCanvas"></canvas>
    </div>

    <div class="plotPanel">
      <div class="small">Heading Trace</div>
      <div class="legendRow">
        <span class="legendItem"><span class="legendSwatch" style="background:#10b981"></span>Yaw</span>
        <span class="legendItem"><span class="legendSwatch" style="background:#f59e0b"></span>Heading Target</span>
        <span class="legendItem"><span class="legendSwatch" style="background:#ef4444"></span>Heading Error</span>
        <span class="legendItem"><span class="legendSwatch" style="background:#06b6d4"></span>Corr RPM (x10)</span>
      </div>
      <canvas id="plot_heading" class="plotCanvas"></canvas>
    </div>
  </div>
</div>

<script>
function f(x,d=2){return Number(x).toFixed(d)}
async function hit(url){const r=await fetch(url);if(!r.ok){alert('Request failed: '+url);return null;}return r.text();}
function syncFieldValue(id,val){
  const el=document.getElementById(id);
  if(!el) return;
  if(document.activeElement===el) return;
  const next=String(val);
  if(el.value!==next) el.value=next;
}

const GRAPH_POINTS = 240;
const WEB_REC_MAX_LINES = 700;
let graphActiveWheel = -1;
let webRecActive = false;
let webRecLines = [];
let webRecLastMs = -1;
let motionCfgTimer = null;
let motionCfgPending = false;
let motionCfgFreezeUntilMs = 0;
const graph = {
  posTarget: [],
  posMeasured: [],
  posErr: [],
  posCmd: [],
  target: [],
  measured: [],
  err: [],
  pwm: [],
  slip: [],
  yaw: [],
  headingTarget: [],
  headingErr: [],
  headingCorr: [],
};

function pushGraph(key, v) {
  const arr = graph[key];
  const n = Number(v);
  arr.push(Number.isFinite(n) ? n : 0);
  if (arr.length > GRAPH_POINTS) arr.shift();
}

function clearWheelGraph() {
  graph.target.length = 0;
  graph.measured.length = 0;
  graph.err.length = 0;
  graph.pwm.length = 0;
  graph.slip.length = 0;
}

function clearPositionGraph() {
  graph.posTarget.length = 0;
  graph.posMeasured.length = 0;
  graph.posErr.length = 0;
  graph.posCmd.length = 0;
}

function scheduleMotionConfigApply() {
  motionCfgFreezeUntilMs = Date.now() + 1200;
  if (motionCfgTimer) clearTimeout(motionCfgTimer);
  motionCfgTimer = setTimeout(() => {
    applyMotionConfig();
  }, 220);
}

function drawPlot(canvasId, series, yMin, yMax) {
  const canvas = document.getElementById(canvasId);
  if (!canvas) return;

  const rect = canvas.getBoundingClientRect();
  const dpr = Math.max(window.devicePixelRatio || 1, 1);
  const w = Math.max(240, Math.floor(rect.width * dpr));
  const h = Math.max(120, Math.floor(rect.height * dpr));
  if (canvas.width !== w || canvas.height !== h) {
    canvas.width = w;
    canvas.height = h;
  }

  const ctx = canvas.getContext('2d');
  ctx.clearRect(0, 0, w, h);

  ctx.fillStyle = '#091123';
  ctx.fillRect(0, 0, w, h);

  const padL = 50 * dpr;
  const padR = 12 * dpr;
  const padT = 10 * dpr;
  const padB = 20 * dpr;
  const pw = w - padL - padR;
  const ph = h - padT - padB;

  ctx.strokeStyle = '#1f2a44';
  ctx.lineWidth = 1;
  for (let i = 0; i <= 4; i++) {
    const gy = padT + (ph * i / 4);
    ctx.beginPath();
    ctx.moveTo(padL, gy);
    ctx.lineTo(w - padR, gy);
    ctx.stroke();
  }

  ctx.font = `${11 * dpr}px Consolas, monospace`;
  ctx.fillStyle = '#8aa0bf';
  for (let i = 0; i <= 4; i++) {
    const v = yMax - ((yMax - yMin) * i / 4);
    const gy = padT + (ph * i / 4);
    ctx.fillText(v.toFixed(0), 4 * dpr, gy + 4 * dpr);
  }

  const toX = (idx, count) => {
    if (count <= 1) return padL;
    return padL + (pw * idx / (count - 1));
  };
  const toY = (v) => {
    const t = (v - yMin) / (yMax - yMin);
    return padT + ph - (t * ph);
  };

  for (const s of series) {
    const data = s.data || [];
    if (data.length === 0) continue;
    ctx.strokeStyle = s.color;
    ctx.lineWidth = Math.max(1, Math.floor(1.6 * dpr));
    ctx.beginPath();
    for (let i = 0; i < data.length; i++) {
      const x = toX(i, data.length);
      const y = toY(Math.max(yMin, Math.min(yMax, data[i])));
      if (i === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    }
    ctx.stroke();
  }
}

function getAutoRange(seriesList, fallbackMin, fallbackMax) {
  let vMin = Number.POSITIVE_INFINITY;
  let vMax = Number.NEGATIVE_INFINITY;

  for (const arr of seriesList) {
    if (!arr || arr.length === 0) continue;
    for (let i = 0; i < arr.length; i++) {
      const v = Number(arr[i]);
      if (!Number.isFinite(v)) continue;
      if (v < vMin) vMin = v;
      if (v > vMax) vMax = v;
    }
  }

  if (!Number.isFinite(vMin) || !Number.isFinite(vMax)) {
    return { min: fallbackMin, max: fallbackMax };
  }

  if (vMin === vMax) {
    const pad = Math.max(1.0, Math.abs(vMin) * 0.2);
    vMin -= pad;
    vMax += pad;
  }

  const span = Math.max(0.5, vMax - vMin);
  const pad = Math.max(0.5, span * 0.12);
  vMin -= pad;
  vMax += pad;

  if (vMin > 0.0) vMin = 0.0;
  if (vMax < 0.0) vMax = 0.0;

  return { min: vMin, max: vMax };
}

function updateGraphs(d) {
  const aw = Number(d.activeWheel) || 0;
  if (graphActiveWheel !== aw) {
    graphActiveWheel = aw;
    clearWheelGraph();
  }

  const t = Number((d.target && d.target[aw]) || 0);
  const m = Number((d.measured && d.measured[aw]) || 0);
  const p = Number((d.pwm && d.pwm[aw]) || 0);
  const e = t - m;
  const posTargetCm = Number((d.position && d.position.targetCm) || 0);
  const posCurrentCm = Number((d.position && d.position.currentCm) || 0);
  const posErrCm = Number((d.position && d.position.errorCm) || 0);
  const posCmdRpm = Number((d.position && d.position.cmdRpm) || 0);

  pushGraph('posTarget', posTargetCm);
  pushGraph('posMeasured', posCurrentCm);
  pushGraph('posErr', posErrCm);
  pushGraph('posCmd', posCmdRpm);

  pushGraph('target', t);
  pushGraph('measured', m);
  pushGraph('err', e);
  pushGraph('pwm', p);
  pushGraph('slip', (d.slip && d.slip.detected) ? 240 : 0);
  pushGraph('yaw', Number((d.imu && d.imu.yawDeg) || 0));
  pushGraph('headingTarget', Number((d.heading && d.heading.targetDeg) || 0));
  pushGraph('headingErr', Number((d.heading && d.heading.errorDeg) || 0));
  pushGraph('headingCorr', Number((d.heading && d.heading.corrRpm) || 0) * 10);

  const posRange = getAutoRange([graph.posTarget, graph.posMeasured, graph.posErr, graph.posCmd], -10, 10);
  drawPlot('plot_pos', [
    { data: graph.posTarget, color: '#22c55e' },
    { data: graph.posMeasured, color: '#38bdf8' },
    { data: graph.posErr, color: '#ef4444' },
    { data: graph.posCmd, color: '#f59e0b' },
  ], posRange.min, posRange.max);

  drawPlot('plot_rpm', [
    { data: graph.target, color: '#38bdf8' },
    { data: graph.measured, color: '#22c55e' },
    { data: graph.err, color: '#ef4444' },
  ], -65, 65);

  drawPlot('plot_pwm', [
    { data: graph.pwm, color: '#f97316' },
    { data: graph.slip, color: '#a855f7' },
  ], -255, 255);

  drawPlot('plot_heading', [
    { data: graph.yaw, color: '#10b981' },
    { data: graph.headingTarget, color: '#f59e0b' },
    { data: graph.headingErr, color: '#ef4444' },
    { data: graph.headingCorr, color: '#06b6d4' },
  ], -180, 180);
}

function recorderHeader(){
  return 'tMs,phase,errSettled,rateSettled,yawDeg,gyroDps,gyroFiltDps,rawErrDeg,usedErrDeg,corrTargetRpm,corrOutRpm,target0,meas0,pwm0,target1,meas1,pwm1,target2,meas2,pwm2,target3,meas3,pwm3';
}

function renderRecorder(){
  const out=document.getElementById('rec_out');
  const st=document.getElementById('rec_status');
  if(st) st.textContent = `Recorder: ${webRecActive ? 'capturing' : 'idle'} | samples=${webRecLines.length}`;
  if(out){
    const txt = webRecLines.length ? (recorderHeader() + '\n' + webRecLines.join('\n')) : recorderHeader();
    if(out.value!==txt) out.value=txt;
  }
}

function appendRecorderSample(d){
  if(!webRecActive || !d || !d.debug) return;
  const ts = Number(d.nowMs || 0);
  if(ts>0){
    if(ts===webRecLastMs) return;
    webRecLastMs = ts;
  } else {
    webRecLastMs += 1;
  }

  const line = [
    ts,
    Number(d.debug.yawPhase || 0),
    d.debug.errSettled ? 1 : 0,
    d.debug.rateSettled ? 1 : 0,
    f((d.imu && d.imu.yawDeg) || 0, 3),
    f((d.imu && d.imu.gyroDps) || 0, 3),
    f(d.debug.gyroDpsFilt || 0, 3),
    f(d.debug.rawHeadingErrDeg || 0, 3),
    f(d.debug.usedHeadingErrDeg || 0, 3),
    f(d.debug.corrTargetRpm || 0, 3),
    f(d.debug.corrAfterSlewRpm || 0, 3),
    f((d.target && d.target[0]) || 0, 2),
    f((d.measured && d.measured[0]) || 0, 2),
    Number((d.pwm && d.pwm[0]) || 0),
    f((d.target && d.target[1]) || 0, 2),
    f((d.measured && d.measured[1]) || 0, 2),
    Number((d.pwm && d.pwm[1]) || 0),
    f((d.target && d.target[2]) || 0, 2),
    f((d.measured && d.measured[2]) || 0, 2),
    Number((d.pwm && d.pwm[2]) || 0),
    f((d.target && d.target[3]) || 0, 2),
    f((d.measured && d.measured[3]) || 0, 2),
    Number((d.pwm && d.pwm[3]) || 0)
  ].join(',');

  webRecLines.push(line);
  if(webRecLines.length > WEB_REC_MAX_LINES) webRecLines.shift();
  renderRecorder();
}

function startWebRecorder(){
  webRecActive = true;
  if(webRecLines.length===0) webRecLastMs = -1;
  renderRecorder();
}

function stopWebRecorder(){
  webRecActive = false;
  renderRecorder();
}

function clearWebRecorder(){
  webRecLines = [];
  webRecLastMs = -1;
  renderRecorder();
}

async function copyWebRecorder(){
  const out=document.getElementById('rec_out');
  if(!out) return;
  try {
    await navigator.clipboard.writeText(out.value || '');
    const st=document.getElementById('rec_status');
    if(st) st.textContent = `Recorder: ${webRecActive ? 'capturing' : 'idle'} | samples=${webRecLines.length} | copied`;
  } catch(e) {
    alert('Copy failed. You can still select text from the recorder and copy manually.');
  }
}

async function applyConfig(){
  const wheel=document.getElementById('wheel').value;
  const target=document.getElementById('target').value;
  const kp=document.getElementById('kp').value;
  const ki=document.getElementById('ki').value;
  const kd=document.getElementById('kd').value;
  const minpwm=document.getElementById('minpwm').value;
  const alpha=document.getElementById('alpha').value;
  await hit(`/cmd/config?wheel=${wheel}&target=${target}&kp=${kp}&ki=${ki}&kd=${kd}&minpwm=${minpwm}&alpha=${alpha}`);
  await refresh();
}
async function setMode(){
  const m=document.getElementById('mode').value;
  await hit(`/cmd/mode?m=${m}`);
  await refresh();
}
async function applyMotionConfig(){
  motionCfgPending = true;
  motionCfgFreezeUntilMs = Date.now() + 1200;
  const stepcm=document.getElementById('pos_stepcm').value;
  const wdcm=document.getElementById('wheel_diam_cm').value;
  const postol=document.getElementById('pos_tol').value;
  const pkp=document.getElementById('pos_kp').value;
  const pki=document.getElementById('pos_ki').value;
  const pkd=document.getElementById('pos_kd').value;
  const hyaw=document.getElementById('head_target').value;
  const hkp=document.getElementById('head_kp').value;
  const hkd=document.getElementById('head_kd').value;
  const hmax=document.getElementById('head_max').value;
  const hhold=document.getElementById('head_hold').value;
  const hholdmax=document.getElementById('head_hold_max').value;
  const hholddb=document.getElementById('head_hold_db').value;
  const hhkp=document.getElementById('head_hold_kp').value;
  const hhkd=document.getElementById('head_hold_kd').value;
  const alpha=document.getElementById('alpha').value;
  try {
    await hit(`/cmd/motionConfig?stepcm=${stepcm}&wdcm=${wdcm}&postol=${postol}&pkp=${pkp}&pki=${pki}&pkd=${pkd}&hyaw=${hyaw}&hkp=${hkp}&hkd=${hkd}&hmax=${hmax}&hhold=${hhold}&hholdmax=${hholdmax}&hholddb=${hholddb}&hhkp=${hhkp}&hhkd=${hhkd}&alpha=${alpha}`);
    await refresh();
  } finally {
    motionCfgPending = false;
    motionCfgFreezeUntilMs = Date.now() + 500;
  }
}
async function startYawHold(){
  const hyaw=document.getElementById('head_target').value;
  await applyMotionConfig();
  await hit(`/cmd/yawHoldStart?hyaw=${hyaw}`);
  await refresh();
}
async function startPosStep(dir){
  const stepcm=document.getElementById('pos_stepcm').value;
  const wdcm=document.getElementById('wheel_diam_cm').value;
  await applyMotionConfig();
  clearPositionGraph();
  await hit(`/cmd/posStep?dir=${dir}&stepcm=${stepcm}&wdcm=${wdcm}`);
  await refresh();
}
async function startPoseMove(){
  const distcm=document.getElementById('pose_distcm').value;
  const hyaw=document.getElementById('pose_yaw').value;
  const wdcm=document.getElementById('wheel_diam_cm').value;
  await applyMotionConfig();
  clearPositionGraph();
  await hit(`/cmd/poseMove?distcm=${distcm}&hyaw=${hyaw}&wdcm=${wdcm}`);
  await refresh();
}
async function applyRpmSign(){
  const m0=document.getElementById('rs0').value;
  const m1=document.getElementById('rs1').value;
  const m2=document.getElementById('rs2').value;
  const m3=document.getElementById('rs3').value;
  await hit(`/cmd/rpmSign?m0=${m0}&m1=${m1}&m2=${m2}&m3=${m3}`);
  await refresh();
}
async function applySlipConfig(){
  const en=document.getElementById('slip_en').value;
  const rpm=document.getElementById('slip_rpm').value;
  const accg=document.getElementById('slip_accg').value;
  const detect=document.getElementById('slip_detect').value;
  const release=document.getElementById('slip_release').value;
  const maxrpm=document.getElementById('slip_maxrpm').value;
  const accalpha=document.getElementById('slip_accalpha').value;
  await hit(`/cmd/slipConfig?en=${en}&rpm=${rpm}&accg=${accg}&detect=${detect}&release=${release}&maxrpm=${maxrpm}&accalpha=${accalpha}`);
  await refresh();
}
async function applyDebugConfig(){
  const en=document.getElementById('dbg_en').value;
  const ms=document.getElementById('dbg_ms').value;
  await hit(`/cmd/debugConfig?en=${en}&ms=${ms}`);
  await refresh();
}
async function startCtrl(){await hit('/cmd/start'); await refresh();}
async function stopCtrl(){await hit('/cmd/stop'); await refresh();}
async function resetI(){await hit('/cmd/resetI'); await refresh();}
async function resetEnc(){await hit('/cmd/resetEnc'); await refresh();}
async function imuZero(){await hit('/cmd/imuZero'); await refresh();}
async function imuCal(){await hit('/cmd/imuCal'); await refresh();}
async function startRaw(dir){
  const pwm=document.getElementById('rawpwm').value;
  const signed = Number(pwm) * Number(dir);
  await hit(`/cmd/rawStart?pwm=${signed}`);
  await refresh();
}
async function stopRaw(){await hit('/cmd/rawStop'); await refresh();}
async function startSweep(dir){
  const s=document.getElementById('ms_start').value;
  const e=document.getElementById('ms_end').value;
  const st=document.getElementById('ms_step').value;
  const h=document.getElementById('ms_hold').value;
  await hit(`/cmd/minpwmStart?dir=${dir}&start=${s}&end=${e}&step=${st}&hold=${h}`);
  await refresh();
}
async function stopSweep(){await hit('/cmd/minpwmStop'); await refresh();}

async function refresh(){
  const r=await fetch('/data');
  if(!r.ok) return;
  const d=await r.json();
  const freezeMotionSync = motionCfgPending || (Date.now() < motionCfgFreezeUntilMs);

  document.getElementById('status').textContent = `Status: ${d.enabled ? 'RUNNING' : 'STOPPED'} | Mode=${d.modeName} | Active wheel=${d.activeWheel} | Target=${f(d.pidTarget,1)} rpm (0..60) | Kp=${f(d.kp,2)} Ki=${f(d.ki,2)} Kd=${f(d.kd,2)} MinPWM=${d.minPwm} | LPF alpha=${f(d.rpmFilterAlpha,2)} | Sign BR/FR/BL/FL=${d.rpmSign[0]}/${d.rpmSign[1]}/${d.rpmSign[2]}/${d.rpmSign[3]}`;
  document.getElementById('rawStatus').textContent = `Raw mode: ${d.raw.enabled ? 'ACTIVE' : 'IDLE'} | PWM=${d.raw.pwm}`;
  document.getElementById('motionStatus').textContent = `Position: target=${f(d.position.targetCm,2)} cm (${f(d.position.targetCounts,0)} cnt) | current=${f(d.position.currentCm,2)} cm (${f(d.position.currentCounts,0)} cnt) | error=${f(d.position.errorCm,2)} cm (${f(d.position.errorCounts,0)} cnt) | tol=${f(d.position.tolCm,2)} cm | cmd=${f(d.position.cmdRpm,2)} rpm | done=${d.position.done ? 'YES' : 'NO'} | Seq active=${d.position.poseMoveActive ? 1 : 0} phase=${f(d.position.poseMovePhase,0)} dist=${f(d.position.poseMoveDistanceCm,2)}cm | MoveHeadPID=(${f(d.heading.kp,2)}, ${f(d.heading.ki,2)}, ${f(d.heading.kd,2)}) | HoldHeadPID=(${f(d.heading.holdKp,2)}, ${f(d.heading.holdKi,2)}, ${f(d.heading.holdKd,2)}) | Hold=${d.heading.holdAtStop ? 'ON' : 'OFF'} active=${d.heading.holdActive ? '1' : '0'} stable=${f(d.heading.holdStableMs,0)}ms | hold min/max=${f(d.heading.holdMinRpm,1)}/${f(d.heading.holdMaxRpm,1)} db=${f(d.heading.holdDeadbandDeg,1)}deg`;
  document.getElementById('imuStatus').textContent = `IMU: ${d.imu.ok ? 'OK' : 'NOT FOUND'} | yaw=${f(d.imu.yawDeg,2)} deg | gyroZ=${f(d.imu.gyroDps,2)} dps | accX=${f(d.imu.accXg,3)}g | accY=${f(d.imu.accYg,3)}g | accXY=${f(d.imu.accXYg,3)}g | bias=${f(d.imu.biasDps,3)} dps | headingErr=${f(d.heading.errorDeg,2)} deg | corr=${f(d.heading.corrRpm,2)} rpm`;
  document.getElementById('debugStatus').textContent = `Debug: serial=${d.debug.serialEnabled ? 'ON' : 'OFF'}@${f(d.debug.serialPeriodMs,0)}ms | phase=${f(d.debug.yawPhase,0)} lock=${d.heading.yawHoldInPlaceMode ? 1 : 0} | rawErr=${f(d.debug.rawHeadingErrDeg,2)} usedErr=${f(d.debug.usedHeadingErrDeg,2)} | corrTarget=${f(d.debug.corrTargetRpm,2)} corrOut=${f(d.debug.corrAfterSlewRpm,2)} | gyroFilt=${f(d.debug.gyroDpsFilt,2)} | settled E/R=${d.debug.errSettled ? 1 : 0}/${d.debug.rateSettled ? 1 : 0}`;
  document.getElementById('slipStatus').textContent = `Slip: ${d.slip.enabled ? 'ENABLED' : 'DISABLED'} | state=${d.slip.detected ? 'DETECTED' : 'clear'} | candidate=${f(d.slip.candidateMs,0)}ms | avgWheel=${f(d.slip.avgWheelRpm,2)} rpm | rpmThr=${f(d.slip.wheelRpmThresh,1)} | accThr=${f(d.slip.accelThreshG,3)}g | detect=${f(d.slip.detectMs,0)}ms | release=${f(d.slip.releaseMs,0)}ms | maxRPM=${f(d.slip.maxRpm,1)}`;

  const sm=d.minSweep;
  document.getElementById('sweepStatus').textContent = `Sweep: ${sm.active ? 'ACTIVE' : 'IDLE'} | Dir=${sm.direction > 0 ? 'FWD' : 'REV'} | PWM=${sm.currentPwm} | Range=${sm.startPwm}-${sm.endPwm} step ${sm.stepPwm} | Hold=${sm.holdMs}ms | AvgAbsRPM=${f(sm.avgAbsRpm,2)} | Detected=${sm.detectedPwm}`;

  syncFieldValue('mode', d.mode);
  syncFieldValue('rs0', d.rpmSign[0]);
  syncFieldValue('rs1', d.rpmSign[1]);
  syncFieldValue('rs2', d.rpmSign[2]);
  syncFieldValue('rs3', d.rpmSign[3]);
  syncFieldValue('slip_en', d.slip.enabled ? '1' : '0');
  syncFieldValue('slip_rpm', f(d.slip.wheelRpmThresh,1));
  syncFieldValue('slip_accg', f(d.slip.accelThreshG,3));
  syncFieldValue('slip_detect', f(d.slip.detectMs,0));
  syncFieldValue('slip_release', f(d.slip.releaseMs,0));
  syncFieldValue('slip_maxrpm', f(d.slip.maxRpm,1));
  syncFieldValue('slip_accalpha', f(d.imu.accAlpha,2));
  if (!freezeMotionSync) syncFieldValue('pos_stepcm', f(d.position.stepCm,2));
  if (!freezeMotionSync) syncFieldValue('wheel_diam_cm', f(d.position.wheelDiameterCm,2));
  if (!freezeMotionSync) syncFieldValue('pos_tol', f(d.position.tolCm,2));
  syncFieldValue('head_target', f(d.heading.targetDeg,1));
  if (!freezeMotionSync) syncFieldValue('head_kp', f(d.heading.kp,2));
  if (!freezeMotionSync) syncFieldValue('head_ki', f(d.heading.ki,3));
  if (!freezeMotionSync) syncFieldValue('head_kd', f(d.heading.kd,3));
  if (!freezeMotionSync) syncFieldValue('head_max', f(d.heading.maxCorrRpm,1));
  if (!freezeMotionSync) syncFieldValue('head_base', f(d.heading.enableBaseRpm,2));
  syncFieldValue('head_sign', String(d.heading.sign));
  if (!freezeMotionSync) syncFieldValue('head_hold', d.heading.holdAtStop ? '1' : '0');
  if (!freezeMotionSync) syncFieldValue('head_hold_max', f(d.heading.holdMaxRpm,1));
  if (!freezeMotionSync) syncFieldValue('head_hold_minpwm', f(d.heading.holdMinPwm,0));
  if (!freezeMotionSync) syncFieldValue('head_db', f(d.heading.deadbandDeg,1));
  if (!freezeMotionSync) syncFieldValue('head_hold_db', f(d.heading.holdDeadbandDeg,1));
  if (!freezeMotionSync) syncFieldValue('head_hold_kp', f(d.heading.holdKp,2));
  if (!freezeMotionSync) syncFieldValue('head_hold_ki', f(d.heading.holdKi,2));
  if (!freezeMotionSync) syncFieldValue('head_hold_kd', f(d.heading.holdKd,2));
  if (!freezeMotionSync) syncFieldValue('head_hold_rate_db', f(d.heading.holdExitRateDps,2));
  if (!freezeMotionSync) syncFieldValue('head_slew', f(d.heading.corrSlewRpmPerSec,1));
  if (!freezeMotionSync) syncFieldValue('pose_yaw', f(d.heading.targetDeg,1));
  if (!freezeMotionSync && d.position.poseMoveActive) syncFieldValue('pose_distcm', f(d.position.poseMoveDistanceCm,2));
  syncFieldValue('dbg_en', d.debug.serialEnabled ? '1' : '0');
  syncFieldValue('dbg_ms', f(d.debug.serialPeriodMs,0));

  for(let i=0;i<4;i++){
    document.getElementById('rpm'+i).textContent=f(d.measured[i],1);
    document.getElementById('tr'+i).textContent=f(d.target[i],1);
    document.getElementById('mr'+i).textContent=f(d.measured[i],1);
    document.getElementById('er'+i).textContent=f(d.target[i]-d.measured[i],1);
    document.getElementById('pw'+i).textContent=d.pwm[i];
  }

  appendRecorderSample(d);
  updateGraphs(d);
}
setInterval(refresh, 200);
renderRecorder();
refresh();
</script>
</body>
</html>
)HTML";
}

String jsonData() {
  String s = "{";
  s += "\"nowMs\":" + String(millis()) + ",";
  s += "\"enabled\":" + String(controllerEnabled ? "true" : "false") + ",";
  s += "\"mode\":" + String((int)controlMode) + ",";
  s += "\"modeName\":\"" + String(controlMode == MODE_POSITION_HEADING ? "position+heading" : "single-wheel velocity") + "\",";
  s += "\"activeWheel\":" + String(activeWheel) + ",";
  s += "\"pidTarget\":" + String(pidTargetRpm, 3) + ",";
  s += "\"kp\":" + String(pid[activeWheel].kp, 4) + ",";
  s += "\"ki\":" + String(pid[activeWheel].ki, 4) + ",";
  s += "\"kd\":" + String(pid[activeWheel].kd, 4) + ",";
  s += "\"minPwm\":" + String(minPwm) + ",";
  s += "\"rpmFilterAlpha\":" + String(rpmFilterAlpha, 3) + ",";

  s += "\"rpmSign\":[";
  for (uint8_t i = 0; i < 4; i++) {
    s += String(rpmSign[i]);
    if (i < 3) s += ",";
  }
  s += "],";

  s += "\"raw\":{";
  s += "\"enabled\":" + String(rawModeEnabled ? "true" : "false") + ",";
  s += "\"pwm\":" + String(rawPwmCmd);
  s += "},";

  s += "\"minSweep\":{";
  s += "\"active\":" + String(minSweep.active ? "true" : "false") + ",";
  s += "\"direction\":" + String(minSweep.direction) + ",";
  s += "\"startPwm\":" + String(minSweep.startPwm) + ",";
  s += "\"endPwm\":" + String(minSweep.endPwm) + ",";
  s += "\"stepPwm\":" + String(minSweep.stepPwm) + ",";
  s += "\"holdMs\":" + String(minSweep.holdMs) + ",";
  s += "\"currentPwm\":" + String(minSweep.currentPwm) + ",";
  s += "\"detectedPwm\":" + String(minSweep.detectedPwm) + ",";
  s += "\"avgAbsRpm\":" + String(minSweep.avgAbsRpm, 3);
  s += "},";

  s += "\"imu\":{";
  s += "\"ok\":" + String(imuOk ? "true" : "false") + ",";
  s += "\"yawDeg\":" + String(imuYawDeg, 3) + ",";
  s += "\"gyroDps\":" + String(imuGyroDps, 3) + ",";
  s += "\"accXg\":" + String(imuAccXg, 4) + ",";
  s += "\"accYg\":" + String(imuAccYg, 4) + ",";
  s += "\"accXYg\":" + String(imuAccXYg, 4) + ",";
  s += "\"accAlpha\":" + String(imuAccAlpha, 3) + ",";
  s += "\"biasDps\":" + String(imuBiasDps, 4);
  s += "},";

  s += "\"slip\":{";
  s += "\"enabled\":" + String(slipEnabled ? "true" : "false") + ",";
  s += "\"detected\":" + String(slipDetected ? "true" : "false") + ",";
  s += "\"candidateMs\":" + String(slipCandidateMs, 1) + ",";
  s += "\"wheelRpmThresh\":" + String(slipWheelRpmThresh, 3) + ",";
  s += "\"accelThreshG\":" + String(slipAccelThreshG, 4) + ",";
  s += "\"detectMs\":" + String(slipDetectMs, 1) + ",";
  s += "\"releaseMs\":" + String(slipReleaseMs, 1) + ",";
  s += "\"maxRpm\":" + String(slipMaxRpm, 3) + ",";
  s += "\"avgWheelRpm\":" + String(slipAvgWheelRpm, 3);
  s += "},";

  s += "\"position\":{";
  s += "\"targetCounts\":" + String(positionTargetCounts, 3) + ",";
  s += "\"currentCounts\":" + String(positionCurrentCounts, 3) + ",";
  s += "\"errorCounts\":" + String(positionErrorCounts, 3) + ",";
  s += "\"targetCm\":" + String(countsToCm(positionTargetCounts), 3) + ",";
  s += "\"currentCm\":" + String(countsToCm(positionCurrentCounts), 3) + ",";
  s += "\"errorCm\":" + String(countsToCm(positionErrorCounts), 3) + ",";
  s += "\"stepCm\":" + String(positionStepCm, 3) + ",";
  s += "\"wheelDiameterCm\":" + String(wheelDiameterCm, 3) + ",";
  s += "\"poseMoveActive\":" + String(poseMoveActive ? "true" : "false") + ",";
  s += "\"poseMovePhase\":" + String(poseMovePhase) + ",";
  s += "\"poseMoveDistanceCm\":" + String(poseMoveDistanceCm, 3) + ",";
  s += "\"poseMoveStartCounts\":" + String(poseMoveStartCounts, 3) + ",";
  s += "\"poseMoveTargetCounts\":" + String(poseMoveTargetCounts, 3) + ",";
  s += "\"tolCounts\":" + String(cmToCounts(positionTolCm), 3) + ",";
  s += "\"tolCm\":" + String(positionTolCm, 3) + ",";
  s += "\"cmdRpm\":" + String(positionCmdRpm, 3) + ",";
  s += "\"done\":" + String(positionDone ? "true" : "false") + ",";
  s += "\"kp\":" + String(posPid.kp, 5) + ",";
  s += "\"ki\":" + String(posPid.ki, 5) + ",";
  s += "\"kd\":" + String(posPid.kd, 5);
  s += "},";

  s += "\"heading\":{";
  s += "\"targetDeg\":" + String(headingTargetDeg, 3) + ",";
  s += "\"errorDeg\":" + String(headingErrorDeg, 3) + ",";
  s += "\"corrRpm\":" + String(headingCorrRpm, 3) + ",";
  s += "\"maxCorrRpm\":" + String(headingCorrMaxRpm, 3) + ",";
  s += "\"holdMaxRpm\":" + String(headingHoldMaxRpm, 3) + ",";
  s += "\"holdMinRpm\":" + String(headingHoldMinRpm, 3) + ",";
  s += "\"holdMinPwm\":" + String(holdMinPwm) + ",";
  s += "\"holdEnterDeg\":" + String(headingHoldEnterDeg, 3) + ",";
  s += "\"holdExitDeg\":" + String(headingHoldExitDeg, 3) + ",";
  s += "\"holdExitRateDps\":" + String(headingHoldExitRateDps, 3) + ",";
  s += "\"holdActive\":" + String(headingHoldActive ? "true" : "false") + ",";
  s += "\"holdStableMs\":" + String(headingHoldStableMs, 1) + ",";
  s += "\"corrSlewRpmPerSec\":" + String(headingCorrSlewRpmPerSec, 3) + ",";
  s += "\"deadbandDeg\":" + String(headingDeadbandDeg, 3) + ",";
  s += "\"holdDeadbandDeg\":" + String(headingHoldDeadbandDeg, 3) + ",";
  s += "\"enableBaseRpm\":" + String(headingEnableBaseRpm, 3) + ",";
  s += "\"sign\":" + String(headingCorrSign) + ",";
  s += "\"yawHoldInPlaceMode\":" + String(yawHoldInPlaceMode ? "true" : "false") + ",";
  s += "\"autoLock\":" + String(autoLockHeadingOnStep ? "true" : "false") + ",";
  s += "\"holdAtStop\":" + String(holdHeadingAtStop ? "true" : "false") + ",";
  s += "\"kp\":" + String(headingPid.kp, 5) + ",";
  s += "\"ki\":" + String(headingPid.ki, 5) + ",";
  s += "\"kd\":" + String(headingPid.kd, 5) + ",";
  s += "\"holdKp\":" + String(headingHoldPid.kp, 5) + ",";
  s += "\"holdKi\":" + String(headingHoldPid.ki, 5) + ",";
  s += "\"holdKd\":" + String(headingHoldPid.kd, 5);
  s += "},";

  s += "\"debug\":{";
  s += "\"serialEnabled\":" + String(debugSerialEnabled ? "true" : "false") + ",";
  s += "\"serialPeriodMs\":" + String(debugSerialPeriodMs) + ",";
  s += "\"yawPhase\":" + String(debugYawPhase) + ",";
  s += "\"errSettled\":" + String(debugErrSettled ? "true" : "false") + ",";
  s += "\"rateSettled\":" + String(debugRateSettled ? "true" : "false") + ",";
  s += "\"rawHeadingErrDeg\":" + String(debugRawHeadingErrDeg, 3) + ",";
  s += "\"usedHeadingErrDeg\":" + String(debugUsedHeadingErrDeg, 3) + ",";
  s += "\"corrTargetRpm\":" + String(debugCorrTargetRpm, 3) + ",";
  s += "\"corrAfterSlewRpm\":" + String(debugCorrAfterSlewRpm, 3) + ",";
  s += "\"gyroDpsFilt\":" + String(imuGyroDpsFilt, 3) + ",";

  s += "\"wheelErr\":[";
  for (uint8_t i = 0; i < 4; i++) {
    s += String(debugWheelErr[i], 3);
    if (i < 3) s += ",";
  }
  s += "],";

  s += "\"wheelPidOut\":[";
  for (uint8_t i = 0; i < 4; i++) {
    s += String(debugWheelPidOut[i], 3);
    if (i < 3) s += ",";
  }
  s += "],";

  s += "\"wheelCmdPreBreakaway\":[";
  for (uint8_t i = 0; i < 4; i++) {
    s += String(debugWheelCmdPreBreakaway[i]);
    if (i < 3) s += ",";
  }
  s += "]";
  s += "},";

  s += "\"target\":[";
  for (uint8_t i = 0; i < 4; i++) {
    s += String(targetRpm[i], 3);
    if (i < 3) s += ",";
  }
  s += "],";

  // measuredRpm[] is already in motor order [MOTA, MOTB, MOTC, MOTD]
  s += "\"measured\":[";
  for (uint8_t i = 0; i < 4; i++) {
    s += String(measuredRpm[i], 3);
    if (i < 3) s += ",";
  }
  s += "],";

  s += "\"measuredRaw\":[";
  for (uint8_t i = 0; i < 4; i++) {
    s += String(measuredRpmRaw[i], 3);
    if (i < 3) s += ",";
  }
  s += "],";

  s += "\"pwm\":[";
  for (uint8_t i = 0; i < 4; i++) {
    s += String(pwmCmd[i]);
    if (i < 3) s += ",";
  }
  s += "]";

  s += "}";
  return s;
}

void handleRoot() {
  server.send(200, "text/html", pageHtml());
}

void handleData() {
  server.send(200, "application/json", jsonData());
}

void handleDebugConfig() {
  if (server.hasArg("en")) {
    debugSerialEnabled = server.arg("en").toInt() != 0;
  }
  if (server.hasArg("ms")) {
    debugSerialPeriodMs = (uint16_t)constrain(server.arg("ms").toInt(), 50, 2000);
  }

  String msg = "Debug config applied | serial=" + String(debugSerialEnabled ? "ON" : "OFF") +
               " | periodMs=" + String(debugSerialPeriodMs);
  server.send(200, "text/plain", msg);
}

void handleRpmSign() {
  for (uint8_t i = 0; i < 4; i++) {
    String argName = String("m") + String(i);
    if (server.hasArg(argName)) {
      int s = server.arg(argName).toInt();
      rpmSign[i] = (s >= 0) ? 1 : -1;
    }
  }

  resetAllPidStates();
  server.send(200, "text/plain", "RPM sign updated");
}

void handleSlipConfig() {
  if (server.hasArg("en")) {
    slipEnabled = server.arg("en").toInt() != 0;
  }
  if (server.hasArg("rpm")) {
    slipWheelRpmThresh = constrain(server.arg("rpm").toFloat(), 1.0f, TARGET_RPM_MAX);
  }
  if (server.hasArg("accg")) {
    slipAccelThreshG = constrain(server.arg("accg").toFloat(), 0.001f, 1.5f);
  }
  if (server.hasArg("detect")) {
    slipDetectMs = constrain(server.arg("detect").toFloat(), 80.0f, 3000.0f);
  }
  if (server.hasArg("release")) {
    slipReleaseMs = constrain(server.arg("release").toFloat(), 40.0f, 3000.0f);
  }
  if (server.hasArg("maxrpm")) {
    slipMaxRpm = constrain(server.arg("maxrpm").toFloat(), 0.0f, TARGET_RPM_MAX);
  }
  if (server.hasArg("accalpha")) {
    imuAccAlpha = constrain(server.arg("accalpha").toFloat(), 0.02f, 1.0f);
  }

  if (!slipEnabled) {
    slipDetected = false;
    slipCandidateMs = 0.0f;
  }

  server.send(200, "text/plain", "Slip config applied");
}

void handleConfig() {
  bool wheelChanged = false;
  if (server.hasArg("wheel")) {
    int w = server.arg("wheel").toInt();
    if (w >= 0 && w < 4) {
      wheelChanged = (activeWheel != (uint8_t)w);
      activeWheel = (uint8_t)w;
    }
  }

  if (server.hasArg("target")) {
    pidTargetRpm = server.arg("target").toFloat();
    pidTargetRpm = constrain(pidTargetRpm, 0.0f, TARGET_RPM_MAX);
  }

  if (server.hasArg("kp")) {
    float v = server.arg("kp").toFloat();
    for (uint8_t i = 0; i < 4; i++) pid[i].kp = v;
  }
  if (server.hasArg("ki")) {
    float v = server.arg("ki").toFloat();
    for (uint8_t i = 0; i < 4; i++) pid[i].ki = v;
  }
  if (server.hasArg("kd")) {
    float v = server.arg("kd").toFloat();
    for (uint8_t i = 0; i < 4; i++) pid[i].kd = v;
  }

  if (server.hasArg("minpwm")) {
    int v = server.arg("minpwm").toInt();
    minPwm = (uint8_t)constrain(v, 0, PWM_MAX);
  }

  if (server.hasArg("alpha")) {
    float a = server.arg("alpha").toFloat();
    rpmFilterAlpha = constrain(a, 0.02f, 1.0f);
  }

  if (rawModeEnabled && wheelChanged) {
    stopAllMotors();
    setMotorPwmSigned(activeWheel, rawPwmCmd);
  }

  updateTargets();
  server.send(200, "text/plain", "Config applied");
}

void handleMode() {
  if (server.hasArg("m")) {
    String m = server.arg("m");
    if (m == "1" || m == "pos" || m == "position") {
      controlMode = MODE_POSITION_HEADING;
    } else {
      controlMode = MODE_SINGLE_WHEEL_VELOCITY;
    }
  }

  if (rawModeEnabled) {
    stopRawMode();
  }
  if (minSweep.active) {
    stopMinPwmSweep();
  }

  cancelPoseMove();

  resetHighLevelControllers();
  updateTargets();
  server.send(200, "text/plain", controlMode == MODE_POSITION_HEADING ? "Mode: POSITION+HEADING" : "Mode: SINGLE-WHEEL VELOCITY");
}

void handleMotionConfig() {
  if (server.hasArg("pos")) {
    positionTargetCounts = server.arg("pos").toFloat();
    positionDone = false;
    positionDoneTicks = 0;
  }
  if (server.hasArg("stepcm")) {
    positionStepCm = constrain(server.arg("stepcm").toFloat(), -1000.0f, 1000.0f);
  }
  if (server.hasArg("wdcm")) {
    wheelDiameterCm = constrain(server.arg("wdcm").toFloat(), 1.0f, 30.0f);
  }
  if (server.hasArg("postol")) {
    positionTolCm = server.arg("postol").toFloat();
    positionTolCm = constrain(positionTolCm, 0.1f, 200.0f);
  }

  if (server.hasArg("pkp")) posPid.kp = server.arg("pkp").toFloat();
  if (server.hasArg("pki")) posPid.ki = server.arg("pki").toFloat();
  if (server.hasArg("pkd")) posPid.kd = server.arg("pkd").toFloat();

  if (server.hasArg("hyaw")) {
    headingTargetDeg = wrapAngleDeg(server.arg("hyaw").toFloat());
  }
  if (server.hasArg("hkp")) headingPid.kp = server.arg("hkp").toFloat();
  if (server.hasArg("hki")) headingPid.ki = server.arg("hki").toFloat();
  if (server.hasArg("hkd")) headingPid.kd = server.arg("hkd").toFloat();
  if (server.hasArg("hhkp")) headingHoldPid.kp = server.arg("hhkp").toFloat();
  if (server.hasArg("hhki")) headingHoldPid.ki = server.arg("hhki").toFloat();
  if (server.hasArg("hhkd")) headingHoldPid.kd = server.arg("hhkd").toFloat();
  if (server.hasArg("hmax")) {
    headingCorrMaxRpm = server.arg("hmax").toFloat();
    headingCorrMaxRpm = constrain(headingCorrMaxRpm, 0.0f, TARGET_RPM_MAX);
  }
  if (server.hasArg("hholdmax")) {
    headingHoldMaxRpm = server.arg("hholdmax").toFloat();
    headingHoldMaxRpm = constrain(headingHoldMaxRpm, 0.0f, TARGET_RPM_MAX);
  }
  if (server.hasArg("hholdmin")) {
    headingHoldMinRpm = constrain(server.arg("hholdmin").toFloat(), 0.0f, TARGET_RPM_MAX);
  }
  if (server.hasArg("hholdminpwm")) {
    holdMinPwm = (uint8_t)constrain(server.arg("hholdminpwm").toInt(), 0, PWM_MAX);
  }
  if (server.hasArg("hhoutrate")) {
    headingHoldExitRateDps = constrain(server.arg("hhoutrate").toFloat(), 0.0f, 40.0f);
  }
  if (server.hasArg("hslew")) {
    headingCorrSlewRpmPerSec = constrain(server.arg("hslew").toFloat(), 0.0f, 400.0f);
  }
  if (server.hasArg("hsign")) {
    int s = server.arg("hsign").toInt();
    headingCorrSign = (s >= 0) ? 1 : -1;
  }
  if (server.hasArg("hdb")) {
    headingDeadbandDeg = constrain(server.arg("hdb").toFloat(), 0.0f, 30.0f);
  }
  if (server.hasArg("hholddb")) {
    headingHoldDeadbandDeg = constrain(server.arg("hholddb").toFloat(), 0.0f, 30.0f);
  }
  if (server.hasArg("hbase")) {
    headingEnableBaseRpm = constrain(server.arg("hbase").toFloat(), 0.0f, TARGET_RPM_MAX);
  }
  if (server.hasArg("hhold")) {
    holdHeadingAtStop = server.arg("hhold").toInt() != 0;
    if (!holdHeadingAtStop) {
      yawHoldInPlaceMode = false;
    }
  }

  if (server.hasArg("alpha")) {
    float a = server.arg("alpha").toFloat();
    rpmFilterAlpha = constrain(a, 0.02f, 1.0f);
  }

  if (headingHoldMinRpm > headingHoldMaxRpm) {
    headingHoldMinRpm = headingHoldMaxRpm;
  }

  resetHighLevelControllers();
  server.send(200, "text/plain", "Motion config applied");
}

void handleYawHoldStart() {
  cancelPoseMove();

  if (rawModeEnabled) {
    stopRawMode();
  }
  if (minSweep.active) {
    stopMinPwmSweep();
  }

  if (server.hasArg("hyaw")) {
    headingTargetDeg = wrapAngleDeg(server.arg("hyaw").toFloat());
  }

  if (!imuOk) {
    controllerEnabled = false;
    yawHoldInPlaceMode = false;
    stopAllMotors();
    server.send(200, "text/plain", "Yaw hold aborted: IMU not found");
    return;
  }

  // Force hold branch active for dedicated yaw-hold tests.
  holdHeadingAtStop = true;

  positionTargetCounts = getAverageMotorEncoderCount();
  controlMode = MODE_POSITION_HEADING;
  controllerEnabled = true;
  yawHoldInPlaceMode = true;
  resetHighLevelControllers();
  yawHoldInPlaceMode = true;
  positionDone = true;
  positionDoneTicks = 5;

  String msg = "Yaw hold active | targetDeg=" + String(headingTargetDeg, 2);
  server.send(200, "text/plain", msg);
}

void handlePositionStep() {
  cancelPoseMove();

  int dir = server.hasArg("dir") ? server.arg("dir").toInt() : 1;
  float stepCm = positionStepCm;
  if (server.hasArg("stepcm")) {
    stepCm = constrain(server.arg("stepcm").toFloat(), -1000.0f, 1000.0f);
  }
  if (server.hasArg("wdcm")) {
    wheelDiameterCm = constrain(server.arg("wdcm").toFloat(), 1.0f, 30.0f);
  }
  positionStepCm = stepCm;
  yawHoldInPlaceMode = false;

  if (rawModeEnabled) {
    stopRawMode();
  }
  if (minSweep.active) {
    stopMinPwmSweep();
  }

  float current = getAverageMotorEncoderCount();
  float stepCounts = cmToCounts(fabsf(stepCm));
  int8_t sign = (dir >= 0) ? 1 : -1;
  if (stepCm < 0.0f) {
    sign = -sign;
  }

  if (imuOk && autoLockHeadingOnStep) {
    headingTargetDeg = imuYawDeg;
  }

  positionTargetCounts = current + ((float)sign) * stepCounts;
  controllerEnabled = true;
  controlMode = MODE_POSITION_HEADING;
  resetHighLevelControllers();

  String msg = "Position step started | stepCm=" + String(positionStepCm, 2) + " | targetCounts=" + String(positionTargetCounts, 1);
  server.send(200, "text/plain", msg);
}

void handlePoseMove() {
  cancelPoseMove();

  if (rawModeEnabled) {
    stopRawMode();
  }
  if (minSweep.active) {
    stopMinPwmSweep();
  }

  if (server.hasArg("wdcm")) {
    wheelDiameterCm = constrain(server.arg("wdcm").toFloat(), 1.0f, 30.0f);
  }

  float distCm = server.hasArg("distcm") ? server.arg("distcm").toFloat() : positionStepCm;
  distCm = constrain(distCm, -1000.0f, 1000.0f);

  if (server.hasArg("hyaw")) {
    headingTargetDeg = wrapAngleDeg(server.arg("hyaw").toFloat());
  }

  if (!imuOk) {
    controllerEnabled = false;
    yawHoldInPlaceMode = false;
    stopAllMotors();
    server.send(200, "text/plain", "Pose move aborted: IMU not found");
    return;
  }

  poseMoveDistanceCm = distCm;
  poseMoveStartCounts = getAverageMotorEncoderCount();
  poseMoveTargetCounts = poseMoveStartCounts + cmToCounts(poseMoveDistanceCm);

  holdHeadingAtStop = true;
  controlMode = MODE_POSITION_HEADING;
  controllerEnabled = true;
  resetHighLevelControllers();

  poseMoveActive = true;
  poseMovePhase = 1;
  yawHoldInPlaceMode = true;
  positionTargetCounts = poseMoveStartCounts;
  positionDone = true;
  positionDoneTicks = 5;

  String msg = "Pose move started | distCm=" + String(poseMoveDistanceCm, 2) + " | targetYawDeg=" + String(headingTargetDeg, 2);
  server.send(200, "text/plain", msg);
}

void handleImuZero() {
  imuZeroYaw();
  resetCascadePid(headingPid);
  server.send(200, "text/plain", "IMU yaw zeroed");
}

void handleImuCal() {
  bool ok = imuCalibrateBias(500);
  server.send(200, "text/plain", ok ? "IMU bias calibrated" : "IMU calibration failed");
}

void handleRawStart() {
  int pwm = server.hasArg("pwm") ? server.arg("pwm").toInt() : PWM_MAX;
  startRawMode((int16_t)pwm);
  server.send(200, "text/plain", "Raw mode started");
}

void handleRawStop() {
  stopRawMode();
  server.send(200, "text/plain", "Raw mode stopped");
}

void handleMinPwmStart() {
  if (rawModeEnabled) {
    stopRawMode();
  }

  int dir = server.hasArg("dir") ? server.arg("dir").toInt() : 1;
  int start = server.hasArg("start") ? server.arg("start").toInt() : 20;
  int end = server.hasArg("end") ? server.arg("end").toInt() : 90;
  int step = server.hasArg("step") ? server.arg("step").toInt() : 2;
  int hold = server.hasArg("hold") ? server.arg("hold").toInt() : 700;

  startMinPwmSweep((dir >= 0) ? 1 : -1, (uint8_t)start, (uint8_t)end, (uint8_t)step, (uint16_t)hold);
  server.send(200, "text/plain", "MinPWM sweep started");
}

void handleMinPwmStop() {
  stopMinPwmSweep();
  server.send(200, "text/plain", "MinPWM sweep stopped");
}

void handleStart() {
  cancelPoseMove();

  if (rawModeEnabled) {
    stopRawMode();
  }

  if (minSweep.active) {
    stopMinPwmSweep();
  }

  resetHighLevelControllers();

  if (controlMode == MODE_POSITION_HEADING) {
    // In Mode 1, Start means execute one relative distance step from current pose.
    yawHoldInPlaceMode = false;
    float current = getAverageMotorEncoderCount();
    if (imuOk && autoLockHeadingOnStep) {
      headingTargetDeg = imuYawDeg;
    }
    positionTargetCounts = current + cmToCounts(positionStepCm);
  }

  controllerEnabled = true;
  updateTargets();
  server.send(200, "text/plain", "Controller started");
}

void handleStop() {
  cancelPoseMove();

  if (rawModeEnabled) {
    stopRawMode();
  }

  if (minSweep.active) {
    stopMinPwmSweep();
  }
  controllerEnabled = false;
  yawHoldInPlaceMode = false;
  updateTargets();
  resetAllPidStates();
  resetHighLevelControllers();
  stopAllMotors();
  server.send(200, "text/plain", "Controller stopped");
}

void handleResetI() {
  resetAllPidStates();
  server.send(200, "text/plain", "PID integrators reset");
}

void handleResetEnc() {
  resetEncoders();
  server.send(200, "text/plain", "Encoders reset");
}

void setup() {
  Serial.begin(115200);
  delay(300);

  imuInit();

  for (uint8_t i = 0; i < 4; i++) {
    setupEncoderPin(encoders[i].a);
    setupEncoderPin(encoders[i].b);
    encoderPrevState[i] = (digitalRead(encoders[i].a) << 1) | digitalRead(encoders[i].b);
  }

  attachInterrupt(digitalPinToInterrupt(encoders[0].a), isrEnc0A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoders[0].b), isrEnc0B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoders[1].a), isrEnc1A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoders[1].b), isrEnc1B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoders[2].a), isrEnc2A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoders[2].b), isrEnc2B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoders[3].a), isrEnc3A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoders[3].b), isrEnc3B, CHANGE);

  for (uint8_t i = 0; i < 4; i++) {
    pwmAttachPinCompat(motors[i].in1, motors[i].ch1);
    pwmAttachPinCompat(motors[i].in2, motors[i].ch2);
  }

  stopAllMotors();
  resetEncoders();
  resetAllPidStates();
  updateTargets();

  WiFi.mode(WIFI_AP);
  bool apOk = WiFi.softAP(AP_SSID, AP_PASS);

  server.on("/", HTTP_GET, handleRoot);
  server.on("/data", HTTP_GET, handleData);
  server.on("/cmd/debugConfig", HTTP_GET, handleDebugConfig);
  server.on("/cmd/config", HTTP_GET, handleConfig);
  server.on("/cmd/rpmSign", HTTP_GET, handleRpmSign);
  server.on("/cmd/slipConfig", HTTP_GET, handleSlipConfig);
  server.on("/cmd/mode", HTTP_GET, handleMode);
  server.on("/cmd/motionConfig", HTTP_GET, handleMotionConfig);
  server.on("/cmd/posStep", HTTP_GET, handlePositionStep);
  server.on("/cmd/poseMove", HTTP_GET, handlePoseMove);
  server.on("/cmd/yawHoldStart", HTTP_GET, handleYawHoldStart);
  server.on("/cmd/imuZero", HTTP_GET, handleImuZero);
  server.on("/cmd/imuCal", HTTP_GET, handleImuCal);
  server.on("/cmd/start", HTTP_GET, handleStart);
  server.on("/cmd/stop", HTTP_GET, handleStop);
  server.on("/cmd/resetI", HTTP_GET, handleResetI);
  server.on("/cmd/resetEnc", HTTP_GET, handleResetEnc);
  server.on("/cmd/rawStart", HTTP_GET, handleRawStart);
  server.on("/cmd/rawStop", HTTP_GET, handleRawStop);
  server.on("/cmd/minpwmStart", HTTP_GET, handleMinPwmStart);
  server.on("/cmd/minpwmStop", HTTP_GET, handleMinPwmStop);
  server.begin();

  Serial.println();
  Serial.println("Wheel PID tuner started.");
  Serial.print("AP started: ");
  Serial.println(apOk ? "YES" : "NO");
  Serial.print("SSID: ");
  Serial.println(AP_SSID);
  Serial.print("PASS: ");
  Serial.println(AP_PASS);
  Serial.print("IP: ");
  Serial.println(WiFi.softAPIP());
  Serial.print("IMU: ");
  Serial.println(imuOk ? "OK" : "NOT FOUND");
}

void loop() {
  server.handleClient();

  uint32_t now = millis();
  if (now - lastControlMs >= CONTROL_INTERVAL_MS) {
    float dt = (now - lastControlMs) / 1000.0f;
    lastControlMs = now;
    if (dt < 0.001f) dt = CONTROL_INTERVAL_MS / 1000.0f;

    if (minSweep.active) {
      updateMeasuredRpm(dt);
      updateMinPwmSweep(now);
    } else if (rawModeEnabled) {
      updateMeasuredRpm(dt);
      stopAllMotors();
      setMotorPwmSigned(activeWheel, rawPwmCmd);
    } else {
      runControlLoop(dt);
    }
  }

  if (now - lastPrintMs >= 1000) {
    lastPrintMs = now;
    Serial.print("RPM BR/FR/BL/FL: ");
    Serial.print(measuredRpm[0], 1);
    Serial.print(" / ");
    Serial.print(measuredRpm[1], 1);
    Serial.print(" / ");
    Serial.print(measuredRpm[2], 1);
    Serial.print(" / ");
    Serial.print(measuredRpm[3], 1);
    Serial.print(" | PWM: ");
    Serial.print(pwmCmd[0]);
    Serial.print(",");
    Serial.print(pwmCmd[1]);
    Serial.print(",");
    Serial.print(pwmCmd[2]);
    Serial.print(",");
    Serial.print(pwmCmd[3]);
    Serial.print(" | MinSweep active=");
    Serial.print(minSweep.active ? "1" : "0");
    Serial.print(" current=");
    Serial.print(minSweep.currentPwm);
    Serial.print(" detected=");
    Serial.print(minSweep.detectedPwm);
    Serial.print(" | Raw=");
    Serial.print(rawModeEnabled ? "1" : "0");
    Serial.print(" pwm=");
    Serial.print(rawPwmCmd);
    Serial.print(" | Mode=");
    Serial.print(controlMode == MODE_POSITION_HEADING ? "POS" : "VEL");
    Serial.print(" | Yaw=");
    Serial.print(imuYawDeg, 1);
    Serial.print(" | PosErr=");
    Serial.println(positionErrorCounts, 1);
  }

  if (debugSerialEnabled && (now - lastDebugSerialMs >= debugSerialPeriodMs)) {
    lastDebugSerialMs = now;
    Serial.print("DBG,t=");
    Serial.print(now);
    Serial.print(",ph=");
    Serial.print(debugYawPhase);
    Serial.print(",eRaw=");
    Serial.print(debugRawHeadingErrDeg, 3);
    Serial.print(",eUse=");
    Serial.print(debugUsedHeadingErrDeg, 3);
    Serial.print(",gyro=");
    Serial.print(imuGyroDps, 3);
    Serial.print(",gyroF=");
    Serial.print(imuGyroDpsFilt, 3);
    Serial.print(",corrT=");
    Serial.print(debugCorrTargetRpm, 3);
    Serial.print(",corr=");
    Serial.print(debugCorrAfterSlewRpm, 3);
    Serial.print(",setE=");
    Serial.print(debugErrSettled ? 1 : 0);
    Serial.print(",setR=");
    Serial.print(debugRateSettled ? 1 : 0);
    Serial.print(",tr=");
    Serial.print(targetRpm[0], 2); Serial.print("/");
    Serial.print(targetRpm[1], 2); Serial.print("/");
    Serial.print(targetRpm[2], 2); Serial.print("/");
    Serial.print(targetRpm[3], 2);
    Serial.print(",mr=");
    Serial.print(measuredRpm[0], 2); Serial.print("/");
    Serial.print(measuredRpm[1], 2); Serial.print("/");
    Serial.print(measuredRpm[2], 2); Serial.print("/");
    Serial.print(measuredRpm[3], 2);
    Serial.print(",pwm=");
    Serial.print(pwmCmd[0]); Serial.print("/");
    Serial.print(pwmCmd[1]); Serial.print("/");
    Serial.print(pwmCmd[2]); Serial.print("/");
    Serial.println(pwmCmd[3]);
  }
}
