// Wheel speed PID tuning sketch for ESP32 mecanum base.
// Display and tuning are done via Wi-Fi web page (no serial interaction required).
// Connect to AP: ESP32-PID-TUNE / pid12345, then open http://192.168.4.1

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <math.h>

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
float measuredRpm[4] = {0, 0, 0, 0};
float targetRpm[4] = {0, 0, 0, 0};
int16_t pwmCmd[4] = {0, 0, 0, 0};

PIDState pid[4] = {
    {1.2f, 0.35f, 0.0f, 0.0f, 0.0f, 0.0f},
    {1.2f, 0.35f, 0.0f, 0.0f, 0.0f, 0.0f},
    {1.2f, 0.35f, 0.0f, 0.0f, 0.0f, 0.0f},
    {1.2f, 0.35f, 0.0f, 0.0f, 0.0f, 0.0f},
};

WebServer server(80);

const char* AP_SSID = "ESP32-PID-TUNE";
const char* AP_PASS = "pid12345";

const float COUNTS_PER_WHEEL_REV = 4346.8f;  // From your calibration data
const uint32_t CONTROL_INTERVAL_MS = 20;      // 50 Hz speed loop
const uint16_t PWM_FREQ = 20000;
const uint8_t PWM_RES_BITS = 8;
const int PWM_MAX = 255;

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

float pidTargetRpm = 20.0f;
uint8_t activeWheel = 0;
bool controllerEnabled = false;
uint8_t minPwm = 35;

uint32_t lastControlMs = 0;
uint32_t lastPrintMs = 0;

bool isInputOnlyPin(uint8_t pin) {
  return pin == 34 || pin == 35 || pin == 36 || pin == 39;
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

void updateTargets() {
  for (uint8_t i = 0; i < 4; i++) {
    targetRpm[i] = 0.0f;
  }
  if (controllerEnabled) {
    targetRpm[activeWheel] = pidTargetRpm;
  }
}

void runControlLoop(float dt) {
  const float rpmScale = (60.0f / COUNTS_PER_WHEEL_REV) / dt;

  for (uint8_t i = 0; i < 4; i++) {
    int32_t countNow = readEncoderCount(i);
    int32_t delta = countNow - lastEncoderCount[i];
    lastEncoderCount[i] = countNow;

    // Your tests showed clockwise rotation gives negative counts.
    // This sign flip makes clockwise-positive RPM easier to reason about.
    measuredRpm[i] = -((float)delta) * rpmScale;

    float err = targetRpm[i] - measuredRpm[i];

    if (fabsf(targetRpm[i]) < 0.01f) {
      resetPidState(i);
      setMotorPwmSigned(i, 0);
      continue;
    }

    pid[i].integral += err * dt;
    pid[i].integral = constrain(pid[i].integral, -300.0f, 300.0f);

    float deriv = (err - pid[i].prevError) / dt;
    pid[i].prevError = err;

    float u = pid[i].kp * err + pid[i].ki * pid[i].integral + pid[i].kd * deriv;
    pid[i].out = u;

    int cmd = (int)lroundf(u);
    cmd = constrain(cmd, -PWM_MAX, PWM_MAX);

    if (abs(cmd) > 0 && abs(cmd) < minPwm) {
      cmd = (cmd > 0) ? minPwm : -minPwm;
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
</style>
</head>
<body>
<div class="wrap">
  <div class="card">
    <h2>Wheel Speed PID Tuner</h2>
    <div class="small">Connect to Wi-Fi <span class="code">ESP32-PID-TUNE</span> (pass <span class="code">pid12345</span>) and open <span class="code">192.168.4.1</span>.</div>
    <div class="small">Tune one wheel at a time. Only selected wheel runs; others remain stopped.</div>
  </div>

  <div class="card">
    <div class="row">
      <label>Wheel</label>
      <select id="wheel">
        <option value="0">MOTA BackRight (ENC4)</option>
        <option value="1">MOTB FrontRight (ENC3)</option>
        <option value="2">MOTC BackLeft (ENC1)</option>
        <option value="3">MOTD FrontLeft (ENC2)</option>
      </select>

      <label>Target RPM</label><input id="target" type="number" value="20" step="1" />
      <label>Kp</label><input id="kp" type="number" value="1.2" step="0.05" />
      <label>Ki</label><input id="ki" type="number" value="0.35" step="0.05" />
      <label>Kd</label><input id="kd" type="number" value="0.00" step="0.01" />
      <label>MinPWM</label><input id="minpwm" type="number" value="35" step="1" />

      <button class="main" onclick="applyConfig()">Apply</button>
      <button class="start" onclick="startCtrl()">Start</button>
      <button class="stop" onclick="stopCtrl()">Stop</button>
      <button onclick="resetI()">Reset I</button>
      <button onclick="resetEnc()">Reset Encoders</button>
    </div>
    <div class="small" id="status">Status: -</div>
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
</div>

<script>
function f(x,d=2){return Number(x).toFixed(d)}
async function hit(url){const r=await fetch(url);if(!r.ok){alert('Request failed: '+url);return null;}return r.text();}
async function applyConfig(){
  const wheel=document.getElementById('wheel').value;
  const target=document.getElementById('target').value;
  const kp=document.getElementById('kp').value;
  const ki=document.getElementById('ki').value;
  const kd=document.getElementById('kd').value;
  const minpwm=document.getElementById('minpwm').value;
  await hit(`/cmd/config?wheel=${wheel}&target=${target}&kp=${kp}&ki=${ki}&kd=${kd}&minpwm=${minpwm}`);
  await refresh();
}
async function startCtrl(){await hit('/cmd/start'); await refresh();}
async function stopCtrl(){await hit('/cmd/stop'); await refresh();}
async function resetI(){await hit('/cmd/resetI'); await refresh();}
async function resetEnc(){await hit('/cmd/resetEnc'); await refresh();}

async function refresh(){
  const r=await fetch('/data');
  if(!r.ok) return;
  const d=await r.json();

  document.getElementById('status').textContent = `Status: ${d.enabled ? 'RUNNING' : 'STOPPED'} | Active wheel=${d.activeWheel} | Target=${f(d.pidTarget,1)} rpm | Kp=${f(d.kp,2)} Ki=${f(d.ki,2)} Kd=${f(d.kd,2)} MinPWM=${d.minPwm}`;

  for(let i=0;i<4;i++){
    document.getElementById('rpm'+i).textContent=f(d.measured[i],1);
    document.getElementById('tr'+i).textContent=f(d.target[i],1);
    document.getElementById('mr'+i).textContent=f(d.measured[i],1);
    document.getElementById('er'+i).textContent=f(d.target[i]-d.measured[i],1);
    document.getElementById('pw'+i).textContent=d.pwm[i];
  }
}
setInterval(refresh, 200);
refresh();
</script>
</body>
</html>
)HTML";
}

String jsonData() {
  String s = "{";
  s += "\"enabled\":" + String(controllerEnabled ? "true" : "false") + ",";
  s += "\"activeWheel\":" + String(activeWheel) + ",";
  s += "\"pidTarget\":" + String(pidTargetRpm, 3) + ",";
  s += "\"kp\":" + String(pid[activeWheel].kp, 4) + ",";
  s += "\"ki\":" + String(pid[activeWheel].ki, 4) + ",";
  s += "\"kd\":" + String(pid[activeWheel].kd, 4) + ",";
  s += "\"minPwm\":" + String(minPwm) + ",";

  s += "\"target\":[";
  for (uint8_t i = 0; i < 4; i++) {
    s += String(targetRpm[i], 3);
    if (i < 3) s += ",";
  }
  s += "],";

  s += "\"measured\":[";
  for (uint8_t i = 0; i < 4; i++) {
    s += String(measuredRpm[i], 3);
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

void handleConfig() {
  if (server.hasArg("wheel")) {
    int w = server.arg("wheel").toInt();
    if (w >= 0 && w < 4) {
      activeWheel = (uint8_t)w;
    }
  }

  if (server.hasArg("target")) {
    pidTargetRpm = server.arg("target").toFloat();
    pidTargetRpm = constrain(pidTargetRpm, -80.0f, 80.0f);
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
    minPwm = (uint8_t)constrain(v, 0, 180);
  }

  updateTargets();
  server.send(200, "text/plain", "Config applied");
}

void handleStart() {
  controllerEnabled = true;
  updateTargets();
  server.send(200, "text/plain", "Controller started");
}

void handleStop() {
  controllerEnabled = false;
  updateTargets();
  resetAllPidStates();
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
  server.on("/cmd/config", HTTP_GET, handleConfig);
  server.on("/cmd/start", HTTP_GET, handleStart);
  server.on("/cmd/stop", HTTP_GET, handleStop);
  server.on("/cmd/resetI", HTTP_GET, handleResetI);
  server.on("/cmd/resetEnc", HTTP_GET, handleResetEnc);
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
}

void loop() {
  server.handleClient();

  uint32_t now = millis();
  if (now - lastControlMs >= CONTROL_INTERVAL_MS) {
    float dt = (now - lastControlMs) / 1000.0f;
    lastControlMs = now;
    if (dt < 0.001f) dt = CONTROL_INTERVAL_MS / 1000.0f;

    updateTargets();
    runControlLoop(dt);
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
    Serial.println(pwmCmd[3]);
  }
}
