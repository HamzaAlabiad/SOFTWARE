// Encoder wheel calibration with on-robot Wi-Fi dashboard (ESP32).
// Display method: phone/laptop browser -> http://192.168.4.1
// No Serial Monitor interaction is required.

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>

struct EncoderPins {
  uint8_t a;
  uint8_t b;
  const char* name;
};

struct CalibrationResult {
  bool valid;
  int32_t lastDelta;
  float lastTurns;
  float lastCountsPerRev;
  float avgCountsPerRev;
  uint16_t samples;
};

struct CalibrationSession {
  bool active;
  uint8_t wheel;
  float turns;
  int32_t startCount;
  uint32_t startMs;
};

// Encoder map from your table
// ENC1 -> Back Left   -> GPIO39, GPIO36
// ENC2 -> Front Left  -> GPIO34, GPIO35
// ENC3 -> Front Right -> GPIO33, GPIO32
// ENC4 -> Back Right  -> GPIO25, GPIO26
EncoderPins encoders[4] = {
    {39, 36, "ENC1 BackLeft"},
    {34, 35, "ENC2 FrontLeft"},
    {33, 32, "ENC3 FrontRight"},
    {25, 26, "ENC4 BackRight"},
};

volatile int32_t encoderCount[4] = {0, 0, 0, 0};
volatile uint8_t encoderPrevState[4] = {0, 0, 0, 0};

CalibrationResult results[4] = {};
CalibrationSession session = {false, 0, 5.0f, 0, 0};

WebServer server(80);

const char* AP_SSID = "ESP32-ENC-CAL";
const char* AP_PASS = "enc12345";
const float FIXED_CAL_TURNS = 5.0f;

uint32_t lastSerialPrintMs = 0;

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

void resetAllCounts() {
  noInterrupts();
  for (uint8_t i = 0; i < 4; i++) {
    encoderCount[i] = 0;
  }
  interrupts();
}

void resetAllResults() {
  for (uint8_t i = 0; i < 4; i++) {
    results[i].valid = false;
    results[i].lastDelta = 0;
    results[i].lastTurns = 0.0f;
    results[i].lastCountsPerRev = 0.0f;
    results[i].avgCountsPerRev = 0.0f;
    results[i].samples = 0;
  }
}

String htmlPage() {
  return R"HTML(
<!doctype html>
<html>
<head>
<meta charset="utf-8" />
<meta name="viewport" content="width=device-width,initial-scale=1" />
<title>Encoder Calibration</title>
<style>
:root { --bg:#0f172a; --panel:#111827; --line:#374151; --text:#e5e7eb; --muted:#9ca3af; --ok:#16a34a; --warn:#f59e0b; }
body { margin:0; font-family:Segoe UI, Tahoma, sans-serif; background:linear-gradient(135deg,#0b1020,#0f172a); color:var(--text); }
.wrap { max-width:900px; margin:20px auto; padding:0 12px; }
.card { background:rgba(17,24,39,.95); border:1px solid #233049; border-radius:12px; padding:14px; margin-bottom:12px; }
h2 { margin:0 0 10px 0; font-size:20px; }
.row { display:flex; gap:10px; flex-wrap:wrap; align-items:center; }
input, select, button { background:#0b1220; color:var(--text); border:1px solid #334155; border-radius:8px; padding:8px 10px; }
button { cursor:pointer; }
button.primary { background:#0b5ed7; border-color:#0b5ed7; }
button.stop { background:#b91c1c; border-color:#b91c1c; }
button.alt { background:#1f2937; }
.grid { display:grid; grid-template-columns:repeat(4,minmax(140px,1fr)); gap:8px; }
.kpi { border:1px solid #334155; border-radius:10px; padding:8px; background:#0b1220; }
.small { color:var(--muted); font-size:12px; }
.value { font-size:18px; font-weight:700; }
.table { width:100%; border-collapse:collapse; }
.table th, .table td { border-bottom:1px solid #2d3748; padding:8px; text-align:left; font-size:14px; }
.status { font-weight:700; }
.status.active { color:var(--warn); }
.status.idle { color:var(--ok); }
.code { font-family:Consolas, monospace; }
</style>
</head>
<body>
<div class="wrap">
  <div class="card">
    <h2>Encoder Wheel Calibration</h2>
    <div class="small">Connect to Wi-Fi <span class="code">ESP32-ENC-CAL</span> (password <span class="code">enc12345</span>) then open <span class="code">192.168.4.1</span>.</div>
    <div class="small">Procedure: reset counts -> select wheel -> Start -> rotate wheel by hand exactly 5 turns -> Stop.</div>
  </div>

  <div class="card">
    <div class="row">
      <label>Wheel:</label>
      <select id="wheel">
        <option value="0">ENC1 BackLeft</option>
        <option value="1">ENC2 FrontLeft</option>
        <option value="2">ENC3 FrontRight</option>
        <option value="3">ENC4 BackRight</option>
      </select>
      <span class="small">Turns: fixed to 5</span>
      <button class="primary" onclick="startCal()">Start</button>
      <button class="stop" onclick="stopCal()">Stop</button>
      <button class="alt" onclick="resetCounts()">Reset Counts</button>
      <button class="alt" onclick="resetResults()">Reset Results</button>
    </div>
    <div class="small" id="sessionStatus">Session: idle</div>
  </div>

  <div class="card">
    <h2>Live Encoder Counts</h2>
    <div class="grid">
      <div class="kpi"><div class="small">ENC1 BackLeft</div><div class="value" id="c0">0</div></div>
      <div class="kpi"><div class="small">ENC2 FrontLeft</div><div class="value" id="c1">0</div></div>
      <div class="kpi"><div class="small">ENC3 FrontRight</div><div class="value" id="c2">0</div></div>
      <div class="kpi"><div class="small">ENC4 BackRight</div><div class="value" id="c3">0</div></div>
    </div>
  </div>

  <div class="card">
    <h2>Calibration Results (Counts Per Wheel Revolution)</h2>
    <table class="table">
      <thead>
        <tr><th>Wheel</th><th>Last Delta</th><th>Turns</th><th>Last CPR</th><th>Average CPR</th><th>Samples</th></tr>
      </thead>
      <tbody>
        <tr><td>ENC1 BackLeft</td><td id="d0">-</td><td id="t0">-</td><td id="l0">-</td><td id="a0">-</td><td id="s0">0</td></tr>
        <tr><td>ENC2 FrontLeft</td><td id="d1">-</td><td id="t1">-</td><td id="l1">-</td><td id="a1">-</td><td id="s1">0</td></tr>
        <tr><td>ENC3 FrontRight</td><td id="d2">-</td><td id="t2">-</td><td id="l2">-</td><td id="a2">-</td><td id="s2">0</td></tr>
        <tr><td>ENC4 BackRight</td><td id="d3">-</td><td id="t3">-</td><td id="l3">-</td><td id="a3">-</td><td id="s3">0</td></tr>
      </tbody>
    </table>
  </div>
</div>

<script>
async function call(path) {
  const r = await fetch(path);
  if (!r.ok) {
    alert('Request failed: ' + path);
    return null;
  }
  return r.text();
}

async function startCal() {
  const wheel = document.getElementById('wheel').value;
  await call('/cmd/start?wheel=' + wheel);
  await refreshData();
}

async function stopCal() {
  await call('/cmd/stop');
  await refreshData();
}

async function resetCounts() {
  await call('/cmd/resetCounts');
  await refreshData();
}

async function resetResults() {
  await call('/cmd/resetResults');
  await refreshData();
}

function fmt(num, digits) {
  return Number(num).toFixed(digits);
}

async function refreshData() {
  const r = await fetch('/data');
  if (!r.ok) return;
  const data = await r.json();

  for (let i = 0; i < 4; i++) {
    document.getElementById('c' + i).textContent = data.counts[i];

    const res = data.results[i];
    if (res.valid) {
      document.getElementById('d' + i).textContent = res.lastDelta;
      document.getElementById('t' + i).textContent = fmt(res.lastTurns, 2);
      document.getElementById('l' + i).textContent = fmt(res.lastCountsPerRev, 2);
      document.getElementById('a' + i).textContent = fmt(res.avgCountsPerRev, 2);
    } else {
      document.getElementById('d' + i).textContent = '-';
      document.getElementById('t' + i).textContent = '-';
      document.getElementById('l' + i).textContent = '-';
      document.getElementById('a' + i).textContent = '-';
    }
    document.getElementById('s' + i).textContent = res.samples;
  }

  const ss = document.getElementById('sessionStatus');
  if (data.session.active) {
    ss.innerHTML = 'Session: <span class="status active">ACTIVE</span> | Wheel=' + data.session.wheel + ' | Turns=' + fmt(data.session.turns, 2) + ' | Elapsed ms=' + data.session.elapsedMs;
  } else {
    ss.innerHTML = 'Session: <span class="status idle">IDLE</span>';
  }
}

setInterval(refreshData, 500);
refreshData();
</script>
</body>
</html>
)HTML";
}

String jsonData() {
  int32_t counts[4];
  noInterrupts();
  for (uint8_t i = 0; i < 4; i++) {
    counts[i] = encoderCount[i];
  }
  interrupts();

  String s = "{";
  s += "\"counts\":[" + String(counts[0]) + "," + String(counts[1]) + "," + String(counts[2]) + "," + String(counts[3]) + "],";

  s += "\"session\":{";
  s += "\"active\":" + String(session.active ? "true" : "false") + ",";
  s += "\"wheel\":" + String(session.wheel) + ",";
  s += "\"turns\":" + String(session.turns, 3) + ",";
  s += "\"elapsedMs\":" + String(session.active ? (millis() - session.startMs) : 0);
  s += "},";

  s += "\"results\":[";
  for (uint8_t i = 0; i < 4; i++) {
    s += "{";
    s += "\"valid\":" + String(results[i].valid ? "true" : "false") + ",";
    s += "\"lastDelta\":" + String(results[i].lastDelta) + ",";
    s += "\"lastTurns\":" + String(results[i].lastTurns, 3) + ",";
    s += "\"lastCountsPerRev\":" + String(results[i].lastCountsPerRev, 3) + ",";
    s += "\"avgCountsPerRev\":" + String(results[i].avgCountsPerRev, 3) + ",";
    s += "\"samples\":" + String(results[i].samples);
    s += "}";
    if (i < 3) s += ",";
  }
  s += "]";

  s += "}";
  return s;
}

void handleRoot() {
  server.send(200, "text/html", htmlPage());
}

void handleData() {
  server.send(200, "application/json", jsonData());
}

void handleStart() {
  if (session.active) {
    server.send(409, "text/plain", "Session already active");
    return;
  }

  if (!server.hasArg("wheel")) {
    server.send(400, "text/plain", "Missing wheel");
    return;
  }

  int wheel = server.arg("wheel").toInt();

  if (wheel < 0 || wheel > 3) {
    server.send(400, "text/plain", "Wheel must be 0..3");
    return;
  }

  session.active = true;
  session.wheel = (uint8_t)wheel;
  session.turns = FIXED_CAL_TURNS;
  session.startCount = readEncoderCount(session.wheel);
  session.startMs = millis();

  server.send(200, "text/plain", "Calibration started");
}

void handleStop() {
  if (!session.active) {
    server.send(409, "text/plain", "No active session");
    return;
  }

  int32_t endCount = readEncoderCount(session.wheel);
  int32_t delta = endCount - session.startCount;
  float countsPerRev = fabsf((float)delta) / session.turns;

  CalibrationResult& r = results[session.wheel];
  r.valid = true;
  r.lastDelta = delta;
  r.lastTurns = session.turns;
  r.lastCountsPerRev = countsPerRev;
  r.samples += 1;

  if (r.samples == 1) {
    r.avgCountsPerRev = countsPerRev;
  } else {
    r.avgCountsPerRev = ((r.avgCountsPerRev * (r.samples - 1)) + countsPerRev) / (float)r.samples;
  }

  session.active = false;

  server.send(200, "text/plain", "Calibration stopped and saved");
}

void handleResetCounts() {
  resetAllCounts();
  session.active = false;
  server.send(200, "text/plain", "Encoder counts reset");
}

void handleResetResults() {
  resetAllResults();
  session.active = false;
  server.send(200, "text/plain", "Calibration results reset");
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

  resetAllResults();
  resetAllCounts();

  WiFi.mode(WIFI_AP);
  bool apOk = WiFi.softAP(AP_SSID, AP_PASS);

  server.on("/", HTTP_GET, handleRoot);
  server.on("/data", HTTP_GET, handleData);
  server.on("/cmd/start", HTTP_GET, handleStart);
  server.on("/cmd/stop", HTTP_GET, handleStop);
  server.on("/cmd/resetCounts", HTTP_GET, handleResetCounts);
  server.on("/cmd/resetResults", HTTP_GET, handleResetResults);
  server.begin();

  Serial.println();
  Serial.println("Encoder calibration server started.");
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

  if (millis() - lastSerialPrintMs > 1000) {
    lastSerialPrintMs = millis();
    Serial.print("Counts BL/FL/FR/BR: ");
    Serial.print(readEncoderCount(0));
    Serial.print(" / ");
    Serial.print(readEncoderCount(1));
    Serial.print(" / ");
    Serial.print(readEncoderCount(2));
    Serial.print(" / ");
    Serial.println(readEncoderCount(3));
  }
}
