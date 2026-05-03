// 3 LED pattern tester for ESP32.
// Uses the same pins as the stepper test pins.
// Open Serial Monitor at 115200 baud and set line ending to "Newline".

const int LED1_PIN = 26;
const int LED2_PIN = 25;
const int LED3_PIN = 27;

const int ledPins[3] = {LED1_PIN, LED2_PIN, LED3_PIN};

String inputLine = "";
int pattern = 1;
int speedMs = 150;
bool enabled = true;

void allOff() {
  for (int i = 0; i < 3; i++) {
    digitalWrite(ledPins[i], LOW);
  }
} 

void setLed(int index) {
  allOff();
  if (index >= 0 && index < 3) {
    digitalWrite(ledPins[index], HIGH);
  }
}

void printHelp() {
  Serial.println();
  Serial.println("Commands:");
  Serial.println("  p 1       chase forward");
  Serial.println("  p 2       bounce");
  Serial.println("  p 3       fill and clear");
  Serial.println("  p 4       sparkle");
  Serial.println("  s 150     speed in milliseconds");
  Serial.println("  on        enable pattern");
  Serial.println("  off       turn all LEDs off");
  Serial.println("  status    print current settings");
  Serial.println();
}

void printStatus() {
  Serial.print("Enabled=");
  Serial.print(enabled ? "YES" : "NO");
  Serial.print(" | Pattern=");
  Serial.print(pattern);
  Serial.print(" | Speed=");
  Serial.print(speedMs);
  Serial.println(" ms");
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
  } else if (cmd == "on") {
    enabled = true;
    Serial.println("Patterns enabled");
  } else if (cmd == "off") {
    enabled = false;
    allOff();
    Serial.println("LEDs off");
  } else if (cmd.startsWith("p ")) {
    pattern = constrain(cmd.substring(2).toInt(), 1, 4);
    allOff();
    Serial.println("Pattern updated");
  } else if (cmd.startsWith("s ")) {
    speedMs = constrain(cmd.substring(2).toInt(), 30, 2000);
    Serial.println("Speed updated");
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

void waitWithSerial(int ms) {
  uint32_t start = millis();
  while ((uint32_t)(millis() - start) < (uint32_t)ms) {
    readSerialCommands();
    delay(2);
  }
}

void patternChase() {
  for (int i = 0; i < 3 && enabled && pattern == 1; i++) {
    setLed(i);
    waitWithSerial(speedMs);
  }
}

void patternBounce() {
  const int order[4] = {0, 1, 2, 1};
  for (int i = 0; i < 4 && enabled && pattern == 2; i++) {
    setLed(order[i]);
    waitWithSerial(speedMs);
  }
}

void patternFillClear() {
  allOff();
  for (int i = 0; i < 3 && enabled && pattern == 3; i++) {
    digitalWrite(ledPins[i], HIGH);
    waitWithSerial(speedMs);
  }
  for (int i = 2; i >= 0 && enabled && pattern == 3; i--) {
    digitalWrite(ledPins[i], LOW);
    waitWithSerial(speedMs);
  }
}

void patternSparkle() {
  int a = random(0, 3);
  int b = random(0, 3);
  allOff();
  digitalWrite(ledPins[a], HIGH);
  digitalWrite(ledPins[b], HIGH);
  waitWithSerial(speedMs);
  allOff();
  waitWithSerial(max(30, speedMs / 2));
}

void setup() {
  Serial.begin(115200);
  delay(500);

  for (int i = 0; i < 3; i++) {
    pinMode(ledPins[i], OUTPUT);
  }
  allOff();
  randomSeed(esp_random());

  Serial.println("3 LED pattern tester started");
  Serial.println("Pins: LED1=GPIO26, LED2=GPIO25, LED3=GPIO27");
  printHelp();
  printStatus();
}

void loop() {
  readSerialCommands();

  if (!enabled) {
    allOff();
    delay(20);
    return;
  }

  if (pattern == 1) {
    patternChase();
  } else if (pattern == 2) {
    patternBounce();
  } else if (pattern == 3) {
    patternFillClear();
  } else if (pattern == 4) {
    patternSparkle();
  }
}
