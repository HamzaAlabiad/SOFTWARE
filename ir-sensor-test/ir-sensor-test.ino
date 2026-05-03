// Simple 6 IR sensor test for ESP32.
// Open Serial Monitor at 115200 baud.

const uint8_t IR_COUNT = 6;

// Connect each IR sensor OUT/SIGNAL pin to these ESP32 GPIO pins.
// Connect all IR VCC pins to 3V3 and all GND pins to ESP32 GND.
const uint8_t irPins[IR_COUNT] = {
  32,  // IR1 OUT
  33,  // IR2 OUT
  25,  // IR3 OUT
  26,  // IR4 OUT
  27,  // IR5 OUT
  14   // IR6 OUT
};

void setup() {
  Serial.begin(115200);
  delay(500);

  for (uint8_t i = 0; i < IR_COUNT; i++) {
    pinMode(irPins[i], INPUT);
  }

  Serial.println("6 IR sensor test started");
  Serial.println("Format: IR1 IR2 IR3 IR4 IR5 IR6");
}

void loop() {
  for (uint8_t i = 0; i < IR_COUNT; i++) {
    int reading = digitalRead(irPins[i]);

    Serial.print("IR");
    Serial.print(i + 1);
    Serial.print("=");
    Serial.print(reading);

    if (i < IR_COUNT - 1) {
      Serial.print("  ");
    }
  }

  Serial.println();
  delay(200);
}
