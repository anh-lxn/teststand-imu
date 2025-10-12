#define START_OUT_PIN 3     // sendet an Motor-Nano D11
#define TRIGGER_IN_PIN 2    // empfängt von Motor-Nano D10

void setup() {
  Serial.begin(115200);
  pinMode(START_OUT_PIN, OUTPUT);
  pinMode(TRIGGER_IN_PIN, INPUT_PULLUP);
  digitalWrite(START_OUT_PIN, HIGH); // Leerlauf HIGH
  Serial.println("IMU bereit.");
  delay(1000);
}

void sendStart() {
  Serial.println("IMU → Motor: START senden...");
  digitalWrite(START_OUT_PIN, LOW);
  delay(100);              // kurzer Low-Puls
  digitalWrite(START_OUT_PIN, HIGH);
}

void loop() {
  // per Tastatur-Trigger aus serieller Konsole
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd == "START") {
      sendStart();
    }
  }

  // Antwort vom Motor prüfen
  if (digitalRead(TRIGGER_IN_PIN) == LOW) {
    Serial.println("Motor → IMU: TRIGGER empfangen!");
    delay(500); // Entprellen
  }
}
