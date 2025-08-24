#include <AccelStepper.h>

// Pins
#define STEP_PIN   4
#define DIR_PIN_1  2   // Richtung Motor 1
#define DIR_PIN_2  3   // Richtung Motor 2

// Microstepping-Pins (A4988-Beispiel)
#define M0_PIN 5
#define M1_PIN 6
#define M2_PIN 7

unsigned long startTime;
int state = 0;  // 0=vorwärts, 1=pause, 2=rückwärts, 3=pause, 4=rückwärts, 5=pause

const float stepsPerRevolution = 200;
int microstepSetting = 16;   // passt zu M0=H, M1=H, M2=L (A4988: 1/16)

// Eine gemeinsame AccelStepper-Instanz (DIR-Dummy: Pin 8, NICHT anschließen)
AccelStepper stepperShared(AccelStepper::DRIVER, STEP_PIN, 8);

void setup() {
  Serial.begin(9600);   // Serieller Monitor starten
  Serial.println("Setup fertig, starte Ablauf...");

  startTime = millis();   // Zeitpunkt merken

  pinMode(M0_PIN, OUTPUT);
  pinMode(M1_PIN, OUTPUT);
  pinMode(M2_PIN, OUTPUT);

  // 1/16 Microstepping (prüfe Tabelle deines Treibers)
  digitalWrite(M0_PIN, HIGH);
  digitalWrite(M1_PIN, HIGH);
  digitalWrite(M2_PIN, LOW);

  pinMode(DIR_PIN_1, OUTPUT);
  pinMode(DIR_PIN_2, OUTPUT);


  stepperShared.setMinPulseWidth(2); // µs, ggf. 3–5 testen

  float desiredRPM = 10;
  float MaxRPM = 40;

  float speedStepsPerSec = (microstepSetting * stepsPerRevolution * desiredRPM) / 60.0;
  float Max_SPS          = (microstepSetting * stepsPerRevolution * MaxRPM)    / 60.0;

  stepperShared.setMaxSpeed(Max_SPS);
  stepperShared.setSpeed(speedStepsPerSec);
}

void loop() {
  unsigned long now = millis();

  switch (state) {
    case 0: // Vorwärts 5s
      digitalWrite(DIR_PIN_1, LOW);
      digitalWrite(DIR_PIN_2, HIGH);
      stepperShared.runSpeed();
      if (now - startTime >= 5000) {
        Serial.println("Wechsel zu Pause 1 (nach Vorwärts)");
        state = 1;
        startTime = now;
      }
      break;

    case 1: // Pause 1s
      if (now - startTime >= 1000) {
        Serial.println("Wechsel zu Rueckwaerts Phase 1");
        state = 2;
        startTime = now;
      }
      break;

    case 2: // Rückwärts 5s (nur Motor 2 low)
      digitalWrite(DIR_PIN_2, LOW);
      stepperShared.runSpeed();
      if (now - startTime >= 5000) {
        Serial.println("Wechsel zu Pause 2 (nach Rueckwaerts Phase 1)");
        state = 3;
        startTime = now;
      }
      break;

    case 3: // Pause 1s
      if (now - startTime >= 1000) {
        Serial.println("Wechsel zu Rueckwaerts Phase 2 (beide)");
        state = 4;
        startTime = now;
      }
      break;
    
    case 4: // Rückwärts 5s (beide low)
      digitalWrite(DIR_PIN_1, HIGH);
      digitalWrite(DIR_PIN_2, LOW);
      stepperShared.runSpeed();
      if (now - startTime >= 5000) {
        Serial.println("Wechsel zu Pause 3 (nach Rueckwaerts Phase 2)");
        state = 5;
        startTime = now;
      }
      break;

    case 5: // Pause 1s
      if (now - startTime >= 1000) {
        Serial.println("Zurueck zu Vorwaerts Phase");
        state = 0;
        startTime = now;
      }
      break;
  }
}
