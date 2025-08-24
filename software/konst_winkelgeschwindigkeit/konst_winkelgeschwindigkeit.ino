#include <AccelStepper.h>

// Sonstige Variablen
bool motor_enabled = false;
float winkelgeschwindigkeit_imu = 30.0;

/// --- Deklarationen KINEMATIK --- ///
// Pins
#define DIR_PIN_1 2
#define DIR_PIN_2 3
#define STEP_PIN 4
#define ENABLE_PIN 9

// Microstepping Pins
#define M0_PIN 5
#define M1_PIN 6
#define M2_PIN 7

// Übersetzungsverhältnis
const float d_Zahnriemen = 15.0f;
const float d_Ritzel = 59.0f;
const float uebersetzung = (d_Ritzel / d_Zahnriemen)/2;

// Schritte pro Umdrehung (360°)
const float vollSchritteproUmdrehung = 200;
// Microstepping Konfiguration
int microstepping = 16; 
float mikroSchritteProUmdrehung = vollSchritteproUmdrehung * microstepping;

// Stepper-Objekt
AccelStepper stepper_1_2(AccelStepper::DRIVER, STEP_PIN, 8);

// Startverzögerung
const unsigned long startDelayMs = 3000;
unsigned long t0;

/// --- FUNKTIONEN KINEMATIK --- ///

// gewünschte Winkelgeschwindigkeit [°/s] → Mikrosteps/s
float berechneMikroschritteProSekunde(float winkelgeschwindigkeit) {
  float winkelgeschwindigkeit_motor = winkelgeschwindigkeit * uebersetzung;
  float umdrehungen_pro_sekunde =  winkelgeschwindigkeit_motor/360;
  float mikroschritte_pro_sekunde = umdrehungen_pro_sekunde * mikroSchritteProUmdrehung;
  return (mikroschritte_pro_sekunde);
}

// gewünschter Winkel[°], um den der IMU drehen soll -> Mikrosteps
float berechneMikroschritteProDrehung(float rotation){
  return (rotation * mikroSchritteProUmdrehung * uebersetzung); // bei rotation = 0.5 macht IMU halbe Umdrehung
}

// --- ALTE FUNKTION: drehe mit maxSpeed/Acceleration ---
void drehe(float rotation, bool dir1High, bool dir2High) {
  digitalWrite(DIR_PIN_1, dir1High ? HIGH : LOW);
  digitalWrite(DIR_PIN_2, dir2High ? HIGH : LOW);

  float mikroSchritteInsgesamt = berechneMikroschritteProDrehung(rotation);
  stepper_1_2.move(mikroSchritteInsgesamt); 
  while (stepper_1_2.distanceToGo() != 0) {
    stepper_1_2.run();
  }

  delay(100);
}



// Setup für Kinematik
void starte_kinematik_setup() {
  Serial.begin(115200);

  pinMode(STEP_PIN, OUTPUT);
  digitalWrite(STEP_PIN, LOW);

  pinMode(DIR_PIN_1, OUTPUT);
  pinMode(DIR_PIN_2, OUTPUT);

  pinMode(M0_PIN, OUTPUT);
  pinMode(M1_PIN, OUTPUT);
  pinMode(M2_PIN, OUTPUT);
  digitalWrite(M0_PIN, HIGH);
  digitalWrite(M1_PIN, HIGH);
  digitalWrite(M2_PIN, LOW); // 1/16 Step

  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, HIGH);  // Treiber AUS

  stepper_1_2.setMinPulseWidth(5);

  /// Maximale Geschwindigkeit und Beschleunigung
  float mikroschritte_pro_sekunde = berechneMikroschritteProSekunde(winkelgeschwindigkeit_imu);
  stepper_1_2.setMaxSpeed(mikroschritte_pro_sekunde);
  stepper_1_2.setAcceleration(10000);
  stepper_1_2.setCurrentPosition(0);
  t0 = millis();
}

/// --- Arduino Standard --- ///
void setup() {
  starte_kinematik_setup();
}

void loop() {
  // Startverzögerung
  if (!motor_enabled) {
    if (millis() - t0 >= startDelayMs) {
      digitalWrite(ENABLE_PIN, LOW); // Treiber aktivieren
      motor_enabled = true;
    } else {
      return;
    }
  }

  // Zeit vor Drehung speichern
  unsigned long tstart = millis();

  // Beispiel: eine volle Umdrehung mit 90 °/s
  digitalWrite(ENABLE_PIN, LOW);
  drehe(0.5, HIGH, LOW);
  digitalWrite(ENABLE_PIN, HIGH);

}
