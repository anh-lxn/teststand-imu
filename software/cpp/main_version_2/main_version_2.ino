#include <AccelStepper.h> // Bibliothek für Stepper Motoren
#include <Wire.h>

// Sonstige Variablen
bool motor_enabled = false;
int transitionSpeed = 5000; // Standard: 5000
float dt = 1; // Zeit pro Phase
int stillstands = 10; // stillstands_gesamt = stillstand *
float degreeZ = 60.0;
int rotation_count = 0;
float beschleunigung = 20000.0;
bool accel_calibration_done = false;
bool gyro_calibration_done = false;

// Zustände der Messung
enum Phase {
  IDLE, // Start
  TO_POS_X, MEAS_POS_X, MEASURE_MOVE_FROM_POS_X_TO_NEG_X,
  TO_NEG_X, MEAS_NEG_X, MEASURE_MOVE_FROM_NEG_X_TO_POS_X,
  TO_POS_Y, MEAS_POS_Y, MEASURE_MOVE_FROM_POS_Y_TO_NEG_Y,
  TO_NEG_Y, MEAS_NEG_Y, MEASURE_MOVE_FROM_NEG_Y_TO_POS_Y,
  TO_POS_Z, MEAS_POS_Z, MEASURE_MOVE_FROM_POS_Z_TO_NEG_Z,
  TO_NEG_Z, MEAS_NEG_Z, MEASURE_MOVE_FROM_NEG_Z_TO_POS_Z,
  HOMING_X, HOMING_Y, HOMING_Z, HOMING, DONE // Ende
};
Phase phase = IDLE; // Enumeration phase

/// --- Deklarationen KINEMATIK --- ///
// Pins
#define DIR_PIN_1 2 // Richtung Motor 1
#define DIR_PIN_2 3 // Richtung Motor 2
#define STEP_PIN 4 // Schritte
#define ENABLE_PIN 9

// Microstepping Pins
#define M0_PIN 5
#define M1_PIN 6
#define M2_PIN 7

#define START_IN_PIN 11   // empfängt Signal von IMU (D3)
#define TRIGGER_OUT_PIN 10   // sendet Signal an IMU (D2)

// Übersetzungsverhältnis
const float zaehne_Zahnriemen = 15.0f;
const float zaehne_Ritzel = 59.0f;
const float uebersetzung = (zaehne_Ritzel/zaehne_Zahnriemen)/2.0;

// Schritte pro Umdrehung (360°)
const float vollSchritteproUmdrehung = 200;
// Microstepping Konfiguration (siehe DOCS)
int microstepping = 16; // 16 Schritte pro Vollschritt (1/16 von 1,8°) kann 1,2,4,8,16,32 sein
// Mikroschritte für eine volle Umdrehung 
float mikroSchritteProUmdrehung = vollSchritteproUmdrehung * microstepping;

/// beide Motoren mit einem Objekt erstellen, weil der STEP_PIN bei beiden Motoren
/// und Treibern der gleiche ist (beide an Eingang D4 vom Arduino Nano angeschlossen)
// Objekt der Unterklasse Driver, welche nur STEP und DIR ansteuert
// Objekt erkennt STEP_PIN als Eingang für beide STEP der beiden Treiber
// Objekt erkennt Dummy-Pin 8 (nicht belegt und freilassen!!) als DIR der beiden Treiber
AccelStepper stepper_1_2(AccelStepper::DRIVER, STEP_PIN, 8);

// wie lange nach Reset NICHT fahren (Treiber auslassen)
const unsigned long startDelayMs = 5000; // 5000ms
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

// rotation=0 ... keine Umdrehung
// rotation=0.25 ... viertel Umdrehung
// rotation=0.5 ... halbe Umdrehung
void drehe(float rotation, bool dir1High, bool dir2High) {
  digitalWrite(DIR_PIN_1, dir1High ? HIGH : LOW);
  digitalWrite(DIR_PIN_2, dir2High ? HIGH : LOW);
  float mikroSchritteInsgesamt = berechneMikroschritteProDrehung(rotation);
  digitalWrite(ENABLE_PIN, LOW);
  stepper_1_2.move(mikroSchritteInsgesamt);
  while (stepper_1_2.distanceToGo() != 0) {stepper_1_2.run();}
  digitalWrite(ENABLE_PIN, HIGH);
}

void fahre_stillstandspositionen_ab(int stillstands, bool dir1High, bool dir2High) {
  float deg = 360.0 / stillstands;
  float rotation = deg / 360.0; 
  float winkelgeschwindigkeit = deg/dt;
  stepper_1_2.setMaxSpeed(berechneMikroschritteProSekunde(winkelgeschwindigkeit));
  

  for (int i=0; i < stillstands; i++) {
    t0 = millis();
    delay(dt*1000); // warte genau dt sekunden an der Stillstandsposition
    Serial.print("B: "); Serial.println(millis() - t0);

    
    t0 = millis();
    drehe(rotation, dir1High, dir2High);
    Serial.print("A: "); Serial.println(millis() - t0);
  }

}

void transition(float rotation, bool dir1High, bool dir2High) {
  stepper_1_2.setMaxSpeed(transitionSpeed);
  drehe(rotation, dir1High, dir2High);
}

void transitionZ(bool dir1High, bool dir2High) {
  stepper_1_2.setMaxSpeed(transitionSpeed);
  float rotation = (degreeZ/360.0)*2; //*2 weil übersetzung
  drehe(rotation, dir1High, dir2High);
  rotation_count++;
}
void starte_kinematik_setup(){
  /// STEP Pins 
  pinMode(STEP_PIN, OUTPUT);
  digitalWrite(STEP_PIN, LOW);     // (neu) sicher LOW halten

  /// Richtungs Pins
  // Deklarations der Arduino Pins D2 & D3 als Ausgangssignalpins
  pinMode(DIR_PIN_1, OUTPUT);
  pinMode(DIR_PIN_2, OUTPUT);
  // Einstellung der Richtung der Motoren
  //digitalWrite(DIR_PIN_1, HIGH);
  //digitalWrite(DIR_PIN_2, LOW);

  /// Mode Pins für Einstellung der Microstepping Schritte (siehe DOCS)
  // Deklarations der Arduino Pins D5 bis D7 als Ausgangssignalpins
  pinMode(M0_PIN, OUTPUT);
  pinMode(M1_PIN, OUTPUT);
  pinMode(M2_PIN, OUTPUT);
  // Einstellung des Microsteppings (siehe DOCS), z.B. M0=HIGH, M1=HIGH, M2=LOW -> 1/16 Step
  // -> StandardSTEP=1,8° -> MicroSTEP_1/16=0,1125°
  // WICHTIG: Variable microstepping anpassen!
  digitalWrite(M0_PIN, HIGH);
  digitalWrite(M1_PIN, HIGH);
  digitalWrite(M2_PIN, LOW);

  /// Enable Pins, Treiber deaktivieren
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, HIGH);  // Treiber AUS bis gestartet wird

  /// Pulse Width (breite des gesendeten Pulses vom Arduiono an die Treiber) -> Dieser muss mind. laut Hersteller 2 Mikrosekunden sein, sonst erkennt der Motor den Schritt eventuell nicht an
  stepper_1_2.setMinPulseWidth(3); // Pulsbreite in µs

  /// Maximale Geschwindigkeit und Beschleunigung
  stepper_1_2.setAcceleration(beschleunigung);
  stepper_1_2.setCurrentPosition(0); // Move to position 0 as the starting point

  t0 = millis(); // Startzeit speichern, millis() -> Fkt. für aktuelle Laufzeit des Arduinos
}


void sendTriggerToIMU(bool active) {
  pinMode(TRIGGER_OUT_PIN, OUTPUT);
  digitalWrite(TRIGGER_OUT_PIN, active ? LOW : HIGH);
}

void setup() 
  {
    Serial.begin(115200);
    starte_kinematik_setup();
    pinMode(START_IN_PIN, INPUT_PULLUP);
    pinMode(TRIGGER_OUT_PIN, INPUT); // hochohmig (noch nicht aktiv)
    Serial.println("Motor bereit.");
  }

bool startLatched = false;

void loop() 
  { 
    /*
    int state = digitalRead(START_IN_PIN);
    Serial.print("START_IN_PIN = ");
    Serial.println(state);
    */
    if (digitalRead(START_IN_PIN) == LOW) {
      Serial.println("IMU → Motor: START empfangen.");
      delay(10);
      // Startverzögerung von startDelaysMS: Treiber bleibt AUS, es passiert NICHTS
      if (!motor_enabled)
        {
          if (millis() - t0 >= startDelayMs)
            {
              digitalWrite(ENABLE_PIN, LOW); // Treiber aktivieren
              motor_enabled = true;
            }
          else {return;} // durchlaufe Schleife solange, bis Treiber aktiviert sind
        }

      if (rotation_count % 2 == 0) {
        Serial.print("Rotation Start: "); Serial.println(rotation_count);
        sendTriggerToIMU(true);
        fahre_stillstandspositionen_ab(10, HIGH, LOW);
        sendTriggerToIMU(false);
        transitionZ(HIGH, HIGH);
      }
      else {
        Serial.print("Rotation Start: "); Serial.println(rotation_count);
        sendTriggerToIMU(true);
        fahre_stillstandspositionen_ab(10, LOW, HIGH);
        sendTriggerToIMU(false);
        transitionZ(HIGH, HIGH);
      }
      
      if(rotation_count == 6) {
        sendTriggerToIMU(false);
        transition(2.0, LOW, LOW);
        while (true) {}
      }
           
    }
    

  }
