#include <AccelStepper.h> // Bibliothek für Stepper Motoren

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

// Übersetzungsverhältnis
const float d_Zahnriemen = 15.0f;
const float d_Ritzel = 59.0f;
const float uebersetzung = d_Ritzel/d_Zahnriemen;

// Schritte pro Umdrehung (360°)
const float stepsPerRevolution = 200;
// Microstepping Konfiguration (siehe DOCS)
int microstepping = 16; // 16 Schritte pro Vollschritt (1/16 von 1,8°) kann 1,2,4,8,16,32 sein
// Mikroschritte für eine volle Umdrehung 
float microStepsPerRevolution = stepsPerRevolution * microstepping;

/// beide Motoren mit einem Objekt erstellen, weil der STEP_PIN bei beiden Motoren
/// und Treibern der gleiche ist (beide an Eingang D4 vom Arduino Nano angeschlossen)
// Objekt der Unterklasse Driver, welche nur STEP und DIR ansteuert
// Objekt erkennt STEP_PIN als Eingang für beide STEP der beiden Treiber
// Objekt erkennt Dummy-Pin 8 (nicht belegt und freilassen!!) als DIR der beiden Treiber
AccelStepper stepper_1_2(AccelStepper::DRIVER, STEP_PIN, 8);

// wie lange nach Reset NICHT fahren (Treiber auslassen)
const unsigned long startDelayMs = 3000; // 5000ms
unsigned long t0;

/// --- FUNKTIONEN KINEMATIK --- ///
// Function to calculate steps based on desired rotations
float convert_rotational_position_to_steps(float rotations) {
  return rotations * microStepsPerRevolution * uebersetzung/2; // rotations=0,..,1 ; 
}

void starte_kinematik_setup(){
  // für jegliche Prints im Terminal
  Serial.begin(9600);
  Serial.println("Anfang");

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
  stepper_1_2.setMinPulseWidth(5); // Pulsbreite in µs

  /// Maximale Geschwindigkeit und Beschleunigung
  float maxRPM = 160;
  float Max_Speed_StepsPerSec = microstepping * stepsPerRevolution * maxRPM / 60; // Specify max speed in steps/sec (converted from RPM)
  stepper_1_2.setMaxSpeed(Max_Speed_StepsPerSec);

  float AccelRPMperSec = 300; // Set acceleration in rpm/sec
  float Accel_StepsPerSec2 = microstepping * stepsPerRevolution * AccelRPMperSec / 60;
  stepper_1_2.setAcceleration(Accel_StepsPerSec2);
  
  // Move to position 0 as the starting point
  stepper_1_2.setCurrentPosition(0);

  t0 = millis(); // Startzeit speichern, millis() -> Fkt. für aktuelle Laufzeit des Arduinos
}
void setup() 
  {
    starte_kinematik_setup();
  }

bool motor_enabled = false;
bool motionDone = false;

void loop() 
  {
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
    
    // Testbewegung für den IMU
    static int phase = 0;
    if (motionDone) return;

    switch (phase) {
      case 0:
        digitalWrite(DIR_PIN_1, HIGH); // Richtung Motor 1
        digitalWrite(DIR_PIN_2, HIGH); // Richtung Motor 2
        stepper_1_2.move(convert_rotational_position_to_steps(0.5)); // Drehung um 90°
        while (stepper_1_2.distanceToGo() != 0) {stepper_1_2.run();} // wenn distanceToGo = 0, dann Ziel erreicht
        delay(100);
        phase = 1;
        break;
      
      case 1:
        digitalWrite(DIR_PIN_1, LOW); // Richtung Motor 1
        digitalWrite(DIR_PIN_2, HIGH); // Richtung Motor 2
        stepper_1_2.move(convert_rotational_position_to_steps(0.25)); // Drehung um 90°
        while (stepper_1_2.distanceToGo() != 0) {stepper_1_2.run();} // wenn distanceToGo = 0, dann Ziel erreicht
        delay(100);
        phase = 2;
        break;
      
      case 2:
        digitalWrite(DIR_PIN_1, LOW); // Richtung Motor 1
        digitalWrite(DIR_PIN_2, LOW); // Richtung Motor 2
        stepper_1_2.move(convert_rotational_position_to_steps(0.5)); // Drehung um 90°
        while (stepper_1_2.distanceToGo() != 0) {stepper_1_2.run();} // wenn distanceToGo = 0, dann Ziel erreicht
        delay(100);
        phase = 3;
        break;

      case 3:
        digitalWrite(DIR_PIN_1, HIGH); // Richtung Motor 1
        digitalWrite(DIR_PIN_2, LOW); // Richtung Motor 2
        stepper_1_2.move(convert_rotational_position_to_steps(0.25)); // Drehung um 90°
        while (stepper_1_2.distanceToGo() != 0) {stepper_1_2.run();} // wenn distanceToGo = 0, dann Ziel erreicht
        delay(100);
        phase = 4;
        break;
      /*
      case 4:
        digitalWrite(DIR_PIN_1, LOW); // Richtung Motor 1
        digitalWrite(DIR_PIN_2, HIGH); // Richtung Motor 2
        stepper_1_2.move(convert_rotational_position_to_steps(0.25)); // Drehung um 90°
        while (stepper_1_2.distanceToGo() != 0) {stepper_1_2.run();} // wenn distanceToGo = 0, dann Ziel erreicht
        delay(100);
        motionDone = true;
        break;
      */
    }
  }
