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
#define DIR_PIN_1 26 // Richtung Motor 1
#define DIR_PIN_2 23 // Richtung Motor 2
#define STEP_PIN 25 // Schritte
#define ENABLE_PIN 33

// Microstepping Pins
#define M0_PIN 14
#define M1_PIN 27
#define M2_PIN 32


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
AccelStepper stepper_1_2(AccelStepper::DRIVER, STEP_PIN, 13);

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
    //t0 = millis();
    delay(dt*1000); // warte genau dt sekunden an der Stillstandsposition
    //Serial.print("B: "); Serial.println(millis() - t0);

    
    //t0 = millis();
    drehe(rotation, dir1High, dir2High);
    //Serial.print("A: "); Serial.println(millis() - t0);
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

// MPU
const uint16_t FS_HZ_DEFAULT = 100;   // Abtastrate (Hz)
const uint8_t MPU = 0x68;             // I2C-Adresse

const float g0 = 9.81;
const float MPUscaleFactorAcc = 16384.0;                  // in LSB/g
const float MPUscaleFactorAccMS2 = g0 / MPUscaleFactorAcc; // (m/s²)/LSB
const float MPUscaleFactorGyro = 131.0;                   // in LSB/(°/s)
const float MPUscaleFactorGyroRADS = MPUscaleFactorGyro / (PI / 180.0);

void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission(true);
}

void mpu_setup(uint16_t rate_hz = FS_HZ_DEFAULT) {
  writeReg(0x6B, 0x01); // PWR_MGMT_1: PLL mit X-Gyro
  writeReg(0x6C, 0x00); // PWR_MGMT_2: alle Achsen aktiv
  writeReg(0x1B, 0x00); // GYRO_CONFIG: ±250°/s
  writeReg(0x1C, 0x00); // ACCEL_CONFIG: ±2g
  writeReg(0x1A, 0x04); // CONFIG: DLPF 20 Hz
  uint8_t div = (uint8_t)(1000 / rate_hz - 1);
  writeReg(0x19, div);  // SMPLRT_DIV
  writeReg(0x6A, 0x04); // USER_CTRL: FIFO Reset
}

void readDataIMU() {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Startadresse: ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 14, true);

  if (Wire.available() == 14) {
    int16_t ax = (Wire.read() << 8) | Wire.read();
    int16_t ay = (Wire.read() << 8) | Wire.read();
    int16_t az = (Wire.read() << 8) | Wire.read();
    Wire.read(); Wire.read(); // Temperatur überspringen
    int16_t wx = (Wire.read() << 8) | Wire.read();
    int16_t wy = (Wire.read() << 8) | Wire.read();
    int16_t wz = (Wire.read() << 8) | Wire.read();

    Serial.print((float)ax * MPUscaleFactorAccMS2, 5); Serial.print(",");
    Serial.print((float)ay * MPUscaleFactorAccMS2, 5); Serial.print(",");
    Serial.print((float)az * MPUscaleFactorAccMS2, 5); Serial.print(",");
    Serial.print((float)wx / MPUscaleFactorGyroRADS, 5); Serial.print(",");
    Serial.print((float)wy / MPUscaleFactorGyroRADS, 5); Serial.print(",");
    Serial.println((float)wz / MPUscaleFactorGyroRADS, 5);
  }
}



void setup() 
  {
    Serial.begin(115200);
    starte_kinematik_setup();
    Wire.begin(21, 22);  // SDA=21, SCL=22 (Standard beim ESP32)
    Wire.setClock(400000);
    mpu_setup(FS_HZ_DEFAULT);
    delay(100);
    Serial.println("Motor bereit.");
  }

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

    if (rotation_count % 2 == 0) {
      fahre_stillstandspositionen_ab(10, HIGH, LOW);
      transitionZ(HIGH, HIGH);
    }
    else {
      Serial.print("Rotation Start: "); Serial.println(rotation_count);
      fahre_stillstandspositionen_ab(10, LOW, HIGH);
      transitionZ(HIGH, HIGH);
    }
    
    if(rotation_count == 6) {
      transition(2.0, LOW, LOW);
      while (true) {}
    }
          


  }
