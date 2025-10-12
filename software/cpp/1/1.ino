#include <AccelStepper.h> 
#include <Wire.h>

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

const uint16_t FS_HZ_DEFAULT = 100;
const uint8_t MPU = 0x68; 

const float g0 = 9.81;
const float MPUscaleFactorAcc = 16384.0; // in LSB/g
const float MPUscaleFactorAccMS2 = g0/MPUscaleFactorAcc; // in (m/s²)/LSB
const float MPUscaleFactorGyro = 131.0; // in LSB/(°/s)

// Sonstige Variablen
bool motor_enabled = false;
int transitionSpeed = 5000; // Standard: 5000
float winkelgeschwindigkeit_imu = 30.0; // ab 15.0 ist konstant, Standard: 40.0
float beschleunigung = 20000.0;
bool accel_calibration_done = false;
bool gyro_calibration_done = false;
int messungen = 100;
const unsigned long startDelayMs = 3000; // 5000ms
unsigned long t0;

// Übersetzungsverhältnis
const float zaehne_Zahnriemen = 15.0f;
const float zaehne_Ritzel = 59.0f;
const float uebersetzung = (zaehne_Ritzel/zaehne_Zahnriemen)/2.0;

const float vollSchritteproUmdrehung = 200;
int microstepping = 16; 
float mikroSchritteProUmdrehung = vollSchritteproUmdrehung * microstepping;

AccelStepper stepper_1_2(AccelStepper::DRIVER, STEP_PIN, 8);



struct IMUSample {
  int16_t ax, ay, az;
  int16_t wx, wy, wz;
};

IMUSample buffer[100];
int8_t buffer_counter = 0;

void writeReg(uint8_t reg, uint8_t val){
  Wire.beginTransmission(MPU); // Verbindung zum Sensor "MPU" beginnen
  Wire.write(reg);             // zuerst die Adresse des Registers senden
  Wire.write(val);             // dann den Wert, den wir dort hineinschreiben wollen
  Wire.endTransmission(true);  // Übertragung beenden (und abschicken)
}
void mpu_setup(uint16_t rate_hz = FS_HZ_DEFAULT){
  // PWR_MGMT_1 (0x6B): Energie-/Takteinstellungen
  // 0x01 bedeutet: "Nimm die PLL mit dem X-Gyro als Takt" -> sehr stabiles Timing.
  // (0x00 wäre interner Oszillator; geht auch, ist aber wackliger.)
  writeReg(0x6B, 0x01);
  // PWR_MGMT_2 (0x6C): Einzelne Sensorachsen ein-/ausschalten.
  // 0x00 = nichts abschalten -> alle Gyro- und Accel-Achsen EIN.
  writeReg(0x6C, 0x00);
  // GYRO_CONFIG (0x1B): Messbereich des Gyros einstellen.
  // 0x00 = ±250 °/s -> höchste Auflösung.
  // Merken: Umrechnung roh -> °/s = raw / 131.0
  writeReg(0x1B, 0x00);
  // ACCEL_CONFIG (0x1C): Messbereich des Beschleunigungssensors.
  // 0x00 = ±2 g -> höchste Auflösung beim Accel.
  writeReg(0x1C, 0x00);
  // CONFIG (0x1A): DLPF-Einstellung (Digital Low-Pass Filter).
  // 0x04 -> ca. 20 Hz Bandbreite: filtert hochfrequentes Zittern/Vibrationen,
  // sorgt für ruhigeres Signal und weniger Aliasing.
  writeReg(0x1A, 0x04);
  // SMPLRT_DIV (0x19): Teiler für die Ausgaberate.
  // WICHTIG: Wenn DLPF > 0, ist der interne Grundtakt 1000 Hz.
  // Also gilt: Ausgabe-Frequenz Fs = 1000 / (DIV + 1)
  // Wir berechnen den passenden Teiler "div" für die gewünschte rate_hz.
  uint8_t div = (uint8_t)(1000 / rate_hz - 1);
  writeReg(0x19, div); // Beispiel: rate_hz=100 -> div=9 -> 1000/(9+1)=100 Hz

  // USER_CTRL (0x6A): FIFO reset.
  // "Eimer leeren", damit keine alten Reste drin sind, bevor wir starten.
  writeReg(0x6A, 0x04);
}

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
Phase phase = IDLE; 

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

void drehe(float rotation, bool dir1High, bool dir2High) {
  digitalWrite(DIR_PIN_1, dir1High ? HIGH : LOW);
  digitalWrite(DIR_PIN_2, dir2High ? HIGH : LOW);

  long mikroSchritteInsgesamt = (long)berechneMikroschritteProDrehung(rotation);
  digitalWrite(ENABLE_PIN, LOW);
  stepper_1_2.move(mikroSchritteInsgesamt);

  long lastStep = stepper_1_2.currentPosition();
  unsigned long lastIMUread = micros();  // hochauflösender Timer
  const unsigned long imuInterval = 9000; // µs ≈ 9 ms → ≈110 Hz

  while (stepper_1_2.distanceToGo() != 0) {
    stepper_1_2.run();

    long pos = stepper_1_2.currentPosition();
    if (pos != lastStep) {
      lastStep = pos;
      // Schritt erfolgt -> prüfen, ob Zeitfenster offen
      unsigned long now = micros();
      if (now - lastIMUread >= imuInterval) {
        // Nur lesen, wenn wirklich Zeit genug vergangen ist
        if (buffer_counter < 100) readDataIMU();
        lastIMUread = now;
      }
    }
  }

  digitalWrite(ENABLE_PIN, HIGH);
  delay(100);
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
  float mikroschritte_pro_sekunde = berechneMikroschritteProSekunde(winkelgeschwindigkeit_imu);
  stepper_1_2.setMaxSpeed(mikroschritte_pro_sekunde);
  stepper_1_2.setAcceleration(beschleunigung);
  stepper_1_2.setCurrentPosition(0); // Move to position 0 as the starting point

  t0 = millis(); // Startzeit speichern, millis() -> Fkt. für aktuelle Laufzeit des Arduinos
}

void starte_imu_setup(){

  // IO
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000); // I2C Fast Mode
  delay(100);
  
  // MPU konfigurieren (DLPF 20 Hz, Fs=100 Hz)
  mpu_setup(FS_HZ_DEFAULT);

  t0 = millis();
}

void readDataIMU() {
  if (buffer_counter >= 100) return;

  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);  // repeated start
  Wire.requestFrom(MPU, (uint8_t)14, (uint8_t)true);

  if (Wire.available() >= 14) {
    IMUSample &s = buffer[buffer_counter];
    s.ax = (Wire.read() << 8) | Wire.read();
    s.ay = (Wire.read() << 8) | Wire.read();
    s.az = (Wire.read() << 8) | Wire.read();
    Wire.read(); Wire.read(); // Temperatur überspringen
    s.wx = (Wire.read() << 8) | Wire.read();
    s.wy = (Wire.read() << 8) | Wire.read();
    s.wz = (Wire.read() << 8) | Wire.read();
    buffer_counter++;
  }
}

void dump_data() {
  for(uint8_t i = 0; i < buffer_counter; i++) {
    Serial.print(buffer[i].ax); Serial.print(",");
    Serial.print(buffer[i].ay); Serial.print(",");
    Serial.print(buffer[i].az); Serial.print(",");
    Serial.print(buffer[i].wx); Serial.print(",");
    Serial.print(buffer[i].wy); Serial.print(",");
    Serial.println(buffer[i].wz);
  }
  buffer_counter = 0;
}

void setup() {
    starte_kinematik_setup();
    starte_imu_setup();
    //warmupMPU();
}

void loop() {
  
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
  for(int i=0; i < 5; i++) {
    drehe(0.1, HIGH, LOW);
    dump_data();
  }
  for(int i=0; i < 5; i++) {
    drehe(0.1, LOW, HIGH);
    dump_data();
  }
  

  }

