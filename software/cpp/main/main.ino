#include <AccelStepper.h> // Bibliothek für Stepper Motoren
#include <Wire.h>

/// --- Deklarationen IMU --- ///
// IMU Sensor
// grün A4
// gelb A5
// rot 5V
// braun GND
// ---- MPU-Grundsetup ----
// Ziel: Den MPU-6050 "richtig einstellen", bevor wir messen.
// - DLPF ~20 Hz  -> digitaler Tiefpass, glättet das Signal
// - Sample-Rate  -> wie oft pro Sekunde ein Wert erzeugt wird
// - Gyro ±250 dps -> größtmögliche Auflösung für Winkelgeschwindigkeit
// - PLL-Clock    -> stabile Taktquelle im Chip, damit Timing sauber ist
const uint16_t FS_HZ_DEFAULT = 100; // Standard: 100 Messungen pro Sekunde (nur 1 Gyro-Achse in FIFO)
                                     // Warum 100 Hz? Bei einer Achse sind das 2 Bytes/Sample.
                                     // Die FIFO ist 1024 Bytes groß -> max ca. 512 Samples ~ 5 s @ 100 Hz.
const uint8_t MPU = 0x68; // Adresse des MPU
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

// ---- I2C-Helfer ----
// ===== Funktion: schreibe in ein Register der MPU =====
void writeReg(uint8_t reg, uint8_t val){
  Wire.beginTransmission(MPU); // Verbindung zum Sensor "MPU" beginnen
  Wire.write(reg);             // zuerst die Adresse des Registers senden
  Wire.write(val);             // dann den Wert, den wir dort hineinschreiben wollen
  Wire.endTransmission(true);  // Übertragung beenden (und abschicken)
}

// ===== Funktion: lese ein einzelnes Byte (=8 Bit) aus einem Register =====
uint8_t readReg(uint8_t reg){
  Wire.beginTransmission(MPU); // Verbindung zum Sensor starten
  Wire.write(reg);             // sagen: "ich möchte dieses Register lesen"
  Wire.endTransmission(false); // Verbindung offen lassen, nur Adresse geschickt
  Wire.requestFrom((uint8_t)MPU,(uint8_t)1,(uint8_t)true); // jetzt 1 Byte anfordern
  return Wire.read();          // empfangenes Byte zurückgeben
}

// ===== Funktion: lese 2 Bytes (16 Bit) aus zwei aufeinanderfolgenden Registern =====
// (bei der MPU liegen viele Sensorwerte so gespeichert: HIGH-Byte + LOW-Byte)
// MPU 6050 schickt seine Werte in 8-Bit-Paketen(HIGH-Byte und LOW-Byte)
uint16_t read16(uint8_t regHi){
  Wire.beginTransmission(MPU); // Verbindung starten
  Wire.write(regHi);           // Adresse des HIGH-Bytes angeben
  Wire.endTransmission(false); // nur Adresse geschickt, Verbindung offen lassen
  Wire.requestFrom((uint8_t)MPU,(uint8_t)2,(uint8_t)true); // jetzt 2 Bytes anfordern
  uint8_t hi=Wire.read();      // erstes Byte = HIGH
  uint8_t lo=Wire.read();      // zweites Byte = LOW
  // HIGH und LOW zu einem 16-Bit-Wert zusammenfügen:
  return ((uint16_t)hi<<8)|lo; // HIGH nach links schieben
  // -> BSP.: lo=0x34, hi=0x12=18dez -> hi<<8=0x1200=18*256dez -> hi<<8|lo=0x1234 
}

// ---- FIFO: Start für eine bestimmte Gyro-Achse ('X','Y','Z') ----
// Warum FIFO?
// Normalerweise würde man bei jeder Messung sofort über I²C
// den Wert vom Sensor abholen. Das Problem: I²C und Serial
// brauchen Zeit -> währenddessen stockt Motor und verliert seine konstante Winkelgeschwindigkeit.
// Die Werte die ankommen sind völlig verfälscht und streuen stark.
// Mit der FIFO ("First In, First Out") funktioniert es so:
// - Der MPU hat intern einen Speicher-Eimer (1024 Byte groß).
// - Jede neue Messung wird automatisch in diesen Eimer gelegt.
// - Während der Motor fährt, muss der Arduino NICHT ständig
//   nachfragen -> der Eimer füllt sich im Hintergrund gleichmäßig.
// - Wenn die Fahrt fertig ist, schaut der Arduino in den Eimer,
//   liest ALLE Werte in einem Rutsch aus und schickt sie per Serial.
// Ergebnis: konstante Motorbewegung + gleichmäßig getaktete Messwerte.
void mpu_fifo_start_axis(char axis) {
  // Zuerst bestimmen wir, welche Achse genommen werden soll:
  // wenn 'X', dann 0x40, wenn 'Y', dann 0x20, sonst 'Z' -> 0x10
  uint8_t mask; // Variablenhalter für gewählte Achse 
  if (axis == 'X') {mask = 0x40;} // X
  else if (axis == 'Y') {mask = 0x20;} // Y
  else {mask = 0x10;} // Z

  // FIFO RESET, FIFO ENABLE, FIFO WRITE -> Adressen sind MPU 6050 spezifisch!
  writeReg(0x6A, 0x04);  // Befehl an den Sensor: "Eimer leeren" (FIFO reset)
  writeReg(0x6A, 0x40);  // Befehl: "Eimer einschalten" (FIFO-Modul aktivieren)
  writeReg(0x23, mask);  // Befehl: "Nur die gewählte Achse soll in den Eimer schreiben"
}

// Nachfüllen stoppen (Count bleibt erhalten), kurz warten
void mpu_fifo_feed_off() {
  writeReg(0x23, 0x00);  // Befehl: "Keine neuen Daten mehr in den Eimer legen"
  //delay(20);             // Warte 20 Millisekunden, damit die letzten Messwerte noch ankommen
}

// FIFO ganz aus (nach dem Auslesen)
void mpu_fifo_disable() {
  writeReg(0x6A, 0x00);  // Befehl: "Eimer-Modul ausschalten" (FIFO ganz aus)
}

// Anzahl der Bytes im FIFO (wie viele Daten liegen im Eimer?)
uint16_t mpu_fifo_count(){ 
  return read16(0x72);   // Register 0x72/0x73: FIFO_COUNT_H/L -> gibt Anzahl Bytes zurück
}

// Nächstes Datenpaket (2 Byte = ein Messwert) aus dem FIFO holen
int16_t  mpu_fifo_pop2(){
  Wire.beginTransmission(MPU); 
  Wire.write(0x74);          // Adresse des Registers FIFO_R_W (0x74): nächster Wert im Eimer
  Wire.endTransmission(false);

  Wire.requestFrom((uint8_t)MPU,(uint8_t)2,(uint8_t)true); // Fordere 2 Bytes vom Sensor an
  // Zusammenbauen: erstes Byte = HIGH, zweites = LOW -> ein 16-Bit Wert
  return (int16_t)((Wire.read()<<8)|Wire.read());
}

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


// Sonstige Variablen
bool motor_enabled = false;
int transitionSpeed = 5000;
float winkelgeschwindigkeit_imu = 40.0; // ab 15.0 ist konstant
float beschleunigung = 20000.0;
bool accel_calibration_done = false;
bool gyro_calibration_done = false;
int messungen = 300;

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
const unsigned long startDelayMs = 3000; // 5000ms
unsigned long t0;

/// --- FUNKTIONEN IMU --- ///
// Liest die 3 Beschleunigungs-Rohwerte (X, Y, Z) vom MPU-Sensor aus
// und speichert sie als 16-Bit-Signed-Integer (int16_t).
// -> Ausgabe ist in Rohwerten, noch NICHT in "g".
void readAccelRaw(const char* id) {
  // Treiber kurz aktivieren, damit Motoren nicht fiepen
  digitalWrite(ENABLE_PIN, HIGH);

  // HEADER
  Serial.println("id,i,ax,ay,az,static_gx,static_gy,static_gz");

  int16_t ax, ay, az, gx, gy, gz, tmp;
  for (int i = 1; i <= messungen; i++) {
    // 1) I²C-Übertragung starten zum Sensor mit Adresse MPU
    Wire.beginTransmission(MPU);   
    Wire.write(0x3B);              
    Wire.endTransmission(false);   // repeated start

    // 2) 14 Bytes vom Sensor anfordern: Accel(6) + Temp(2) + Gyro(6)
    Wire.requestFrom((uint8_t)MPU, (uint8_t)12, (uint8_t)true);

    // 3) Bytes zusammensetzen
    ax  = (Wire.read() << 8) | Wire.read();
    ay  = (Wire.read() << 8) | Wire.read();
    az  = (Wire.read() << 8) | Wire.read();
    tmp = 0; // Temperatur unnötig
    gx  = (Wire.read() << 8) | Wire.read();
    gy  = (Wire.read() << 8) | Wire.read();
    gz  = (Wire.read() << 8) | Wire.read();

    // 4) Messung als CSV ausgeben
    Serial.print(id); Serial.print(",");
    Serial.print(i); Serial.print(",");
    Serial.print(ax); Serial.print(",");
    Serial.print(ay); Serial.print(",");
    Serial.print(az); Serial.print(",");
    Serial.print(gx); Serial.print(",");
    Serial.print(gy); Serial.print(",");
    Serial.println(gz);
 
    //delay(40);   // kleine Abtastrate
  }

  // Treiber wieder deaktivieren
  digitalWrite(ENABLE_PIN, LOW);
}

/// ====== CSV-Dump (eine Achse) ======
/// Liest alle gespeicherten Messwerte (für genau eine Gyro-Achse)
/// aus der FIFO des MPU-6050 aus und gibt sie im CSV-Format über
/// die serielle Schnittstelle aus.
void dump_fifo_csv_1axis(const char* id, char axis) {
  uint16_t bytes   = mpu_fifo_count(); // Wie viele Bytes liegen im FIFO-Puffer?
  uint16_t samples = bytes / 2;       // 2 Bytes pro Sample (eine Gyro-Achse)
  if (samples > 512) samples = 512;   // Sicherheit: FIFO kann max. 1024 Bytes halten -> also höchstens 512 Samples

  Serial.println("id,i,axis,gyro_raw,gyro_dps,w"); // Header CSV

  // Jetzt alle Samples der Reihe nach auslesen
  for (uint16_t i=1; i<=samples; ++i){
    int16_t g  = mpu_fifo_pop2(); // Nächstes Sample (16-Bit Rohwert) aus FIFO holen
    float dps  = g / 131.0f;          // Umrechnung: Rohwert in °/s; Skala ±250 dps => 131 LSB pro °/s
    Serial.print(id);   Serial.print(",");
    Serial.print(i);  Serial.print(",");
    Serial.print(axis); Serial.print(",");
    Serial.print(g);    Serial.print(","); // Rohwert einer Gyro Achse (raw)
    Serial.print(dps,4);Serial.print(","); // umgewandelter Wert (in °/s)
    Serial.println((uint8_t) winkelgeschwindigkeit_imu);  // tatsächliche Winkelgeschwindigkeit
  }
}

// liest die ersten 5 Werte, aber sendet sie nicht weiter
// dient zur Erwärmung des MPU, damit die ersten Messungen nicht fehlerhaft sind
void warmupMPU() {
  for (int i = 0; i < 5; i++) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU, (uint8_t)14, (uint8_t)true);
    while (Wire.available()) Wire.read(); // alles verwerfen
    delay(10);
  }
}

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
  delay(100);
}
    
void drehe_lese_gyro(float rotation, bool dir1High, bool dir2High, const char* id, char axis) {
  // Insgesamte Anzahl an Mikroschritten
  long mikroSchritteInsgesamt = (long)berechneMikroschritteProDrehung(rotation);

  // Zielgeschwindigkeit (in Schritten/s) und Beschleunigung (Schritte/s^2)
  float v_steps = berechneMikroschritteProSekunde(winkelgeschwindigkeit_imu);
  float a_steps = beschleunigung; // AccelStepper erwartet Schritte/s^2

  // Weg bzw. Mikrosteps in Abhängigkeit der Mikrosteps/s und Mikrosteps/s² (s=v²/2a)
  long s = (long)floor((v_steps*v_steps) / (2.0*a_steps)); 

  // Plateau-Grenzen (nur innerhalb der Grenzen wird gemessen). Sicherheitsabstand von ein paar Steps, um Messung bei Beschleunigung zu vernachlässigen
  const long safety = 10;
  long start_plateau = s + safety;
  long end_plateau = mikroSchritteInsgesamt - s - safety;

  // Falls die Drehung zu kurz ist: Plateau gar nicht vorhanden -> alles loggen (Fallback)
  bool has_plateau = (end_plateau > start_plateau + 20);

  // Motoren vorbereiten
  digitalWrite(DIR_PIN_1, dir1High ? HIGH : LOW);
  digitalWrite(DIR_PIN_2, dir2High ? HIGH : LOW);
  stepper_1_2.setCurrentPosition(0); // dient zum zählen der Schritte
  stepper_1_2.move(mikroSchritteInsgesamt);
  digitalWrite(ENABLE_PIN, LOW);

  // Variablen für FIFO und Befüllung des FIFO
  bool fifo_started = false;
  bool feed_stopped = false;

  // Motoren drehen so lange bis Ziel erreicht ist, nebenbei FIFO laufen lassen
  while (stepper_1_2.distanceToGo() != 0) {
    stepper_1_2.run();
    long steps = stepper_1_2.currentPosition();

    if (has_plateau) {
      // FIFO erst starten, wenn wir innerhalb der Plateau Grenzen sind
      if (!fifo_started && steps >= start_plateau) {
        mpu_fifo_start_axis(axis); // FIFO für gewählte Achse starten
        fifo_started = true;
      }
      // FEED stoppen kurz bevor wieder gebremst wird (am Ende der Rotation)
      if (fifo_started && steps >= end_plateau) {
        mpu_fifo_feed_off();
        feed_stopped = true;
      }
    }
    else {
      // // Fallback: Bewegung ist so kurz, dass praktisch kein Plateau existiert
      if (!fifo_started) {
        mpu_fifo_start_axis(axis);
        fifo_started = true;
      }
    }
  }

  // Nach dem Loop: falls wir gestartet haben, aber das Feed noch nicht gestoppt ist
  if (fifo_started && !feed_stopped) {
    mpu_fifo_feed_off();
  }

  digitalWrite(ENABLE_PIN, HIGH); // Motoren aus, wegen fiepen

  // FIFO dumpen, FIFO aus
  dump_fifo_csv_1axis(id, axis);
  mpu_fifo_disable();

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
  Wire.setClock(400000); // I2C Fast Mode
  delay(100);
  
  // MPU konfigurieren (DLPF 20 Hz, Fs=100 Hz)
  mpu_setup(FS_HZ_DEFAULT);

  t0 = millis();

}

void run_calibration_accel() {
  stepper_1_2.setMaxSpeed(transitionSpeed);
  switch (phase) {

    case IDLE:
      Serial.println("STARTING ACCEL CALIBRATION ...");
      phase = TO_POS_X;
      break;

    // --- +X ---
    case TO_POS_X:
      drehe(0.25, LOW, HIGH);   
      phase = MEAS_POS_X;
      break;

    case MEAS_POS_X:
      readAccelRaw("axp");
      Serial.println("Positive X-Accel aufgenommen ✔");
      phase = TO_NEG_X;
      break;

    // --- -X ---
    case TO_NEG_X:
      drehe(0.25, HIGH, LOW); // homing
      drehe(0.25, HIGH, LOW);   
      phase = MEAS_NEG_X;
      break;

    case MEAS_NEG_X:
      readAccelRaw("axn");
      Serial.println("Negative X-Accel aufgenommen ✔");
      phase = TO_POS_Y;
      break;

    // --- +Y ---
    case TO_POS_Y:
      drehe(0.25, LOW, HIGH);  // homing
      drehe(0.5, HIGH, HIGH); drehe(0.25, HIGH, LOW);
      phase = MEAS_POS_Y;
      break;

    case MEAS_POS_Y:
      readAccelRaw("ayp");
      Serial.println("Positive Y-Accel aufgenommen ✔");
      phase = TO_NEG_Y;
      break;

    // --- -Y ---
    case TO_NEG_Y:
      drehe(0.25, LOW, HIGH);  // homing
      drehe(0.25, LOW, HIGH);  
      phase = MEAS_NEG_Y;
      break;

    case MEAS_NEG_Y:
      readAccelRaw("ayn");
      Serial.println("Negative Y-Accel aufgenommen ✔");
      phase = TO_POS_Z;
      break;

    // --- +Z ---
    case TO_POS_Z:
      drehe(0.25, HIGH, LOW); // homing 
      phase = MEAS_POS_Z;
      break;

    case MEAS_POS_Z:
      readAccelRaw("azp");
      Serial.println("Positive Z-Accel aufgenommen ✔");
      phase = TO_NEG_Z;
      break;

    // --- -Z ---
    case TO_NEG_Z:
      drehe(0.5, HIGH, LOW);   
      phase = MEAS_NEG_Z;
      break;

    case MEAS_NEG_Z:
      readAccelRaw("azn");
      Serial.println("Negative Z-Accel aufgenommen ✔");
      phase = HOMING;
      break;
    
    case HOMING:
      drehe(0.5, LOW, HIGH);
      drehe(0.5, LOW, LOW);
      phase = DONE;
      break;

    case DONE:
      Serial.println("ACCEL CALIBRATION DONE");
      Serial.println("");
      accel_calibration_done = true;
      break;
  }
}

void run_calibration_gyro() {
  switch (phase) {

    case IDLE:
      Serial.println("STARTING GYRO CALIBRATION ...");
      phase = TO_POS_X;
      break;

    // --- Y-ACHSE ---
    case TO_POS_X:
      stepper_1_2.setMaxSpeed(transitionSpeed);
      drehe(0.25, LOW, HIGH);   
      phase = MEASURE_MOVE_FROM_POS_X_TO_NEG_X;
      break;

    case MEASURE_MOVE_FROM_POS_X_TO_NEG_X:
      stepper_1_2.setMaxSpeed(berechneMikroschritteProSekunde(winkelgeschwindigkeit_imu));
      drehe_lese_gyro(0.5, HIGH, LOW, "gyp", 'Y');
      phase = MEASURE_MOVE_FROM_NEG_X_TO_POS_X;
      break;

    case MEASURE_MOVE_FROM_NEG_X_TO_POS_X:
      drehe_lese_gyro(0.5, LOW, HIGH, "gyn", 'Y');
      Serial.println("Y-Gyro aufgenommen ✔");
      phase = HOMING_X;
      break;

    case HOMING_X:
      stepper_1_2.setMaxSpeed(transitionSpeed);
      drehe(0.25, HIGH, LOW);
      phase = TO_POS_Y;
      break;

    // --- X-ACHSE ---
    case TO_POS_Y:
      drehe(0.5, HIGH, HIGH); drehe(0.25, HIGH, LOW);
      phase = MEASURE_MOVE_FROM_POS_Y_TO_NEG_Y;
      break;

    case MEASURE_MOVE_FROM_POS_Y_TO_NEG_Y:
      stepper_1_2.setMaxSpeed(berechneMikroschritteProSekunde(winkelgeschwindigkeit_imu));
      drehe_lese_gyro(0.5, LOW, HIGH, "gxn", 'X');
      phase = MEASURE_MOVE_FROM_NEG_Y_TO_POS_Y;
      break;

    case MEASURE_MOVE_FROM_NEG_Y_TO_POS_Y:
      drehe_lese_gyro(0.5, HIGH, LOW, "gxp", 'X');
      Serial.println("X-Gyro aufgenommen ✔");
      phase = HOMING_Y;
      break;

    case HOMING_Y:
      stepper_1_2.setMaxSpeed(transitionSpeed);
      drehe(0.25, LOW, HIGH);
      phase = TO_POS_Z;
      break;

    // --- Z-Achse ---
    case TO_POS_Z:
      drehe(0.5, LOW, LOW);
      phase = MEASURE_MOVE_FROM_POS_Z_TO_NEG_Z;
      break;

    case MEASURE_MOVE_FROM_POS_Z_TO_NEG_Z:
      stepper_1_2.setMaxSpeed(berechneMikroschritteProSekunde(winkelgeschwindigkeit_imu*2)); // *2 aufgrund der übersetzung!
      drehe_lese_gyro(1.0, HIGH, HIGH, "gzp", 'Z');
      phase = MEASURE_MOVE_FROM_NEG_Z_TO_POS_Z;
      break;

    // --- -Z ---
    case MEASURE_MOVE_FROM_NEG_Z_TO_POS_Z:
      drehe_lese_gyro(1.0, LOW, LOW, "gzn", 'Z');
      Serial.println("Z-Gyro aufgenommen ✔"); 
      phase = HOMING_Z;
      break;

    case HOMING_Z:
      phase = DONE;
      break;

    case DONE:
      Serial.println("GYRO CALIBRATION DONE");
      Serial.println("");
      gyro_calibration_done = true;
      break;
  }
}

void setup() 
  {
    starte_kinematik_setup();
    starte_imu_setup();
    warmupMPU();
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
    
    
    // --- Accelerometer-Kalibrierung ---
    if (!accel_calibration_done){
      run_calibration_accel();
      if (accel_calibration_done){
        phase=IDLE; // Reset für Gyro Kalibrierung
        digitalWrite(ENABLE_PIN, HIGH);
      } 
      return;
    } 
    
    
    // --- Gyroskop-Kalibrierung ---
    if (!gyro_calibration_done) {
      run_calibration_gyro();
      if (gyro_calibration_done){
        digitalWrite(ENABLE_PIN, HIGH);
      }
      return;
    }
    

    if (accel_calibration_done && accel_calibration_done) {Serial.println("CALIBRATION DONE");}
    
  }
