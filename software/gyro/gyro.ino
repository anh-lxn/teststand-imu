#include <Wire.h>

/// ====== Pins ======
#define DIR_PIN_1   2
#define DIR_PIN_2   3
#define STEP_PIN    4
#define ENABLE_PIN  9
#define M0_PIN      5
#define M1_PIN      6
#define M2_PIN      7

/// ====== Mechanik / Motor ======
/// Übersetzung: (Ritzel/Zahnriemen)/2  -> anpassen falls nötig
const float zaehne_Zahnriemen = 15.0f;
const float zaehne_Ritzel     = 59.0f;
const float uebersetzung      = (zaehne_Ritzel / zaehne_Zahnriemen) / 2.0f;

/// Schritte
const float vollSchritteproUmdrehung = 200.0f;   // Vollschritte pro 360°
const int   microstepping             = 16;      // 1/16
const float mikroSchritteProUmdrehung = vollSchritteproUmdrehung * microstepping;

/// ====== IMU / MPU-6050 ======
const uint8_t MPU = 0x68;   // I2C-Adresse

// ---- I2C-Helfer ----
void writeReg(uint8_t reg, uint8_t val){
  Wire.beginTransmission(MPU); Wire.write(reg); Wire.write(val); Wire.endTransmission(true);
}
uint8_t readReg(uint8_t reg){
  Wire.beginTransmission(MPU); Wire.write(reg); Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU,(uint8_t)1,(uint8_t)true); return Wire.read();
}
uint16_t read16(uint8_t regHi){
  Wire.beginTransmission(MPU); Wire.write(regHi); Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU,(uint8_t)2,(uint8_t)true);
  uint8_t hi=Wire.read(), lo=Wire.read();
  return ((uint16_t)hi<<8)|lo;
}

// ---- FIFO: Start für eine bestimmte Gyro-Achse ('X','Y','Z') ----
// X -> 0x40, Y -> 0x20, Z -> 0x10
void mpu_fifo_start_axis(char axis) {
  uint8_t mask = (axis=='X') ? 0x40 : (axis=='Y') ? 0x20 : 0x10;

  writeReg(0x6A, 0x04);  // USER_CTRL: FIFO reset (Eimer leeren)
  writeReg(0x6A, 0x40);  // USER_CTRL: FIFO-Modul AN (Eimer aktiv)
  writeReg(0x23, mask);  // FIFO_EN: nur gewählte Gyro-Achse in FIFO
}

// Nachfüllen stoppen (Count bleibt erhalten), kurz warten
void mpu_fifo_feed_off() {
  writeReg(0x23, 0x00);  // nichts Neues mehr in FIFO
  delay(20);             // > 1 Sampleperiode bei 100 Hz
}

// FIFO ganz aus (nach dem Auslesen)
void mpu_fifo_disable() {
  writeReg(0x6A, 0x00);  // USER_CTRL: FIFO AUS
}

uint16_t mpu_fifo_count(){ return read16(0x72); } // FIFO_COUNTH/L
int16_t  mpu_fifo_pop2(){
  Wire.beginTransmission(MPU); Wire.write(0x74);   // FIFO_R_W
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU,(uint8_t)2,(uint8_t)true);
  return (int16_t)((Wire.read()<<8)|Wire.read());
}

// ---- MPU-Grundsetup ----
// DLPF ~20 Hz, Sample-Rate FS_HZ, Gyro ±250 dps (131 LSB/°/s), PLL-Clock
const uint16_t FS_HZ_DEFAULT = 100; // 1 Achse -> 2 B/Sample -> max ~512 Samples ~ 5s @100 Hz
void mpu_setup(uint16_t rate_hz = FS_HZ_DEFAULT){
  writeReg(0x6B, 0x01);  // PWR_MGMT_1: PLL mit X-Gyro (stabile Clock)
  writeReg(0x6C, 0x00);  // PWR_MGMT_2: alle Sensorachsen EIN
  writeReg(0x1B, 0x00);  // GYRO_CONFIG: ±250 °/s
  writeReg(0x1C, 0x00);  // ACCEL_CONFIG: ±2 g
  writeReg(0x1A, 0x04);  // CONFIG: DLPF_CFG=4 -> ~20 Hz
  uint8_t div = (uint8_t)(1000 / rate_hz - 1); // bei DLPF>0 ist Basistakt 1 kHz
  writeReg(0x19, div);   // SMPLRT_DIV: Fs = 1000/(DIV+1)
  writeReg(0x6A, 0x04);  // FIFO reset
}

/// ====== Rechnen (Schritte <-> Winkelgeschwindigkeit) ======
float microsteps_per_second(float omega_dps){
  float omega_motor = omega_dps * uebersetzung;     // dps am Motor
  float rev_per_s   = omega_motor / 360.0f;
  return rev_per_s * mikroSchritteProUmdrehung;
}
unsigned long period_us_for_omega(float omega_dps){
  float f = microsteps_per_second(omega_dps);
  if (f < 1) f = 1;
  return (unsigned long)(1000000.0f / f);
}
long microsteps_for_rotation(float rotations){
  return lroundf(rotations * mikroSchritteProUmdrehung * uebersetzung);
}

/// ====== Parameter für die Fahrt ======
float omega_set_dps = 40.0f;     // gewünschte Winkelgeschwindigkeit am Prüfling
const int STEP_HIGH_US = 3;      // STEP-Pulsbreite (DRV8825 min ~2 µs)
const float rotations_per_run = 0.5f; // pro Run 0.5 U

/// ====== CSV-Dump (eine Achse) ======
void dump_fifo_csv_1axis(const char* id, char axis) {
  uint16_t bytes   = mpu_fifo_count();
  uint16_t samples = bytes / 2;       // 2 Bytes pro Sample (eine Gyro-Achse)
  if (samples > 512) samples = 512;   // Schutz (FIFO max 1024 B)

  Serial.println("id,i,axis,gyro_raw,gyro_dps,omega_set_dps");
  for (uint16_t i=0; i<samples; ++i){
    int16_t g  = mpu_fifo_pop2();
    float dps  = g / 131.0f;          // ±250 dps -> 131 LSB/(°/s)
    Serial.print(id);   Serial.print(',');
    Serial.print(i+1);  Serial.print(',');
    Serial.print(axis); Serial.print(',');
    Serial.print(g);    Serial.print(',');
    Serial.print(dps,4);Serial.print(',');
    Serial.println(omega_set_dps,4);
  }
}

/// ====== Eine konstante Fahrt + FIFO-Logging für gewählte Achse ======
void run_constant_omega(const char* id, char axis, float rotations, bool dir1High, bool dir2High){
  // Richtungen & Enable
  digitalWrite(DIR_PIN_1, dir1High ? HIGH : LOW);
  digitalWrite(DIR_PIN_2, dir2High ? HIGH : LOW);
  digitalWrite(ENABLE_PIN, LOW);   // DRV8825: LOW = EIN

  long steps_total   = microsteps_for_rotation(rotations);
  unsigned long T    = period_us_for_omega(omega_set_dps);

  // FIFO für gewählte Achse starten
  mpu_fifo_start_axis(axis);

  // Gleichmäßige Schrittfolge
  uint32_t t_next = micros();
  for (long i=0; i<steps_total; ++i){
    while ((int32_t)(micros() - t_next) < 0) { /* warten */ }
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(STEP_HIGH_US);
    digitalWrite(STEP_PIN, LOW);
    t_next += T;
  }

  // Feed stoppen, FIFO dumpen, FIFO aus
  mpu_fifo_feed_off();
  dump_fifo_csv_1axis(id, axis);
  mpu_fifo_disable();

  digitalWrite(ENABLE_PIN, HIGH);  // Treiber AUS
}

/// ====== Setup / Loop ======
const unsigned long startDelayMs = 2000;
unsigned long t0;
bool done = false;

// Kleine Struktur für die geplanten Läufe
struct RunSpec {
  const char* id;   // z.B. "Yp", "Yn", ...
  char axis;        // 'X','Y','Z' -> welche Gyro-Achse in FIFO
  float rot;        // +0.5 oder -0.5 U
  bool dir1H;       // DIR1-Pegel
  bool dir2H;       // DIR2-Pegel
};

// >>> HINWEIS ZU DEN RICHTUNGEN:
// Passe dir die DIR-Pegel so an, dass jeweilige mechanische Achse wirklich dreht.
// Für Demo nutzen wir dieselben Pegel wie bisher (eine DOF-Achse):
RunSpec runs[] = {
  {"Yp", 'Y', +rotations_per_run, HIGH, LOW}, // Y +
  {"Yn", 'Y', -rotations_per_run, LOW,  HIGH},// Y -
  {"Xp", 'X', +rotations_per_run, HIGH, LOW}, // X +  (wenn du mechanisch X drehst)
  {"Xn", 'X', -rotations_per_run, LOW,  HIGH},// X -
  {"Zp", 'Z', +rotations_per_run, HIGH, LOW}, // Z +
  {"Zn", 'Z', -rotations_per_run, LOW,  HIGH} // Z -
};

void setup(){
  // Pins
  pinMode(STEP_PIN, OUTPUT); digitalWrite(STEP_PIN, LOW);
  pinMode(DIR_PIN_1, OUTPUT); pinMode(DIR_PIN_2, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT); digitalWrite(ENABLE_PIN, HIGH); // DRV8825 AUS
  pinMode(M0_PIN, OUTPUT); pinMode(M1_PIN, OUTPUT); pinMode(M2_PIN, OUTPUT);
  // 1/16 Microstep (DRV8825: M0=H, M1=H, M2=L)
  digitalWrite(M0_PIN, HIGH); digitalWrite(M1_PIN, HIGH); digitalWrite(M2_PIN, LOW);

  // IO
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000); // I2C Fast Mode
  delay(100);

  // MPU konfigurieren (DLPF 20 Hz, Fs=100 Hz)
  mpu_setup(FS_HZ_DEFAULT);

  t0 = millis();
}

void loop(){
  if (done) return;
  if (millis() - t0 < startDelayMs) return;

  // Nacheinander Y+, Y-, X+, X-, Z+, Z- (du kannst hier Runs auskommentieren)
  for (size_t k=0; k<sizeof(runs)/sizeof(runs[0]); ++k){
    run_constant_omega(runs[k].id, runs[k].axis, runs[k].rot, runs[k].dir1H, runs[k].dir2H);
    delay(300); // kurze Pause zwischen den Blöcken
  }

  Serial.println("DONE");
  done = true;
}
