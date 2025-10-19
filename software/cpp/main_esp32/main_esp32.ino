#include <Arduino.h>
#include <AccelStepper.h>
#include <Wire.h>

// ===== Threads =====
TaskHandle_t TaskMotorHandle;   // Task-Handle-Variable
TaskHandle_t TaskIMUHandle;
// ===== Zustände & Variablen =====
bool motor_enabled = false;
volatile bool motor_done = false;
int transitionSpeed = 10000;
float dt = 0.05;
int stillstands = 10;
float degreeZ = 60.0;
int rotation_count = 0;
float beschleunigung = 20000.0;

// ===== Pins =====
#define DIR_PIN_1 26
#define DIR_PIN_2 23
#define STEP_PIN 25
#define ENABLE_PIN 33
#define M0_PIN 14
#define M1_PIN 27
#define M2_PIN 32

// ===== Kinematik =====
const float zaehne_Zahnriemen = 15.0f;
const float zaehne_Ritzel = 59.0f;
const float uebersetzung = (zaehne_Ritzel / zaehne_Zahnriemen) / 2.0;
const float vollSchritteproUmdrehung = 200;
int microstepping = 16;
float mikroSchritteProUmdrehung = vollSchritteproUmdrehung * microstepping;

AccelStepper stepper_1_2(AccelStepper::DRIVER, STEP_PIN, 13);
const unsigned long startDelayMs = 3000;
unsigned long t0;

// ===== Funktionen =====
float berechneMikroschritteProSekunde(float winkelgeschwindigkeit) {
  float winkelgeschwindigkeit_motor = winkelgeschwindigkeit * uebersetzung;
  float umdrehungen_pro_sekunde = winkelgeschwindigkeit_motor / 360;
  return umdrehungen_pro_sekunde * mikroSchritteProUmdrehung;
}

float berechneMikroschritteProDrehung(float rotation) {
  return rotation * mikroSchritteProUmdrehung * uebersetzung;
}

// ===== Setup Motorpins =====
void starte_kinematik_setup() {
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN_1, OUTPUT);
  pinMode(DIR_PIN_2, OUTPUT);
  pinMode(M0_PIN, OUTPUT);
  pinMode(M1_PIN, OUTPUT);
  pinMode(M2_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, HIGH); // deaktiviert
  digitalWrite(M0_PIN, HIGH);
  digitalWrite(M1_PIN, HIGH);
  digitalWrite(M2_PIN, LOW);

  stepper_1_2.setMinPulseWidth(5);
  stepper_1_2.setAcceleration(beschleunigung);
  stepper_1_2.setCurrentPosition(0);
  t0 = millis();
}

// ===== Bewegung in Stillstandspositionen =====
void fahre_stillstandspositionen_ab(int stillstands, bool dir1High, bool dir2High) {
  float deg = 360.0 / stillstands;
  float rotation = deg / 360.0;
  float winkelgeschwindigkeit = deg / dt;
  stepper_1_2.setMaxSpeed(berechneMikroschritteProSekunde(winkelgeschwindigkeit));

  for (int i = 0; i < stillstands; i++) {
    vTaskDelay(dt * 1000 / portTICK_PERIOD_MS); // Stillstandzeit

    digitalWrite(DIR_PIN_1, dir1High ? HIGH : LOW);
    digitalWrite(DIR_PIN_2, dir2High ? HIGH : LOW);
    float mikroSchritteInsgesamt = berechneMikroschritteProDrehung(rotation);
    long ziel = stepper_1_2.currentPosition() + mikroSchritteInsgesamt;
    stepper_1_2.moveTo(ziel);

    // Bewegung laufen lassen, ohne zu blockieren
    while (stepper_1_2.distanceToGo() != 0) {
      stepper_1_2.run();
    }
  }
}

void disableMotor() {
  digitalWrite(ENABLE_PIN, HIGH);
}

void enableMotor() {
  digitalWrite(ENABLE_PIN, LOW);
}


void transitionZ(bool dir1High, bool dir2High) {
  // Geschwindigkeit für Übergangsbewegung setzen
  stepper_1_2.setMaxSpeed(transitionSpeed);

  // Rotation berechnen: degreeZ (z. B. 60 °)
  float rotation = (degreeZ / 360.0) * 2.0;   // *2 wegen Übersetzung
  
  // Richtung setzen
  digitalWrite(DIR_PIN_1, dir1High ? HIGH : LOW);
  digitalWrite(DIR_PIN_2, dir2High ? HIGH : LOW);

  // Zielposition berechnen und Bewegung starten
  long ziel = stepper_1_2.currentPosition() + berechneMikroschritteProDrehung(rotation);
  stepper_1_2.moveTo(ziel);

  // Bewegung ausführen, nicht blockierend
  while (stepper_1_2.distanceToGo() != 0) {
    stepper_1_2.run();
  }
}

void transition(float rotation, bool dir1High, bool dir2High) {
  // Geschwindigkeit
  stepper_1_2.setMaxSpeed(transitionSpeed);

  // Richtung setzen
  digitalWrite(DIR_PIN_1, dir1High ? HIGH : LOW);
  digitalWrite(DIR_PIN_2, dir2High ? HIGH : LOW);

  // Ziel in Mikroschritten
  long ziel = stepper_1_2.currentPosition() + berechneMikroschritteProDrehung(rotation);
  stepper_1_2.moveTo(ziel);

  // Non-blocking Bewegung
  while (stepper_1_2.distanceToGo() != 0) {
    stepper_1_2.run();
  }
}


// ===== MPU =====
const uint8_t MPU = 0x68;
const float g0 = 9.81;
const float MPUscaleFactorAcc = 16384.0;
const float MPUscaleFactorAccMS2 = g0 / MPUscaleFactorAcc;
const float MPUscaleFactorGyro = 131.0;
const float MPUscaleFactorGyroRADS = MPUscaleFactorGyro / (PI / 180.0);

void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission(true);
}

void mpu_setup(uint16_t rate_hz = 100) {
  writeReg(0x6B, 0x01);
  writeReg(0x6C, 0x00);
  writeReg(0x1B, 0x00);
  writeReg(0x1C, 0x00);
  writeReg(0x1A, 0x04);
  uint8_t div = (uint8_t)(1000 / rate_hz - 1);
  writeReg(0x19, div);
  writeReg(0x6A, 0x04);
}

void readDataIMU() {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 14, true);
  if (Wire.available() == 14) {
    int16_t ax = (Wire.read() << 8) | Wire.read();
    int16_t ay = (Wire.read() << 8) | Wire.read();
    int16_t az = (Wire.read() << 8) | Wire.read();
    Wire.read(); Wire.read();
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

// ===== Task: IMU =====
void TaskReadIMU(void *pvParameters) {
  for (;;) {
    if (!motor_done) {
      readDataIMU();
      vTaskDelay(10 / portTICK_PERIOD_MS);
    } else {
      vTaskDelay(100 / portTICK_PERIOD_MS);
    }
  }
}

// ===== Task: Motor =====
void TaskMotor(void *pvParameters) {
  motor_done = false;
  for (;;) {
    if (!motor_enabled) {
      if (millis() - t0 >= startDelayMs) {
        digitalWrite(ENABLE_PIN, LOW);
        motor_enabled = true;
      } else {
        vTaskDelay(10 / portTICK_PERIOD_MS);
        continue;
      }
    }
    if (rotation_count % 2 == 0) {
      fahre_stillstandspositionen_ab(10, HIGH, LOW);
      transitionZ(HIGH, HIGH);   // Z-Übergang
      rotation_count++;
    } else {
      fahre_stillstandspositionen_ab(10, LOW, HIGH);
      transitionZ(HIGH, HIGH);
      rotation_count++;
    }

    if (rotation_count >= (int)(360.0 / degreeZ)) {
      motor_done = true;
      transition(2.0, LOW, LOW);
      disableMotor();
      Serial.println("Motor fertig!");
      vTaskDelete(NULL); // TaskMotor beenden
    }

    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

// ===== Setup =====
void setup() {
  Serial.begin(115200);
  starte_kinematik_setup();
  Wire.begin(21, 22);
  Wire.setClock(400000);
  mpu_setup();
  delay(100);
  Serial.println("ESP32 Dual-Core Motor+IMU gestartet.");

  xTaskCreatePinnedToCore(TaskReadIMU, "TaskIMU", 8192, NULL, 2, &TaskIMUHandle, 0);
  xTaskCreatePinnedToCore(TaskMotor, "TaskMotor", 8192, NULL, 2, &TaskMotorHandle, 1);
}

void loop() {}
