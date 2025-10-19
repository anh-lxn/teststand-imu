#include <Arduino.h>
#include <AccelStepper.h>
#include <Wire.h>

//
// ==========================================================
//                ESP32 Dual-Core Stepper + IMU
// ==========================================================
//  - Core 0: IMU-Lesen über I²C (MPU6050)
//  - Core 1: Motorsteuerung mit AccelStepper
//
//  Autor: Anh Le Xuan
//  Version: Final Cleaned & Documented
// ==========================================================
//

// ------------------- FreeRTOS Threads -------------------
TaskHandle_t MotorTaskHandle;
TaskHandle_t ImuTaskHandle;

// ------------------- Globale Zustände -------------------
bool motorEnabled = false;
volatile bool motorDone = false;
int transitionSpeed = 10000;      // Max. Drehgeschwindigkeit für Übergänge
float dwellTime = 0.05;           // Stillstandszeit an Messposition (s)
int numPositions = 10;            // Anzahl Messpositionen pro Umdrehung
float degreePerStep = 60.0;       // Rotationswinkel pro Z-Schritt (°)
int rotationCount = 0;            // Zählt Anzahl Übergänge
float motorAcceleration = 20000.0; // Beschleunigung (Steps/s²)
const unsigned long startupDelayMs = 3000; // Wartezeit nach Start
unsigned long startTime;

// ------------------- Motor-Pins -------------------
#define DIR_PIN_1 26
#define DIR_PIN_2 23
#define STEP_PIN 25
#define ENABLE_PIN 33
#define M0_PIN 14
#define M1_PIN 27
#define M2_PIN 32

// ------------------- Kinematik -------------------
const float teethBelt = 15.0f;
const float teethPulley = 59.0f;
const float gearRatio = (teethPulley / teethBelt) / 2.0; // mechanische Übersetzung
const float fullStepsPerRev = 200;
int microstepping = 16;  // 1/16 Microstep
float microStepsPerRev = fullStepsPerRev * microstepping;

// Stepper-Objekt (beide Motoren synchron über ein STEP-Signal)
AccelStepper dualStepper(AccelStepper::DRIVER, STEP_PIN, 13);

// ==========================================================
//                     Hilfsfunktionen
// ==========================================================

// Berechnet Mikroschritte pro Sekunde aus gewünschter Winkelgeschwindigkeit (°/s)
float calcMicrostepsPerSecond(float angularVelocityDegPerSec) {
  float motorAngularVelocity = angularVelocityDegPerSec * gearRatio;
  float revolutionsPerSec = motorAngularVelocity / 360.0;
  return revolutionsPerSec * microStepsPerRev;
}

// Berechnet Mikroschritte aus gewünschtem Drehwinkel (in Umdrehungsanteilen)
float calcMicrostepsPerRotation(float rotations) {
  return rotations * microStepsPerRev * gearRatio;
}

// ==========================================================
//                   Motor-Setup
// ==========================================================
void setupMotorPins() {
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN_1, OUTPUT);
  pinMode(DIR_PIN_2, OUTPUT);
  pinMode(M0_PIN, OUTPUT);
  pinMode(M1_PIN, OUTPUT);
  pinMode(M2_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);

  // Microstepping auf 1/16 konfigurieren
  digitalWrite(M0_PIN, HIGH);
  digitalWrite(M1_PIN, HIGH);
  digitalWrite(M2_PIN, LOW);

  // Motor initial deaktivieren
  digitalWrite(ENABLE_PIN, HIGH);

  // Stepper konfigurieren
  dualStepper.setMinPulseWidth(5);
  dualStepper.setAcceleration(motorAcceleration);
  dualStepper.setCurrentPosition(0);

  startTime = millis();
}

// Motor aktivieren/deaktivieren
void enableMotor()  { digitalWrite(ENABLE_PIN, LOW); }
void disableMotor() { digitalWrite(ENABLE_PIN, HIGH); }

// ==========================================================
//           Bewegung: Stillstands- und Übergangsphasen
// ==========================================================

// Fährt n Stillstandspositionen mit definiertem Pausenintervall ab
void moveToMeasurementPositions(int numPositions, bool dir1High, bool dir2High) {
  float deg = 360.0 / numPositions;
  float rotations = deg / 360.0;
  float angularSpeed = deg / dwellTime;

  dualStepper.setMaxSpeed(calcMicrostepsPerSecond(angularSpeed));

  for (int i = 0; i < numPositions; i++) {
    // Stillstand (Messung)
    vTaskDelay(dwellTime * 1000 / portTICK_PERIOD_MS);

    // Richtung setzen
    digitalWrite(DIR_PIN_1, dir1High ? HIGH : LOW);
    digitalWrite(DIR_PIN_2, dir2High ? HIGH : LOW);

    // Zielposition berechnen
    long target = dualStepper.currentPosition() + calcMicrostepsPerRotation(rotations);
    dualStepper.moveTo(target);

    // Bewegung ausführen
    while (dualStepper.distanceToGo() != 0) {
      dualStepper.run();
    }
  }
}

// Dreht nach jeder Messrunde um einen festen Winkel (z. B. 60°)
void transitionZ(bool dir1High, bool dir2High) {
  dualStepper.setMaxSpeed(transitionSpeed);

  digitalWrite(DIR_PIN_1, dir1High ? HIGH : LOW);
  digitalWrite(DIR_PIN_2, dir2High ? HIGH : LOW);

  float rotation = (degreePerStep / 360.0) * 2.0; // *2 wegen Übersetzung
  long target = dualStepper.currentPosition() + calcMicrostepsPerRotation(rotation);
  dualStepper.moveTo(target);

  while (dualStepper.distanceToGo() != 0) {
    dualStepper.run();
  }

  rotationCount++;
}

// Führt eine größere Gesamtrotation durch (z. B. Heimfahrt)
void transitionFull(float rotation, bool dir1High, bool dir2High) {
  dualStepper.setMaxSpeed(transitionSpeed);
  digitalWrite(DIR_PIN_1, dir1High ? HIGH : LOW);
  digitalWrite(DIR_PIN_2, dir2High ? HIGH : LOW);

  long target = dualStepper.currentPosition() + calcMicrostepsPerRotation(rotation);
  dualStepper.moveTo(target);

  while (dualStepper.distanceToGo() != 0) {
    dualStepper.run();
  }
}

// ==========================================================
//                       MPU6050
// ==========================================================
const uint8_t MPU = 0x68;
const float g0 = 9.81;
const float accScale = g0 / 16384.0;
const float gyroScale = 131.0 / (PI / 180.0); // rad/s

void writeMPU(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission(true);
}

void setupMPU(uint16_t sampleRateHz = 100) {
  writeMPU(0x6B, 0x01); // PWR_MGMT_1
  writeMPU(0x6C, 0x00); // PWR_MGMT_2
  writeMPU(0x1B, 0x00); // GYRO_CONFIG
  writeMPU(0x1C, 0x00); // ACCEL_CONFIG
  writeMPU(0x1A, 0x04); // DLPF
  uint8_t div = (uint8_t)(1000 / sampleRateHz - 1);
  writeMPU(0x19, div);
  writeMPU(0x6A, 0x04); // FIFO Reset
}

// Liest aktuelle Sensordaten und gibt sie über Serial aus
void readIMUData() {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 14, true);

  if (Wire.available() == 14) {
    int16_t ax = (Wire.read() << 8) | Wire.read();
    int16_t ay = (Wire.read() << 8) | Wire.read();
    int16_t az = (Wire.read() << 8) | Wire.read();
    Wire.read(); Wire.read(); // Temp überspringen
    int16_t gx = (Wire.read() << 8) | Wire.read();
    int16_t gy = (Wire.read() << 8) | Wire.read();
    int16_t gz = (Wire.read() << 8) | Wire.read();

    Serial.printf("%.5f,%.5f,%.5f,%.5f,%.5f,%.5f\n",
                  ax * accScale, ay * accScale, az * accScale,
                  gx / gyroScale, gy / gyroScale, gz / gyroScale);
  }
}

// ==========================================================
//                     Tasks (Threads)
// ==========================================================

// Task 1: IMU-Daten lesen (Core 0)
void TaskReadIMU(void *pvParameters) {
  for (;;) {
    if (!motorDone) {
      readIMUData();
      vTaskDelay(10 / portTICK_PERIOD_MS); // 100 Hz
    } else {
      vTaskDelay(100 / portTICK_PERIOD_MS);
    }
  }
}

// Task 2: Motorsteuerung (Core 1)
void TaskMotor(void *pvParameters) {
  motorDone = false;

  for (;;) {
    // Wartephase beim Start
    if (!motorEnabled) {
      if (millis() - startTime >= startupDelayMs) {
        enableMotor();
        motorEnabled = true;
      } else {
        vTaskDelay(10 / portTICK_PERIOD_MS);
        continue;
      }
    }

    // Bewegungslogik (abwechselnde Drehrichtungen)
    if (rotationCount % 2 == 0) {
      moveToMeasurementPositions(numPositions, HIGH, LOW);
      transitionZ(HIGH, HIGH);
    } else {
      moveToMeasurementPositions(numPositions, LOW, HIGH);
      transitionZ(HIGH, HIGH);
    }

    // Nach kompletter 360°-Drehung: Motor deaktivieren
    if (rotationCount >= (int)(360.0 / degreePerStep)) {
      transitionFull(2.0, LOW, LOW);
      disableMotor();
      motorDone = true;
      Serial.println("✅ Motorlauf abgeschlossen");
      vTaskDelete(NULL);
    }

    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

// ==========================================================
//                         Setup
// ==========================================================
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== ESP32 Dual-Core Motor + IMU gestartet ===");

  setupMotorPins();
  Wire.begin(21, 22);
  Wire.setClock(400000);
  setupMPU();
  delay(100);

  // Core-Zuweisung: IMU -> Core 0, Motor -> Core 1
  xTaskCreatePinnedToCore(TaskReadIMU, "IMU_Task", 8192, NULL, 2, &ImuTaskHandle, 0);
  xTaskCreatePinnedToCore(TaskMotor,  "Motor_Task", 8192, NULL, 2, &MotorTaskHandle, 1);
}

void loop() {
  // Alle Aktionen laufen in Tasks
}
