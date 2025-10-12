#include <Wire.h>

// MPU6050 Adresse
const int MPU = 0x68;

// Variablen für Winkel
float pitch = 0.0;
float roll = 0.0;
unsigned long lastTime;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // MPU aufwecken (aus Sleep-Modus holen)
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);   // PWR_MGMT_1 Register
  Wire.write(0);      // Setze auf 0 = Clock intern, Sleep aus
  Wire.endTransmission(true);

  lastTime = millis();
}

void loop() {
  // Zeitdifferenz
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  // 14 Bytes ab 0x3B auslesen: Accel(6), Temp(2), Gyro(6)
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 14, true);

  int16_t ax = (Wire.read() << 8) | Wire.read();
  int16_t ay = (Wire.read() << 8) | Wire.read();
  int16_t az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read(); // Temp ignorieren
  int16_t gx = (Wire.read() << 8) | Wire.read();
  int16_t gy = (Wire.read() << 8) | Wire.read();
  int16_t gz = (Wire.read() << 8) | Wire.read();

  // In G und °/s umrechnen
  float axf = ax / 16384.0;  // ±2g → 16384 LSB/g
  float ayf = ay / 16384.0;
  float azf = az / 16384.0;

  float gxf = gx / 131.0;    // ±250°/s → 131 LSB/(°/s)
  float gyf = gy / 131.0;
  float gzf = gz / 131.0;

  // --- Accelerometer-Winkel ---
  float accelPitch = atan2(ayf, azf) * 180 / PI;
  float accelRoll  = atan2(-axf, sqrt(ayf * ayf + azf * azf)) * 180 / PI;

  // --- Gyro Integration ---
  pitch += gxf * dt;
  roll  += gyf * dt;

  // --- Komplementärfilter ---
  pitch = 0.98 * pitch + 0.02 * accelPitch;
  roll  = 0.98 * roll  + 0.02 * accelRoll;

  // Ausgabe
  Serial.print("Pitch: "); Serial.print(pitch);
  Serial.print("  Roll: "); Serial.println(roll);

  delay(10); // Abtastrate ~100 Hz
}
