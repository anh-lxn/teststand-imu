#include <Wire.h>

const uint8_t MPU = 0x68;
const uint16_t NUM_SAMPLES = 100;

struct IMUSample {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
};
IMUSample buffer[NUM_SAMPLES];

/// ---- I2C Helfer ----
void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission(true);
}

uint16_t read16(uint8_t regHi) {
  Wire.beginTransmission(MPU);
  Wire.write(regHi);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 2, true);
  uint8_t hi = Wire.read();
  uint8_t lo = Wire.read();
  return ((uint16_t)hi << 8) | lo;
}

/// ---- MPU Setup ----
void mpu_setup() {
  writeReg(0x6B, 0x01); // PLL Clock
  writeReg(0x6C, 0x00); // alle Sensoren an
  writeReg(0x1B, 0x00); // Gyro ±250 dps
  writeReg(0x1C, 0x00); // Accel ±2 g
  writeReg(0x1A, 0x04); // DLPF ~20 Hz
  writeReg(0x19, 9);    // SMPLRT_DIV -> 100 Hz
}

/// ---- FIFO Steuerung ----
void mpu_fifo_reset() {
  writeReg(0x6A, 0x04); // FIFO Reset
}

void mpu_fifo_start() {
  mpu_fifo_reset();
  writeReg(0x6A, 0x40); // FIFO enable
  writeReg(0x23, 0x78); // Accel + Gyro in FIFO
}

void mpu_fifo_stop() {
  writeReg(0x23, 0x00); // keine neuen Daten
}

uint16_t mpu_fifo_count() {
  return read16(0x72);
}

uint16_t lastSamples = 0; // global merken, wie viele Frames wirklich gelesen wurden

void readFIFOBlock() {
  mpu_fifo_start();
  delay(1000);            // 1 Sekunde sammeln
  mpu_fifo_stop();        // jetzt nix Neues mehr

  uint16_t bytes = mpu_fifo_count();
  uint16_t samples = bytes / 12;
  if (samples > NUM_SAMPLES) samples = NUM_SAMPLES;

  lastSamples = samples;  // <--- merken, wie viele Frames wirklich da sind

  for (uint16_t i = 0; i < samples; i++) {
    Wire.beginTransmission(MPU);
    Wire.write(0x74);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 12, true);

    buffer[i].ax = (Wire.read()<<8) | Wire.read();
    buffer[i].ay = (Wire.read()<<8) | Wire.read();
    buffer[i].az = (Wire.read()<<8) | Wire.read();
    buffer[i].gx = (Wire.read()<<8) | Wire.read();
    buffer[i].gy = (Wire.read()<<8) | Wire.read();
    buffer[i].gz = (Wire.read()<<8) | Wire.read();
  }
}

void dumpBuffer() {
  Serial.println("AX,AY,AZ,GX,GY,GZ");
  for (uint16_t i = 0; i < lastSamples; i++) {   // <--- nur die echten Samples!
    Serial.print(buffer[i].ax); Serial.print(",");
    Serial.print(buffer[i].ay); Serial.print(",");
    Serial.print(buffer[i].az); Serial.print(",");
    Serial.print(buffer[i].gx); Serial.print(",");
    Serial.print(buffer[i].gy); Serial.print(",");
    Serial.println(buffer[i].gz);
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000);
  delay(100);

  mpu_setup();
  Serial.println("Starte MPU FIFO Test...");
}

void loop() {
  readFIFOBlock();  // genau 1 Block einlesen
  dumpBuffer();     // Ausgabe
  delay(5000);      // Pause
}
