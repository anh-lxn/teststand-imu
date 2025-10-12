#include <Wire.h>

#define START_OUT_PIN 3    // geht an D11 vom Motor-Nano
#define TRIGGER_IN_PIN 2      // kommt von D10 des Motor-Nano


const uint16_t FS_HZ_DEFAULT = 100;
const uint8_t MPU = 0x68; 

const float g0 = 9.81;
const float MPUscaleFactorAcc = 16384.0; // in LSB/g
const float MPUscaleFactorAccMS2 = g0/MPUscaleFactorAcc; // in (m/s²)/LSB
const float MPUscaleFactorGyro = 131.0; // in LSB/(°/s)
const float MPUscaleFactorGyroRADS = MPUscaleFactorGyro / (PI/180.0);

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

void starte_imu_setup(){
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000); // I2C Fast Mode
  delay(100);
  // MPU konfigurieren (DLPF 20 Hz, Fs=100 Hz)
  mpu_setup(FS_HZ_DEFAULT);
}

void readDataIMU() {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);  // repeated start
  Wire.requestFrom(MPU, 14, true);

  int16_t ax, ay, az, wx, wy, wz;
  if (Wire.available() == 14) { 
    ax = (Wire.read() << 8) | Wire.read();
    ay = (Wire.read() << 8) | Wire.read();
    az = (Wire.read() << 8) | Wire.read();
    Wire.read(); Wire.read(); // Temperatur überspringen
    wx = (Wire.read() << 8) | Wire.read();
    wy = (Wire.read() << 8) | Wire.read();
    wz = (Wire.read() << 8) | Wire.read();
    /*
    Serial.print(ax); Serial.print(",");
    Serial.print(ay); Serial.print(",");
    Serial.print(az); Serial.print(",");
    Serial.print(wx); Serial.print(",");
    Serial.print(wy); Serial.print(",");
    Serial.print(wz); Serial.print(" | ");
    */
    Serial.print((float)ax * MPUscaleFactorAccMS2, 5); Serial.print(",");
    Serial.print((float)ay * MPUscaleFactorAccMS2, 5); Serial.print(",");
    Serial.print((float)az * MPUscaleFactorAccMS2, 5); Serial.print(",");
    Serial.print((float)wx / MPUscaleFactorGyroRADS, 5); Serial.print(",");
    Serial.print((float)wy / MPUscaleFactorGyroRADS, 5); Serial.print(",");
    Serial.println((float)wz / MPUscaleFactorGyroRADS, 5);

  }

  //delay(10);
}

void sendStartToMotor() {
  pinMode(START_OUT_PIN, OUTPUT);
  digitalWrite(START_OUT_PIN, LOW);
  delay(50);
  pinMode(START_OUT_PIN, INPUT); // wieder hochohmig
}

void setup() {
  pinMode(START_OUT_PIN, INPUT);   // hochohmig, kein Stromfluss
  pinMode(TRIGGER_IN_PIN, INPUT_PULLUP);
  starte_imu_setup();
  Serial.println("IMU bereit.");
  delay(100);
}

bool end = false;

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "START") { 
      end = false;
      Serial.println("PC → IMU: START");
      sendStartToMotor();
      delay(100);
      //Serial.println("IMU → Motor: Startsignal gesendet.");
    }
  }
  
  if (digitalRead(TRIGGER_IN_PIN) == LOW) {
    //Serial.println("Motor → IMU: Trigger erhalten!");
    readDataIMU();
    end = true;

  }

  if (digitalRead(TRIGGER_IN_PIN) == HIGH && end) {
    Serial.println("Ende");
  }
}
