#include <Wire.h>
const int MPU_ADDR = 0x68;

int counter = 0;  // Zähler für Messungen

void setup() {
  Serial.begin(115200);
  delay(200);
  Wire.begin();

  // MPU aufwecken
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0); 
  Wire.endTransmission(true);

  // CSV-Header
  Serial.println("n,ax,ay,az,gx,gy,gz");
}

void loop() {
  int16_t ax,ay,az,gx,gy,gz,tmp;

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  ax = (Wire.read()<<8) | Wire.read();
  ay = (Wire.read()<<8) | Wire.read();
  az = (Wire.read()<<8) | Wire.read();
  tmp= (Wire.read()<<8) | Wire.read(); // Temp ungenutzt
  gx = (Wire.read()<<8) | Wire.read();
  gy = (Wire.read()<<8) | Wire.read();
  gz = (Wire.read()<<8) | Wire.read();

  // Messung ausgeben: Nummer + Rohdaten
  Serial.print(counter); Serial.print(",");
  Serial.print(ax); Serial.print(",");
  Serial.print(ay); Serial.print(",");
  Serial.print(az); Serial.print(",");
  Serial.print(gx); Serial.print(",");
  Serial.print(gy); Serial.print(",");
  Serial.println(gz);

  counter++;        // Zähler hochzählen
  delay(100);       // Abtastrate
}
