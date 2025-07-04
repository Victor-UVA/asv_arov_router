#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void setup() {
  Serial.begin(115200);
  delay(10000);
  while (!Serial) delay(10);

  if (!bno.begin()) {
    Serial.println("ERROR: BNO055 not detected.");
    while (1);
  }
  bno.setExtCrystalUse(true);
  Serial.println("BNO055 ready.");
}

void loop() {
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  unsigned long t_ms = millis();
  Serial.print(t_ms); Serial.print(",");
  Serial.print(gyro.x()); Serial.print(",");
  Serial.print(gyro.y()); Serial.print(",");
  Serial.print(gyro.z()); //Serial.print("\n")
}
// a4 -> sda
// a5 -> scl