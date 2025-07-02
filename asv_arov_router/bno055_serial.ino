#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  if (!bno.begin()) {
    Serial.println("ERROR: BNO055 not detected.");
    while (1);
  }

  bno.setExtCrystalUse(true);
  Serial.println("BNO055 ready.");
}

void loop() {
  imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
//   imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
//   imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<4> quat = bno.getQuat()

  unsigned long timestamp = millis();

  Serial.print(timestamp); Serial.print(",");
  Serial.print(acc.x()); Serial.print(","); Serial.print(acc.y()); Serial.print(","); Serial.print(acc.z()); Serial.print(",");
  Serial.print(gyro.x()); Serial.print(","); Serial.print(gyro.y()); Serial.print(","); Serial.print(gyro.z()); Serial.print(",");
  Serial.print(quat.x()); Serial.print(","); Serial.print(quat.y()); Serial.print(","); Serial.print(quat.z()); Serial.print(","); Serial.print(quat.w())
//   Serial.print(mag.x()); Serial.print(","); Serial.print(mag.y()); Serial.print(","); Serial.print(mag.z()); Serial.print(",");
//   Serial.print(euler.x()); Serial.print(","); Serial.print(euler.y()); Serial.print(","); Serial.println(euler.z());

  delay(100);  // 10Hz
}
