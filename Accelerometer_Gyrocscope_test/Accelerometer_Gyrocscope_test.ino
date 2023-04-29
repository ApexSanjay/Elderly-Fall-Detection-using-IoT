#include <M5StickC.h>

void setup() {
  M5.begin();
  M5.IMU.Init();
}

void loop() {
  float accX, accY, accZ;
  float gyroX, gyroY, gyroZ;

  M5.IMU.getAccelData(&accX, &accY, &accZ);
  M5.IMU.getGyroData(&gyroX, &gyroY, &gyroZ);

  Serial.printf("Acceleration: X=%.2f, Y=%.2f, Z=%.2f\n", accX, accY, accZ);
  Serial.printf("Gyroscope: X=%.2f, Y=%.2f, Z=%.2f\n", gyroX, gyroY, gyroZ);

  delay(100);
}