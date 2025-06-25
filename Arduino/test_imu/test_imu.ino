#include "MPU9250.h"

//---------DEFINITIONS-----------

// SERIAL
#define SERIAL 115200

// IMU
MPU9250 mpu;
#define T_S_IMU 50 // ms


//---------SETUP-----------
void setup() {
  Serial.begin(SERIAL);

  // imu
  Wire.begin();
  delay(500);
  if (!mpu.setup(0x68)) {  // change to your own address
        while (1) {
            Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
            delay(5000);
        }
    }
  mpu.verbose(true);
  delay(1000);
  mpu.calibrateAccelGyro();
  delay(1000);
  mpu.calibrateMag();
  print_calibration();
  mpu.verbose(false);

}


//---------LOOP-----------
void loop() {

  if (mpu.update()){
    Serial.print("Accx:");
    Serial.print(mpu.getAccX());
    Serial.print(",");
    Serial.print("Accy:");
    Serial.print(",");
    Serial.print(mpu.getAccY());
    Serial.print(",");
    Serial.print("Accz:");
    Serial.print(mpu.getAccZ());
    Serial.print(",");
    Serial.print("Gyrox:");
    Serial.print(mpu.getGyroX());
    Serial.print(",");
    Serial.print("Gyroy:");
    Serial.print(mpu.getGyroY());
    Serial.print(",");
    Serial.print("Gyroz:");
    Serial.print(mpu.getGyroZ());
    Serial.println(",");
  }

}


void print_calibration() {
    Serial.println("< calibration parameters >");
    Serial.println("accel bias [g]: ");
    Serial.print(mpu.getAccBiasX() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getAccBiasY() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getAccBiasZ() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.println();
    Serial.println("gyro bias [deg/s]: ");
    Serial.print(mpu.getGyroBiasX() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasY() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasZ() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.println();
    Serial.println("mag bias [mG]: ");
    Serial.print(mpu.getMagBiasX());
    Serial.print(", ");
    Serial.print(mpu.getMagBiasY());
    Serial.print(", ");
    Serial.print(mpu.getMagBiasZ());
    Serial.println();
    Serial.println("mag scale []: ");
    Serial.print(mpu.getMagScaleX());
    Serial.print(", ");
    Serial.print(mpu.getMagScaleY());
    Serial.print(", ");
    Serial.print(mpu.getMagScaleZ());
    Serial.println();
}
