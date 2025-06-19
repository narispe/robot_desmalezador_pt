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


