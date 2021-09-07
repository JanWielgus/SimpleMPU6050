/**
 * @file AccCalibration.ino
 * @author Jan Wielgus
 * @brief Accelerometer calibraion
 * @date 2021-08-17
 */

#include "SimpleMPU6050.h"

SimpleMPU6050 mpu;


void setup()
{
    Serial.begin(115200);
    Serial.println("Program has started");

    Wire.begin();
    Serial.println(mpu.initialize());

    Wire.setClock(400000L);

    Serial.println("Starting calibration");
}


void loop()
{
    mpu.calibrateGyroscope(10000);

    auto offset = mpu.getGyroOffset();

    Serial.print(offset.x);
    Serial.print('\t');
    Serial.print(offset.y);
    Serial.print('\t');
    Serial.println(offset.z);


    // mpu.readRawData();
    // auto gyro = mpu.getRawRotation();
    // Serial.print(gyro.x);
    // Serial.print('\t');
    // Serial.print(gyro.y);
    // Serial.print('\t');
    // Serial.println(gyro.z);
    // delay(4);
}
