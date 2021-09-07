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
    mpu.initialize();

    Wire.setClock(400000L);

    Serial.println("Starting calibration");
    delay(1000);
}


void loop()
{
    mpu.calibrateAccelerometer(6000);

    auto offset = mpu.getAccOffset();

    Serial.print(offset.x);
    Serial.print('\t');
    Serial.print(offset.y);
    Serial.print('\t');
    Serial.println(offset.z);
}
