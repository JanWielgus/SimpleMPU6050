/*
    Created:	04/06/2019
    Author:     Jan Wielgus
*/
// 
// 
// 

#include <FC_MPU6050Lib.h>


FC_MPU6050Lib::FC_MPU6050Lib()
{
	rawAcceleration.x = 0;
	rawAcceleration.y = 0;
	rawAcceleration.z = 0;
	
	rawRotation.x = 0;
	rawRotation.y = 0;
	rawRotation.z = 0;
	
	temperature = 0;
	
	accAngle.x = 0;
	accAngle.y = 0;
	accAngle.z = 0;
	
	gyroCalVal.xPitch = 0;
	gyroCalVal.yRoll = 0;
	gyroCalVal.zYaw = 0;
	
	accCalVal.x = 0;
	accCalVal.y = 0;
	accCalVal.z = 0;
	
	// default gyro calculation multipliers values
	Multiplier1 = 0.0000611;
	Multiplier2 = 0.000001066;
}


bool FC_MPU6050Lib::initialize(bool needToBeginWire_flag)
{
	if (needToBeginWire_flag)
		Wire.begin();
	
	// Check if the MPU-6050 is responding
	Wire.beginTransmission(MPU6050_Address);
	if (Wire.endTransmission() != 0)
	{
		// Cannot connect with the MPU-6050
		
		return false;
	}
	
	
	// Device setup
	Wire.beginTransmission(MPU6050_Address);                     //Start communication with the MPU-6050.
	Wire.write(0x6B);                                            //We want to write to the PWR_MGMT_1 register (6B hex).
	Wire.write(0x00);                                            //Set the register bits as 00000000 to activate the gyro.
	Wire.endTransmission();                                      //End the transmission with the gyro.

	Wire.beginTransmission(MPU6050_Address);                     //Start communication with the MPU-6050.
	Wire.write(0x1B);                                            //We want to write to the GYRO_CONFIG register (1B hex).
	Wire.write(0x08);                                            //Set the register bits as 00001000 (500dps full scale).
	Wire.endTransmission();                                      //End the transmission with the gyro.

	Wire.beginTransmission(MPU6050_Address);                     //Start communication with the MPU-6050.
	Wire.write(0x1C);                                            //We want to write to the ACCEL_CONFIG register (1A hex).
	Wire.write(0x10);                                            //Set the register bits as 00010000 (+/- 8g full scale range).
	Wire.endTransmission();                                      //End the transmission with the gyro.

	Wire.beginTransmission(MPU6050_Address);                     //Start communication with the MPU-6050.
	Wire.write(0x1A);                                            //We want to write to the CONFIG register (1A hex).
	Wire.write(0x03);                                            //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz).
	Wire.endTransmission();                                      //End the transmission with the gyro.
	
	
	// Set start position to angles got from the gyro
	fusedAngle.x = 0;
	fusedAngle.y = 0;
	int samples = 50;
	for (int i=0; i<samples; i++)
	{
		read6AxisMotion();
		getAccAngles();
		fusedAngle.x += accAngle.x;
		fusedAngle.y += accAngle.y;
		delay(4);
	}
	fusedAngle.x /= samples;
	fusedAngle.y /= samples;
	
	
	return true;
}


void FC_MPU6050Lib::setFastClock()
{
	Wire.setClock(400000L);
}


void FC_MPU6050Lib::read6AxisMotion()
{
	Wire.beginTransmission(MPU6050_Address);
	Wire.write(0x3B);
	Wire.endTransmission();
	
	Wire.requestFrom(MPU6050_Address, 14);
	rawAcceleration.x = Wire.read() << 8 | Wire.read();
	rawAcceleration.y = Wire.read() << 8 | Wire.read();
	rawAcceleration.z = Wire.read() << 8 | Wire.read();
	temperature = Wire.read() << 8 | Wire.read();
	rawRotation.y = Wire.read() << 8 | Wire.read(); // roll
	rawRotation.x = Wire.read() << 8 | Wire.read(); // pitch
	rawRotation.z = Wire.read() << 8 | Wire.read();
	
	
	rawRotation.x *= -1; // pitch
	rawRotation.z *= -1; // yaw
	
	// Use the calibration data
	rawRotation.x -= gyroCalVal.xPitch;
	rawRotation.y -= gyroCalVal.yRoll;
	rawRotation.z -= gyroCalVal.zYaw;
	rawAcceleration.x -= accCalVal.x;
	rawAcceleration.y -= accCalVal.y;
	rawAcceleration.z -= accCalVal.z;
	
	// Temperature
	temperature = ((float)temperature/340 + 36.53) + 0.5; // 0.5 to average for int
}


void FC_MPU6050Lib::calibrateGyro(int samples)
{
	// !!!!
	// Whole process last about 8 seconds !!
	// (when 2000 samples)
	// !!!!
	
	int32_t sumX = 0;
	int32_t sumY = 0;
	int32_t sumZ = 0;
	
	for (int i=0; i<samples; i++)
	{
		read6AxisMotion();
		
		sumX += rawRotation.x;
		sumY += rawRotation.y;
		sumZ += rawRotation.z;
		
		delay(4); // simulate 250Hz loop
	}
	
	gyroCalVal.xPitch += sumX / samples;
	gyroCalVal.yRoll += sumY / samples;
	gyroCalVal.zYaw += sumZ / samples;
}


void FC_MPU6050Lib::calibrateAccelerometer(int samples)
{
	int32_t sumX = 0;
	int32_t sumY = 0;
	int32_t sumZ = 0;
	
	for (int i=0; i<samples; i++)
	{
		read6AxisMotion();
		
		sumX += rawAcceleration.x;
		sumY += rawAcceleration.y;
		sumZ += (rawAcceleration.z - 4096); // 4096 is the value for 1g (from the datasheet)
		
		delay(4);
	}
	
	accCalVal.x += sumX / samples;
	accCalVal.y += sumY / samples;
	accCalVal.z += sumZ / samples;
	
	
	// Set initial gyro values after calibration
	fusedAngle.x = 0; // reset values
	fusedAngle.y = 0;
	for (int i=0; i<50; i++)
	{
		read6AxisMotion();
		getAccAngles();
		fusedAngle.x += accAngle.x;
		fusedAngle.y += accAngle.y;
		delay(4);
	}
	fusedAngle.x /= 50;
	fusedAngle.y /= 50;
}


FC_MPU6050Lib::vector3Int FC_MPU6050Lib::getAccelerometerCalibrationValues()
{
	return accCalVal;
}


FC_MPU6050Lib::vector3Int FC_MPU6050Lib::getGyroCalibrationValues()
{
	vector3Int temp;
	temp.x = gyroCalVal.xPitch;
	temp.y = gyroCalVal.yRoll;
	temp.z = gyroCalVal.zYaw;
	return temp;
}


void FC_MPU6050Lib::setAccelerometerCalibrationValues(int16_t offX, int16_t offY, int16_t offZ)
{
	accCalVal.x = offX;
	accCalVal.y = offY;
	accCalVal.z = offZ;
}


void FC_MPU6050Lib::setGyroCalibrationValues(int16_t offXpitch, int16_t offYroll, int16_t offZyaw)
{
	gyroCalVal.xPitch = offXpitch;
	gyroCalVal.yRoll = offYroll;
	gyroCalVal.zYaw = offZyaw;
}


void FC_MPU6050Lib::setGyroFusionMultiplier(float mpr)
{
	GyroFusionMultiplier = mpr;
	AccFusionMultiplier = 1.0-mpr;
}


void FC_MPU6050Lib::setCalculationsFrequency(uint16_t freq)
{
	//0.0000611 = 1 / (250Hz * 65.5)
	Multiplier1 = 1.0 / ((float)freq * 65.5);
	
	//0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians and not degrees.
	Multiplier2 = radians(Multiplier1);
}


FC_MPU6050Lib::vector3Int& FC_MPU6050Lib::getRawAcceleration()
{
	return rawAcceleration;
}


FC_MPU6050Lib::vector3Int& FC_MPU6050Lib::getRawRotation()
{
	return rawRotation;
}


int16_t FC_MPU6050Lib::getTemperature()
{
	return temperature;
}


FC_MPU6050Lib::vector3Float& FC_MPU6050Lib::getAccAngles()
{
	static int32_t accTotalVector;
	
	accTotalVector = sqrt(((int32_t)rawAcceleration.x * rawAcceleration.x) +
							((int32_t)rawAcceleration.y * rawAcceleration.y) +
							((int32_t)rawAcceleration.z * rawAcceleration.z));
	//Serial.println(accTotalVector);
	//Serial.println((int32_t)rawAcceleration.x * rawAcceleration.x);
	
	if (abs(rawAcceleration.x) < accTotalVector)
	{
		accAngle.x = asin((float)rawAcceleration.x / accTotalVector) * 57.296;
	}
	
	if (abs(rawAcceleration.y) < accTotalVector)
	{
		accAngle.y = asin((float)rawAcceleration.y / accTotalVector) * 57.296;
	}
	
	return accAngle;
}


FC_MPU6050Lib::vector3Float& FC_MPU6050Lib::getFusedXYAngles()
{
	//Gyro angle calculations

	// X & Y axis
	fusedAngle.x += (float)rawRotation.x * Multiplier1;
	fusedAngle.y += (float)rawRotation.y * Multiplier1;

	static float temp;
	temp = fusedAngle.x;
	fusedAngle.x -= fusedAngle.y * sin((float)rawRotation.z * Multiplier2);
	fusedAngle.y += temp * sin((float)rawRotation.z * Multiplier2);
	
	
	// Z axis is calculated in getZAngle() method
	// because to calculate heading (which is needed for Z axis calculations)
	// you have to calculate angle X and Y at first
	
	
	getAccAngles();
	
	// Make fusion with accelerometer data
	fusedAngle.x = fusedAngle.x * GyroFusionMultiplier + accAngle.x * AccFusionMultiplier;
	fusedAngle.y = fusedAngle.y * GyroFusionMultiplier + accAngle.y * AccFusionMultiplier;
	
	
	return fusedAngle;
}


float FC_MPU6050Lib::getZAngle(float heading)
{
	// Z axis
	fusedAngle.z += (float)rawRotation.z * Multiplier1;
	
	// 0-359.99 correction
	if(fusedAngle.z < 0.0)
		fusedAngle.z += 360.0;
	else if (fusedAngle.z >= 360.0)
		fusedAngle.z -= 360.0;
	
	
	// USE COMPASS DATA IF PROVIDED (if not, compass is == -1)  !!!!   <<<-----
	if (heading != -1)
	{
		// fuse compass with gyro
		
		// eg. if compass is 359 but gyro is 1 degree
		// this solve this error
		if (abs(fusedAngle.z - heading) > 100)
		{
			if (heading > 180)
				heading -= 360.0;
			else
				heading += 360.0;
		}
		
		// complementary filter
		fusedAngle.z = 0.98 * fusedAngle.z + 0.02 * heading;
		
		// 0-359.99 correction
		if(fusedAngle.z < 0.0)
			fusedAngle.z += 360.0;
		else if (fusedAngle.z >= 360.0)
			fusedAngle.z -= 360.0;
	}
	
	
	return fusedAngle.z;
}


void FC_MPU6050Lib::setInitialZAxisValue(float compassHeading)
{
	fusedAngle.z = compassHeading;
}


