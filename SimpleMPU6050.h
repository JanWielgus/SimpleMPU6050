/**
 * @file SimpleMPU6050.h
 * @author Jan Wielgus
 * @brief Parts of this program are based on the YMFC-32 made by Joop Brokking
 * Link: http://www.brokking.net/YMFC-32_auto_downloads.html
 * @date 2019-06-04 (modified: 2020-11-25)
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#ifndef SIMPLEMPU6050_H
#define SIMPLEMPU6050_H

#include <Wire.h>

#ifdef ARDUINO
    #include <Arduino.h>
#endif


class SimpleMPU6050
{
public:
	struct vector3Int16
	{              // IN GYRO
		int16_t x; // pitch
		int16_t y; // roll
		int16_t z; // yaw
	};
	
	struct vector3Float
	{            // IN GYRO
		float x; // pitch
		float y; // roll
		float z; // yaw
	};


private:
	// Raw readings
	vector3Int16 rawAcceleration;
	vector3Int16 rawRotation;
	vector3Float normAcceleration; // updated in getter
	vector3Float normRotation; // updated in getter
	int16_t temperature;

	// Calibration values
	vector3Int16 accOffset;
	vector3Int16 gyroOffset;

	// Normalize multipliters
	const float accNormalizeMultiplier = 0.0002441406f; // = 1 / 4096
	const float gyroNormalizeMultiplier = 0.01526717f; // = 1 / 65.5
	const uint16_t acc1G_value = 4096; // (from datasheet)

	static const uint8_t MPU6050_Address = 0x68;


public:
	SimpleMPU6050();

	/**
	 * @brief Perform one attempt to connect with MPU6050.
	 * If failed, try checking several times.
	 * CALL Wire.begin() OUTSIDE!!!
	 * @return false if initialization was not successful
	 * (for example connection problem). 
	 */
	bool initialize();

	/**
	 * @brief Asks device for the raw data and receives it.
	 */
	void readRawData();

	/**
	 * @brief Getter of raw acceleration.
	 * @return Raw acceleration int16 vector.
	 */
	vector3Int16 getRawAcceleration();
	
	/**
	 * @brief Getter of raw angular velocity.
	 * @return Raw angular velocity int16 vector.
	 */
	vector3Int16 getRawRotation();

	/**
	 * @brief Getter of normalized acceleration [in G].
	 * @return Normalized acceleration float vector.
	 */
	vector3Float getNormalizedAcceleration();

	/**
	 * @brief Getter of normalized angular velocity
	 * [in revolutions per second].
	 * @return Normalized angular velocity float vector.
	 */
	vector3Float getNormalizedRotation();

	/**
	 * @brief Get the temperature in degrees Celsius.
	 */
	int16_t getTemperature();

	/**
	 * @brief Block the program and perform accelerometer calibration.
	 * Device have to be positioned horizontally and not move!
	 * Sum samplesToAverage values and calculate average of them (offset).
	 * @param samplesToAverage Amount of readings to include in the average
	 * (250 by default).
	 */
	void calibrateAccelerometer(uint16_t samplesToAverage = 250);

	/**
	 * @brief Block the program and perform gyro calibration.
	 * Device have to be as steady as possible (cannot rotate at all).
	 * Sum samplesToAverage values and calculate average of them (offset). 
	 * @param samplesToAverage Amount of readings to include in the average
	 * (2000 by default).
	 */
	void calibrateGyroscope(uint16_t samplesToAverage = 2000);

	/**
	 * @brief Getter of the accelerometer offset
	 * (calibration values).
	 */
	vector3Int16 getAccOffset();

	/**
	 * @brief Getter of the gyro offset
	 * (calibration values).
	 */
	vector3Int16 getGyroOffset();

	/**
	 * @brief Accelerometer offset setter.
	 * @param offX X axis offset.
	 * @param offY Y axis offset.
	 * @param offZ Z axis offset.
	 */
	void setAccOffset(int16_t offX, int16_t offY, int16_t offZ);

	/**
	 * @brief Gyroscope offset setter. 
	 * @param offX X axis offset.
	 * @param offY Y axis offset.
	 * @param offZ Z axis offset.
	 */
	void setGyroOffset(int16_t offX, int16_t offY, int16_t offZ);

	/**
	 * @brief This method is used to enable compass on GY86
	 * sensor board, because it is connected via MPU6050.
	 */
	void enableCompassBypass();

	/**
	 * @brief Return the value whick accelerometer axis has if have 1G.
	 */
	uint16_t getRawAccValueFor1G()
	{
		return acc1G_value;
	}
};


#endif

