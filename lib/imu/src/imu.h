#ifndef IMU_h
#define IMU_h

#include <stdint.h>
#include <Wire.h>

#include "kalmanFilter.h"

union imuData
{
	float axis[3];
	struct RollPitchYaw
	{
		float roll;
		float pitch;
		float yaw;
	} rpy;
	struct XYZ
	{
		float x;
		float y;
		float z;
	} xyz;
};

class Imu
{
	public:
		Imu();

		// @brief Trigger IMU data update
		const imuData& updateAttitude();
		
		// @brief Get current RPY attitude in degrees
		const imuData& getAttitude() const { return m_attitude; }
		
		// @brief Start IMU calibration
		void calibrate(int num_samples = 200);
		
		private:
		void updateGyroAccelData();
		void getRawGyroAccelData(int16_t *rawGyroAccelData);
		
		void updateMagnetometerData();
		void getMagnetometerData(int16_t *rawMagnetometerData);

		float calculatePitchAccel(const imuData& a);
		float calculateRollAccel(const imuData& a);
		
		void writeAK8963Register(uint8_t reg, uint8_t value);
		void readAK8963Registers(uint8_t reg, uint8_t *value, uint8_t count);

		void writeRegister(uint8_t addr, uint8_t reg, uint8_t value);
		void readRegister(uint8_t addr, uint8_t reg, uint8_t *value, uint8_t length = 0);

	private:
		// IMU I2C address
		uint8_t m_imuAddress;

		// IMU calibration data
		imuData m_axisCalibration;

		// current IMU Attitude use Data() to update values
		// IMU attitude data, use getAttitude() to access
		imuData m_attitude;

		// IMU gyro, accel & mag data
		imuData m_gyroData;
		imuData m_accelData;
		imuData m_magData;

		// Previous gyro output data for high-pass filter
		imuData m_prevGyroOutput[2];

		// IMU temperature in degrees Celsius
		int16_t m_temperatureCelcius;

		KalmanFilter kalmanPitch;
		KalmanFilter kalmanRoll;
		KalmanFilter kalmanYaw;

		// Wire object for I2C communication
		TwoWire* m_wire;
};

#endif
