#ifndef IMU_h
#define IMU_h

#include <stdint.h>
#include <Wire.h>
class Imu
{
public:
	union imu_data
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
	

public:
	Imu();

	// @brief Get current RPY attitude in degrees
	const imu_data& getAttitude() const { return m_attitude; }
	
	// @brief Trigger IMU data update
	void updateAttitude();
	
	void updateGyroAccelData();

	// @brief Start IMU calibration
	void calibrate(int num_samples = 200);

private:
	void calculateAngle(imu_data *gyroData, imu_data *accelData, unsigned long dt);
	void getRawGyroAccelData(int16_t *rawGyroAccelData);

	void writeRegister(uint8_t addr, uint8_t reg, uint8_t value);
	void readRegister(uint8_t addr, uint8_t reg, uint8_t *value);
	void readRegister(uint8_t addr, uint8_t reg, uint8_t *value, uint8_t length);

private:
	uint8_t m_imuAddress;
	imu_data m_axisCalibration;

	// current IMU Attitude use Data() to update values
	// IMU attitude data, use getAttitude() to access
	imu_data m_attitude;

	imu_data m_gyroData;
	imu_data m_accelData;

	imu_data m_prevGyroOutput;

	int16_t m_temperatureCelcius;

	unsigned long m_lastTime;

	TwoWire* m_wire;
};

#endif
