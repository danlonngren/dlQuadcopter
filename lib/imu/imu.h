#ifndef IMU_h
#define IMU_h

#include <stdint.h>

class imu
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
	imu();

	// @brief Get current RPY attitude in degrees
	const imu_data& getAttitude() const { return m_attitude; }
	
	// @brief Trigger IMU data update
	void updateAttitude();
	
	// @brief Start IMU calibration
	void calibrate(int num_samples = 2000);

private:
	void calculateAngle(float roll, float pitch, float yaw, float x, float y, float z);
	void getRawImuData(int* dataOut);

private:
	uint8_t m_imuAddress;
	imu_data m_axis_cal;

	float m_roll_adjust; 
	float m_pitch_adjust;

	// current IMU Attitude use Data() to update values
	// IMU attitude data, use getAttitude() to access
	imu_data m_attitude;

	// Raw axis data for lowpass filter
	float m_raw_axis[6];
};

#endif
