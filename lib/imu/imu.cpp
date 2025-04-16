#include "imu.h"

// Arduino includes
#include <Wire.h>

// Lib includes
#include "filters.h"

// Constants
#define MPU6050 0x68
#define FILTER_SNAP 0.01f

imu::imu() :
    m_imuAddress(MPU6050),
    m_axis_cal{50, -30, -4},
    m_roll_adjust(0),
    m_pitch_adjust(0),
    m_attitude{0}
{
    Wire.begin();
    Wire.setClock(400000L);
    Wire.beginTransmission(m_imuAddress);
    Wire.write(0x6B); // PWR_MGMT_1 register (6B hex)
    Wire.write(0x00); // activate
    Wire.endTransmission();

    Wire.beginTransmission(m_imuAddress);
    Wire.write(0x1B); // gyro dps register
    Wire.write(0x08); // 00001000 (500dps scale)
    Wire.endTransmission();

    Wire.beginTransmission(m_imuAddress);
    Wire.write(0x1C); // accelerometer sensisitivity register
    Wire.write(0x10); // 00010000  (8g full scale range)
    Wire.endTransmission();

    Wire.beginTransmission(m_imuAddress);
    Wire.write(0x1A); //
    Wire.write(0x00); //(Set Digital Low Pass Filter to ~43Hz)
    Wire.endTransmission();
}

void imu::updateAttitude()
{
    int new_raw_axis[6] = {0};

    getRawImuData(new_raw_axis);
    
    // Adjust for calibration
    new_raw_axis[3] -= m_axis_cal.axis[0];                            
    new_raw_axis[4] -= m_axis_cal.axis[1];                          
    new_raw_axis[5] -= m_axis_cal.axis[2];                            

    LowPassFilter(new_raw_axis[0], &m_raw_axis[0], FILTER_SNAP); // X
    LowPassFilter(new_raw_axis[1], &m_raw_axis[1], FILTER_SNAP); // Y
    LowPassFilter(new_raw_axis[2], &m_raw_axis[2], FILTER_SNAP); // Z
    LowPassFilter(new_raw_axis[3], &m_raw_axis[3], FILTER_SNAP); // Pitch
    LowPassFilter(new_raw_axis[4], &m_raw_axis[4], FILTER_SNAP); // Roll
    LowPassFilter(new_raw_axis[5], &m_raw_axis[5], FILTER_SNAP); // Yaw

    calculateAngle( m_raw_axis[4] / 65.5,
                    m_raw_axis[3] / 65.5,
                    m_raw_axis[5] / 65.5 * (-1),
                    m_raw_axis[0], 
                    m_raw_axis[1], 
                    m_raw_axis[2] * (-1));
}

void imu::getRawImuData(int* dataOut)
{
    if (!dataOut)
        return;

    // Read 14 bytes of data starting from register 0x3B
    // 0x3B: ACCEL_XOUT_H
    Wire.beginTransmission(m_imuAddress);
    Wire.write(0x3B);
    Wire.endTransmission();
    Wire.requestFrom(m_imuAddress, (uint8_t)14);

    while (Wire.available() < 14);
    // Accelerometer
    dataOut[0] = Wire.read() << 8 | Wire.read(); // x
    dataOut[1] = Wire.read() << 8 | Wire.read(); // y
    dataOut[2] = Wire.read() << 8 | Wire.read(); // z
    int temperature = Wire.read() << 8 | Wire.read();
    (void)temperature;
    // Gyroscope
    dataOut[3] = Wire.read() << 8 | Wire.read(); // Pitch
    dataOut[4] = Wire.read() << 8 | Wire.read(); // Roll
    dataOut[5] = Wire.read() << 8 | Wire.read(); // Yaw
}

void imu::calculateAngle(float roll, float m_pitch, float m_yaw, float x, float y, float z)
{
    m_attitude.rpy.roll += roll * 0.0000611;
    m_attitude.rpy.pitch += m_pitch * 0.0000611;

    m_attitude.rpy.pitch -= m_attitude.rpy.roll * sin(m_yaw * 0.000001066);
    m_attitude.rpy.roll += m_attitude.rpy.pitch * sin(m_yaw * 0.000001066);

    float acc_vector = sqrt((x * x) + (y * y) + (z * z));

    float pitch_acc = 0.0f;
    float roll_acc = 0.0f;
    if (abs(y) < acc_vector)
    {
        pitch_acc = asin((float)y / acc_vector) * 57.296;
    }

    if (abs(x) < acc_vector)
    {
        roll_acc = asin((float)x / acc_vector) * -57.296;
    }

    pitch_acc -= 1.0; // 1.0;
    roll_acc -= -1.0; // -1.0;

    m_attitude.rpy.pitch = m_attitude.rpy.pitch * 0.9995 + pitch_acc * 0.0005;
    m_attitude.rpy.roll = m_attitude.rpy.roll * 0.9995 + roll_acc * 0.0005;
    m_attitude.rpy.yaw = m_yaw;

    m_pitch_adjust = m_attitude.rpy.pitch * 15.0;
    m_roll_adjust = m_attitude.rpy.roll * 15.0;
}

void imu::calibrate(int num_samples)
{
    m_axis_cal.axis[0] = 0;
    m_axis_cal.axis[1] = 0;
    m_axis_cal.axis[2] = 0;

    for (int cal_int = 0; cal_int < num_samples; cal_int++)
    {
        //    if (cal_int % 15 == 0)digitalWriteFast(led, !digitalReadFast(led));
        int raw_axis[6] = {0};
        getRawImuData(raw_axis);
        m_axis_cal.axis[0] += raw_axis[1];
        m_axis_cal.axis[0] += raw_axis[2];
        m_axis_cal.axis[0] += raw_axis[3];
        delay(300);
    }
    m_axis_cal.axis[0] /= num_samples;
    m_axis_cal.axis[0] /= num_samples;
    m_axis_cal.axis[0] /= num_samples;
}