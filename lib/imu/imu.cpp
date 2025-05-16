#include "imu.h"

// Arduino includes
#include <math.h>
#include <Arduino.h>

// Lib includes
#include "filters.h"
#include "MPU9250RegisterMap.h"

// Constants
#define MPU6050 0x68
// #define MPU6050 0xE8
// #define MPU6050 0x80
#define FILTER_SNAP 0.99f

const float IMU_GYRO_RES_500 = 65.5f;
const float IMU_ACC_RES_8 = 4096.0f;
const float IMU_DEG_TO_RAD = 0.0174533f;

Imu::Imu() :
    m_imuAddress(MPU6050),
    m_axisCalibration{0.0, 0.0, 0.0},
    m_attitude{0}
{
    m_wire = &Wire;
    m_wire->begin();
    m_wire->setClock(400000L);

    writeRegister(m_imuAddress, PWR_MGMT_1, 0x00); // activate
    writeRegister(m_imuAddress, GYRO_CONFIG, 0x08); // Gyro 500dps 65.5
    writeRegister(m_imuAddress, ACCEL_CONFIG, 0x10); // Accel 8g 4096
    writeRegister(m_imuAddress, 0x1A, 0x00); // Set Digital Low Pass Filter to ~43Hz

    for (int i = 0; i < 250; i++)
    {
        uint8_t out = 0;
        readRegister(i, WHO_AM_I_MPU9250, &out); // Accel 8g 4096
        Serial.print("MPU9250 WHO_AM_I: 0x");
        Serial.print(i, HEX);
        Serial.print(" = 0x");
        Serial.println(out, HEX);
    }
}

void Imu::updateAttitude()
{
    imu_data prevGyroInput = m_gyroData;
    imu_data oldAccelData = m_accelData;

    // Get raw gyro and accel data
    updateGyroAccelData();

    for (int i = 0; i < 3; i++)
    {
        m_accelData.axis[i] = lowPassFilter(m_accelData.axis[i], oldAccelData.axis[i], 0.999f);    
        m_gyroData.axis[i] = highPassFilter(m_gyroData.axis[i], prevGyroInput.axis[i], m_prevGyroOutput.axis[i], 0.9f);
        m_prevGyroOutput.axis[i] = m_gyroData.axis[i];
    }
    
    // Adjust for calibration
    m_gyroData.axis[0] -= m_axisCalibration.axis[0];                            
    m_gyroData.axis[1] -= m_axisCalibration.axis[1];                          
    m_gyroData.axis[2] -= m_axisCalibration.axis[2];                            

    unsigned long now = millis();
    float dt = (now - m_lastTime) / 1000.0;
    m_lastTime = now;

    calculateAngle(&m_gyroData, &m_accelData, dt);
}

void Imu::calculateAngle(imu_data *gyroData, imu_data *accelData, unsigned long dt)
{
    float x = accelData->xyz.x;
    float y = accelData->xyz.y;
    float z = accelData->xyz.z;

    float pitch_acc = atan2(y, sqrt((x * x) + (z * z))) * 180.0f / PI;
    float roll_acc = atan2(-x, z) * 180.0f / PI;

    float alpha = 0.97f;
    m_attitude.rpy.pitch = alpha * (m_attitude.rpy.pitch + gyroData->xyz.x * dt) + (1 - alpha) * pitch_acc;
    m_attitude.rpy.roll  = alpha * (m_attitude.rpy.roll - gyroData->xyz.y * dt) + (1 - alpha) * roll_acc;
    m_attitude.rpy.yaw = gyroData->rpy.yaw;
}

void Imu::updateGyroAccelData()
{
    int16_t dataOut[7];

    getRawGyroAccelData(dataOut);
    
    m_accelData.xyz.x = ((float)dataOut[0]) / IMU_ACC_RES_8;
    m_accelData.xyz.y = ((float)dataOut[1]) / IMU_ACC_RES_8;
    m_accelData.xyz.z = ((float)dataOut[2]) / IMU_ACC_RES_8;
    m_temperatureCelcius = ((uint16_t)dataOut[3] - 521) / 340 + 36.53; // Convert to Celsius
    m_gyroData.xyz.x = ((float)dataOut[4]) / IMU_GYRO_RES_500; // * IMU_DEG_TO_RAD;
    m_gyroData.xyz.y = ((float)dataOut[5]) / IMU_GYRO_RES_500; // * IMU_DEG_TO_RAD;
    m_gyroData.xyz.z = ((float)dataOut[6]) / IMU_GYRO_RES_500; // * IMU_DEG_TO_RAD;
}

void Imu::getRawGyroAccelData(int16_t *rawGyroAccelData)
{
    uint8_t rawDataOut[14];

    // Get raw gyro and accel data
    readRegister(m_imuAddress, ACCEL_XOUT_H, rawDataOut, 14);
    rawGyroAccelData[0] = (int16_t)rawDataOut[0] << 8 | (int16_t)rawDataOut[1];    // Accel x
    rawGyroAccelData[1] = (int16_t)rawDataOut[2] << 8 | (int16_t)rawDataOut[3];    // Accel y
    rawGyroAccelData[2] = (int16_t)rawDataOut[4] << 8 | (int16_t)rawDataOut[5];    // Accel z
    rawGyroAccelData[3] = (int16_t)rawDataOut[6] << 8 | (int16_t)rawDataOut[7];    // Temp
    rawGyroAccelData[4] = (int16_t)rawDataOut[8] << 8 | (int16_t)rawDataOut[9];    // Gyro x
    rawGyroAccelData[5] = (int16_t)rawDataOut[10] << 8 | (int16_t)rawDataOut[11];  // Gyro y
    rawGyroAccelData[6] = (int16_t)rawDataOut[12] << 8 | (int16_t)rawDataOut[13];  // Gyro z
}

void Imu::calibrate(int num_samples)
{
    m_axisCalibration.axis[0] = 0;
    m_axisCalibration.axis[1] = 0;
    m_axisCalibration.axis[2] = 0;

    for (int cal_int = 0; cal_int < num_samples; cal_int++)
    {
        int16_t data[7];
        getRawGyroAccelData(data);
        m_axisCalibration.axis[0] += data[0];
        m_axisCalibration.axis[1] += data[1];
        m_axisCalibration.axis[2] += data[2];
        delay(100);
    }
    m_axisCalibration.axis[0] /= num_samples;
    m_axisCalibration.axis[0] /= num_samples;
    m_axisCalibration.axis[0] /= num_samples;
}

void Imu::writeRegister(uint8_t addr, uint8_t reg, uint8_t value)
{
    m_wire->beginTransmission(m_imuAddress);
    m_wire->write(reg);
    m_wire->write(value);
    m_wire->endTransmission(true);
}

void Imu::readRegister(uint8_t addr, uint8_t reg, uint8_t *value)
{
    m_wire->beginTransmission(addr);
    m_wire->write(reg);
    m_wire->endTransmission(false);
    m_wire->requestFrom(addr, (uint8_t)1);
    if (m_wire->available()) *value = m_wire->read();
}

void Imu::readRegister(uint8_t addr, uint8_t reg, uint8_t *value, uint8_t length)
{
    m_wire->beginTransmission(m_imuAddress);
    m_wire->write(reg);
    m_wire->endTransmission(false);
    m_wire->requestFrom(m_imuAddress, length);
    uint8_t i = 0;
    while (m_wire->available())
    {
        value[i++] = m_wire->read();
    }
}