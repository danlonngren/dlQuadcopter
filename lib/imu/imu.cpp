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
    
    // Setup MPU9250
    writeRegister(m_imuAddress, PWR_MGMT_1, 0x00);      // activate
    writeRegister(m_imuAddress, GYRO_CONFIG, 0x08);     // Gyro 500dps 65.5
    writeRegister(m_imuAddress, ACCEL_CONFIG, 0x10);    // Accel 8g 4096
    writeRegister(m_imuAddress, MPU_CONFIG, 0x00);      // Set Digital Low Pass Filter to ~43Hz
    
    writeRegister(m_imuAddress, USER_CTRL, 0x20); 
    writeRegister(m_imuAddress, I2C_MST_CTRL, 0x0D); // I2C Master mode, 400kHz


    // Setup magnetometer
    writeRegister(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down
    writeRegister(AK8963_ADDRESS, AK8963_CNTL, 0x16); // Set to 16 bit mode
}

const imuData& Imu::updateAttitude()
{
    static unsigned long lastTime = millis();
    imuData prevAccelData = m_accelData;

    // Get raw gyro and accel data
    updateGyroAccelData();
    updateMagnetometerData();

    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;
    lastTime = now;

    for (int i = 0; i < 3; i++)
    {
        m_accelData.axis[i] = lowPassFilter(m_accelData.axis[i], prevAccelData.axis[i], 0.999f);    
        m_gyroData.axis[i]  = highPassFilter(m_gyroData.axis[i], m_prevGyroOutput[0].axis[i], m_prevGyroOutput[0].axis[i], 0.9f);
        m_prevGyroOutput[1].axis[i] = m_prevGyroOutput[0].axis[i];
        m_prevGyroOutput[0].axis[i] = m_gyroData.axis[i];
    }
    
    // Adjust for calibration
    m_gyroData.axis[0] -= m_axisCalibration.axis[0];                          
    m_gyroData.axis[1] -= m_axisCalibration.axis[1];                        
    m_gyroData.axis[2] -= m_axisCalibration.axis[2];                          

    float pitchAccel = calculatePitchAccel(m_accelData);
    float rollAccel  = calculateRollAccel(m_accelData);

    // Calc tilt compensation
    float sinRoll = sin(rollAccel);
    float sinPitch = sin(pitchAccel);
    float cosRoll = cos(rollAccel);
    float cosPitch = cos(pitchAccel);

    float mxh = m_magData.xyz.x * cosPitch + m_magData.xyz.z * sinPitch;
    float myh = m_magData.xyz.x * sinRoll * sinPitch + m_magData.xyz.y * cosRoll - m_magData.xyz.z * sinRoll * cosPitch;
    float magYaw = atan2(-myh, mxh);

#if 1
    // Kalman filter for pitch and roll
    m_attitude.rpy.pitch = kalmanPitch.update(pitchAccel, m_gyroData.axis[0], dt);
    m_attitude.rpy.roll  = kalmanRoll.update(rollAccel, m_gyroData.axis[1], dt);
    m_attitude.rpy.yaw  = kalmanYaw.update(magYaw, m_gyroData.axis[2], dt);
#else
    float alpha = 0.97f;
    m_attitude.rpy.pitch = alpha * (m_attitude.rpy.pitch + m_gyroData.xyz.x * dt) + (1 - alpha) * pitchAccel;
    m_attitude.rpy.roll  = alpha * (m_attitude.rpy.roll - m_gyroData.xyz.y * dt) + (1 - alpha) * rollAccel;
    m_attitude.rpy.yaw = m_gyroData->rpy.yaw;
#endif

    return m_attitude;
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

///////////////////////////////////////////////////////////////////////////
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

void Imu::updateMagnetometerData()
{
    int16_t rawMagnetometerData[3];
    getMagnetometerData(rawMagnetometerData);
    m_magData.xyz.x = ((float)rawMagnetometerData[0]) / 0.15f; // 0.15uT/LSB
    m_magData.xyz.y = ((float)rawMagnetometerData[1]) / 0.15f; // 0.15uT/LSB
    m_magData.xyz.z = ((float)rawMagnetometerData[2]) / 0.15f; // 0.15uT/LSB
}

void Imu::getMagnetometerData(int16_t *rawMagnetometerData)
{
    uint8_t rawDataOut[7];
    // Get raw magnetometer data
    readAK8963Registers(AK8963_ST1, rawDataOut, 7);
    rawMagnetometerData[0] = (int16_t)rawDataOut[0] << 8 | (int16_t)rawDataOut[1];    // Mag x
    rawMagnetometerData[1] = (int16_t)rawDataOut[2] << 8 | (int16_t)rawDataOut[3];    // Mag y
    rawMagnetometerData[2] = (int16_t)rawDataOut[4] << 8 | (int16_t)rawDataOut[5];    // Mag z
}

float Imu::calculatePitchAccel(const imuData& a)
{
    return atan2(a.xyz.y, sqrt((a.xyz.x * a.xyz.x) + (a.xyz.z * a.xyz.z))) * 180.0f / PI;
}

float Imu::calculateRollAccel(const imuData& a)
{
    return atan2(-a.xyz.x, a.xyz.z) * 180.0f / PI;
}

///////////////////////////////////////////////////////////////////////////
void Imu::writeAK8963Register(uint8_t reg, uint8_t value)
{
    writeRegister(AK8963_ADDRESS, I2C_SLV0_ADDR, AK8963_ADDRESS);
    writeRegister(AK8963_ADDRESS, I2C_SLV0_REG, reg);
    writeRegister(AK8963_ADDRESS, I2C_SLV0_CTRL, 0x81); // 1 byte read
    writeRegister(AK8963_ADDRESS, 0x63, value);
}

void Imu::readAK8963Registers(uint8_t reg, uint8_t *value, uint8_t count)
{
    writeRegister(AK8963_ADDRESS, I2C_SLV0_ADDR, AK8963_ADDRESS);
    writeRegister(AK8963_ADDRESS, I2C_SLV0_REG, reg);
    writeRegister(AK8963_ADDRESS, I2C_SLV0_CTRL, 0x80 | count); // 1 byte read
    readRegister(AK8963_ADDRESS, EXT_SENS_DATA_00, value, count);
}

///////////////////////////////////////////////////////////////////////////
void Imu::writeRegister(uint8_t addr, uint8_t reg, uint8_t value)
{
    m_wire->beginTransmission(m_imuAddress);
    m_wire->write(reg);
    m_wire->write(value);
    m_wire->endTransmission(true);
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