/*
 * MPU9250.c
 *
 *  Created on: Feb 25, 2025
 *      Author: jasev
 */

#include "MPU9250.h"

/// @brief Set the IMU address, check for connection, reset IMU, and set full range scale.
/// @param I2Cx Pointer to I2C structure config.
/// @param addr Hex address based on AD0 pin - 0x68 low or 0x69 high.
/// @param aScale Set accelerometer full scale range: 0 for ±2g, 1 for ±4g, 2 for ±8g, and 3 for ±16g.
/// @param gScale Set gyroscope full scale range: 0 for ±250°/s, 1 for ±500°/s, 2 for ±1000°/s, and 3 for ±2000°/s.
/// @param tau Set tau value for the complementary filter (typically 0.98).
/// @param dt Set sampling rate in seconds determined by the timer interrupt.

// Data variables

 int16_t rawData_ax, rawData_ay, rawData_az, rawData_gx, rawData_gy, rawData_gz;

 float sensorData_ax, sensorData_ay, sensorData_az, sensorData_gx, sensorData_gy, sensorData_gz;

 float gyroCalx, gyroCaly, gyroCalz;

 float attitude_r, attitude_p, attitude_y;


// Variables
 uint8_t _addr;
 float _dt, _tau;
float aScaleFactor, gScaleFactor;

uint8_t MPU_begin(I2C_HandleTypeDef *I2Cx, uint8_t addr, uint8_t aScale, uint8_t gScale, float tau, float dt)
{
    // Save values
    _addr = addr << 1;
    _tau = tau;
    _dt = dt;

    // Initialize variables
    uint8_t check;
    uint8_t select;

    // Confirm device
    HAL_I2C_Mem_Read(I2Cx, _addr, WHO_AM_I, 1, &check, 1, I2C_TIMOUT_MS);

    // TODO: If 9250 or 6050 fails could it trigger the opposite check???
    if ((check == WHO_AM_I_9250_ANS) || (check == WHO_AM_I_6050_ANS))
    {
        // Startup / reset the sensor
        select = 0x00;
        HAL_I2C_Mem_Write(I2Cx, _addr, PWR_MGMT_1, 1, &select, 1, I2C_TIMOUT_MS);

        // Set the full scale ranges
        MPU_writeAccFullScaleRange(I2Cx, aScale);
        MPU_writeGyroFullScaleRange(I2Cx, gScale);

        return 1;
    }
    else
    {
        return 0;
    }
}

/// @brief Set the accelerometer full scale range.
/// @param I2Cx Pointer to I2C structure config.
/// @param aScale Set 0 for ±2g, 1 for ±4g, 2 for ±8g, and 3 for ±16g.
void MPU_writeAccFullScaleRange(I2C_HandleTypeDef *I2Cx, uint8_t aScale)
{
    // Variable init
    uint8_t select;

    // Set the value
    switch (aScale)
    {
    case AFSR_2G:
        aScaleFactor = 16384.0;
        select = 0x00;
        HAL_I2C_Mem_Write(I2Cx, _addr, ACCEL_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    case AFSR_4G:
        aScaleFactor = 8192.0;
        select = 0x08;
        HAL_I2C_Mem_Write(I2Cx, _addr, ACCEL_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    case AFSR_8G:
        aScaleFactor = 4096.0;
        select = 0x10;
        HAL_I2C_Mem_Write(I2Cx, _addr, ACCEL_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    case AFSR_16G:
        aScaleFactor = 2048.0;
        select = 0x18;
        HAL_I2C_Mem_Write(I2Cx, _addr, ACCEL_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    default:
        aScaleFactor = 8192.0;
        select = 0x08;
        HAL_I2C_Mem_Write(I2Cx, _addr, ACCEL_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    }
}

/// @brief Set the gyroscope full scale range.
/// @param I2Cx Pointer to I2C structure config.
/// @param gScale Set 0 for ±250°/s, 1 for ±500°/s, 2 for ±1000°/s, and 3 for ±2000°/s.
void MPU_writeGyroFullScaleRange(I2C_HandleTypeDef *I2Cx, uint8_t gScale)
{
    // Variable init
    uint8_t select;

    // Set the value
    switch (gScale)
    {
    case GFSR_250DPS:
        gScaleFactor = 131.0;
        select = 0x00;
        HAL_I2C_Mem_Write(I2Cx, _addr, GYRO_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    case GFSR_500DPS:
        gScaleFactor = 65.5;
        select = 0x08;
        HAL_I2C_Mem_Write(I2Cx, _addr, GYRO_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    case GFSR_1000DPS:
        gScaleFactor = 32.8;
        select = 0x10;
        HAL_I2C_Mem_Write(I2Cx, _addr, GYRO_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    case GFSR_2000DPS:
        gScaleFactor = 16.4;
        select = 0x18;
        HAL_I2C_Mem_Write(I2Cx, _addr, GYRO_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    default:
        gScaleFactor = 65.5;
        select = 0x08;
        HAL_I2C_Mem_Write(I2Cx, _addr, GYRO_CONFIG, 1, &select, 1, I2C_TIMOUT_MS);
        break;
    }
}

/// @brief Read raw data from IMU.
/// @param I2Cx Pointer to I2C structure config.
void MPU_readRawData(I2C_HandleTypeDef *I2Cx)
{
    // Init buffer
    uint8_t buf[14];

    // Subroutine for reading the raw data
    HAL_I2C_Mem_Read(I2Cx, _addr, ACCEL_XOUT_H, 1, buf, 14, I2C_TIMOUT_MS);

    // Bit shift the data
    rawData_ax = buf[0] << 8 | buf[1];
    rawData_ay = buf[2] << 8 | buf[3];
    rawData_az = buf[4] << 8 | buf[5];
    // temperature = buf[6] << 8 | buf[7];
    rawData_gx = buf[8] << 8 | buf[9];
    rawData_gy = buf[10] << 8 | buf[11];
    rawData_gz = buf[12] << 8 | buf[13];
}

/// @brief Find offsets for each axis of gyroscope.
/// @param I2Cx Pointer to I2C structure config.
/// @param numCalPoints Number of data points to average.
void MPU_calibrateGyro(I2C_HandleTypeDef *I2Cx, uint16_t numCalPoints)
{
    // Init
    int32_t x = 0;
    int32_t y = 0;
    int32_t z = 0;

    // Zero guard
    if (numCalPoints == 0)
    {
        numCalPoints = 1;
    }

    // Save specified number of points
    for (uint16_t ii = 0; ii < numCalPoints; ii++)
    {
        MPU_readRawData(I2Cx);
        x += rawData_gx;
        y += rawData_gy;
        z += rawData_gz;
        HAL_Delay(3);
    }

    // Average the saved data points to find the gyroscope offset
    gyroCalx = (float)x / (float)numCalPoints;
    gyroCaly = (float)y / (float)numCalPoints;
    gyroCalz = (float)z / (float)numCalPoints;
}

/// @brief Calculate the real world sensor values.
/// @param I2Cx Pointer to I2C structure config.
void MPU_readProcessedData(I2C_HandleTypeDef *I2Cx)
{
    // Get raw values from the IMU
    MPU_readRawData(I2Cx);

    // Convert accelerometer values to g's
    sensorData_ax = rawData_ax / aScaleFactor;
    sensorData_ay = rawData_ay / aScaleFactor;
    sensorData_az = rawData_az / aScaleFactor;

    // Compensate for gyro offset
    sensorData_gx = rawData_gx - gyroCalx;
    sensorData_gy = rawData_gy - gyroCaly;
    sensorData_gz = rawData_gz - gyroCalz;

    // Convert gyro values to deg/s
    sensorData_gx /= gScaleFactor;
    sensorData_gy /= gScaleFactor;
    sensorData_gz /= gScaleFactor;
}

/// @brief Calculate the attitude of the sensor in degrees using a complementary filter.
/// @param I2Cx Pointer to I2C structure config.
void MPU_calcAttitude(I2C_HandleTypeDef *I2Cx)
{
    // Read processed data
    MPU_readProcessedData(I2Cx);

    // Complementary filter
    float accelPitch = atan2(sensorData_ay, sensorData_az) * RAD2DEG;
    float accelRoll = atan2(sensorData_ax, sensorData_az) * RAD2DEG;

    attitude_r = _tau * (attitude_r - sensorData_gy * _dt) + (1 - _tau) * accelRoll;
    attitude_p = _tau * (attitude_p + sensorData_gx * _dt) + (1 - _tau) * accelPitch;
    attitude_y += sensorData_gz * _dt;
}

// Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
// (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
// which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute
// device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
// The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
// but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!

//void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
//{
//	float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
//	float norm;
//	float hx, hy, _2bx, _2bz;
//	float s1, s2, s3, s4;
//	float qDot1, qDot2, qDot3, qDot4;
//
//	// Auxiliary variables to avoid repeated arithmetic
//	float _2q1mx;
//	float _2q1my;
//	float _2q1mz;
//	float _2q2mx;
//	float _4bx;
//	float _4bz;
//	float _2q1 = 2.0f * q1;
//	float _2q2 = 2.0f * q2;
//	float _2q3 = 2.0f * q3;
//	float _2q4 = 2.0f * q4;
//	float _2q1q3 = 2.0f * q1 * q3;
//	float _2q3q4 = 2.0f * q3 * q4;
//	float q1q1 = q1 * q1;
//	float q1q2 = q1 * q2;
//	float q1q3 = q1 * q3;
//	float q1q4 = q1 * q4;
//	float q2q2 = q2 * q2;
//	float q2q3 = q2 * q3;
//	float q2q4 = q2 * q4;
//	float q3q3 = q3 * q3;
//	float q3q4 = q3 * q4;
//	float q4q4 = q4 * q4;
//
//	// Normalise accelerometer measurement
//	norm = sqrt(ax * ax + ay * ay + az * az);
//	if (norm == 0.0f) return; // handle NaN
//	norm = 1.0f/norm;
//	ax *= norm;
//	ay *= norm;
//	az *= norm;
//
//	// Normalise magnetometer measurement
//	norm = sqrt(mx * mx + my * my + mz * mz);
//	if (norm == 0.0f) return; // handle NaN
//	norm = 1.0f/norm;
//	mx *= norm;
//	my *= norm;
//	mz *= norm;
//
//	// Reference direction of Earth's magnetic field
//	_2q1mx = 2.0f * q1 * mx;
//	_2q1my = 2.0f * q1 * my;
//	_2q1mz = 2.0f * q1 * mz;
//	_2q2mx = 2.0f * q2 * mx;
//	hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
//	hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
//	_2bx = sqrt(hx * hx + hy * hy);
//	_2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
//	_4bx = 2.0f * _2bx;
//	_4bz = 2.0f * _2bz;
//
//	// Gradient decent algorithm corrective step
//	s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
//	s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
//	s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
//	s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
//	norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
//	norm = 1.0f/norm;
//	s1 *= norm;
//	s2 *= norm;
//	s3 *= norm;
//	s4 *= norm;
//
//	// Compute rate of change of quaternion
//	qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
//	qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
//	qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
//	qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;
//
//	// Integrate to yield quaternion
//	q1 += qDot1 * deltat;
//	q2 += qDot2 * deltat;
//	q3 += qDot3 * deltat;
//	q4 += qDot4 * deltat;
//	norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
//	norm = 1.0f/norm;
//	q[0] = q1 * norm;
//	q[1] = q2 * norm;
//	q[2] = q3 * norm;
//	q[3] = q4 * norm;
//
//}

