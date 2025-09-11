#ifndef AHRS_H
#define AHRS_H

#include "ICM_20948.h"
#include "imu_config.h"

// AHRS functions
#ifdef USE_SPI
void get_scaled_IMU(ICM_20948_SPI* sensor, float Gxyz[3], float Axyz[3], float Mxyz[3]);
#else
void get_scaled_IMU(ICM_20948_I2C* sensor, float Gxyz[3], float Axyz[3], float Mxyz[3]);
#endif
void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, 
                           float mx, float my, float mz, float deltat);
void quaternion_to_euler(float* yaw, float* pitch, float* roll);

// Vector math functions
float vector_dot(float a[3], float b[3]);
void vector_normalize(float a[3]);

// Global variables
extern float q[4];
extern float yaw, pitch, roll;

#endif // AHRS_H