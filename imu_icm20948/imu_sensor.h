#ifndef IMU_SENSOR_H
#define IMU_SENSOR_H

#include "ICM_20948.h"
#include "imu_config.h"

// Global sensor object declaration
#ifdef USE_SPI
extern ICM_20948_SPI myICM;
#else
extern ICM_20948_I2C myICM;
#endif

// Function declarations
bool init_imu_sensor();
void printPaddedInt16b(int16_t val);
void printRawAGMT(ICM_20948_AGMT_t agmt);
void printFormattedFloat(float val, uint8_t leading, uint8_t decimals);

#ifdef USE_SPI
void printScaledAGMT(ICM_20948_SPI *sensor);
#else
void printScaledAGMT(ICM_20948_I2C *sensor);
#endif

#endif // IMU_SENSOR_H
