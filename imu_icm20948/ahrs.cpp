#include "ahrs.h"
#include <Arduino.h>

// Calibration values - UPDATE THESE WITH YOUR CALIBRATION DATA
float G_offset[3] = {-59.9, -3.1, 7.2};

float A_B[3] = {-472.32, -93.47, 214.8};

float A_Ainv[3][3] = {
  {0.06192, 0.00109, -0.0001},
  {0.00109, 0.05765, 0.00129},
  {-0.0001, 0.00129, 0.0602}
};

float M_B[3] = {-81.46, 100.42, -279.41};

float M_Ainv[3][3] = {
  {2.76839, -0.00048, 0.00619},
  {-0.00048, 2.84903, 0.02794},
  {0.00619, 0.02794, 2.59648}
};

float declination = 4.109; // Local magnetic declination in degrees

// Global variables
float q[4] = {1.0, 0.0, 0.0, 0.0}; // Quaternion
float yaw = 0.0, pitch = 0.0, roll = 0.0; // Euler angles

// Vector math functions
float vector_dot(float a[3], float b[3])
{
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

void vector_normalize(float a[3])
{
  float mag = sqrt(vector_dot(a, a));
  a[0] /= mag;
  a[1] /= mag;
  a[2] /= mag;
}

// Function to subtract offsets and apply scale/correction matrices to IMU data
#ifdef USE_SPI
void get_scaled_IMU(ICM_20948_SPI* sensor, float Gxyz[3], float Axyz[3], float Mxyz[3])
#else
void get_scaled_IMU(ICM_20948_I2C* sensor, float Gxyz[3], float Axyz[3], float Mxyz[3])
#endif
{
  byte i;
  float temp[3];

  Gxyz[0] = GSCALE * (sensor->agmt.gyr.axes.x - G_offset[0]);
  Gxyz[1] = GSCALE * (sensor->agmt.gyr.axes.y - G_offset[1]);
  Gxyz[2] = GSCALE * (sensor->agmt.gyr.axes.z - G_offset[2]);

  Axyz[0] = sensor->agmt.acc.axes.x;
  Axyz[1] = sensor->agmt.acc.axes.y;
  Axyz[2] = sensor->agmt.acc.axes.z;
  Mxyz[0] = sensor->agmt.mag.axes.x;
  Mxyz[1] = sensor->agmt.mag.axes.y;
  Mxyz[2] = sensor->agmt.mag.axes.z;

  // Apply accel offsets and scale factors
  for (i = 0; i < 3; i++) temp[i] = (Axyz[i] - A_B[i]);
  Axyz[0] = A_Ainv[0][0] * temp[0] + A_Ainv[0][1] * temp[1] + A_Ainv[0][2] * temp[2];
  Axyz[1] = A_Ainv[1][0] * temp[0] + A_Ainv[1][1] * temp[1] + A_Ainv[1][2] * temp[2];
  Axyz[2] = A_Ainv[2][0] * temp[0] + A_Ainv[2][1] * temp[1] + A_Ainv[2][2] * temp[2];
  vector_normalize(Axyz);

  // Apply mag offsets and scale factors
  for (i = 0; i < 3; i++) temp[i] = (Mxyz[i] - M_B[i]);
  Mxyz[0] = M_Ainv[0][0] * temp[0] + M_Ainv[0][1] * temp[1] + M_Ainv[0][2] * temp[2];
  Mxyz[1] = M_Ainv[1][0] * temp[0] + M_Ainv[1][1] * temp[1] + M_Ainv[1][2] * temp[2];
  Mxyz[2] = M_Ainv[2][0] * temp[0] + M_Ainv[2][1] * temp[1] + M_Ainv[2][2] * temp[2];
  vector_normalize(Mxyz);

  // Reconcile magnetometer and accelerometer axes
  Mxyz[1] = -Mxyz[1];
  Mxyz[2] = -Mxyz[2];
}

// Mahony AHRS filter
void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, 
                           float mx, float my, float mz, float deltat)
{
  static float eInt[3] = {0.0, 0.0, 0.0};
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];
  float norm;
  float hx, hy, hz;
  float ux, uy, uz, wx, wy, wz;
  float ex, ey, ez;

  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q1q4 = q1 * q4;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q2q4 = q2 * q4;
  float q3q3 = q3 * q3;
  float q3q4 = q3 * q4;
  float q4q4 = q4 * q4;

  hx = ay * mz - az * my;
  hy = az * mx - ax * mz;
  hz = ax * my - ay * mx;
  norm = sqrt(hx * hx + hy * hy + hz * hz);
  if (norm == 0.0f) return;

  norm = 1.0f / norm;
  hx *= norm;
  hy *= norm;
  hz *= norm;

  ux = 2.0f * (q2q4 - q1q3);
  uy = 2.0f * (q1q2 + q3q4);
  uz = q1q1 - q2q2 - q3q3 + q4q4;

  wx = 2.0f * (q2q3 + q1q4);
  wy = q1q1 - q2q2 + q3q3 - q4q4;
  wz = 2.0f * (q3q4 - q1q2);

  ex = (ay * uz - az * uy) + (hy * wz - hz * wy);
  ey = (az * ux - ax * uz) + (hz * wx - hx * wz);
  ez = (ax * uy - ay * ux) + (hx * wy - hy * wx);

  if (Ki > 0.0f) {
    eInt[0] += ex;
    eInt[1] += ey;
    eInt[2] += ez;
    gx += Ki * eInt[0];
    gy += Ki * eInt[1];
    gz += Ki * eInt[2];
  }

  gx = gx + Kp * ex;
  gy = gy + Kp * ey;
  gz = gz + Kp * ez;

  gx = gx * (0.5 * deltat);
  gy = gy * (0.5 * deltat);
  gz = gz * (0.5 * deltat);
  float qa = q1;
  float qb = q2;
  float qc = q3;
  q1 += (-qb * gx - qc * gy - q4 * gz);
  q2 += (qa * gx + qc * gz - q4 * gy);
  q3 += (qa * gy - qb * gz + q4 * gx);
  q4 += (qa * gz + qb * gy - qc * gx);

  norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
  norm = 1.0f / norm;
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;
}

void quaternion_to_euler(float* yaw_out, float* pitch_out, float* roll_out)
{
  roll = atan2((q[0] * q[1] + q[2] * q[3]), 0.5 - (q[1] * q[1] + q[2] * q[2]));
  pitch = asin(2.0 * (q[0] * q[2] - q[1] * q[3]));
  yaw = atan2((q[1] * q[2] + q[0] * q[3]), 0.5 - (q[2] * q[2] + q[3] * q[3]));
  
  yaw *= 180.0 / PI;
  pitch *= 180.0 / PI;
  roll *= 180.0 / PI;

  yaw = -(yaw + declination);
  if (yaw < 0) yaw += 360.0;
  if (yaw >= 360.0) yaw -= 360.0;

  *yaw_out = yaw;
  *pitch_out = pitch;
  *roll_out = roll;
}