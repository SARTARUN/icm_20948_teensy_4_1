/****************************************************************
 * IMU ROS2 Publisher with AHRS Integration
 * ICM 20948 Arduino Library with micro-ROS and Mahony AHRS filter
 * 
 * Data Flow:
 * Raw sensor data → Calibration → Mahony AHRS filter → published to "imu/imu_data"
 * 
 * Published topic:
 * - "imu/imu_data": AHRS-filtered sensor data with quaternion orientation
 * 
 * Features:
 * - Robust ROS2 connection handling
 * - Mahony AHRS filter for orientation estimation
 * - Sensor calibration and bias correction
 * - Real-time Euler angle calculation (YPR)
 * 
 * Based on SparkFun ICM-20948 Example and Mahony AHRS filter
 * Original Creation Date: April 17 2019
 * Modified for ROS2 integration: August 8 2025
 * AHRS Integration: August 19 2025
 *
 * NOTES:
 * - Update calibration values in ahrs.cpp with your sensor-specific data
 * - Architecture warning for micro-ROS is expected on Teensy (but works fine)
 * - Library warnings from FNET/mbedTLS are harmless and can be ignored
 * - All RCL return values are now properly checked
 * 
 * Please see License.md for the license information.
 * Distributed as-is; no warranty is given.
 ***************************************************************/

#include "imu_config.h"
#include "imu_sensor.h"
#include "imu_ros.h"

void setup()
{
  SERIAL_PORT.begin(115200);
  while (!SERIAL_PORT)
  {
  };

  // Initialize LED pin
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Initialize IMU sensor
  init_imu_sensor();

  // Initialize micro-ROS
  init_microros();
}

void loop()
{
  // Handle micro-ROS state machine
  handle_microros_state_machine();

  // Handle IMU data reading and publishing
  if (state == AGENT_CONNECTED && myICM.dataReady())
  {
    myICM.getAGMT();         // The values are only updated when you call 'getAGMT'
    
    // Process IMU data: Raw → Calibration → Mahony Filter → IMU_Data
    populateIMUMessage(&myICM);
    
    // Publish IMU message
    publishIMUData();

    //Publish TF data
    publishTFData();

    // Print debug information
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 1000) { // Print every second
      printScaledAGMT(&myICM);
      
      // Print Euler angles from AHRS
      float ypr[3];
      getEulerAngles(&ypr[0], &ypr[1], &ypr[2]);
      SERIAL_PORT.print("YPR (degrees): ");
      SERIAL_PORT.print(ypr[0], 1);
      SERIAL_PORT.print(", ");
      SERIAL_PORT.print(ypr[1], 1);
      SERIAL_PORT.print(", ");
      SERIAL_PORT.println(ypr[2], 1);
      
      SERIAL_PORT.println("Data Flow: Raw → Calibration → Mahony Filter → imu/imu_data");
      
      lastPrint = millis();
    }
  }
  else if (state != AGENT_CONNECTED)
  {
    EXECUTE_EVERY_N_MS(STATUS_PRINT_INTERVAL_MS, SERIAL_PORT.println("Waiting for micro-ROS agent connection..."););
  }
} 