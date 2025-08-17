/****************************************************************
 * IMU ROS2 Publisher
 * ICM 20948 Arduino Library with micro-ROS integration
 * Publishes 9-axis IMU data to ROS2 with robust connection handling
 * 
 * Based on SparkFun ICM-20948 Example
 * Original Creation Date: April 17 2019
 * Modified for ROS2 integration: August 8 2025
 *
 * NOTES:
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
    
    // Populate IMU message with sensor data
    populateIMUMessage(&myICM);
    
    // Publish IMU message
    rcl_ret_t ret = rcl_publish(&publisher, &imu_msg, NULL);
    if (ret != RCL_RET_OK) {
      SERIAL_PORT.println("Failed to publish IMU message");
    }
    
    printScaledAGMT(&myICM); // This function takes into account the scale settings from when the measurement was made to calculate the values with units
  }
  else if (state != AGENT_CONNECTED)
  {
    EXECUTE_EVERY_N_MS(STATUS_PRINT_INTERVAL_MS, SERIAL_PORT.println("Waiting for micro-ROS agent connection..."););
  }
} 