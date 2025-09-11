#ifndef IMU_ROS_H
#define IMU_ROS_H

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/transform_stamped.h>  
#include <tf2_msgs/msg/tf_message.h>  
#include "ICM_20948.h"
#include "imu_config.h"

#include "ahrs.h"
// micro-ROS state machine
enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
};

// Macro for timed execution
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)

// Global variables declaration
extern rcl_allocator_t allocator;
extern rclc_support_t support;
extern rcl_node_t node;
extern rcl_publisher_t publisher;
extern rcl_publisher_t tf_publisher;
extern sensor_msgs__msg__Imu imu_msg;
extern tf2_msgs__msg__TFMessage tf_msg; 
extern rclc_executor_t executor;
extern enum states state;

// Function declarations
bool create_entities();
void destroy_entities();
void init_microros();
void handle_microros_state_machine();
void publishIMUData();
void publishTFData();
void getEulerAngles(float* yaw_out, float* pitch_out, float* roll_out);

#ifdef USE_SPI
void populateIMUMessage(ICM_20948_SPI *sensor);
#else
void populateIMUMessage(ICM_20948_I2C *sensor);
#endif

#endif // IMU_ROS_H
