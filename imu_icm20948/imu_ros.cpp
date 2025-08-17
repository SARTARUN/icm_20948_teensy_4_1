#include "imu_ros.h"
#include <string.h>
#include <stdlib.h>

// Global variables definition
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_publisher_t publisher;
sensor_msgs__msg__Imu imu_msg;
rclc_executor_t executor;
enum states state;

bool create_entities()
{
  allocator = rcl_get_default_allocator();
  
  // Create init_options
  rcl_ret_t ret = rclc_support_init(&support, 0, NULL, &allocator);
  if (ret != RCL_RET_OK) {
    Serial.println("Failed to initialize support");
    return false;
  }
  
  // Create node
  ret = rclc_node_init_default(&node, ROS_NODE_NAME, "", &support);
  if (ret != RCL_RET_OK) {
    Serial.println("Failed to initialize node");
    return false;
  }
  
  // Create publisher
  ret = rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    ROS_TOPIC_NAME);
  if (ret != RCL_RET_OK) {
    Serial.println("Failed to initialize publisher");
    return false;
  }
    
  // Initialize IMU message
  sensor_msgs__msg__Imu__init(&imu_msg);
  
  // Set frame_id - allocate memory and copy string
  const char* frame_id = IMU_FRAME_ID;
  imu_msg.header.frame_id.data = (char*)malloc(strlen(frame_id) + 1);
  if (imu_msg.header.frame_id.data == NULL) {
    Serial.println("Failed to allocate memory for frame_id");
    return false;
  }
  strcpy(imu_msg.header.frame_id.data, frame_id);
  imu_msg.header.frame_id.size = strlen(frame_id);
  imu_msg.header.frame_id.capacity = imu_msg.header.frame_id.size + 1;
  
  // Create executor
  ret = rclc_executor_init(&executor, &support.context, 1, &allocator);
  if (ret != RCL_RET_OK) {
    Serial.println("Failed to initialize executor");
    return false;
  }
  
  Serial.println("ROS entities created successfully");
  return true;
}

void destroy_entities()
{
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  if (imu_msg.header.frame_id.data != NULL) {
    free(imu_msg.header.frame_id.data);
    imu_msg.header.frame_id.data = NULL;
  }
  
  rcl_ret_t ret;
  
  ret = rcl_publisher_fini(&publisher, &node);
  if (ret != RCL_RET_OK) {
    Serial.println("Warning: Failed to finalize publisher");
  }
  
  ret = rcl_node_fini(&node);
  if (ret != RCL_RET_OK) {
    Serial.println("Warning: Failed to finalize node");
  }
  
  ret = rclc_support_fini(&support);
  if (ret != RCL_RET_OK) {
    Serial.println("Warning: Failed to finalize support");
  }
}

void init_microros()
{
  // Initialize micro-ROS
  set_microros_transports();
  
  // Set initial state
  state = WAITING_AGENT;
  
  Serial.println("Setup complete, waiting for micro-ROS agent...");
}

void handle_microros_state_machine()
{
  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(AGENT_PING_INTERVAL_MS, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
                                          ? AGENT_AVAILABLE
                                          : WAITING_AGENT;);
      break;
    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        destroy_entities();
      };
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(CONNECTION_CHECK_INTERVAL_MS, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
                                          ? AGENT_CONNECTED
                                          : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
      }
      break;
    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;
    default:
      break;
  }

  // Update LED status
  if (state == AGENT_CONNECTED) {
    digitalWrite(LED_PIN, 1);
  } else {
    digitalWrite(LED_PIN, 0);
  }
}

// Function to populate IMU message with sensor data
#ifdef USE_SPI
void populateIMUMessage(ICM_20948_SPI *sensor)
{
#else
void populateIMUMessage(ICM_20948_I2C *sensor)
{
#endif
  // Set timestamp
  imu_msg.header.stamp.sec = millis() / 1000;
  imu_msg.header.stamp.nanosec = (millis() % 1000) * 1000000;
  
  // Convert accelerometer data from mg to m/s^2
  imu_msg.linear_acceleration.x = sensor->accX() * 0.00980665; // mg to m/s^2
  imu_msg.linear_acceleration.y = sensor->accY() * 0.00980665;
  imu_msg.linear_acceleration.z = sensor->accZ() * 0.00980665;
  
  // Convert gyroscope data from DPS to rad/s
  imu_msg.angular_velocity.x = sensor->gyrX() * 0.017453292; // DPS to rad/s
  imu_msg.angular_velocity.y = sensor->gyrY() * 0.017453292;
  imu_msg.angular_velocity.z = sensor->gyrZ() * 0.017453292;
  
  // Set covariance matrices (diagonal with small values for good sensors)
  for(int i = 0; i < 9; i++) {
    imu_msg.linear_acceleration_covariance[i] = 0.0;
    imu_msg.angular_velocity_covariance[i] = 0.0;
    imu_msg.orientation_covariance[i] = 0.0;
  }
  
  // Set diagonal covariance values
  imu_msg.linear_acceleration_covariance[0] = 0.01; // x
  imu_msg.linear_acceleration_covariance[4] = 0.01; // y  
  imu_msg.linear_acceleration_covariance[8] = 0.01; // z
  
  imu_msg.angular_velocity_covariance[0] = 0.01; // x
  imu_msg.angular_velocity_covariance[4] = 0.01; // y
  imu_msg.angular_velocity_covariance[8] = 0.01; // z
  
  // Set orientation covariance to -1 to indicate orientation is not available
  imu_msg.orientation_covariance[0] = -1.0;
  
  // Set orientation to zero (not calculated)
  imu_msg.orientation.x = 0.0;
  imu_msg.orientation.y = 0.0;
  imu_msg.orientation.z = 0.0;
  imu_msg.orientation.w = 1.0; // Valid quaternion
}
