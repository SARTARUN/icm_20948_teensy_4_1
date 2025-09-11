#include "imu_ros.h"
#include "ahrs.h"
#include <string.h>
#include <stdlib.h>

// Global variables definition
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_publisher_t publisher;
rcl_publisher_t tf_publisher; 
sensor_msgs__msg__Imu imu_msg;
tf2_msgs__msg__TFMessage tf_msg;
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
  
   // Create IMU publisher
  ret = rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    ROS_TOPIC_NAME);
  if (ret != RCL_RET_OK) {
    Serial.println("Failed to initialize publisher");
    return false;
  }
  // Create TF publisher
  ret = rclc_publisher_init_default(
    &tf_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage),
    "/tf");
  if (ret != RCL_RET_OK) {
    Serial.println("Failed to initialize TF publisher");
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

  // Initialize TF message
  tf2_msgs__msg__TFMessage__init(&tf_msg);
  tf_msg.transforms.capacity = 1;
  tf_msg.transforms.size = 1;
  tf_msg.transforms.data = (geometry_msgs__msg__TransformStamped*)malloc(sizeof(geometry_msgs__msg__TransformStamped));
  
  if (tf_msg.transforms.data == NULL) {
    Serial.println("Failed to allocate memory for TF transforms");
    return false;
  }
  
  geometry_msgs__msg__TransformStamped__init(&tf_msg.transforms.data[0]);
  
  // Set TF frame IDs
  const char* parent_frame = BASE_FRAME_ID;
  const char* child_frame = IMU_FRAME_ID;
  
  tf_msg.transforms.data[0].header.frame_id.data = (char*)malloc(strlen(parent_frame) + 1);
  tf_msg.transforms.data[0].child_frame_id.data = (char*)malloc(strlen(child_frame) + 1);
  
  if (tf_msg.transforms.data[0].header.frame_id.data == NULL || 
      tf_msg.transforms.data[0].child_frame_id.data == NULL) {
    Serial.println("Failed to allocate memory for TF frame IDs");
    return false;
  }
  
  strcpy(tf_msg.transforms.data[0].header.frame_id.data, parent_frame);
  tf_msg.transforms.data[0].header.frame_id.size = strlen(parent_frame);
  tf_msg.transforms.data[0].header.frame_id.capacity = tf_msg.transforms.data[0].header.frame_id.size + 1;
  
  strcpy(tf_msg.transforms.data[0].child_frame_id.data, child_frame);
  tf_msg.transforms.data[0].child_frame_id.size = strlen(child_frame);
  tf_msg.transforms.data[0].child_frame_id.capacity = tf_msg.transforms.data[0].child_frame_id.size + 1;
  
  // Set static transform (IMU position relative to base_link)
  // Adjust these values based on where your IMU is mounted
  tf_msg.transforms.data[0].transform.translation.x = 0.0;
  tf_msg.transforms.data[0].transform.translation.y = 0.0;
  tf_msg.transforms.data[0].transform.translation.z = 0.0;
  tf_msg.transforms.data[0].transform.rotation.x = 0.0;
  tf_msg.transforms.data[0].transform.rotation.y = 0.0;
  tf_msg.transforms.data[0].transform.rotation.z = 0.0;
  tf_msg.transforms.data[0].transform.rotation.w = 1.0;

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

  if (tf_msg.transforms.data != NULL) {
    if (tf_msg.transforms.data[0].header.frame_id.data != NULL) {
      free(tf_msg.transforms.data[0].header.frame_id.data);
    }
    if (tf_msg.transforms.data[0].child_frame_id.data != NULL) {
      free(tf_msg.transforms.data[0].child_frame_id.data);
    }
    free(tf_msg.transforms.data);
    tf_msg.transforms.data = NULL;
  }
  
  rcl_ret_t ret;
  
  ret = rcl_publisher_fini(&publisher, &node);
  if (ret != RCL_RET_OK) {
    Serial.println("Warning: Failed to finalize publisher");
  }


  ret = rcl_publisher_fini(&tf_publisher, &node);
  if (ret != RCL_RET_OK) {
    Serial.println("Warning: Failed to finalize TF publisher");
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
  static unsigned long last_time = 0;
  static float Gxyz[3], Axyz[3], Mxyz[3];
  
  unsigned long current_time = micros();
  float deltat = (current_time - last_time) * 1.0e-6;
  last_time = current_time;

  // Set timestamp
  uint32_t current_millis = millis();
  imu_msg.header.stamp.sec = current_millis / 1000;
  imu_msg.header.stamp.nanosec = (current_millis % 1000) * 1000000;

  // Get scaled IMU data and run AHRS filter
  get_scaled_IMU(sensor, Gxyz, Axyz, Mxyz);
  
  if (deltat > 0) {
    MahonyQuaternionUpdate(Axyz[0], Axyz[1], Axyz[2], 
                          Gxyz[0], Gxyz[1], Gxyz[2],
                          Mxyz[0], Mxyz[1], Mxyz[2], deltat);
  }

  // Populate imu_msg with AHRS-filtered data (published to "imu/imu_data")
  // Set processed acceleration (calibrated and normalized)
  imu_msg.linear_acceleration.x = Axyz[0] * 9.80665; // Convert to m/s^2
  imu_msg.linear_acceleration.y = Axyz[1] * 9.80665;
  imu_msg.linear_acceleration.z = Axyz[2] * 9.80665;
  
  // Set processed angular velocity (calibrated)
  imu_msg.angular_velocity.x = Gxyz[0];
  imu_msg.angular_velocity.y = Gxyz[1];
  imu_msg.angular_velocity.z = Gxyz[2];
  
  // Set quaternion orientation from AHRS filter
  imu_msg.orientation.w = q[0];
  imu_msg.orientation.x = q[1];
  imu_msg.orientation.y = q[2];
  imu_msg.orientation.z = q[3];

  // Set covariance matrices
  for(int i = 0; i < 9; i++) {
    imu_msg.linear_acceleration_covariance[i] = 0.0;
    imu_msg.angular_velocity_covariance[i] = 0.0;
    imu_msg.orientation_covariance[i] = 0.0;
  }
  
  // Set diagonal covariance values for AHRS-filtered data
  imu_msg.linear_acceleration_covariance[0] = 0.005;
  imu_msg.linear_acceleration_covariance[4] = 0.005;
  imu_msg.linear_acceleration_covariance[8] = 0.005;
  imu_msg.angular_velocity_covariance[0] = 0.005;
  imu_msg.angular_velocity_covariance[4] = 0.005;
  imu_msg.angular_velocity_covariance[8] = 0.005;
  imu_msg.orientation_covariance[0] = 0.01;
  imu_msg.orientation_covariance[4] = 0.01;
  imu_msg.orientation_covariance[8] = 0.01;
}

// Function to get Euler angles from quaternion (for debugging)
void getEulerAngles(float* yaw_out, float* pitch_out, float* roll_out) {
  quaternion_to_euler(yaw_out, pitch_out, roll_out);
}

// Function to publish IMU data
void publishIMUData()
{
  if (state == AGENT_CONNECTED) {
    rcl_ret_t ret = rcl_publish(&publisher, &imu_msg, NULL);
    if (ret != RCL_RET_OK) {
      Serial.println("Failed to publish IMU data");
    }
  }
}

// Function to publish TF data
void publishTFData()
{
  if (state == AGENT_CONNECTED) {
    // Update timestamp
    uint32_t current_millis = millis();
    tf_msg.transforms.data[0].header.stamp.sec = current_millis / 1000;
    tf_msg.transforms.data[0].header.stamp.nanosec = (current_millis % 1000) * 1000000;
    
    rcl_ret_t ret = rcl_publish(&tf_publisher, &tf_msg, NULL);
    if (ret != RCL_RET_OK) {
      Serial.println("Failed to publish TF data");
    }
  }
}
