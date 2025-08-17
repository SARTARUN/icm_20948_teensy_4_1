#ifndef IMU_CONFIG_H
#define IMU_CONFIG_H

// Suppress micro-ROS architecture warning for Teensy
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"

// ICM-20948 Configuration
#define USE_SPI       // Uncomment this to use SPI

#define SERIAL_PORT Serial

#define SPI_PORT SPI // Your desired SPI port.       Used only when "USE_SPI" is defined
#define CS_PIN 10     // Which pin you connect CS to. Used only when "USE_SPI" is defined

#define WIRE_PORT Wire // Your desired Wire port.      Used when "USE_SPI" is not defined
// The value of the last bit of the I2C address.
// On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper is closed the value becomes 0
#define AD0_VAL 1

// micro-ROS Configuration
#define LED_PIN 13
#define IMU_FRAME_ID "imu_link"
#define ROS_NODE_NAME "imu_publisher"
#define ROS_TOPIC_NAME "imu/data"

// Timing Configuration
#define AGENT_PING_INTERVAL_MS 500
#define CONNECTION_CHECK_INTERVAL_MS 200
#define STATUS_PRINT_INTERVAL_MS 1000

#pragma GCC diagnostic pop

#endif // IMU_CONFIG_H
