# IMU ROS2 Publisher

A modular Arduino project that publishes ICM-20948 9-axis IMU data to ROS2 using micro-ROS with robust connection handling.

## Project Structure

```
imu_icm20948/
├── imu_icm20948.ino      # Main Arduino sketch
├── imu_config.h          # Configuration definitions
├── imu_sensor.h          # IMU sensor interface
├── imu_sensor.cpp        # IMU sensor implementation
├── imu_ros.h             # ROS2 interface
├── imu_ros.cpp           # ROS2 implementation
└── README.md             # This file
```

## File Descriptions

### `imu_config.h`
- Hardware configuration (SPI/I2C settings)
- Pin definitions (LED, CS, etc.)
- ROS2 configuration (node name, topic, frame_id)
- Timing constants

### `imu_sensor.h/cpp`
- ICM-20948 sensor initialization and management
- Data formatting and printing functions
- Hardware abstraction layer

### `imu_ros.h/cpp`
- micro-ROS state machine implementation
- ROS2 entity creation/destruction
- IMU message population and publishing
- Connection handling

### `imu_icm20948.ino`
- Main application entry point
- Setup and loop functions
- Coordinates between sensor and ROS modules

## Features

- ✅ **Modular Design** - Clean separation of concerns
- ✅ **Robust Connection** - Automatic reconnection handling
- ✅ **LED Status** - Visual connection indicator
- ✅ **Configurable** - Easy to modify settings
- ✅ **Production Ready** - Memory management and error handling

## Hardware Support

- **Board**: Teensy 4.1 (or other micro-ROS compatible boards)
- **Sensor**: SparkFun ICM-20948 9-axis IMU
- **Connection**: SPI or I2C (configurable)

## Dependencies

- SparkFun ICM-20948 Arduino Library
- micro_ros_arduino library

## Configuration

Edit `imu_config.h` to customize:
- Communication protocol (SPI/I2C)
- Pin assignments
- ROS2 node and topic names
- Timing parameters

## Usage

1. Install required libraries
2. Configure settings in `imu_config.h`
3. Upload to Teensy 4.1
4. Start micro-ROS agent: `ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0`
5. Monitor IMU data: `ros2 topic echo /imu/data`

## ROS2 Output

The node publishes `sensor_msgs/Imu` messages containing:
- Linear acceleration (m/s²)
- Angular velocity (rad/s)
- Proper timestamps and covariance matrices
- Frame ID: "imu_link"

## LED Status

- **OFF**: Waiting for micro-ROS agent
- **ON**: Connected and publishing data
