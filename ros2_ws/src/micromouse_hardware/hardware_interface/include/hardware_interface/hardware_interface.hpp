#ifndef RATZE_HARDWARE_INTERFACE__RATZE_HARDWARE_INTERFACE_HPP_
#define RATZE_HARDWARE_INTERFACE__RATZE_HARDWARE_INTERFACE_HPP_

#include <chrono>
#include <string.h>
#include <sstream>
#include <memory>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <vector>
#include <libserial/SerialPort.h>  // Add LibSerial header

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_msgs/msg/char.hpp"

namespace ratze_hardware_interface
{

class RatzeHardwareInterface : public rclcpp::Node
{
public:
  RatzeHardwareInterface();
  ~RatzeHardwareInterface();

private:
  // ROS-related members
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr heading_pub_;
  std::vector<rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr> tof_pubs_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr encoder_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Char>::SharedPtr user_command_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> laser_transform_broadcaster_;
  
  // Serial communication
  std::unique_ptr<LibSerial::SerialPort> serial_port_;  // LibSerial object
  std::string port_;
  int baudrate_;
  std::atomic<bool> connected_;
  std::thread read_thread_;
  std::atomic<bool> running_;

  // Sensor data storage
  mutable std::mutex data_mutex_;
  float heading_ = 0.0;
  float roll_ = 0.0;
  float pitch_ = 0.0;
  float yaw_ = 0.0;
  float gyro_x_ = 0.0;
  float gyro_y_ = 0.0;
  float gyro_z_ = 0.0;
  float accel_x_ = 0.0;
  float accel_y_ = 0.0;
  float accel_z_ = 0.0;
  int sys_calib_ = 0;
  int gyro_calib_ = 0;
  int accel_calib_ = 0;
  int mag_calib_ = 0;
  std::vector<uint16_t> tof_distances_;
  int32_t encoder1_ = 0;
  int32_t encoder2_ = 0;
  int32_t encoder3_ = 0;
  int32_t encoder4_ = 0;
  
  nav_msgs::msg::Path path_msg_;

  // Odometry calculation
  double x_ = 0.0, y_ = 0.0, odom_yaw_ = 0.0;
  double last_encoder_[4] = {0.0, 0.0, 0.0, 0.0};
  bool encoder_initialized_ = false;
  rclcpp::Time last_odom_time_;
  
  // Constants
  const double WHEEL_RADIUS = 0.035;  // 3.5cm radius
  const double TICKS_PER_REVOLUTION = 100;  // encoder ticks per wheel revolution
  const double WHEEL_BASE = 0.145;  // distance between wheels

  // Serial connection methods
  bool connect();
  void disconnect();
  bool sendCommand(char cmd, int value = 0);
  bool waitForAck(char cmd, int timeout_ms = 1000);
  std::string readLine(int timeout_ms = 100);  // Updated to use LibSerial timeout
  void readThread();
  
  // Command methods
  bool moveForward(int speed);
  bool moveBackward(int speed);
  bool turnLeft(int speed);
  bool turnRight(int speed);
  bool stop();
  bool setSpeed(int speed);
  bool resetEncoders();
  bool calibrateHeading();
  bool requestSensorData();
  
  // Data parsing methods
  bool parseImuData(const std::string& line);
  bool parseTofData(const std::string& line);
  bool parseEncoderData(const std::string& line);
  
  // ROS publishing methods
  void publishImuData();
  void publishTofData();
  void publishEncoderData();
  void publishOdometry(const rclcpp::Time& current_time, double distance, double delta_theta);
  void publishPath();
  void publishLaserScan(const std::vector<uint16_t>& distances);
  void publishLaserTransform();

  
  // ROS callback methods
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void userCommandCallback(const std_msgs::msg::Char::SharedPtr msg);
  void timerCallback();
};

}  // namespace ratze_hardware_interface

#endif  // RATZE_HARDWARE_INTERFACE__RATZE_HARDWARE_INTERFACE_HPP_
