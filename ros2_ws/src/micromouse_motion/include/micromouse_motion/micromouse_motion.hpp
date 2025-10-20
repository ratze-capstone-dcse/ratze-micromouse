#ifndef RATZE_MOTION__RATZE_MOTION_HPP_
#define RATZE_MOTION__RATZE_MOTION_HPP_

#include <chrono>
#include <string.h>
#include <sstream>
#include <memory>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <vector>

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
#include "yaml-cpp/yaml.h"

namespace ratze_motion
{

// Structure to represent a 2D point
struct Point2D
{
  double x;
  double y;
};

class RatzeMotion : public rclcpp::Node
{
public:
  RatzeMotion();
  ~RatzeMotion();

private:
  // Helper methods
  bool loadWaypointsFromConfig();
  void visualizeWaypoints();
  Point2D getCurrentPose();
  int computeTargetWaypoint();
  double computeSteeringAngle(const Point2D& target_point);
  char selectDiscreteAction(double steering_angle);
  void publishCommand(char command);
  void controlLoop();

  // Subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // Publishers
  rclcpp::Publisher<std_msgs::msg::Char>::SharedPtr command_pub_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // State variables
  std::vector<Point2D> waypoints_;
  int current_waypoint_index_;
  Point2D current_pose_;
  double current_heading_;
  double lookahead_distance_;
  bool is_turning_;
  char last_command_;
};

}  // namespace ratze_motion

#endif  // RATZE_MOTION__RATZE_MOTION_HPP_
