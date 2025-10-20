#include <micromouse_motion/micromouse_motion.hpp>
#include <cmath>
#include <iostream>

namespace ratze_motion {
    RatzeMotion::RatzeMotion() : Node("ratze_motion") {
        // Initialize parameters
        lookahead_distance_ = this->declare_parameter<double>("lookahead_distance", 0.3);
        current_waypoint_index_ = 0;
        current_pose_ = {0.0, 0.0};
        current_heading_ = 0.0;
        is_turning_ = false;
        last_command_ = 'F';  // Default to forward

        // Load waypoints
        if (!loadWaypointsFromConfig()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load waypoints from config file");
            rclcpp::shutdown();
            return;
        }

        // Create subscribers
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                // Extract position
                current_pose_.x = msg->pose.pose.position.x;
                current_pose_.y = msg->pose.pose.position.y;

                // Extract heading (yaw) from quaternion
                auto q = msg->pose.pose.orientation;
                double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
                double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
                current_heading_ = std::atan2(siny_cosp, cosy_cosp);  // in radians
            }
        );

        // Create publishers
        command_pub_ = this->create_publisher<std_msgs::msg::Char>("/ratze_command", 10);

        // Create timer for control loop
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),  // 10Hz
            std::bind(&RatzeMotion::controlLoop, this)
        );

        RCLCPP_INFO(this->get_logger(), "Ratze Motion node initialized");
    }

    RatzeMotion::~RatzeMotion() {}

    bool RatzeMotion::loadWaypointsFromConfig() {
        try {
            // Path to config file
            std::string config_path = "src/micromouse_motion/config/waypoints.yaml";
            YAML::Node config = YAML::LoadFile(config_path);

            if (config["waypoints"]) {
                auto waypoints_node = config["waypoints"];
                for (const auto& wp : waypoints_node) {
                    if (wp.size() >= 2) {
                        Point2D point;
                        point.x = wp[0].as<double>();
                        point.y = wp[1].as<double>();
                        waypoints_.push_back(point);
                    }
                }
                RCLCPP_INFO(this->get_logger(), "Loaded %zu waypoints", waypoints_.size());
                return !waypoints_.empty();
            }
            return false;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load waypoints: %s", e.what());
            return false;
        }
    }

    Point2D RatzeMotion::getCurrentPose() {
        return current_pose_;
    }

    int RatzeMotion::computeTargetWaypoint() {
        if (waypoints_.empty()) {
            return -1;
        }

        // If we've reached the end of the waypoints, stop
        if (current_waypoint_index_ >= static_cast<int>(waypoints_.size())) {
            return -1;
        }

        // Check if we've reached the current waypoint
        const auto& target = waypoints_[current_waypoint_index_];
        double dx = target.x - current_pose_.x;
        double dy = target.y - current_pose_.y;
        double distance = std::sqrt(dx*dx + dy*dy);

        // If we've reached the waypoint, move to the next one
        if (distance < 0.1) {  // Threshold for reaching a waypoint
            current_waypoint_index_++;
            RCLCPP_INFO(this->get_logger(), "Reached waypoint %d", current_waypoint_index_ - 1);
            
            // Check if we've reached the end of the waypoints
            if (current_waypoint_index_ >= static_cast<int>(waypoints_.size())) {
                RCLCPP_INFO(this->get_logger(), "Reached final waypoint");
                return -1;
            }
        }

        return current_waypoint_index_;
    }

    double RatzeMotion::computeSteeringAngle(const Point2D& target_point) {
        // Calculate vector to target
        double dx = target_point.x - current_pose_.x;
        double dy = target_point.y - current_pose_.y;
        
        // Calculate desired heading
        double desired_heading = std::atan2(dy, dx);
        
        // Calculate steering angle (error between desired and current heading)
        double steering_angle = desired_heading - current_heading_;
        
        // Normalize to [-π, π]
        while (steering_angle > M_PI) steering_angle -= 2.0 * M_PI;
        while (steering_angle < -M_PI) steering_angle += 2.0 * M_PI;
        
        return steering_angle;
    }

    char RatzeMotion::selectDiscreteAction(double steering_angle) {
        // Convert radians to degrees for easier thresholding
        double angle_degrees = steering_angle * 180.0 / M_PI;
        
        // Determine discrete action based on angle
        if (std::abs(angle_degrees) < 12.5) {
            // Almost straight ahead, go forward
            return 'F';
        } else if (angle_degrees >= 12.5 && angle_degrees < 35.0) {
            // 25 degree left turn
            return '<';
        } else if (angle_degrees <= -12.5 && angle_degrees > -35.0) {
            // 25 degree right turn
            return '>';
        } else if (angle_degrees >= 35.0 && angle_degrees < 67.5) {
            // 45 degree left turn
            return 'l';
        } else if (angle_degrees <= -35.0 && angle_degrees > -67.5) {
            // 45 degree right turn
            return 'r';
        } else if (angle_degrees >= 67.5) {
            // 90 degree left turn
            return 'L';
        } else {  // angle_degrees <= -67.5
            // 90 degree right turn
            return 'R';
        }
    }

    void RatzeMotion::publishCommand(char command) {
        std_msgs::msg::Char msg;
        msg.data = command;
        command_pub_->publish(msg);
        
        // Check if we're starting a turn
        if (command != 'F' && command != 'S' && last_command_ != command) {
            // First stop the robot before turning
            msg.data = 'S';
            command_pub_->publish(msg);
            RCLCPP_DEBUG(this->get_logger(), "Stopping before turn: %c", command);
        }
        
        last_command_ = command;
        
        // If turning, set flag to stop forward motion
        is_turning_ = (command != 'F' && command != 'S');
    }

    void RatzeMotion::controlLoop() {
        // Skip if no waypoints or not properly initialized
        if (waypoints_.empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "No waypoints available");
            return;
        }

        // Compute target waypoint
        int target_idx = computeTargetWaypoint();
        if (target_idx < 0) {
            // We've reached the end of the path
            publishCommand('S');  // Stop the robot when path is complete
            return;
        }

        const auto& target = waypoints_[target_idx];
        
        // Compute steering angle to the target waypoint
        double steering_angle = computeSteeringAngle(target);
        
        // Select discrete action based on the steering angle
        char action = selectDiscreteAction(steering_angle);
        
        // If currently turning and the previous command wasn't 'F', keep turning
        if (is_turning_) {
            // Continue with the current turn until we're aligned
            if (std::abs(steering_angle) < 0.05) {  // ~3 degrees threshold
                is_turning_ = false;
                publishCommand('F');  // Resume forward motion
            } else {
                publishCommand(last_command_);  // Continue turning
            }
        } else {
            // Normal operation
            publishCommand(action);
        }
        
        RCLCPP_DEBUG(this->get_logger(), "Target: (%.2f, %.2f), Steering: %.2f°, Action: %c", 
                    target.x, target.y, steering_angle * 180.0 / M_PI, action);
    }
}

int main (int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<ratze_motion::RatzeMotion>();
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}