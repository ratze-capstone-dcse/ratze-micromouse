#ifndef MICROMOUSE_COMMAND__COMMANDS_HPP_
#define MICROMOUSE_COMMAND__COMMANDS_HPP_

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <iostream>
#include "std_msgs/msg/char.hpp"

namespace micromouse_command
{
    class UserCommandNode : public rclcpp::Node
    {
        public:
        UserCommandNode();
        ~UserCommandNode();

        void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);


    private:
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_; // publisher for cmd_vel
        rclcpp::Publisher<std_msgs::msg::Char>::SharedPtr char_pub_; // publisher for char commands
        void handleUserInput();
    };
}


#endif // MICROMOUSE_COMMAND__COMMANDS_HPP_