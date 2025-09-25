#include <micromouse_command/commands.hpp>

namespace micromouse_command
{

    UserCommandNode::UserCommandNode() : Node("user_command_node")
    {
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        // start thread to handle user input
        std::thread input_thread(&UserCommandNode::handleUserInput, this);
        input_thread.detach();
    }

    UserCommandNode::~UserCommandNode()
    {
    }

    void UserCommandNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received cmd_vel: linear.x=%f, linear.y=%f, angular.z=%f",
                    msg->linear.x, msg->linear.y, msg->angular.z);
    }

    void UserCommandNode::handleUserInput()
    {
        std::string input;
        while (rclcpp::ok())
        {
            std::cout << "Enter command (F, B, L, R, S): ";
            std::getline(std::cin, input);
            RCLCPP_INFO(this->get_logger(), "User input: %s", input.c_str());

            auto msg = geometry_msgs::msg::Twist();

            if (input == "F")
            {
                msg.linear.x = 0.5;
            }
            else if (input == "B")
            {
                msg.linear.x = -0.5;
            }
            else if (input == "L")
            {
                msg.angular.z = 0.5;
            }
            else if (input == "R")
            {
                msg.angular.z = -0.5;
            }
            else if (input == "S")
            {
                msg.linear.x = 0.0;
                msg.angular.z = 0.0;
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Unknown command");
                continue;
            }
            RCLCPP_INFO(this->get_logger(), "Publishing command: %s", input.c_str());
            cmd_vel_pub_->publish(msg);
        }
    }

} // namespace micromouse_command

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<micromouse_command::UserCommandNode>());
    rclcpp::shutdown();
    return 0;
}