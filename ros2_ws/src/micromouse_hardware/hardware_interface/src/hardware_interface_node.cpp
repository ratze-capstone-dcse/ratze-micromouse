#include <hardware_interface/hardware_interface.hpp>

int main (int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<micromouse_hardware::RatzeHardwareInterface>();
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}