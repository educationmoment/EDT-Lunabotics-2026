#include "rclcpp/rclcpp.hpp"
#include "librealsense2/rs.hpp"

int main() {
    rclcpp::init(0, nullptr);
    rclcpp::spin(std::make_shared<rclcpp::Node>("rs_camera_node"));
    rclcpp::shutdown();
    return 0;
}