#include    <list>
#include    <iomanip>
#include    "rclcpp/rclcpp.hpp"
#include    "opencv2/opencv.hpp"
#include    "librealsense2/rs.hpp"
#include    "std_msgs/msg/string.hpp"
#include    "geometry_msgs/msg/accel_stamped.hpp"
#include    "sensor_msgs/msg/compressed_image.hpp"
#include    "sensor_msgs/msg/image.hpp"


class CameraNode : public rclcpp::Node {
public:
    CameraNode() : Node("camera_node") {
        RCLCPP_INFO( this->get_logger(), "Hello World!");
    }


private:
    void init_camera();
};

void CameraNode::init_camera() {
    rs2::context ctx;
    rs2::pipeline pipe;
    
    return;
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin( std::make_shared<CameraNode>() );
    rclcpp::shutdown();
    return 0;
}