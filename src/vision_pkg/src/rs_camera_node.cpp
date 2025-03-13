#include    <list>
#include    <iomanip>
#include    <thread>
#include    "rclcpp/rclcpp.hpp"
#include    "librealsense2/rs.hpp"
#include    "std_msgs/msg/string.hpp"
#include    "sensor_msgs/msg/compressed_image.hpp"


/*******************************************************************************
Class: Camera
    Description: 
        Camera Class handles all instances of Intel Realsense Cameras
        and publishes RGB, Depth, and IMU measurements to the below
        topics:
            - /camera/rgb/image_raw
            - /camera/depth/image_raw
            - /camera/imu/data
        The Camera Class exposes the following services
            - NaN at the moment
        And the following parameters
            - NaN at the moment
        
        Public Functions:
            - Camera() : Constructor


*******************************************************************************/
class Camera : public rclcpp::Node {
public:

    /****************************************
    Constructor: Camera()
        Parameters: None
        Description: Constructor for Camera Class
    *****************************************/
    Camera() : Node("camera_node") {
        RCLCPP_WARN( this->get_logger(), "Enumerating Cameras...");
        (void)enumerate_cameras();
        (void)init_publishers();
        (void)init_timers();


        // Initialize all cameras
        for( auto device : this->devices ) {
            rs2::pipeline pipe = init_camera(device);
            // while (true) {
                // (void)camera_runtime( pipe );
            // }
        }
        return;
    }
    
private:
    ////////////////////////////////////////
    // Variable Declarations
    //      - ctx: Realsense Context
    //      - devices: List of Realsense Devices
    //      - timer_: Timer Object
    //      - _string_pub_: String Publisher
    rs2::context ctx;
    std::list<rs2::device> devices;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _string_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr _rgb_camera_pub_;

    /****************************************
    Function: init_publishers()
        Parameters: None
        Description: Initialize all publishers
    *****************************************/
    void init_publishers(std::list<std::string> topics = {}) {
        RCLCPP_WARN( this->get_logger(), "Initializing Publishers...");

        // Initialize String Publisher
        this->_string_pub_ = this->create_publisher<std_msgs::msg::String>(
            "camera_info", 10
        );

        // TODO: Implement RGB Image Publisher (Use Compressed Image Topic)
        this->_rgb_camera_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
            "camera/rgb/image_raw", 10
        );

        // TODO: Implement Depth Image Publisher (Use Image Topic)

        // TODO: Implemenet IMU Data Publisher (Use geometry_msgs::msg::AccelStamped Topic)
        return;
    }

    rs2::pipeline init_camera( rs2::device& device ) {
        RCLCPP_WARN( this->get_logger(), "Initializing Camera %s", device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) );
        rs2::config cfg;
        rs2::pipeline pipe;
        try {
            cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
            cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
            cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F, 62);
            cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F, 200);
            pipe.start(cfg);    
        } catch( const rs2::error& e ) {
            RCLCPP_ERROR( this->get_logger(), "Error: %s", e.what() );
        }
        return pipe;
    }

    /****************************************
    Function: init_timers()
        Parameters: None
        Description: Initialize all timers
    *****************************************/
    void init_timers() {
        RCLCPP_WARN( this->get_logger(), "Initializing Timer...");
        this->timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&Camera::timer_callback, this)
        );
        return;
    }

    /*******************************************************************************
    Function: enumerate_cameras()
        Parameters: None
        Description: 
            Enumerate all connected cameras. Discovered cameras will be 
            added to the devices list. The model and serial number of each camera
            will be printed to the console.
    ********************************************************************************/
    void enumerate_cameras() {
        int num_cameras = this->ctx.query_devices().size();
        RCLCPP_WARN( this->get_logger(), "Number of Cameras: %d", num_cameras);
        for( auto camera : this->ctx.query_devices() ) {
            this->devices.push_back(camera);
            RCLCPP_INFO( this->get_logger(), "Model: %s", camera.get_info(RS2_CAMERA_INFO_NAME));
            RCLCPP_INFO( this->get_logger(), "Serial Number: %s", camera.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
        }
        return;
    }

    /****************************************
    Function: timer_callback()
        Parameters: None
        Description: Timer Callback Function
    *****************************************/
    void timer_callback() {
        std_msgs::msg::String msg;
        msg.data = "Hello World!";
        this->_string_pub_->publish(msg);
        RCLCPP_INFO( this->get_logger(), "Timer Callback");
    }

    void camera_runtime( rs2::pipeline& pipe ) {
        rs2::frameset frames = pipe.wait_for_frames();
        

        rs2::frame color_frame = frames.get_color_frame();
        
        // Publish RGB Image

        // Publish Depth Image

        // Publish IMU Data
    }
};


int main() {
    rclcpp::init(0, nullptr);
    rclcpp::spin(std::make_shared<Camera>());
    rclcpp::shutdown();
    return 0;
}