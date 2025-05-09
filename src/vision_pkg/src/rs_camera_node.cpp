// rs_camera_node.cpp
// Optimized for low latency and high performance on Jetson

#include <list>
#include <vector>
#include <iostream>
#include <cstring>
#include <chrono>
#include <thread>
#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "librealsense2/rs.hpp"
#include "SparkMax.hpp"

using namespace std::chrono_literals;

class MultiCameraNode : public rclcpp::Node {
public:
  MultiCameraNode() : Node("multi_camera_node") {
    RCLCPP_INFO(this->get_logger(), "Multi-camera node startup.");

    // Known D455 serials
    std::string serial1 = "318122303486";
    //std::string serial2 = "308222300472";

    // Initialize RealSense pipeline for D455 #1
    {
      rs2::pipeline pipeline;
      rs2::config cfg;
      cfg.enable_device(serial1);
      cfg.enable_stream(RS2_STREAM_COLOR, 424, 240, RS2_FORMAT_BGR8, 15); // Lower resolution + FPS
      cfg.enable_stream(RS2_STREAM_DEPTH, RS2_FORMAT_Z16, 10);
      try {
        pipeline.start(cfg);
        pipelines_.push_back(pipeline);
        RCLCPP_INFO(this->get_logger(), "Started D455 pipeline on device %s", serial1.c_str());
      } catch (const rs2::error &e) {
        RCLCPP_ERROR(this->get_logger(), "Error starting pipeline for D455 #1: %s", e.what());
      }
    }
    
    // Open USB RGB cameras explicitly
    cap_rgb1_.open("/dev/video0");
    cap_rgb2_.open("/dev/video3");
    
    if (!cap_rgb1_.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open USB RGB camera 1 (/dev/video0).\n");
    }
    if (!cap_rgb2_.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open USB RGB camera 2 (/dev/video2).\n");
    }

    // Publishers
    d455_cam1_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("rs_node/camera1/compressed_video", 10);
    rgb_cam1_pub_  = this->create_publisher<sensor_msgs::msg::CompressedImage>("rgb_cam1/compressed", 10);
    rgb_cam2_pub_  = this->create_publisher<sensor_msgs::msg::CompressedImage>("rgb_cam2/compressed", 10);
    L_obstacle_detection_pub_ = this->create_publisher<std_msgs::msg::Bool>("obstacle_detection/left", 10);
    R_obstacle_detection_pub_ = this->create_publisher<std_msgs::msg::Bool>("obstacle_detection/right", 10);

    // Timer
    timer_ = this->create_wall_timer(66ms, std::bind(&MultiCameraNode::timer_callback, this)); // ~15 FPS
  }

private:
  rs2::context ctx;
  std::vector<rs2::pipeline> pipelines_;
  cv::VideoCapture cap_rgb1_;
  cv::VideoCapture cap_rgb2_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr d455_cam1_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr rgb_cam1_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr rgb_cam2_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr L_obstacle_detection_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr R_obstacle_detection_pub_;
  rclcpp::TimerBase::SharedPtr timer_;


  void timer_callback() {
    // Use poll_for_frames to avoid blocking and stalling pipeline
    if (pipelines_.size() >= 1) {
      rs2::frameset frames1;
      if (pipelines_[0].poll_for_frames(&frames1)) {
        rs2::frame color_frame1 = frames1.get_color_frame();
        rs2::depth_frame depth_frame1 = frames1.get_depth_frame();
        if (color_frame1) publish_realsense_image(color_frame1, d455_cam1_pub_);
        if (depth_frame1 && depth_frame1.get_data()) obstacle_detection_callback(depth_frame1, L_obstacle_detection_pub_, R_obstacle_detection_pub_);
      } else {
        RCLCPP_WARN(this->get_logger(), "No frame available from D455 camera 1.");
      }
    }

    publish_rgb_camera(cap_rgb1_, rgb_cam1_pub_);
    publish_rgb_camera(cap_rgb2_, rgb_cam2_pub_);
  }

  void publish_realsense_image(rs2::frame & color_frame, rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub) {
    std::vector<uchar> buffer;
    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 40}; // Lower quality for faster encoding
    sensor_msgs::msg::CompressedImage msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "camera_rgb_optical_frame";
    msg.format = "jpeg";

    cv::Mat frame(cv::Size(color_frame.as<rs2::video_frame>().get_width(),
                           color_frame.as<rs2::video_frame>().get_height()),
                  CV_8UC3,
                  (void*)color_frame.get_data(),
                  cv::Mat::AUTO_STEP);
    if (!cv::imencode(".jpg", frame, buffer, params)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to encode RealSense image to JPEG.");
      return;
    }
    msg.data = buffer;
    pub->publish(msg);
  }

  void publish_rgb_camera(cv::VideoCapture & cap, rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub) {
    cv::Mat frame;
    if (!cap.read(frame)) {
      //RCLCPP_WARN(this->get_logger(), "Failed to capture frame from USB RGB camera.");
      return;
    }
    std::vector<uchar> buffer;
    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 40};
    sensor_msgs::msg::CompressedImage msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "rgb_camera_frame";
    msg.format = "jpeg";

    if (!cv::imencode(".jpg", frame, buffer, params)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to encode USB RGB image to JPEG.");
      return;
    }
    msg.data = buffer;
    pub->publish(msg);
  }

  void obstacle_detection_callback(const rs2::frame& depth_frame, rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_left, 
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_right){
    const int GROUND_DEPTH_MM = 700;
    const int POS_THRESHOLD = 250; 
    const int NEG_THRESHOLD = 800;
    const int OBSTACLE_THRESHOLD = 200;

    const rs2::depth_frame depth = depth_frame.as<rs2::depth_frame>();
    const int width = depth.get_width();
    const int height = depth.get_height(); //848 x 480

    int pos_count = 0;
    int neg_count = 0;
    int left_count = 0;
    int right_count = 0;

    for (int y = height / 2; y < height; y += 5) {
      for (int x = 0; x < width / 2; x += 5) {
        float dist_m = depth.get_distance(x, y);
        if (dist_m <= 0.0f) continue;

        int depth_mm = static_cast<int>(dist_m * 1000);
        int delta = GROUND_DEPTH_MM - depth_mm;

        if (delta > POS_THRESHOLD){ 
          pos_count++;
          right_count++;
        }
        else if (-delta > NEG_THRESHOLD){
          neg_count++;
          right_count++;
        } 
      }
    }//right side detection

    for (int y = height / 2; y < height; y += 5) {
      for (int x = width / 2; x < width; x += 5) {
        float dist_m = depth.get_distance(x, y);
        if (dist_m <= 0.0f) continue;

        int depth_mm = static_cast<int>(dist_m * 1000);
        int delta = GROUND_DEPTH_MM - depth_mm;

        if (delta > POS_THRESHOLD){ 
          pos_count++;
          left_count++;
        }
        else if (-delta > NEG_THRESHOLD){
          neg_count++;
          left_count++;
        } 
      }
    }//right side detection

    std_msgs::msg::Bool left_msg;
    std_msgs::msg::Bool right_msg;

    if (pos_count > OBSTACLE_THRESHOLD || neg_count > OBSTACLE_THRESHOLD){
      //RCLCPP_INFO(this->get_logger(), "Obstacle Detected :3 Positive = %i, Negative = %i", pos_count, neg_count);
      if (left_count >= right_count){
        //RCLCPP_INFO(this->get_logger(), "Obstacle on left side");
        left_msg.data = true;
        right_msg.data = false;
      }
      else {
        //RCLCPP_INFO(this->get_logger(), "Obstacle on right side");
        left_msg.data = false;
        right_msg.data = true;
      }
    }
    else {
      //RCLCPP_INFO(this->get_logger(), "Obstacle Not Detected :( Positive = %i, Negative = %i", pos_count, neg_count);
      left_msg.data = false;
      right_msg.data = false;
    }
    pub_left->publish(left_msg);
    pub_right->publish(right_msg);
  }
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MultiCameraNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
