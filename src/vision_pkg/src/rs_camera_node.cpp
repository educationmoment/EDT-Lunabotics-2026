//rs_camera_node.cpp
//added low frame for d455s it works ok so fuck you

#include <list>
#include <vector>
#include <iostream>
#include <cstring>
#include <chrono>
#include <thread>
#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
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
    std::string serial2 = "308222300472";

    // Initialize RealSense pipeline for D455 #1
    {
      rs2::pipeline pipeline;
      rs2::config cfg;
      cfg.enable_device(serial1);
      cfg.enable_stream(RS2_STREAM_COLOR, 424, 240, RS2_FORMAT_BGR8, 15); // Lower resolution + FPS
      try {
        pipeline.start(cfg);
        pipelines_.push_back(pipeline);
        RCLCPP_INFO(this->get_logger(), "Started D455 pipeline on device %s", serial1.c_str());
      } catch (const rs2::error &e) {
        RCLCPP_ERROR(this->get_logger(), "Error starting pipeline for D455 #1: %s", e.what());
      }
    }

    // Initialize RealSense pipeline for D455 #2
    {
      rs2::pipeline pipeline;
      rs2::config cfg;
      cfg.enable_device(serial2);
      cfg.enable_stream(RS2_STREAM_COLOR, 424, 240, RS2_FORMAT_BGR8, 15);
      try {
        pipeline.start(cfg);
        pipelines_.push_back(pipeline);
        RCLCPP_INFO(this->get_logger(), "Started D455 pipeline on device %s", serial2.c_str());
      } catch (const rs2::error &e) {
        RCLCPP_ERROR(this->get_logger(), "Error starting pipeline for D455 #2: %s", e.what());
      }
    }

    // Open USB RGB cameras explicitly
    cap_rgb1_.open("/dev/video0");
    cap_rgb2_.open("/dev/video8");

    if (!cap_rgb1_.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open USB RGB camera 1 (/dev/video0).\n");
    }
    if (!cap_rgb2_.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open USB RGB camera 2 (/dev/video8).\n");
    }

    // Publishers
    d455_cam1_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("rs_node/camera1/compressed_video", 10);
    d455_cam2_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("rs_node/camera2/compressed_video", 10);
    rgb_cam1_pub_  = this->create_publisher<sensor_msgs::msg::CompressedImage>("rgb_cam1/compressed", 10);
    rgb_cam2_pub_  = this->create_publisher<sensor_msgs::msg::CompressedImage>("rgb_cam2/compressed", 10);

    // Timer
    timer_ = this->create_wall_timer(66ms, std::bind(&MultiCameraNode::timer_callback, this)); // ~15 FPS
  }

private:
  rs2::context ctx;
  std::vector<rs2::pipeline> pipelines_;
  cv::VideoCapture cap_rgb1_;
  cv::VideoCapture cap_rgb2_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr d455_cam1_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr d455_cam2_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr rgb_cam1_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr rgb_cam2_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  void timer_callback() {
    // Use poll_for_frames to avoid blocking and stalling pipeline
    if (pipelines_.size() >= 1) {
      rs2::frameset frames1;
      if (pipelines_[0].poll_for_frames(&frames1)) {
        rs2::frame color_frame1 = frames1.get_color_frame();
        if (color_frame1) publish_realsense_image(color_frame1, d455_cam1_pub_);
      } else {
        RCLCPP_WARN(this->get_logger(), "No frame available from D455 camera 1.");
      }
    }

    if (pipelines_.size() >= 2) {
      rs2::frameset frames2;
      if (pipelines_[1].poll_for_frames(&frames2)) {
        rs2::frame color_frame2 = frames2.get_color_frame();
        if (color_frame2) publish_realsense_image(color_frame2, d455_cam2_pub_);
      } else {
        RCLCPP_WARN(this->get_logger(), "No frame available from D455 camera 2.");
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
      RCLCPP_WARN(this->get_logger(), "Failed to capture frame from USB RGB camera.");
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
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MultiCameraNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
