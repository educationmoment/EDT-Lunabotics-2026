// rs_camera_node.cpp

#include <vector>
#include <iostream>
#include <chrono>
#include <thread>
#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

using namespace std::chrono_literals;

class MultiCameraNode : public rclcpp::Node {
public:
  MultiCameraNode() : Node("multi_camera_node") {
    RCLCPP_INFO(this->get_logger(), "Camera node starting up...");

    // search up 15 video devices. this should allow for each one to be found- can take up to five seconds
    int found = 0;
    for (int i = 0; i < 15; ++i) {
      cv::VideoCapture cap(i);
      if (cap.isOpened()) {
        RCLCPP_INFO(this->get_logger(), "Opened /dev/video%d", i);
        switch (found) {
          case 0: cap_usb1_ = std::move(cap); break;
          case 1: cap_usb2_ = std::move(cap); break;
          case 2: cap_rs_ = std::move(cap); break;
        }
        found++;
      }
      if (found >= 3) break;
    }

    if (!cap_usb1_.isOpened()) RCLCPP_ERROR(this->get_logger(), "Could not open USB Webcam 1.");
    if (!cap_usb2_.isOpened()) RCLCPP_ERROR(this->get_logger(), "Could not open USB Webcam 2.");
    if (!cap_rs_.isOpened())   RCLCPP_ERROR(this->get_logger(), "Could not open RealSense V4L2 stream.");

    // sent publishers
    usb1_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("rgb_cam1/compressed", 10);
    usb2_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("rgb_cam2/compressed", 10);
    rs_pub_   = this->create_publisher<sensor_msgs::msg::CompressedImage>("rs_node/camera1/compressed_video", 10);

    // tmer loop for fps shit
    timer_ = this->create_wall_timer(33ms, std::bind(&MultiCameraNode::timer_callback, this)); // ~30 FPS -> to switch to 15, put it at 66ms, to 60, put it at 16ms
  }

private:
  cv::VideoCapture cap_usb1_;
  cv::VideoCapture cap_usb2_;
  cv::VideoCapture cap_rs_;

  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr usb1_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr usb2_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr rs_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  void timer_callback() {
    publish_camera(cap_usb1_, usb1_pub_, "usb1_frame");
    publish_camera(cap_usb2_, usb2_pub_, "usb2_frame");
    publish_camera(cap_rs_,   rs_pub_,   "rs_frame");
  }

  void publish_camera(cv::VideoCapture& cap,
                      rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub,
                      const std::string& frame_id) {
    if (!cap.isOpened()) return;

    cv::Mat frame;
    if (!cap.read(frame)) {
      RCLCPP_WARN(this->get_logger(), "Failed to read frame from device.");
      return;
    }

    std::vector<uchar> buffer;
    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 40};

    sensor_msgs::msg::CompressedImage msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = frame_id;
    msg.format = "jpeg";

    if (!cv::imencode(".jpg", frame, buffer, params)) {
      RCLCPP_ERROR(this->get_logger(), "JPEG encoding failed.");
      return;
    }

    msg.data = buffer;
    pub->publish(msg);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MultiCameraNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
