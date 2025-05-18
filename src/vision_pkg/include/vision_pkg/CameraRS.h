#ifndef RSCAMERA_H
#define RSCAMERA_H

#include <vector>
#include <cstring>
#include <chrono>
#include <thread>
#include <opencv2/opencv.hpp>
#include <opencv4/opencv2/core/cuda.hpp>
#include <opencv4/opencv2/cudaarithm.hpp>
#include <opencv4/opencv2/cudaimgproc.hpp>
#include <opencv4/opencv2/cudafilters.hpp>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "librealsense2/rs.hpp"

/**
 * @class MultiCameraNode
 * @brief Handles a single realsense camera (by serial ID) and two Web Cameras.
 *        Realsense camera provides both RGB and Depth frames.
 ******************************************************************************/
class MultiCameraNode : public rclcpp::Node
{
public:
  /**
   * @brief MultiCameraNode is the main constructor of the MultiCameraNode class.
   */
  MultiCameraNode();

private:
  /* Constants */
  // const int POS_THRESHOLD = 250;
  // const int NEG_THRESHOLD = 800;
  // const int GROUND_DEPTH_MM = 700;
  
  /* Realsense2 Config */
  rs2::context ctx;

  /* Realsense2 Pipelines */
  rs2::pipeline pipeline_;
  rs2::pipeline pipeline2_;

  /* Filter Objects*/
  rs2::decimation_filter deci_;
  rs2::spatial_filter spat_;
  rs2::temporal_filter temp_;

  /* Video Capture Streams: Webcam */
  cv::VideoCapture cap_rgb1_;
  cv::VideoCapture cap_rgb2_;
  // rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr edge_cam1_pub_;

  /* Publisher Objects */
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr d455_cam1_pub_;      // RGB D455 Data
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr d455_cam2_edge_pub_; // Edge Detection Data
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr rgb_cam1_pub_;       // RGB Webcam Data
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr rgb_cam2_pub_;       // RGB Webcam Data
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr L_obstacle_detection_pub_;         // Left Obstacle Detected
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr R_obstacle_detection_pub_;         // Right Obstacle Detected
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr depth_detection_pub_;           // Depth Distance Center Average

  /* Timer Object */
  rclcpp::TimerBase::SharedPtr timer_;

  /**
   * @brief Callback to timer object. Callback polls the Realsense pipeline for available frams, avoiding locking the thread.
   *        If a frame is successfully polled, extracts both the depth and the RGB frmes. Publishes frames upon successful retrieval.
   * @param None
   *******************************************************/
  void timer_callback();

  // Use poll_for_frames to avoid blocking and stalling pipeline

  /**
   * @brief Publishes a realsense frame to the passed publisher.
   * @param color_frame rs2::frame passed by reference. The color_frame to be sent.
   * @param pub a rclcpp::publisher to a Compressed Image.
   *******************************************************/
  void publish_realsense_image(rs2::frame &color_frame, rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub);

  /**
   * @brief Takes a cv::VideoCapture by reference and reads a frame. Frame is encoded and sent along message topic.
   * @param cap cv::VideoCapture reference object.
   * @param pub sensor_msgs::msg::CompressedImage.
   */
  void publish_rgb_camera(cv::VideoCapture &cap, rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub);

  void publish_canny_from_realsense(rs2::frame &color_frame, rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub);

  /**
   * Function: average_depth
   * @brief Get the average depth pixel value across a rectangular region in the depth camera. Does not provide bounds checking.
   *
   * @param depth The depth frame passed by reference to the function call
   * @param x_bounds The lower and the upper bounds in the x-direction
   * @param y_bounds The lower and the upper bounds in the y-direction
   * @param increment The value to increment by between pixels
   * @returns double The average depth value across a region
   **********************************************************************/
  double average_depth(const rs2::depth_frame &depth, std::pair<int, int> x_bounds, std::pair<int, int> y_bounds, int increment = 1);

  /**
   * TODO: If possible, try to modularize the obstacle detection callback function.
   *  TODO: Aim to have the function length extend no further than 60 lines of code.
   */
  void obstacle_detection_callback(const rs2::frame &depth_frame, rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_left,
                                   rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_right, rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_depth);
  /**
   * Function: DetectLR
   * @brief 
   * 
   * @param
   */
  void DetectLR( const rs2::depth_frame&, int&, int&, int&, int& );
};
#endif