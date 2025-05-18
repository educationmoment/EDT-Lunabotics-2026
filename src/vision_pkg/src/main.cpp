// Optimized for low latency and high performance on Jetson
#define CUDA_IMPORTS true
#include <list>
#include <vector>
#include <iostream>
#include <cstring>
#include <chrono>
#include <thread>
#include <opencv2/opencv.hpp>

//
#if defined(CUDA_IMPORTS) && CUDA_IMPORTS == true
#include <opencv4/opencv2/core/cuda.hpp>
#include <opencv4/opencv2/cudaarithm.hpp>
#include <opencv4/opencv2/cudaimgproc.hpp>
#include <opencv4/opencv2/cudafilters.hpp>
#endif


#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "librealsense2/rs.hpp"
// #include "SparkMax.hpp"

#include "vision_pkg/CameraRS.h"

/**
 * @brief Entry point
 * @param argc Number of arguments
 * @param argv Arguments
 **********************************/
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MultiCameraNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
