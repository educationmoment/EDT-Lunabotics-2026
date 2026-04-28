/**
 * @file apriltag_to_landmarks.cpp
 * @brief Converts apriltag_ros detections → rtabmap_msgs/LandmarkDetections.
 *
 * apriltag_msgs::msg::AprilTagDetection does NOT carry a pose field in ROS2.
 * Instead, apriltag_ros publishes each detected tag as a TF frame named
 * "tag36h11:<id>" (verify with: ros2 run tf2_tools view_frames).
 *
 * This node:
 *   1. Subscribes to /detections (AprilTagDetectionArray) to know which IDs are visible
 *   2. Looks up each tag's pose from TF (camera_frame → tag36h11:<id>)
 *   3. Publishes /landmarks (LandmarkDetections) for rtabmap to consume
 *
 * rtabmap must have:
 *   subscribe_landmarks: true
 *   RGBD/MarkerDetection: "false"   ← disable internal detection when using this node
 */

#include "rclcpp/rclcpp.hpp"
#include "apriltag_msgs/msg/april_tag_detection_array.hpp"
#include "rtabmap_msgs/msg/landmark_detection.hpp"
#include "rtabmap_msgs/msg/landmark_detections.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class AprilTagToLandmarks : public rclcpp::Node
{
public:
    AprilTagToLandmarks() : Node("apriltag_to_landmarks")
    {
        this->declare_parameter("tag_linear_variance",  0.01);
        this->declare_parameter("tag_angular_variance", 0.05);
        this->declare_parameter("tag_size",             0.1651);
        // The frame that tag poses are expressed relative to.
        // Use the optical frame of whichever camera is running apriltag_ros.
        this->declare_parameter("reference_frame", std::string("d455_color_optical_frame"));
        // TF lookup timeout in seconds
        this->declare_parameter("tf_timeout", 0.1);

        tag_linear_variance_  = this->get_parameter("tag_linear_variance").as_double();
        tag_angular_variance_ = this->get_parameter("tag_angular_variance").as_double();
        tag_size_             = this->get_parameter("tag_size").as_double();
        reference_frame_      = this->get_parameter("reference_frame").as_string();
        tf_timeout_           = this->get_parameter("tf_timeout").as_double();

        tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        sub_ = this->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
            "/detections", 10,
            std::bind(&AprilTagToLandmarks::callback, this, std::placeholders::_1));

        pub_ = this->create_publisher<rtabmap_msgs::msg::LandmarkDetections>(
            "/landmark_detections", 10);

        RCLCPP_INFO(get_logger(),
            "apriltag_to_landmarks ready — reference_frame: %s, tag_size: %.4fm",
            reference_frame_.c_str(), tag_size_);
    }

private:
    void callback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg)
    {
        if (msg->detections.empty()) return;

        rtabmap_msgs::msg::LandmarkDetections out;
        out.header = msg->header;


        for (const auto & det : msg->detections)
        {
            // Build the TF frame name: apriltag_ros uses "family:id" format
            // e.g. "tag36h11:7"  — verify with: ros2 run tf2_tools view_frames
            std::string tag_frame = det.family + ":" + std::to_string(det.id);

            geometry_msgs::msg::TransformStamped tf_stamped;
            try
            {
                tf_stamped = tf_buffer_->lookupTransform(
                    reference_frame_,
                    tag_frame,
                    tf2::TimePointZero,
                    tf2::durationFromSec(tf_timeout_));
            }
            catch (const tf2::TransformException & ex)
            {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                    "Could not look up TF for %s: %s", tag_frame.c_str(), ex.what());
                continue;
            }

            // Convert TF transform → PoseWithCovariance
            // rtabmap_msgs/LandmarkDetection.pose is PoseWithCovariance (NOT stamped)
            rtabmap_msgs::msg::LandmarkDetection lm;
            lm.id   = det.id;
            lm.size = static_cast<float>(tag_size_);
            lm.header.frame_id = reference_frame_;        // ADD THIS
            lm.header.stamp    = tf_stamped.header.stamp; // ADD THIS

            // Position from TF translation
            lm.pose.pose.position.x = tf_stamped.transform.translation.x;
            lm.pose.pose.position.y = tf_stamped.transform.translation.y;
            lm.pose.pose.position.z = tf_stamped.transform.translation.z;

            // Orientation from TF rotation
            lm.pose.pose.orientation.x = tf_stamped.transform.rotation.x;
            lm.pose.pose.orientation.y = tf_stamped.transform.rotation.y;
            lm.pose.pose.orientation.z = tf_stamped.transform.rotation.z;
            lm.pose.pose.orientation.w = tf_stamped.transform.rotation.w;

            // Diagonal covariance [x, y, z, roll, pitch, yaw]
            lm.pose.covariance[0]  = 0.05;   // was 0.01 — looser position trust
            lm.pose.covariance[7]  = 0.05;
            lm.pose.covariance[14] = 0.05;
            lm.pose.covariance[21] = 0.1;    // was 0.05 — looser angular trust
            lm.pose.covariance[28] = 0.1;
            lm.pose.covariance[35] = 0.1;

            // LandmarkDetections uses "landmarks", not "detections"
            out.landmarks.push_back(lm);

            RCLCPP_DEBUG(get_logger(),
                "Tag %d: pos=(%.3f, %.3f, %.3f)",
                det.id,
                lm.pose.pose.position.x,
                lm.pose.pose.position.y,
                lm.pose.pose.position.z);
        }

        if (!out.landmarks.empty())
        {
            pub_->publish(out);
        }
    }

    // Parameters
    double      tag_linear_variance_;
    double      tag_angular_variance_;
    double      tag_size_;
    std::string reference_frame_;
    double      tf_timeout_;

    // ROS
    std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr sub_;
    rclcpp::Publisher<rtabmap_msgs::msg::LandmarkDetections>::SharedPtr pub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AprilTagToLandmarks>());
    rclcpp::shutdown();
    return 0;
}