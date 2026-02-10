/**
 * @file navigation_client.cpp
 * @brief Navigation client that waits for localization, sets initial pose in Nav2,
 *        then navigates to goals using Nav2 with obstacle avoidance via lidar.
 */

#include <array>
#include <chrono>
#include <cmath>
#include <thread>
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "msg_pkg/action/localization.hpp"

class NavigationClient : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateToPose>;
    using Localization = msg_pkg::action::Localization;
    using GoalHandleLocalization = rclcpp_action::ClientGoalHandle<Localization>;

    NavigationClient()
        : Node("navigation_client"),
          localization_in_progress_(false),
          localized_(false),
          nav2_initialized_(false),
          start_navigation_(false),
          navigation_in_progress_(false),
          current_goal_index_(0)
    {
        // Publishers
        initial_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10);
            
        tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);


        // Action clients
        navigation_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
        localization_client_ = rclcpp_action::create_client<Localization>(this, "localization_action");

        // Main execution timer
        execution_timer_ = create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&NavigationClient::execute, this));

        RCLCPP_INFO(get_logger(), "Navigation client initialized");
    }

private:
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;

    // Goal positions - grab these using coordinate_grabber.py
    struct GoalXY { double x; double y; };
    const std::array<GoalXY, 3> goals_{{
        {-0.928, 3.165},  // Goal 0
        {-0.928, 3.165},  // Goal 1 (duplicate for now)
        {-0.928, 3.165}   // Goal 2 (duplicate for now)
    }};
// Recorded position 1: x=-0.928, y=-3.165, yaw=0.000
    // Goal 0: x=0.326, y=-2.660
  // 5.0, 1.7}, {5.0, 1.3}, {5.0, 1.2
    // State flags
    bool localization_in_progress_;
    bool localized_;
    bool nav2_initialized_;
    bool start_navigation_;
    bool navigation_in_progress_;

    // Robot pose from localization
    double robot_x_;
    double robot_y_;
    double robot_yaw_;

    // ROS interfaces
    
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr navigation_client_;
    rclcpp_action::Client<Localization>::SharedPtr localization_client_;
    rclcpp::TimerBase::SharedPtr execution_timer_;

    std::size_t current_goal_index_;

    /**
     * @brief Main execution loop
     */
    void execute()
    {
        // Step 1: Run localization first
        if (!localized_ && !localization_in_progress_)
        {
            request_localization();
        }
        // Step 2: After localization, initialize Nav2 with our position
        else if (localized_ && !nav2_initialized_)
        {
            initialize_nav2();
        }
        // Step 3: Navigate to goals
        else if (nav2_initialized_ && start_navigation_ && !navigation_in_progress_)
        {
            request_navigation();
        }
    }

    /**
     * @brief Request localization from localization server
     */
    void request_localization()
    {
        if (!localization_client_->wait_for_action_server(std::chrono::seconds(1)))
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "Waiting for localization action server...");
            return;
        }

        auto goal_msg = Localization::Goal();
        auto send_goal_options = rclcpp_action::Client<Localization>::SendGoalOptions();
        send_goal_options.result_callback =
            std::bind(&NavigationClient::handle_localization_result, this, std::placeholders::_1);

        localization_client_->async_send_goal(goal_msg, send_goal_options);
        localization_in_progress_ = true;

        RCLCPP_INFO(get_logger(), "Localization request sent");
    }

    /**
     * @brief Handle localization result
     */
    void handle_localization_result(const GoalHandleLocalization::WrappedResult &result)
    {
        localization_in_progress_ = false;

        if (result.code == rclcpp_action::ResultCode::SUCCEEDED &&
            result.result && result.result->success)
        {
            robot_x_ = result.result->x;
            robot_y_ = result.result->y;
            robot_yaw_ = 0.0;  // Facing forward after localization

            RCLCPP_INFO(get_logger(),
                "\033[1;32mLocalization succeeded: x=%.2f y=%.2f\033[0m",
                robot_x_, robot_y_);

            localized_ = true;
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Localization failed");
            rclcpp::shutdown();
        }
    }

    /**
     * @brief Initialize Nav2 with our localized position
     */
    void initialize_nav2()
    {
        RCLCPP_INFO(get_logger(), "Initializing Nav2 with localized position...");

        // Wait for Nav2 to be ready
        if (!navigation_client_->wait_for_action_server(std::chrono::seconds(5)))
        {
            RCLCPP_WARN(get_logger(), "Nav2 not ready yet, waiting...");
            return;
        }

        // Publish initial pose to Nav2
        publish_initial_pose();

        // Give Nav2 time to process the initial pose
        std::this_thread::sleep_for(std::chrono::seconds(3));

        nav2_initialized_ = true;
        start_navigation_ = true;

        RCLCPP_INFO(get_logger(), "\033[1;32mNav2 initialized, starting navigation\033[0m");
    }

    /**
     * @brief Publish initial pose to Nav2's /initialpose topic
     */
    void publish_initial_pose()
    {
        // Get current odom -> base_link transform (where odometry thinks we are)
        // For simplicity, assume odometry starts at (0,0,0)
        double odom_x = 0.0;
        double odom_y = 0.0;
        double odom_yaw = 0.0;
        
        // Calculate map -> odom transform
        // map_pose = map_to_odom * odom_pose
        // So: map_to_odom = map_pose * inverse(odom_pose)
        // Since odom starts at origin: map_to_odom = map_pose
        
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->now();
        transform.header.frame_id = "map";
        transform.child_frame_id = "odom";
        
        transform.transform.translation.x = robot_x_ - odom_x;
        transform.transform.translation.y = robot_y_ - odom_y;
        transform.transform.translation.z = 0.0;
        
        double yaw_diff = robot_yaw_ - odom_yaw;
        transform.transform.rotation.x = 0.0;
        transform.transform.rotation.y = 0.0;
        transform.transform.rotation.z = std::sin(yaw_diff / 2.0);
        transform.transform.rotation.w = std::cos(yaw_diff / 2.0);
        
        tf_broadcaster_->sendTransform(transform);
        
        RCLCPP_INFO(get_logger(), "Published map->odom transform: x=%.2f, y=%.2f, yaw=%.2f",
            robot_x_, robot_y_, robot_yaw_);
    }

    /**
     * @brief Request navigation to current goal via Nav2
     */
    void request_navigation()
    {
        if (!navigation_client_->wait_for_action_server(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(get_logger(), "Nav2 action server not available");
            return;
        }

        navigation_in_progress_ = true;

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.header.stamp = this->now();
        goal_msg.pose.header.frame_id = "map";

        goal_msg.pose.pose.position.x = goals_[current_goal_index_].x;
        goal_msg.pose.pose.position.y = goals_[current_goal_index_].y;
        goal_msg.pose.pose.position.z = 0.0;

        // Face forward (east) - 90 degrees
        goal_msg.pose.pose.orientation.z = 0.707;
        goal_msg.pose.pose.orientation.w = 0.707;

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        
        send_goal_options.goal_response_callback =
            std::bind(&NavigationClient::nav_goal_response_callback, this, std::placeholders::_1);
        
        send_goal_options.feedback_callback =
            std::bind(&NavigationClient::nav_feedback_callback, this, 
                std::placeholders::_1, std::placeholders::_2);
        
        send_goal_options.result_callback =
            std::bind(&NavigationClient::handle_navigation_result, this, std::placeholders::_1);

        navigation_client_->async_send_goal(goal_msg, send_goal_options);
        start_navigation_ = false;

        RCLCPP_INFO(get_logger(), "Navigating to goal %zu: (%.2f, %.2f)",
            current_goal_index_, goals_[current_goal_index_].x, goals_[current_goal_index_].y);
    }

    /**
     * @brief Callback when Nav2 accepts/rejects the goal
     */
    void nav_goal_response_callback(const GoalHandleNavigate::SharedPtr &goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_ERROR(get_logger(), "Navigation goal was rejected by Nav2");
            navigation_in_progress_ = false;
        }
        else
        {
            RCLCPP_INFO(get_logger(), "Navigation goal accepted by Nav2");
        }
    }

    /**
     * @brief Callback for navigation feedback (progress updates)
     */
    void nav_feedback_callback(
        GoalHandleNavigate::SharedPtr,
        const std::shared_ptr<const NavigateToPose::Feedback> feedback)
    {
        double distance = feedback->distance_remaining;
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
            "Distance remaining: %.2f m", distance);
    }

    /**
     * @brief Handle navigation result
     */
    void handle_navigation_result(const GoalHandleNavigate::WrappedResult &result)
    {
        navigation_in_progress_ = false;

        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(get_logger(), "\033[1;32mGoal %zu reached!\033[0m", current_goal_index_);
            
            // Move to next goal
            ++current_goal_index_;
            if (current_goal_index_ < goals_.size())
            {
                RCLCPP_INFO(get_logger(), "Moving to next goal...");
                start_navigation_ = true;
            }
            else
            {
                RCLCPP_INFO(get_logger(), "\033[1;32mALL GOALS COMPLETED!\033[0m");
                rclcpp::shutdown();
            }
            break;

        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(get_logger(), "\033[1;31mNavigation aborted\033[0m");
            rclcpp::shutdown();
            break;

        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(get_logger(), "Navigation canceled");
            rclcpp::shutdown();
            break;

        default:
            RCLCPP_ERROR(get_logger(), "Unknown navigation result");
            rclcpp::shutdown();
            break;
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NavigationClient>());
    rclcpp::shutdown();
    return 0;
}
