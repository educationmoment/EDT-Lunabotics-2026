/**
 * @file navigation_client.cpp
 * @brief Navigation client that uses localization to know where it is relative
 *        to AprilTag 7, then navigates to goals in the map frame.
 *        rtabmap handles map->odom transform. Robot starts at map origin (0,0).
 *        Goals are defined relative to the robot's starting position.
 */

#include <array>
#include <chrono>
#include <cmath>
#include <thread>
#include "geometry_msgs/msg/pose_stamped.hpp"
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
        navigation_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
        localization_client_ = rclcpp_action::create_client<Localization>(this, "localization_action");

        execution_timer_ = create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&NavigationClient::execute, this));

        RCLCPP_INFO(get_logger(), "Navigation client initialized");
    }

private:
    // Goals relative to robot start position (map origin)
    // +X = robot's initial forward direction, +Y = robot's initial left
    // Adjust these for your field layout
    struct GoalXY { double x; double y; };
    const std::array<GoalXY, 1> goals_{{
        {0.0, 3.0},  // 3 meters forward from start
    }};

    bool localization_in_progress_;
    bool localized_;
    bool nav2_initialized_;
    bool start_navigation_;
    bool navigation_in_progress_;

    double robot_x_;
    double robot_y_;

    rclcpp_action::Client<NavigateToPose>::SharedPtr navigation_client_;
    rclcpp_action::Client<Localization>::SharedPtr localization_client_;
    rclcpp::TimerBase::SharedPtr execution_timer_;
    std::size_t current_goal_index_;

    void execute()
    {
        if (!localized_ && !localization_in_progress_)
        {
            request_localization();
        }
        else if (localized_ && !nav2_initialized_)
        {
            initialize_nav2();
        }
        else if (nav2_initialized_ && start_navigation_ && !navigation_in_progress_)
        {
            request_navigation();
        }
    }

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

    void handle_localization_result(const GoalHandleLocalization::WrappedResult &result)
    {
        localization_in_progress_ = false;

        if (result.code == rclcpp_action::ResultCode::SUCCEEDED &&
            result.result && result.result->success)
        {
            robot_x_ = result.result->x;
            robot_y_ = result.result->y;

            RCLCPP_INFO(get_logger(),
                "\033[1;32mLocalization succeeded: depth_from_tag=%.2f lateral_from_tag=%.2f\033[0m",
                robot_x_, robot_y_);

            localized_ = true;
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Localization failed");
            rclcpp::shutdown();
        }
    }

    void initialize_nav2()
    {
        RCLCPP_INFO(get_logger(), "Waiting for Nav2...");

        if (!navigation_client_->wait_for_action_server(std::chrono::seconds(5)))
        {
            RCLCPP_WARN(get_logger(), "Nav2 not ready yet, waiting...");
            return;
        }

        // No initial pose needed — rtabmap starts at origin = robot start
        std::this_thread::sleep_for(std::chrono::seconds(2));

        nav2_initialized_ = true;
        start_navigation_ = true;

        RCLCPP_INFO(get_logger(), "\033[1;32mNav2 ready, starting navigation\033[0m");
    }

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

        // Face forward (+X)
        goal_msg.pose.pose.orientation.w = 1.0;

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

    void nav_feedback_callback(
        GoalHandleNavigate::SharedPtr,
        const std::shared_ptr<const NavigateToPose::Feedback> feedback)
    {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
            "Distance remaining: %.2f m", feedback->distance_remaining);
    }

    void handle_navigation_result(const GoalHandleNavigate::WrappedResult &result)
    {
        navigation_in_progress_ = false;

        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(get_logger(), "\033[1;32mGoal %zu reached!\033[0m", current_goal_index_);
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