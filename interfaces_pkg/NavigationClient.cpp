/**
 * @file navigation_client.cpp
 * @brief Navigation client for autonomous robot navigation
 * @date 2025
 */

#include <chrono>
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

/**
 * @class NavigationClient
 * @brief Handles navigation requests for autonomous robot movement
 */
class NavigationClient : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    /**
     * @brief Constructor for the NavigationClient class
     */
    NavigationClient()
        : Node("navigation_client"), 
          navigation_active_(false),
          goal_sent_(false)
    {
        navigation_client_ = rclcpp_action::create_client<NavigateToPose>(
            this, "navigate_to_pose");

        // Timer to check for navigation requests
        execution_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500), 
            std::bind(&NavigationClient::execute, this));

        RCLCPP_INFO(this->get_logger(), "Navigation Client Initialized");
    }

    /**
     * @brief Initiates navigation to a specified pose
     * @param x Target x coordinate
     * @param y Target y coordinate
     * @param orientation_z Quaternion z component (for rotation)
     * @param orientation_w Quaternion w component (for rotation)
     */
    void navigate_to_pose(double x, double y, double orientation_z, double orientation_w)
    {
        target_x_ = x;
        target_y_ = y;
        target_orientation_z_ = orientation_z;
        target_orientation_w_ = orientation_w;
        navigation_active_ = true;
        goal_sent_ = false;
        
        RCLCPP_INFO(this->get_logger(), 
                    "Navigation requested to position [%.2f, %.2f]", x, y);
    }

    /**
     * @brief Cancels current navigation goal
     */
    void cancel_navigation()
    {
        if (goal_handle_)
        {
            navigation_client_->async_cancel_goal(goal_handle_);
            RCLCPP_WARN(this->get_logger(), "Navigation goal cancelled");
        }
        navigation_active_ = false;
        goal_sent_ = false;
    }

    /**
     * @brief Checks if navigation is currently active
     */
    bool is_navigation_active() const
    {
        return navigation_active_;
    }

private:
    /**
     * @brief Runs the main execution sequence
     */
    void execute()
    {
        if (navigation_active_ && !goal_sent_)
        {
            request_navigation();
        }
    }

    /**
     * @brief Sends goal request to navigation server
     */
    void request_navigation()
    {
        if (!navigation_client_->wait_for_action_server(std::chrono::seconds(1)))
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), 
                                 *this->get_clock(), 
                                 5000,
                                 "Navigation action server not available");
            return;
        }

        auto goal_msg = NavigateToPose::Goal();
        geometry_msgs::msg::Pose goal_pose;

        goal_pose.position.x = target_x_;
        goal_pose.position.y = target_y_;
        goal_pose.position.z = 0.0;
        goal_pose.orientation.x = 0.0;
        goal_pose.orientation.y = 0.0;
        goal_pose.orientation.z = target_orientation_z_;
        goal_pose.orientation.w = target_orientation_w_;

        goal_msg.pose.pose = goal_pose;
        goal_msg.pose.header.stamp = this->now();
        goal_msg.pose.header.frame_id = "map";

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback = 
            std::bind(&NavigationClient::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.result_callback = 
            std::bind(&NavigationClient::handle_navigation_result, this, std::placeholders::_1);

        RCLCPP_INFO(this->get_logger(), "Sending navigation goal to [%.2f, %.2f]", 
                    target_x_, target_y_);
        
        auto goal_handle_future = navigation_client_->async_send_goal(goal_msg, send_goal_options);
        goal_sent_ = true;
    }

    /**
     * @brief Callback when goal is accepted or rejected by server
     */
    void goal_response_callback(const GoalHandleNavigate::SharedPtr &goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
            navigation_active_ = false;
            goal_sent_ = false;
        }
        else
        {
            goal_handle_ = goal_handle;
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server");
        }
    }

    /**
     * @brief Callback for the result of the navigation goal
     * @param result The result of the goal execution
     */
    void handle_navigation_result(const GoalHandleNavigate::WrappedResult &result)
    {
        navigation_active_ = false;
        goal_sent_ = false;

        switch (result.code)
        {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), 
                           "\033[1;32mNavigation goal reached successfully!\033[0m");
                break;
            
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), 
                            "\033[1;31mNavigation goal aborted\033[0m");
                break;
            
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_WARN(this->get_logger(), 
                           "\033[1;33mNavigation goal cancelled\033[0m");
                break;
            
            default:
                RCLCPP_ERROR(this->get_logger(), 
                            "Navigation goal failed with unknown result code");
                break;
        }
    }

    rclcpp_action::Client<NavigateToPose>::SharedPtr navigation_client_;
    rclcpp::TimerBase::SharedPtr execution_timer_;
    GoalHandleNavigate::SharedPtr goal_handle_;

    bool navigation_active_;
    bool goal_sent_;
    
    double target_x_;
    double target_y_;
    double target_orientation_z_;
    double target_orientation_w_;
};

/**
 * @brief Main function
 * Initializes and runs the NavigationClient node
 */
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NavigationClient>());
    rclcpp::shutdown();
    return 0;
}
