py/**
 * @file localization_server.cpp
 * @brief Handles localization by rotating the robot to find AprilTag 7
 *        and aligning to a target yaw.
 */

#include <chrono>
#include <cmath>
#include <memory>
#include <thread>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "geometry_msgs/msg/twist.hpp"
#include "msg_pkg/action/localization.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class LocalizationServer : public rclcpp::Node
{
public:
    using Localization = msg_pkg::action::Localization;
    using GoalHandleLocalization = rclcpp_action::ServerGoalHandle<Localization>;

    LocalizationServer()
        : Node("localization_server"),
          success_(false),
          tag7_visible_(false),
          search_state_(SearchState::SEARCHING),
          rotation_direction_(1.0)
    {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        action_server_ = rclcpp_action::create_server<Localization>(
            this, "localization_action",
            std::bind(&LocalizationServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&LocalizationServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&LocalizationServer::handle_accepted, this, std::placeholders::_1));

        localization_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&LocalizationServer::localize, this));

        RCLCPP_INFO(get_logger(), "Localization server initialized (Tag 7 only)");
    }

    ~LocalizationServer()
    {
        stopRobot();
    }

private:
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp_action::Server<Localization>::SharedPtr action_server_;
    rclcpp::TimerBase::SharedPtr localization_timer_;

    bool success_;
    bool tag7_visible_;
    double depth_distance_;
    double lateral_distance_;
    double current_yaw_;

    enum class SearchState
    {
        SEARCHING,
        ALIGNING,
        LOCALIZED
    };
    SearchState search_state_;
    double rotation_direction_;

    static constexpr double ROTATION_SPEED = 0.3;
    static constexpr double ALIGN_ROTATION_SPEED = 0.15;
    static constexpr double YAW_TOLERANCE = 0.05;
    static constexpr double TARGET_YAW = 1.57;
    static constexpr double TARGET_BEARING   = M_PI;  // tag is behind robot (rear camera)
    static constexpr double BEARING_TOLERANCE = 0.1;   // slightly looser — ±6°

    void stopRobot()
    {
        geometry_msgs::msg::Twist msg;
        cmd_vel_publisher_->publish(msg);
    }

    void rotateInPlace(double angular_velocity)
    {
        geometry_msgs::msg::Twist msg;
        msg.angular.z = angular_velocity;
        cmd_vel_publisher_->publish(msg);
    }

    /**
     * @brief Try to find Tag 7 from either camera. Returns true if found.
        */
    bool lookupTag7(double &depth, double &lateral, double &bearing)
    {
        try
        {
            // Only accept transforms newer than 500ms
            auto when = tf2_ros::toMsg(tf2::TimePoint(
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::steady_clock::now().time_since_epoch()) -
                std::chrono::milliseconds(500)));

            auto tag_tf = tf_buffer_->lookupTransform(
                "base_link", "tag36h11:7",
                tf2::TimePointZero,
                tf2::durationFromSec(0.1));  // 100ms timeout

            // Check transform age manually
            auto transform_time = tf_buffer_->lookupTransform("base_link", "tag36h11:7", tf2::TimePointZero);
            auto age = this->now() - transform_time.header.stamp;
            if (age.seconds() > 0.5)
            {
                return false;  // transform too old, treat as not visible
            }

            depth   = transform_time.transform.translation.x;
            lateral = transform_time.transform.translation.y;
            bearing = std::atan2(lateral, depth);
            return true;
        }
        catch (tf2::TransformException &) { return false; }
    }

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const Localization::Goal> goal)
    {
        (void)uuid;
        (void)goal;
        RCLCPP_INFO(get_logger(), "Received localization goal - searching for Tag 7");
        search_state_ = SearchState::SEARCHING;
        tag7_visible_ = false;
        success_ = false;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleLocalization> goal_handle)
    {
        (void)goal_handle;
        RCLCPP_INFO(get_logger(), "Received request to cancel goal");
        stopRobot();
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleLocalization> goal_handle)
    {
        std::thread{std::bind(&LocalizationServer::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleLocalization> goal_handle)
    {
        auto result = std::make_shared<Localization::Result>();
        auto timeout = std::chrono::seconds(60);
        auto start = std::chrono::steady_clock::now();

        while (!success_ && rclcpp::ok())
        {
            if (std::chrono::steady_clock::now() - start > timeout)
            {
                RCLCPP_ERROR(get_logger(), "Localization timed out");
                stopRobot();
                result->success = false;
                goal_handle->abort(result);
                return;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        if (success_)
        {
            stopRobot();
            result->x = depth_distance_ + 0.1;
            result->y = lateral_distance_ + 1.0;
            result->success = true;
            goal_handle->succeed(result);
            RCLCPP_INFO(get_logger(), "Localization complete: x=%.2f, y=%.2f", result->x, result->y);
        }
        else
        {
            stopRobot();
            result->success = false;
            goal_handle->abort(result);
        }
    }

    void localize()
    {
        double depth, lateral, bearing;
        tag7_visible_ = lookupTag7(depth, lateral, bearing);

        if (tag7_visible_)
        {
            depth_distance_   = depth;
            lateral_distance_ = lateral;
            current_yaw_      = bearing;
        }

        switch (search_state_)
        {
        case SearchState::SEARCHING:
            if (tag7_visible_)
            {
                stopRobot();
                RCLCPP_INFO(get_logger(), "Tag 7 found! depth=%.2f, lateral=%.2f, bearing=%.2f",
                            depth_distance_, lateral_distance_, current_yaw_);
                success_ = true;
                search_state_ = SearchState::LOCALIZED;
            }
            else
            {
                rotateInPlace(ROTATION_SPEED * rotation_direction_);
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Searching for Tag 7...");
            }
            break;

        case SearchState::ALIGNING:
            // No longer used — kept to avoid enum warning
            search_state_ = SearchState::SEARCHING;
            break;

        case SearchState::LOCALIZED:
            break;
        }
    }
    };

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalizationServer>());
    rclcpp::shutdown();
    return 0;
}