py/**
 * @file localization_server.cpp
 * @brief Handles localization by rotating the robot to find AprilTag 7.
 *        Publishes the tag's position (with fixed offsets) as the localization result.
 *
 * Flow:
 *   SEARCHING → rotates in place until tag36h11:7 is visible in TF
 *   LOCALIZED → stops robot, returns result via action server
 *
 * TF source: apriltag_ros publishes "tag36h11:7" frame when tag is detected.
 * Verify frame name on hardware with: ros2 run tf2_tools view_frames
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
    using Localization     = msg_pkg::action::Localization;
    using GoalHandleLocalization = rclcpp_action::ServerGoalHandle<Localization>;

    LocalizationServer()
        : Node("localization_server"),
          success_(false),
          tag7_visible_(false),
          depth_distance_(0.0),
          lateral_distance_(0.0),
          search_state_(SearchState::SEARCHING),
          rotation_direction_(1.0)
    {
        tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        action_server_ = rclcpp_action::create_server<Localization>(
            this, "localization_action",
            std::bind(&LocalizationServer::handle_goal,     this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&LocalizationServer::handle_cancel,   this, std::placeholders::_1),
            std::bind(&LocalizationServer::handle_accepted, this, std::placeholders::_1));

        localization_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&LocalizationServer::localize, this));

        RCLCPP_INFO(get_logger(), "Localization server initialized — searching for tag36h11:7");
    }

    ~LocalizationServer()
    {
        stopRobot();
    }

private:
    // ── TF ────────────────────────────────────────────────────────────────────
    std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // ── ROS interfaces ────────────────────────────────────────────────────────
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp_action::Server<Localization>::SharedPtr          action_server_;
    rclcpp::TimerBase::SharedPtr                            localization_timer_;

    // ── State ─────────────────────────────────────────────────────────────────
    bool   success_;
    bool   tag7_visible_;
    double depth_distance_;
    double lateral_distance_;

    enum class SearchState { SEARCHING, LOCALIZED };
    SearchState search_state_;
    double rotation_direction_;

    // ── Tuning constants ──────────────────────────────────────────────────────
    // Rotation speed while sweeping for the tag (rad/s)
    static constexpr double ROTATION_SPEED = 0.3;

    // Maximum age of a TF transform before it is considered stale (seconds)
    static constexpr double MAX_TRANSFORM_AGE_S = 0.5;

    // TF lookup timeout (seconds) — how long to wait for a transform to appear
    static constexpr double TF_LOOKUP_TIMEOUT_S = 0.1;

    // Offset from tag face to desired robot goal position.
    // Adjust these to match the physical arena layout before competition.
    //   depth_offset:   distance forward from the tag face (meters)
    //   lateral_offset: lateral shift toward excavation zone (meters)
    static constexpr double TAG_TO_GOAL_DEPTH_OFFSET   = 0.1;   // 10 cm
    static constexpr double TAG_TO_GOAL_LATERAL_OFFSET = 1.0;   // 1 m

    // ── Motion helpers ────────────────────────────────────────────────────────
    void stopRobot()
    {
        cmd_vel_publisher_->publish(geometry_msgs::msg::Twist{});
    }

    void rotateInPlace(double angular_velocity)
    {
        geometry_msgs::msg::Twist msg;
        msg.angular.z = angular_velocity;
        cmd_vel_publisher_->publish(msg);
    }

    // ── TF lookup ─────────────────────────────────────────────────────────────
    /**
     * @brief Looks up tag36h11:7 in the TF tree relative to base_link.
     *        Returns true only if the transform exists and is fresh.
     *
     * @param depth    Output: X distance from base_link to tag (forward)
     * @param lateral  Output: Y distance from base_link to tag (left positive)
     * @param bearing  Output: Bearing angle to tag (radians, from forward axis)
     */
    bool lookupTag7(double & depth, double & lateral, double & bearing)
    {
        try
        {
            auto transform = tf_buffer_->lookupTransform(
                "base_link",
                "tag36h11:7",
                tf2::TimePointZero,
                tf2::durationFromSec(TF_LOOKUP_TIMEOUT_S));

            // Reject stale transforms — tag may have been visible previously
            auto age = this->now() - transform.header.stamp;
            if (age.seconds() > MAX_TRANSFORM_AGE_S)
            {
                RCLCPP_DEBUG(get_logger(), "Tag7 transform is stale (%.2fs) — ignoring", age.seconds());
                return false;
            }

            depth   = transform.transform.translation.x;
            lateral = transform.transform.translation.y;
            bearing = std::atan2(lateral, depth);
            return true;
        }
        catch (const tf2::TransformException & ex)
        {
            RCLCPP_DEBUG(get_logger(), "Tag7 TF lookup failed: %s", ex.what());
            return false;
        }
    }

    // ── Action server callbacks ───────────────────────────────────────────────
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const Localization::Goal> goal)
    {
        (void)uuid;
        (void)goal;
        RCLCPP_INFO(get_logger(), "Received localization goal — starting tag search");
        search_state_ = SearchState::SEARCHING;
        tag7_visible_ = false;
        success_      = false;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleLocalization> goal_handle)
    {
        (void)goal_handle;
        RCLCPP_INFO(get_logger(), "Localization goal cancelled");
        stopRobot();
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleLocalization> goal_handle)
    {
        std::thread{
            std::bind(&LocalizationServer::execute, this, std::placeholders::_1),
            goal_handle
        }.detach();
    }

    // ── Action execution ──────────────────────────────────────────────────────
    void execute(const std::shared_ptr<GoalHandleLocalization> goal_handle)
    {
        auto result   = std::make_shared<Localization::Result>();
        auto start    = std::chrono::steady_clock::now();
        auto timeout  = std::chrono::seconds(60);

        // Spin-wait for the localize() timer to set success_
        while (!success_ && rclcpp::ok())
        {
            if (std::chrono::steady_clock::now() - start > timeout)
            {
                RCLCPP_ERROR(get_logger(), "Localization timed out after 60s — tag not found");
                stopRobot();
                result->success = false;
                goal_handle->abort(result);
                return;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        stopRobot();

        if (success_)
        {
            result->x       = depth_distance_   + TAG_TO_GOAL_DEPTH_OFFSET;
            result->y       = lateral_distance_ + TAG_TO_GOAL_LATERAL_OFFSET;
            result->success = true;
            goal_handle->succeed(result);
            RCLCPP_INFO(get_logger(),
                "Localization complete: tag at (%.2f, %.2f) → goal at (%.2f, %.2f)",
                depth_distance_, lateral_distance_, result->x, result->y);
        }
        else
        {
            result->success = false;
            goal_handle->abort(result);
        }
    }

    // ── Timer callback (10 Hz) ────────────────────────────────────────────────
    void localize()
    {
        double depth, lateral, bearing;
        tag7_visible_ = lookupTag7(depth, lateral, bearing);

        if (tag7_visible_)
        {
            depth_distance_   = depth;
            lateral_distance_ = lateral;
        }

        switch (search_state_)
        {
        case SearchState::SEARCHING:
            if (tag7_visible_)
            {
                stopRobot();
                RCLCPP_INFO(get_logger(),
                    "Tag 7 found — depth=%.3fm  lateral=%.3fm  bearing=%.3frad",
                    depth_distance_, lateral_distance_, bearing);
                success_      = true;
                search_state_ = SearchState::LOCALIZED;
            }
            else
            {
                rotateInPlace(ROTATION_SPEED * rotation_direction_);
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                    "Searching for tag36h11:7 — rotating at %.2f rad/s", ROTATION_SPEED);
            }
            break;

        case SearchState::LOCALIZED:
            // Nothing to do — execute() thread handles result publishing
            break;
        }
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalizationServer>());
    rclcpp::shutdown();
    return 0;
}