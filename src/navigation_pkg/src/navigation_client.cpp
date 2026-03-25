/**
 * @file navigation_client.cpp
 * @brief Navigation client that:
 *        1. Localizes via AprilTag
 *        2. Navigates to the dig zone
 *        3. Calls the excavation action server
 *        4. Navigates to the deposit zone
 *        5. Calls the depositing action server
 *        6. Repeats from step 2 for additional cycles (configurable)
 *
 *        rtabmap handles map->odom transform. Robot starts at map origin (0,0).
 *        Goals are defined relative to the robot's starting position.
 */

#include <array>
#include <chrono>
#include <cmath>
#include <string>
#include <thread>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "msg_pkg/action/localization.hpp"
#include "msg_pkg/action/excavation.hpp"
#include "msg_pkg/action/depositing.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

// ── STATE MACHINE ─────────────────────────────────────────────────────────────
enum class State
{
    LOCALIZING,
    INITIALIZING_NAV2,
    NAVIGATING_TO_DIG,
    EXCAVATING,
    NAVIGATING_TO_DEPOSIT,
    DEPOSITING,
    NAVIGATING_TO_START,   // optional return-to-start between cycles
    COMPLETE,
    FAILED
};

static const char* state_name(State s)
{
    switch (s)
    {
    case State::LOCALIZING:             return "LOCALIZING";
    case State::INITIALIZING_NAV2:      return "INITIALIZING_NAV2";
    case State::NAVIGATING_TO_DIG:      return "NAVIGATING_TO_DIG";
    case State::EXCAVATING:             return "EXCAVATING";
    case State::NAVIGATING_TO_DEPOSIT:  return "NAVIGATING_TO_DEPOSIT";
    case State::DEPOSITING:             return "DEPOSITING";
    case State::NAVIGATING_TO_START:    return "NAVIGATING_TO_START";
    case State::COMPLETE:               return "COMPLETE";
    case State::FAILED:                 return "FAILED";
    default:                            return "UNKNOWN";
    }
}

// ── NODE ─────────────────────────────────────────────────────────────────────
class NavigationClient : public rclcpp::Node
{
public:
    using NavigateToPose        = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigate    = rclcpp_action::ClientGoalHandle<NavigateToPose>;
    using Localization          = msg_pkg::action::Localization;
    using GoalHandleLocalize    = rclcpp_action::ClientGoalHandle<Localization>;
    using ExcavationAction      = msg_pkg::action::Excavation;   // Excavation.action → ::Excavation
    using GoalHandleExcavation  = rclcpp_action::ClientGoalHandle<ExcavationAction>;
    using DepositingAction      = msg_pkg::action::Depositing;   // Depositing.action → ::Depositing
    using GoalHandleDepositing  = rclcpp_action::ClientGoalHandle<DepositingAction>;

    NavigationClient()
        : Node("navigation_client"),
          state_(State::LOCALIZING),
          action_in_progress_(false),
          current_cycle_(0),
          max_cycles_(1)   // set to >1 to repeat dig→deposit
    {
        // ── ACTION CLIENTS ────────────────────────────────────────────────────
        navigation_client_  = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
        localization_client_ = rclcpp_action::create_client<Localization>(this, "localization_action");
        excavation_client_  = rclcpp_action::create_client<ExcavationAction>(this, "excavation_action");
        depositing_client_  = rclcpp_action::create_client<DepositingAction>(this, "depositing_action");

        // ── TIMER ─────────────────────────────────────────────────────────────
        execution_timer_ = create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&NavigationClient::tick, this));

        RCLCPP_INFO(get_logger(), "Navigation client initialized");
    }

private:
    // ── GOALS (map frame, relative to robot start = origin) ──────────────────
    // Adjust these coordinates for your field layout.
    // +X = robot's initial forward direction, +Y = robot's initial left.
    struct GoalXY { double x; double y; double yaw_deg; };

    const GoalXY dig_zone_      = { 3.0,  1.0,   0.0 };   // drive forward to dig area
    const GoalXY deposit_zone_  = { 0.5,  0.0, 180.0 };   // back near start to deposit
    const GoalXY start_pose_    = { 0.0,  0.0,   0.0 };   // true home (used between cycles)

    // ── STATE ─────────────────────────────────────────────────────────────────
    State state_;
    bool  action_in_progress_;
    int   current_cycle_;
    int   max_cycles_;

    double robot_x_ = 0.0;
    double robot_y_ = 0.0;
    bool map_ready_ = false;

    // ── CLIENTS ───────────────────────────────────────────────────────────────
    rclcpp_action::Client<NavigateToPose>::SharedPtr   navigation_client_;
    rclcpp_action::Client<Localization>::SharedPtr     localization_client_;
    rclcpp_action::Client<ExcavationAction>::SharedPtr excavation_client_;
    rclcpp_action::Client<DepositingAction>::SharedPtr depositing_client_;
    rclcpp::TimerBase::SharedPtr execution_timer_;

    // ── HELPERS ───────────────────────────────────────────────────────────────
    void transition(State next)
    {
        RCLCPP_INFO(get_logger(), "State: %s → %s",
                    state_name(state_), state_name(next));
        state_            = next;
        action_in_progress_ = false;
    }

    /** Build a yaw quaternion (rotation about Z). yaw_deg: 0=forward, 180=backward */
    static geometry_msgs::msg::Quaternion yaw_to_quat(double yaw_deg)
    {
        double yaw_rad = yaw_deg * M_PI / 180.0;
        geometry_msgs::msg::Quaternion q;
        q.x = 0.0;
        q.y = 0.0;
        q.z = std::sin(yaw_rad / 2.0);
        q.w = std::cos(yaw_rad / 2.0);
        return q;
    }

    // ── MAIN TICK (runs every 500 ms) ─────────────────────────────────────────
    void tick()
    {
        if (action_in_progress_) return;  // wait for current action callback

        switch (state_)
        {
        case State::LOCALIZING:
            request_localization();
            break;

        case State::INITIALIZING_NAV2:
            initialize_nav2();
            break;

        case State::NAVIGATING_TO_DIG:
            request_navigation(dig_zone_);
            break;

        case State::EXCAVATING:
            request_excavation();
            break;

        case State::NAVIGATING_TO_DEPOSIT:
            request_navigation(deposit_zone_);
            break;

        case State::DEPOSITING:
            request_depositing();
            break;

        case State::NAVIGATING_TO_START:
            request_navigation(start_pose_);
            break;

        case State::COMPLETE:
            RCLCPP_INFO_ONCE(get_logger(), "\033[1;32mAll cycles complete!\033[0m");
            execution_timer_->cancel();
            rclcpp::shutdown();
            break;

        case State::FAILED:
            RCLCPP_ERROR_ONCE(get_logger(), "\033[1;31mMission failed.\033[0m");
            execution_timer_->cancel();
            rclcpp::shutdown();
            break;
        }
    }

    // ══════════════════════════════════════════════════════════════════════════
    //  LOCALIZATION
    // ══════════════════════════════════════════════════════════════════════════
    void request_localization()
    {
        if (!localization_client_->wait_for_action_server(std::chrono::seconds(1)))
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "Waiting for localization action server...");
            return;
        }

        action_in_progress_ = true;

        auto goal_msg = Localization::Goal();
        auto opts     = rclcpp_action::Client<Localization>::SendGoalOptions();
        opts.result_callback =
            std::bind(&NavigationClient::on_localization_result, this, std::placeholders::_1);

        localization_client_->async_send_goal(goal_msg, opts);
        RCLCPP_INFO(get_logger(), "Localization request sent");
    }

    void on_localization_result(const GoalHandleLocalize::WrappedResult & result)
    {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED &&
            result.result && result.result->success)
        {
            robot_x_ = result.result->x;
            robot_y_ = result.result->y;
            RCLCPP_INFO(get_logger(),
                "\033[1;32mLocalization succeeded: x=%.2f y=%.2f\033[0m",
                robot_x_, robot_y_);
            transition(State::INITIALIZING_NAV2);
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Localization failed");
            transition(State::FAILED);
        }
    }

    // ══════════════════════════════════════════════════════════════════════════
    //  NAV2 INIT
    // ══════════════════════════════════════════════════════════════════════════
    void initialize_nav2()
    {
        if (!navigation_client_->wait_for_action_server(std::chrono::seconds(5)))
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "Waiting for Nav2 action server...");
            return;
        }
        action_in_progress_ = true;  // prevent tick() from re-entering while we wait

        waitForMap();  // ← blocks until rtabmap publishes a valid map
        // Brief settle time so Nav2 costmaps populate
        std::this_thread::sleep_for(std::chrono::seconds(2));
        RCLCPP_INFO(get_logger(), "\033[1;32mNav2 ready — starting cycle %d\033[0m",
                    current_cycle_ + 1);
        transition(State::NAVIGATING_TO_DIG);
    }

    // ══════════════════════════════════════════════════════════════════════════
    //  NAVIGATION (shared for all nav goals)
    // ══════════════════════════════════════════════════════════════════════════
    void request_navigation(const GoalXY & goal)
    {
        if (!navigation_client_->wait_for_action_server(std::chrono::seconds(1)))
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "Nav2 not available");
            return;
        }

        action_in_progress_ = true;

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.header.stamp    = this->now();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.pose.position.x = goal.x;
        goal_msg.pose.pose.position.y = goal.y;
        goal_msg.pose.pose.position.z = 0.0;
        goal_msg.pose.pose.orientation = yaw_to_quat(goal.yaw_deg);

        auto opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        opts.goal_response_callback =
            std::bind(&NavigationClient::on_nav_goal_response, this, std::placeholders::_1);
        opts.feedback_callback =
            std::bind(&NavigationClient::on_nav_feedback, this,
                      std::placeholders::_1, std::placeholders::_2);
        opts.result_callback =
            std::bind(&NavigationClient::on_nav_result, this, std::placeholders::_1);

        navigation_client_->async_send_goal(goal_msg, opts);
        RCLCPP_INFO(get_logger(), "Navigating to (%.2f, %.2f) yaw=%.0f°",
                    goal.x, goal.y, goal.yaw_deg);
    }

    void on_nav_goal_response(const GoalHandleNavigate::SharedPtr & goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_ERROR(get_logger(), "Navigation goal rejected by Nav2");
            transition(State::FAILED);
        }
        else
        {
            RCLCPP_INFO(get_logger(), "Navigation goal accepted");
        }
    }
    
    void waitForMap()
    {
        RCLCPP_INFO(get_logger(), "Waiting for valid map from rtabmap...");
        auto map_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", rclcpp::QoS(1).transient_local(),
            [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
                if (!msg->data.empty()) {
                    map_ready_ = true;
                    RCLCPP_INFO(get_logger(), "Map received (%zu cells) — ready to navigate",
                                msg->data.size());
                }
            });

        rclcpp::Rate rate(2);
        while (rclcpp::ok() && !map_ready_) {
            rclcpp::spin_some(this->get_node_base_interface());
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000, "Still waiting for map...");
            rate.sleep();
        }
    }

    void on_nav_feedback(GoalHandleNavigate::SharedPtr,
                         const std::shared_ptr<const NavigateToPose::Feedback> feedback)
    {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                             "Distance remaining: %.2f m", feedback->distance_remaining);
    }

    void on_nav_result(const GoalHandleNavigate::WrappedResult & result)
    {
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(get_logger(), "\033[1;32mNavigation goal reached\033[0m");
            advance_from_navigation();
            break;

        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(get_logger(), "\033[1;31mNavigation aborted\033[0m");
            transition(State::FAILED);
            break;

        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(get_logger(), "Navigation canceled");
            transition(State::FAILED);
            break;

        default:
            RCLCPP_ERROR(get_logger(), "Unknown navigation result");
            transition(State::FAILED);
            break;
        }
    }

    /** Called after any nav goal succeeds — advances to the next logical state. */
    void advance_from_navigation()
    {
        switch (state_)
        {
        case State::NAVIGATING_TO_DIG:
            transition(State::EXCAVATING);
            break;

        case State::NAVIGATING_TO_DEPOSIT:
            transition(State::DEPOSITING);
            break;

        case State::NAVIGATING_TO_START:
            // Finished returning home — start next cycle or finish
            ++current_cycle_;
            if (current_cycle_ < max_cycles_)
            {
                RCLCPP_INFO(get_logger(), "Starting cycle %d of %d",
                            current_cycle_ + 1, max_cycles_);
                transition(State::NAVIGATING_TO_DIG);
            }
            else
            {
                transition(State::COMPLETE);
            }
            break;

        default:
            RCLCPP_ERROR(get_logger(), "Unexpected nav result in state %s", state_name(state_));
            transition(State::FAILED);
            break;
        }
    }

    // ══════════════════════════════════════════════════════════════════════════
    //  EXCAVATION
    // ══════════════════════════════════════════════════════════════════════════
    void request_excavation()
    {
        if (!excavation_client_->wait_for_action_server(std::chrono::seconds(1)))
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "Waiting for excavation action server...");
            return;
        }

        action_in_progress_ = true;

        auto goal_msg = ExcavationAction::Goal();
        auto opts     = rclcpp_action::Client<ExcavationAction>::SendGoalOptions();

        opts.feedback_callback =
            std::bind(&NavigationClient::on_excavation_feedback, this,
                      std::placeholders::_1, std::placeholders::_2);
        opts.result_callback =
            std::bind(&NavigationClient::on_excavation_result, this, std::placeholders::_1);

        excavation_client_->async_send_goal(goal_msg, opts);
        RCLCPP_INFO(get_logger(), "Excavation request sent");
    }

    void on_excavation_feedback(
        GoalHandleExcavation::SharedPtr,
        const std::shared_ptr<const ExcavationAction::Feedback> feedback)
    {
        RCLCPP_INFO(get_logger(), "[Excavation] %s", feedback->feedback_message.c_str());
    }

    void on_excavation_result(const GoalHandleExcavation::WrappedResult & result)
    {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result->success)
        {
            RCLCPP_INFO(get_logger(), "\033[1;32mExcavation complete\033[0m");
            transition(State::NAVIGATING_TO_DEPOSIT);
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Excavation failed or was cancelled");
            transition(State::FAILED);
        }
    }

    // ══════════════════════════════════════════════════════════════════════════
    //  DEPOSITING
    // ══════════════════════════════════════════════════════════════════════════
    void request_depositing()
    {
        if (!depositing_client_->wait_for_action_server(std::chrono::seconds(1)))
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "Waiting for depositing action server...");
            return;
        }

        action_in_progress_ = true;

        auto goal_msg = DepositingAction::Goal();
        auto opts     = rclcpp_action::Client<DepositingAction>::SendGoalOptions();

        opts.feedback_callback =
            std::bind(&NavigationClient::on_depositing_feedback, this,
                      std::placeholders::_1, std::placeholders::_2);
        opts.result_callback =
            std::bind(&NavigationClient::on_depositing_result, this, std::placeholders::_1);

        depositing_client_->async_send_goal(goal_msg, opts);
        RCLCPP_INFO(get_logger(), "Depositing request sent");
    }

    void on_depositing_feedback(
        GoalHandleDepositing::SharedPtr,
        const std::shared_ptr<const DepositingAction::Feedback> feedback)
    {
        RCLCPP_INFO(get_logger(), "[Depositing] %s", feedback->feedback_message.c_str());
    }

    void on_depositing_result(const GoalHandleDepositing::WrappedResult & result)
    {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result->success)
        {
            RCLCPP_INFO(get_logger(), "\033[1;32mDepositing complete\033[0m");

            // After depositing: return to start, then decide whether to loop
            if (max_cycles_ > 1)
                transition(State::NAVIGATING_TO_START);
            else
            {
                ++current_cycle_;
                transition(State::COMPLETE);
            }
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Depositing failed or was cancelled");
            transition(State::FAILED);
        }
    }
};

// ── MAIN ─────────────────────────────────────────────────────────────────────
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NavigationClient>());
    rclcpp::shutdown();
    return 0;
}