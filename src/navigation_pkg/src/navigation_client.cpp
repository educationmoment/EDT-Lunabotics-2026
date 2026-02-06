
#include <array>
#include <chrono>
#include <cmath>
#include <thread>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "msg_pkg/action/localization.hpp"
#include "msg_pkg/action/excavation.hpp"
#include "SparkMax.hpp"

// Motor CAN IDs (matching controller_node)
enum CAN_IDs
{
    LEFT_MOTOR = 1,
    RIGHT_MOTOR = 2
};

class NavigationClient : public rclcpp::Node
{
public:
    using Excavation = msg_pkg::action::Excavation;
    using GoalHandleExcavation = rclcpp_action::ClientGoalHandle<Excavation>;
    using Localization = msg_pkg::action::Localization;
    using GoalHandleLocalization = rclcpp_action::ClientGoalHandle<Localization>;

    /**
     * @brief Constructor for NavigationClient
     * @param can_interface The CAN interface for motor communication
     */
    NavigationClient(const std::string &can_interface)
        : Node("navigation_client"),
          leftMotor_(can_interface, LEFT_MOTOR),
          rightMotor_(can_interface, RIGHT_MOTOR),
          start_navigation_(false),
          start_excavation_(false),
          localization_in_progress_(false),
          localized_(false),
          navigation_in_progress_(false),
          current_x_(0.0),
          current_y_(0.0),
          current_yaw_(0.0)
    {
        // Initialize motors
        initializeMotors();

        // Create action clients
        excavation_client_ = rclcpp_action::create_client<Excavation>(this, "excavation_action");
        localization_client_ = rclcpp_action::create_client<Localization>(this, "localization_action");

        // Main execution timer - runs every second
        execution_timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&NavigationClient::execute, this)
        );

        RCLCPP_INFO(get_logger(), "Navigation client initialized with duty cycle control");
    }

    ~NavigationClient()
    {
        stopMotors();
    }

private:
    // Motor controllers
    SparkMax leftMotor_;
    SparkMax rightMotor_;

    // Goal positions
    struct GoalXY { double x; double y; };
    const std::array<GoalXY, 3> goals_{{{5.0, 1.7}, {5.0, 1.3}, {5.0, 1.2}}};

    // State flags
    bool start_navigation_;
    bool start_excavation_;
    bool localization_in_progress_;
    bool localized_;
    bool navigation_in_progress_;

    // Current robot pose (from localization)
    double current_x_;
    double current_y_;
    double current_yaw_;

    // Action clients
    rclcpp_action::Client<Excavation>::SharedPtr excavation_client_;
    rclcpp_action::Client<Localization>::SharedPtr localization_client_;
    rclcpp::TimerBase::SharedPtr execution_timer_;

    std::size_t current_goal_index_{0};

    // Navigation parameters
    static constexpr double DRIVE_DUTY_CYCLE = 0.35;      // Forward/backward movement
    static constexpr double TURN_DUTY_CYCLE = 0.25;       // Turning in place
    static constexpr double SLOW_DUTY_CYCLE = 0.2;        // Slow approach to goal
    static constexpr double POSITION_TOLERANCE = 0.15;    // 15cm tolerance
    static constexpr double HEADING_TOLERANCE = 0.1;      // ~6 degrees tolerance

    /**
     * @brief Initialize motor controllers
     */
    void initializeMotors()
    {
        leftMotor_.SetIdleMode(IdleMode::kBrake);
        rightMotor_.SetIdleMode(IdleMode::kBrake);
        leftMotor_.SetMotorType(MotorType::kBrushless);
        rightMotor_.SetMotorType(MotorType::kBrushless);
        leftMotor_.SetSensorType(SensorType::kHallSensor);
        rightMotor_.SetSensorType(SensorType::kHallSensor);
        leftMotor_.SetInverted(false);
        rightMotor_.SetInverted(true);

        leftMotor_.SetP(0, 0.0002f);
        leftMotor_.SetI(0, 0.0f);
        leftMotor_.SetD(0, 0.0f);
        leftMotor_.SetF(0, 0.00021f);

        rightMotor_.SetP(0, 0.0002f);
        rightMotor_.SetI(0, 0.0f);
        rightMotor_.SetD(0, 0.0f);
        rightMotor_.SetF(0, 0.00021f);

        leftMotor_.BurnFlash();
        rightMotor_.BurnFlash();

        RCLCPP_INFO(get_logger(), "Motors initialized");
    }

    /**
     * @brief Stop both drive motors
     */
    void stopMotors()
    {
        leftMotor_.SetDutyCycle(0.0f);
        rightMotor_.SetDutyCycle(0.0f);
    }

    /**
     * @brief Drive robot forward/backward using duty cycle
     * @param duty_cycle Positive = forward, negative = backward
     */
    void driveForward(double duty_cycle)
    {
        leftMotor_.SetDutyCycle(static_cast<float>(duty_cycle));
        rightMotor_.SetDutyCycle(static_cast<float>(duty_cycle));
    }

    /**
     * @brief Rotate robot in place using duty cycle
     * @param duty_cycle Positive = counterclockwise, negative = clockwise
     */
    void rotateInPlace(double duty_cycle)
    {
        leftMotor_.SetDutyCycle(static_cast<float>(-duty_cycle));
        rightMotor_.SetDutyCycle(static_cast<float>(duty_cycle));
    }

    /**
     * @brief Arc turn while moving (differential drive)
     * @param forward_duty Forward component
     * @param turn_bias Turn component (positive = turn left)
     */
    void arcTurn(double forward_duty, double turn_bias)
    {
        double left_duty = forward_duty - turn_bias;
        double right_duty = forward_duty + turn_bias;

        // Clamp values
        left_duty = std::clamp(left_duty, -1.0, 1.0);
        right_duty = std::clamp(right_duty, -1.0, 1.0);

        leftMotor_.SetDutyCycle(static_cast<float>(left_duty));
        rightMotor_.SetDutyCycle(static_cast<float>(right_duty));
    }

    /**
     * @brief Main execution loop
     */
    void execute()
    {
        if (!localized_ && !localization_in_progress_)
        {
            request_localization();
        }
        else if (localized_ && start_navigation_ && !navigation_in_progress_)
        {
            navigate_to_goal();
        }
        else if (start_excavation_)
        {
            request_excavation();
        }
    }

    /**
     * @brief Request localization from the localization server
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
            current_x_ = result.result->x;
            current_y_ = result.result->y;
            current_yaw_ = 1.57;  // Facing east after localization

            RCLCPP_INFO(get_logger(),
                "Localization succeeded: x=%.2f y=%.2f",
                current_x_, current_y_);

            localized_ = true;
            start_navigation_ = true;

            // Brief pause before navigation
            std::this_thread::sleep_for(std::chrono::seconds(2));
        }
        else if (result.code == rclcpp_action::ResultCode::SUCCEEDED &&
                 result.result && !result.result->success)
        {
            RCLCPP_WARN(get_logger(),
                "Localization attempt failed – will retry");

            // Small rotation to try different orientation
            rotateInPlace(TURN_DUTY_CYCLE);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            stopMotors();
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Localization failed completely");
            rclcpp::shutdown();
        }
    }

    /**
     * @brief Navigate to the current goal using duty cycle control
     */
    void navigate_to_goal()
    {
        navigation_in_progress_ = true;

        GoalXY goal = goals_[current_goal_index_];
        RCLCPP_INFO(get_logger(), "Navigating to goal %zu: (%.2f, %.2f)",
            current_goal_index_, goal.x, goal.y);

        // Run navigation loop in separate thread
        std::thread([this, goal]() {
            bool goal_reached = navigate_with_duty_cycle(goal.x, goal.y);

            if (goal_reached)
            {
                RCLCPP_INFO(get_logger(), "\033[1;32mGoal %zu reached!\033[0m",
                    current_goal_index_);
                navigation_in_progress_ = false;
                start_navigation_ = false;
                start_excavation_ = true;
            }
            else
            {
                RCLCPP_ERROR(get_logger(), "\033[1;31mNAVIGATION FAILED\033[0m");
                stopMotors();
                rclcpp::shutdown();
            }
        }).detach();
    }

    /**
     * @brief Navigate to a goal position using duty cycle control
     * @param goal_x Target X position
     * @param goal_y Target Y position
     * @return true if goal reached, false otherwise
     */
    bool navigate_with_duty_cycle(double goal_x, double goal_y)
    {
        const int MAX_ITERATIONS = 1000;  // Timeout protection
        int iteration = 0;

        while (rclcpp::ok() && iteration < MAX_ITERATIONS)
        {
            // Calculate distance and heading to goal
            double dx = goal_x - current_x_;
            double dy = goal_y - current_y_;
            double distance = std::sqrt(dx * dx + dy * dy);
            double target_heading = std::atan2(dy, dx);
            double heading_error = normalizeAngle(target_heading - current_yaw_);

            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                "Distance: %.2f, Heading error: %.2f, Pos: (%.2f, %.2f)",
                distance, heading_error, current_x_, current_y_);

            // Check if goal reached
            if (distance < POSITION_TOLERANCE)
            {
                stopMotors();
                RCLCPP_INFO(get_logger(), "Goal reached!");
                return true;
            }

            // Phase 1: Turn to face the goal if heading error is large
            if (std::abs(heading_error) > HEADING_TOLERANCE)
            {
                double turn_duty = (heading_error > 0) ? TURN_DUTY_CYCLE : -TURN_DUTY_CYCLE;

                // Reduce turn speed for small errors
                if (std::abs(heading_error) < 0.3)
                {
                    turn_duty *= 0.6;
                }

                rotateInPlace(turn_duty);

                // Estimate yaw change (rough approximation)
                double yaw_rate = 0.3 * turn_duty;  // Tune this based on robot
                current_yaw_ += yaw_rate * 0.1;     // 100ms update rate
                current_yaw_ = normalizeAngle(current_yaw_);
            }
            // Phase 2: Drive toward goal with minor corrections
            else
            {
                double drive_duty = DRIVE_DUTY_CYCLE;

                // Slow down as we approach
                if (distance < 0.5)
                {
                    drive_duty = SLOW_DUTY_CYCLE;
                }

                // Apply small correction while driving
                double correction = heading_error * 0.5;  // Proportional correction
                arcTurn(drive_duty, correction);

                // Estimate position change (rough approximation)
                double speed = 0.2 * drive_duty;  // Tune based on robot speed
                current_x_ += speed * std::cos(current_yaw_) * 0.1;
                current_y_ += speed * std::sin(current_yaw_) * 0.1;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            iteration++;
        }

        stopMotors();
        RCLCPP_WARN(get_logger(), "Navigation timed out");
        return false;
    }

    /**
     * @brief Normalize angle to [-pi, pi]
     */
    double normalizeAngle(double angle)
    {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    /**
     * @brief Request excavation action
     */
    void request_excavation()
    {
        if (!excavation_client_->wait_for_action_server(std::chrono::seconds(1)))
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "Waiting for excavation action server...");
            return;
        }

        auto goal_msg = Excavation::Goal();
        auto send_goal_options = rclcpp_action::Client<Excavation>::SendGoalOptions();
        send_goal_options.result_callback =
            std::bind(&NavigationClient::handle_excavation_result, this, std::placeholders::_1);

        excavation_client_->async_send_goal(goal_msg, send_goal_options);
        start_excavation_ = false;

        RCLCPP_INFO(get_logger(), "Excavation request sent");
    }

    /**
     * @brief Handle excavation result
     */
    void handle_excavation_result(const GoalHandleExcavation::WrappedResult &result)
    {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
        {
            ++current_goal_index_;

            if (current_goal_index_ < goals_.size())
            {
                RCLCPP_INFO(get_logger(), "Excavation complete, moving to next goal");
                start_navigation_ = true;
            }
            else
            {
                RCLCPP_INFO(get_logger(), "\033[1;32mCOMPLETED ALL THREE CYCLES\033[0m");
                stopMotors();
                rclcpp::shutdown();
            }
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "\033[1;31mEXCAVATION FAILED\033[0m");
            stopMotors();
            rclcpp::shutdown();
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Get CAN interface from parameter
    std::string can_interface = "can0";
    auto temp_node = rclcpp::Node::make_shared("navigation_param_node");
    temp_node->declare_parameter<std::string>("can_interface", "can0");
    temp_node->get_parameter("can_interface", can_interface);

    auto node = std::make_shared<NavigationClient>(can_interface);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
