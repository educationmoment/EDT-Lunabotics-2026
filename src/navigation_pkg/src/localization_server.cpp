#include <chrono>
#include <cmath>
#include <memory>
#include <thread>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "msg_pkg/action/localization.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "SparkMax.hpp"

// Motor CAN IDs (matching controller_node)
enum CAN_IDs
{
    LEFT_MOTOR = 1,
    RIGHT_MOTOR = 2
};

/**
 * @class LocalizationServer
 * @brief Handles localization by physically moving the robot using duty cycles
 *        to find and align with AprilTags 7 and 11.
 */
class LocalizationServer : public rclcpp::Node
{
public:
    using Localization = msg_pkg::action::Localization;
    using GoalHandleLocalization = rclcpp_action::ServerGoalHandle<Localization>;

    /**
     * @brief Constructor for LocalizationServer.
     * @param can_interface The CAN interface for motor communication
     */
    LocalizationServer(const std::string &can_interface)
        : Node("localization_server"),
          leftMotor(can_interface, LEFT_MOTOR),
          rightMotor(can_interface, RIGHT_MOTOR),
          success_(false),
          tag7_visible_(false),
          tag11_visible_(false),
          d455_found_tag7_(false),
          d455_found_tag11_(false),
          d456_found_tag7_(false),
          d456_found_tag11_(false),
          search_state_(SearchState::SEARCHING),
          rotation_direction_(1.0)  // 1.0 = counterclockwise, -1.0 = clockwise
    {
        // Initialize motors
        initializeMotors();

        // Create TF buffer and listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        action_server_ = rclcpp_action::create_server<Localization>(
            this, "localization_action",
            std::bind(&LocalizationServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&LocalizationServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&LocalizationServer::handle_accepted, this, std::placeholders::_1)
        );

        // Timer that runs localization logic every 100 ms
        localization_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&LocalizationServer::localize, this)
        );

        start_time_ = this->now();
        RCLCPP_INFO(get_logger(), "Localization server initialized with duty cycle control");
    }

    ~LocalizationServer()
    {
        stopMotors();
    }

private:
    // Motor controllers
    SparkMax leftMotor;
    SparkMax rightMotor;

    // TF components
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // ROS components
    rclcpp_action::Server<Localization>::SharedPtr action_server_;
    rclcpp::TimerBase::SharedPtr localization_timer_;
    rclcpp::Time start_time_;

    // Localization state
    bool success_;
    bool tag7_visible_;
    bool tag11_visible_;
    bool d455_found_tag7_;
    bool d455_found_tag11_;
    bool d456_found_tag7_;
    bool d456_found_tag11_;
    double depth_distance_;
    double lateral_distance_;
    double depth_distance_7_;
    double lateral_distance_7_;
    double depth_distance_11_;
    double lateral_distance_11_;
    double current_yaw_7_;
    double current_yaw_11_;

    // Search state machine
    enum class SearchState
    {
        SEARCHING,      // Rotating to find tags
        ALIGNING,       // Fine-tuning orientation
        LOCALIZED       // Successfully localized
    };
    SearchState search_state_;
    double rotation_direction_;

    // Duty cycle constants
    static constexpr double DUTY_CYCLE = 0.3;            // Static duty cycle for all movement
    static constexpr double YAW_TOLERANCE = 0.05;        // ~3 degrees tolerance
    static constexpr double TARGET_YAW = -1.57;           // 90 degrees (facing east)

    /**
     * @brief Initialize motor controllers with proper settings
     */
    void initializeMotors()
    {
        leftMotor.SetIdleMode(IdleMode::kBrake);
        rightMotor.SetIdleMode(IdleMode::kBrake);
        leftMotor.SetMotorType(MotorType::kBrushless);
        rightMotor.SetMotorType(MotorType::kBrushless);
        leftMotor.SetSensorType(SensorType::kHallSensor);
        rightMotor.SetSensorType(SensorType::kHallSensor);
        leftMotor.SetInverted(false);
        rightMotor.SetInverted(true);

        // PID settings
        leftMotor.SetP(0, 0.0002f);
        leftMotor.SetI(0, 0.0f);
        leftMotor.SetD(0, 0.0f);
        leftMotor.SetF(0, 0.00021f);

        rightMotor.SetP(0, 0.0002f);
        rightMotor.SetI(0, 0.0f);
        rightMotor.SetD(0, 0.0f);
        rightMotor.SetF(0, 0.00021f);

        leftMotor.BurnFlash();
        rightMotor.BurnFlash();

        RCLCPP_INFO(get_logger(), "Motors initialized");
    }

    /**
     * @brief Stop both drive motors
     */
    void stopMotors()
    {
        leftMotor.SetDutyCycle(0.0f);
        rightMotor.SetDutyCycle(0.0f);
        leftMotor.Heartbeat();
        rightMotor.Heartbeat();
    }

    /**
     * @brief Rotate robot in place using duty cycle
     * @param duty_cycle Positive = counterclockwise, negative = clockwise
     */
    void rotateInPlace(double duty_cycle)
    {
        // For rotation in place: left motor backward, right motor forward (or vice versa)
        leftMotor.SetDutyCycle(static_cast<float>(-duty_cycle));
        rightMotor.SetDutyCycle(static_cast<float>(duty_cycle));
        leftMotor.Heartbeat();
        rightMotor.Heartbeat();
    }

    // Action server callbacks
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const Localization::Goal> goal)
    {
        (void)uuid;
        (void)goal;
        RCLCPP_INFO(get_logger(), "Received localization goal - starting tag search");
        search_state_ = SearchState::SEARCHING;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleLocalization> goal_handle)
    {
        (void)goal_handle;
        RCLCPP_INFO(get_logger(), "Received request to cancel goal");
        stopMotors();
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleLocalization> goal_handle)
    {
        std::thread{std::bind(&LocalizationServer::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    /**
     * @brief Execute the localization goal
     */
    void execute(const std::shared_ptr<GoalHandleLocalization> goal_handle)
    {
        auto result = std::make_shared<Localization::Result>();

        // Wait until localization succeeds or times out
        auto timeout = std::chrono::seconds(60);
        auto start = std::chrono::steady_clock::now();

        while (!success_ && rclcpp::ok())
        {
            if (std::chrono::steady_clock::now() - start > timeout)
            {
                RCLCPP_ERROR(get_logger(), "Localization timed out");
                stopMotors();
                result->success = false;
                goal_handle->abort(result);
                return;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        if (success_)
        {
            stopMotors();
            result->x = depth_distance_ + 0.1;      // Center of robot offset
            result->y = lateral_distance_ + 1.0;    // Tag offset from corner
            result->success = true;
            goal_handle->succeed(result);
            RCLCPP_INFO(get_logger(), "Localization complete: x=%.2f, y=%.2f", result->x, result->y);
        }
        else
        {
            stopMotors();
            result->success = false;
            goal_handle->abort(result);
        }
    }

    /**
     * @brief Main localization loop - runs at 10Hz
     */
    void localize()
    {
        if (!d455_found_tag7_ && !d455_found_tag11_)
        {
            try
            {
                auto tag7_to_d455 = tf_buffer_->lookupTransform("d455_link", "tag36h11:7", tf2::TimePointZero);
                auto d455_to_base = tf_buffer_->lookupTransform("base_link", "d455_link", tf2::TimePointZero);

                tf2::Transform tag7_to_d455_tf;
                tf2::fromMsg(tag7_to_d455.transform, tag7_to_d455_tf);

                tf2::Transform d455_to_base_tf;
                tf2::fromMsg(d455_to_base.transform, d455_to_base_tf);

                tf2::Transform tag7_to_base_tf = d455_to_base_tf * tag7_to_d455_tf;

                depth_distance_7_ = -tag7_to_base_tf.getOrigin().x();
                lateral_distance_7_ = -tag7_to_base_tf.getOrigin().y();
                current_yaw_7_ = tf2::getYaw(tag7_to_base_tf.getRotation());
                d455_found_tag7_ = true;

                RCLCPP_INFO(get_logger(), "D455 found Tag 7: depth=%.2f, lateral=%.2f, yaw=%.2f",
                    depth_distance_7_, lateral_distance_7_, current_yaw_7_);
            }
            catch (tf2::TransformException &ex)
            {
                // D455 tries Tag 11 if Tag 7 not visible and D456 hasn't found Tag 11
                if (!d456_found_tag11_)
                {
                    try
                    {
                        auto tag11_to_d455 = tf_buffer_->lookupTransform("d455_link", "tag36h11:11", tf2::TimePointZero);
                        auto d455_to_base = tf_buffer_->lookupTransform("base_link", "d455_link", tf2::TimePointZero);

                        tf2::Transform tag11_to_d455_tf;
                        tf2::fromMsg(tag11_to_d455.transform, tag11_to_d455_tf);

                        tf2::Transform d455_to_base_tf;
                        tf2::fromMsg(d455_to_base.transform, d455_to_base_tf);

                        tf2::Transform tag11_to_base_tf = d455_to_base_tf * tag11_to_d455_tf;

                        depth_distance_11_ = -tag11_to_base_tf.getOrigin().x();
                        lateral_distance_11_ = -tag11_to_base_tf.getOrigin().y();
                        current_yaw_11_ = tf2::getYaw(tag11_to_base_tf.getRotation());
                        d455_found_tag11_ = true;

                        RCLCPP_INFO(get_logger(), "D455 found Tag 11: depth=%.2f, lateral=%.2f, yaw=%.2f",
                            depth_distance_11_, lateral_distance_11_, current_yaw_11_);
                    }
                    catch (tf2::TransformException &ex2)
                    {
                        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000,
                            "D455: No tags visible");
                    }
                }
            }
        }

        // D456 camera - look for the OTHER tag that D455 didn't find
        if (!d456_found_tag7_ && !d456_found_tag11_)
        {
            // If D455 found Tag 7, D456 MUST look for Tag 11
            if (d455_found_tag7_)
            {
                try
                {
                    auto tag11_to_d456 = tf_buffer_->lookupTransform("d456_link", "tag36h11:11", tf2::TimePointZero);
                    auto d456_to_base = tf_buffer_->lookupTransform("base_link", "d456_link", tf2::TimePointZero);

                    tf2::Transform tag11_to_d456_tf;
                    tf2::fromMsg(tag11_to_d456.transform, tag11_to_d456_tf);

                    tf2::Transform d456_to_base_tf;
                    tf2::fromMsg(d456_to_base.transform, d456_to_base_tf);

                    tf2::Transform tag11_to_base_tf = d456_to_base_tf * tag11_to_d456_tf;

                    depth_distance_11_ = -tag11_to_base_tf.getOrigin().x();
                    lateral_distance_11_ = -tag11_to_base_tf.getOrigin().y();
                    current_yaw_11_ = tf2::getYaw(tag11_to_base_tf.getRotation());
                    d456_found_tag11_ = true;

                    RCLCPP_INFO(get_logger(), "D456 found Tag 11: depth=%.2f, lateral=%.2f, yaw=%.2f",
                        depth_distance_11_, lateral_distance_11_, current_yaw_11_);
                }
                catch (tf2::TransformException &ex)
                {
                    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000,
                        "D456: Tag 11 not visible");
                }
            }
            else if (d455_found_tag11_)
            {
                try
                {
                    auto tag7_to_d456 = tf_buffer_->lookupTransform("d456_link", "tag36h11:7", tf2::TimePointZero);
                    auto d456_to_base = tf_buffer_->lookupTransform("base_link", "d456_link", tf2::TimePointZero);

                    tf2::Transform tag7_to_d456_tf;
                    tf2::fromMsg(tag7_to_d456.transform, tag7_to_d456_tf);

                    tf2::Transform d456_to_base_tf;
                    tf2::fromMsg(d456_to_base.transform, d456_to_base_tf);

                    tf2::Transform tag7_to_base_tf = d456_to_base_tf * tag7_to_d456_tf;

                    depth_distance_7_ = -tag7_to_base_tf.getOrigin().x();
                    lateral_distance_7_ = -tag7_to_base_tf.getOrigin().y();
                    current_yaw_7_ = tf2::getYaw(tag7_to_base_tf.getRotation());
                    d456_found_tag7_ = true;

                    RCLCPP_INFO(get_logger(), "D456 found Tag 7: depth=%.2f, lateral=%.2f, yaw=%.2f",
                        depth_distance_7_, lateral_distance_7_, current_yaw_7_);
                }
                catch (tf2::TransformException &ex)
                {
                    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000,
                        "D456: Tag 7 not visible");
                }
            }
            else
            {
                try
                {
                    auto tag11_to_d456 = tf_buffer_->lookupTransform("d456_link", "tag36h11:11", tf2::TimePointZero);
                    auto d456_to_base = tf_buffer_->lookupTransform("base_link", "d456_link", tf2::TimePointZero);

                    tf2::Transform tag11_to_d456_tf;
                    tf2::fromMsg(tag11_to_d456.transform, tag11_to_d456_tf);

                    tf2::Transform d456_to_base_tf;
                    tf2::fromMsg(d456_to_base.transform, d456_to_base_tf);

                    tf2::Transform tag11_to_base_tf = d456_to_base_tf * tag11_to_d456_tf;

                    depth_distance_11_ = -tag11_to_base_tf.getOrigin().x();
                    lateral_distance_11_ = -tag11_to_base_tf.getOrigin().y();
                    current_yaw_11_ = tf2::getYaw(tag11_to_base_tf.getRotation());
                    d456_found_tag11_ = true;

                    RCLCPP_INFO(get_logger(), "D456 found Tag 11: depth=%.2f, lateral=%.2f, yaw=%.2f",
                        depth_distance_11_, lateral_distance_11_, current_yaw_11_);
                }
                catch (tf2::TransformException &ex)
                {
                    // D456 tries Tag 7 if Tag 11 not visible
                    try
                    {
                        auto tag7_to_d456 = tf_buffer_->lookupTransform("d456_link", "tag36h11:7", tf2::TimePointZero);
                        auto d456_to_base = tf_buffer_->lookupTransform("base_link", "d456_link", tf2::TimePointZero);

                        tf2::Transform tag7_to_d456_tf;
                        tf2::fromMsg(tag7_to_d456.transform, tag7_to_d456_tf);

                        tf2::Transform d456_to_base_tf;
                        tf2::fromMsg(d456_to_base.transform, d456_to_base_tf);

                        tf2::Transform tag7_to_base_tf = d456_to_base_tf * tag7_to_d456_tf;

                        depth_distance_7_ = -tag7_to_base_tf.getOrigin().x();
                        lateral_distance_7_ = -tag7_to_base_tf.getOrigin().y();
                        current_yaw_7_ = tf2::getYaw(tag7_to_base_tf.getRotation());
                        d456_found_tag7_ = true;

                        RCLCPP_INFO(get_logger(), "D456 found Tag 7: depth=%.2f, lateral=%.2f, yaw=%.2f",
                            depth_distance_7_, lateral_distance_7_, current_yaw_7_);
                    }
                    catch (tf2::TransformException &ex2)
                    {
                        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000,
                            "D456: No tags visible");
                    }
                }
            }
        }

        // Update visibility based on what was found
        tag7_visible_ = d455_found_tag7_ || d456_found_tag7_;
        tag11_visible_ = d455_found_tag11_ || d456_found_tag11_;

        // State machine for localization
        switch (search_state_)
        {
        case SearchState::SEARCHING:
            handleSearching();
            break;

        case SearchState::ALIGNING:
            handleAligning();
            break;

        case SearchState::LOCALIZED:
            // Already done, motors should be stopped
            break;
        }
    }

    /**
     * @brief Handle SEARCHING state - rotate to find tags
     */
    void handleSearching()
    {
        if (tag7_visible_ && tag11_visible_)
        {
            stopMotors();
            
            // Average both tag positions
            depth_distance_ = (depth_distance_7_ + depth_distance_11_) / 2.0;
            lateral_distance_ = (lateral_distance_7_ + lateral_distance_11_) / 2.0;
            
            RCLCPP_INFO(get_logger(), "Both tags detected! Transitioning to face forward (Tag 7 behind robot).");
            search_state_ = SearchState::ALIGNING;
        }
        else
        {
            // Continue rotating to search for tags
            rotateInPlace(DUTY_CYCLE * rotation_direction_);
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                "Searching for AprilTags... (duty cycle: %.2f) Tag7: %s, Tag11: %s", 
                DUTY_CYCLE,
                tag7_visible_ ? "YES" : "NO",
                tag11_visible_ ? "YES" : "NO");
        }
    }

    /**
     * @brief Handle ALIGNING state - rotate until yaw matches TARGET_YAW
     */
    void handleAligning()
    {
        // Re-check for Tag 7 to get current yaw
        bool tag7_still_visible = false;
        double current_tag7_yaw = 0.0;
        
        // Check D455 for Tag 7
        try
        {
            auto tag7_to_d455 = tf_buffer_->lookupTransform("d455_link", "tag36h11:7", tf2::TimePointZero);
            auto d455_to_base = tf_buffer_->lookupTransform("base_link", "d455_link", tf2::TimePointZero);

            tf2::Transform tag7_to_d455_tf;
            tf2::fromMsg(tag7_to_d455.transform, tag7_to_d455_tf);

            tf2::Transform d455_to_base_tf;
            tf2::fromMsg(d455_to_base.transform, d455_to_base_tf);

            tf2::Transform tag7_to_base_tf = d455_to_base_tf * tag7_to_d455_tf;
            current_tag7_yaw = tf2::getYaw(tag7_to_base_tf.getRotation());
            tag7_still_visible = true;
        }
        catch (tf2::TransformException &ex)
        {
            // Try D456 for Tag 7
            try
            {
                auto tag7_to_d456 = tf_buffer_->lookupTransform("d456_link", "tag36h11:7", tf2::TimePointZero);
                auto d456_to_base = tf_buffer_->lookupTransform("base_link", "d456_link", tf2::TimePointZero);

                tf2::Transform tag7_to_d456_tf;
                tf2::fromMsg(tag7_to_d456.transform, tag7_to_d456_tf);

                tf2::Transform d456_to_base_tf;
                tf2::fromMsg(d456_to_base.transform, d456_to_base_tf);

                tf2::Transform tag7_to_base_tf = d456_to_base_tf * tag7_to_d456_tf;
                current_tag7_yaw = tf2::getYaw(tag7_to_base_tf.getRotation());
                tag7_still_visible = true;
            }
            catch (tf2::TransformException &ex2)
            {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                    "Lost Tag 7 during alignment, continuing rotation...");
            }
        }

        if (tag7_still_visible)
        {
            double yaw_error = current_tag7_yaw - TARGET_YAW;
            
            while (yaw_error > M_PI) yaw_error -= 2.0 * M_PI;
            while (yaw_error < -M_PI) yaw_error += 2.0 * M_PI;
            
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                "Aligning: Tag7 yaw=%.3f, target=%.3f, error=%.3f", current_tag7_yaw, TARGET_YAW, yaw_error);

            if (std::abs(yaw_error) < YAW_TOLERANCE)
            {
                stopMotors();
                success_ = true;
                search_state_ = SearchState::LOCALIZED;
                RCLCPP_INFO(get_logger(), "Alignment complete! Yaw matches target. Localization successful.");
            }
            else
            {
                double correction_direction = (yaw_error > 0) ? 1.0 : -1.0;
                rotateInPlace(DUTY_CYCLE * correction_direction);
            }
        }
        else
        {
            rotateInPlace(DUTY_CYCLE * 0.5);
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    std::string can_interface = "can0";
    auto temp_node = rclcpp::Node::make_shared("localization_param_node");
    temp_node->declare_parameter<std::string>("can_interface", "can0");
    temp_node->get_parameter("can_interface", can_interface);

    auto node = std::make_shared<LocalizationServer>(can_interface);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
