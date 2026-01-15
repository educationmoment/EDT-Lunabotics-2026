/**
 * @file controller_teleop.cpp
 * @author Grayson Arendt
 * @date 4/17/2025
 */

#include <algorithm>
#include <array>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "SparkMax.hpp"

/**
 * @class ControllerTeleop
 * @brief Handles joystick/navigation commands and drives motors.
 */
class ControllerTeleop : public rclcpp::Node
{
public:
  /**
   * @brief Constructor for ControllerTeleop.
   */
  ControllerTeleop()
  : Node("controller_teleop"),
    left_wheel_motor_("can0", 1),
    right_wheel_motor_("can0", 2),
    lift_actuator_motor_("can0", 3)
    //oscillation_state_(OscillationState::IDLE),
    //last_oscillation_time_(std::chrono::steady_clock::now()),
    //fast_duration_(std::chrono::milliseconds(100)),
    //slow_duration_(std::chrono::milliseconds(200))
  {
    velocity_subscriber_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10,
      std::bind(&ControllerTeleop::velocity_callback, this, std::placeholders::_1));

    joystick_subscriber_ = create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10,
      std::bind(&ControllerTeleop::joy_callback, this, std::placeholders::_1));

    //lift_actuator_motor_.SetAnalogSensorMode(1);
    //lift_actuator_motor_.SetAnalogPositionConversion(1.0f);
    //lift_actuator_motor_.SetAnalogVelocityConversion(1.0f);
    //lift_actuator_motor_.SetAnalogAverageDepth(0);
    //lift_actuator_motor_.SetAnalogSampleDelta(0);
    /*
    left_wheel_motor_.SetIdleMode(IdleMode::kBrake);
    right_wheel_motor_.SetIdleMode(IdleMode::kBrake);
    left_wheel_motor_.SetMotorType(MotorType::kBrushless);
    right_wheel_motor_.SetMotorType(MotorType::kBrushless);
    left_wheel_motor_.SetSensorType(SensorType::kHallSensor);
    right_wheel_motor_.SetSensorType(SensorType::kHallSensor);
    left_wheel_motor_.SetInverted(false);
    right_wheel_motor_.SetInverted(true);
    left_wheel_motor_.SetP(0, 0.0002f);
    left_wheel_motor_.SetI(0, 0.0f);
    left_wheel_motor_.SetD(0, 0.0f);
    left_wheel_motor_.SetF(0, 0.00021f);
    right_wheel_motor_.SetP(0, 0.0002f);
    right_wheel_motor_.SetI(0, 0.0f);
    right_wheel_motor_.SetD(0, 0.0f);
    right_wheel_motor_.SetF(0, 0.00021f);
    left_wheel_motor_.BurnFlash();
    right_wheel_motor_.BurnFlash();
    */

    RCLCPP_INFO(get_logger(), "\033[1;35mMANUAL CONTROL:\033[0m \033[1;32mENABLED\033[0m");
  }

private:
  /**
   * @enum OscillationState
   * @brief State machine states for the lift oscillation pattern.
   */

   
  enum class OscillationState { IDLE, MOVING_DOWN, MOVING_UP };
  
  /**
   * @brief Clamp helper for motor duty‑cycle.
   * @param v Value to clamp.
   * @return Value clamped between -1.0 and 1.0.
   */
   
  static double clamp(double v) {return std::clamp(v, -1.0, 1.0);}

  /**
   * @brief Drive the robot with the given left and right speeds.
   * @param left_speed Left wheel speed.
   * @param right_speed Right wheel speed.
   */
  void drive(double left_speed, double right_speed)
  {
    if (abs(left_speed) <= 0.1 && abs(right_speed) <= 0.1) {
      left_wheel_motor_.SetDutyCycle(0.0);
      right_wheel_motor_.SetDutyCycle(0.0);
      return;
    }

    left_wheel_motor_.SetDutyCycle(clamp(left_speed));
    right_wheel_motor_.SetDutyCycle(clamp(right_speed));
  }

  /**
   * @brief Update the oscillation motion state machine.
   * @param trigger_value The current trigger value (0.0 to 1.0).
   * @param reverse_direction Whether to reverse the oscillation pattern.
   */

   /** 
void update_oscillation(double trigger_value, bool reverse_direction)
{
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_oscillation_time_);

    // Different speeds for movements
    const double fast_speed = 1.0;
    const double slow_speed = 0.8;
    const auto& current_down_duration = reverse_direction ? fast_duration_ : slow_duration_;
    const auto& current_up_duration = reverse_direction ? slow_duration_ : fast_duration_;

    switch (oscillation_state_) {
        case OscillationState::IDLE:
            oscillation_state_ = OscillationState::MOVING_UP;
            last_oscillation_time_ = now;
            lift_actuator_motor_.SetDutyCycle(
                reverse_direction ? fast_speed * trigger_value : -fast_speed * trigger_value);
            break;

        case OscillationState::MOVING_DOWN:
            if (elapsed >= current_down_duration) {
                oscillation_state_ = OscillationState::MOVING_UP;
                last_oscillation_time_ = now;
                lift_actuator_motor_.SetDutyCycle(
                    reverse_direction ? -slow_speed * trigger_value : slow_speed * trigger_value);
            }
            break;

        case OscillationState::MOVING_UP:
            if (elapsed >= current_up_duration) {
                oscillation_state_ = OscillationState::MOVING_DOWN;
                last_oscillation_time_ = now;
                lift_actuator_motor_.SetDutyCycle(
                    reverse_direction ? fast_speed * trigger_value : -fast_speed * trigger_value);
            }
            break;
    }
}
*/
  /**
   * @brief Process joystick messages in manual mode.
   * @param msg The joystick message.
   */
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    auto clock = rclcpp::Clock();

    // Xbox controller button mapping
    share_button_ = msg->buttons[7];
    menu_button_ = msg->buttons[6];
    home_button_ = msg->buttons[8];
    x_button_ = msg->buttons[2];

    // Xbox controller axes
    left_joystick_x_ = msg->axes[0];
    left_joystick_y_ = msg->axes[1];
    right_joystick_y_ = -msg->axes[4];  // Right stick Y (inverted)
    right_trigger_ = (1.0 - msg->axes[5]) / 2.0;  // (0.0 to 1.0)
    left_trigger_ = (1.0 - msg->axes[2]) / 2.0;   // (0.0 to 1.0)
    d_pad_vertical_ = msg->axes[7];  // D-pad vertical axis

    if (share_button_) {
      manual_enabled_ = true;
      RCLCPP_INFO_THROTTLE(
        get_logger(), clock, 1000, "\033[1;35mMANUAL CONTROL:\033[0m \033[1;32mENABLED\033[0m");
    }

    if (menu_button_) {
      manual_enabled_ = false;
      RCLCPP_INFO_THROTTLE(
        get_logger(), clock, 1000, "\033[1;33mAUTONOMOUS CONTROL:\033[0m \033[1;32mENABLED\033[0m");
    }

    if (home_button_) {
      robot_disabled_ = true;
      RCLCPP_ERROR(get_logger(), "\033[1;31mROBOT DISABLED\033[0m");
    }

    if (!manual_enabled_ || robot_disabled_) {return;}

    //lift_actuator_motor_.Heartbeat();
    left_wheel_motor_.Heartbeat();
    right_wheel_motor_.Heartbeat();

    // Driving logic
    double drive_speed_multiplier = x_button_ ? 1.0 : 0.7;
    double left_speed = left_joystick_y_ - left_joystick_x_;
    double right_speed = left_joystick_y_ + left_joystick_x_;

    if (d_pad_vertical_ != 0.0){
      left_speed = d_pad_vertical_;
      right_speed = d_pad_vertical_;
    }

    drive(left_speed * drive_speed_multiplier, right_speed * drive_speed_multiplier);

    // Lift control logic
    double lift_speed = right_joystick_y_;
    lift_speed = clamp(lift_speed);
    /** 
    if (right_trigger_ > 0.1) {  // Right trigger - down/up oscillation
      update_oscillation(right_trigger_, false);
    } else if (left_trigger_ > 0.1) {  // Left trigger - up/down oscillation
      update_oscillation(left_trigger_, true);
    } else if (abs(lift_speed) > 0.4) {  // Manual control
      oscillation_state_ = OscillationState::IDLE;
      //lift_actuator_motor_.SetDutyCycle(lift_speed);
    } else {  // No input
      oscillation_state_ = OscillationState::IDLE;
      //lift_actuator_motor_.SetDutyCycle(0.0);
    }
    */
    // Read actuator encoder position
    //double encoder_position = lift_actuator_motor_.GetAnalogPosition();
    //RCLCPP_INFO(get_logger(), "\033[1;36mLift Actuator Encoder Position: %f\033[0m", encoder_position);
  }

  /**
   * @brief Process /cmd_vel in autonomous mode.
   * @param msg The velocity command message.
   */
  void velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    if (!manual_enabled_) {
    
    left_wheel_motor_.Heartbeat();
    right_wheel_motor_.Heartbeat();
    //lift_actuator_motor_.Heartbeat();

    // Calculate left and right wheel speeds
    double wheel_radius = 0.1651;
    double wheel_base = 0.7;

    double left_cmd = -0.075 * (msg->linear.x + msg->angular.z * wheel_base / 2.0) / wheel_radius;
    double right_cmd = -0.075 * (msg->linear.x - msg->angular.z * wheel_base / 2.0) / wheel_radius;

    // Debug output
    RCLCPP_INFO(get_logger(), "Autonomous cmd: linear.x=%.3f, angular.z=%.3f -> left=%.3f, right=%.3f", 
                msg->linear.x, msg->angular.z, left_cmd, right_cmd);

    // Minimum command threshold
    if (left_cmd > 0.0 && left_cmd < 0.1) left_cmd = 0.1;
    if (right_cmd > 0.0 && right_cmd < 0.1) right_cmd = 0.1;

    drive(left_cmd, right_cmd);
    }
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joystick_subscriber_;

  SparkMax left_wheel_motor_, right_wheel_motor_, lift_actuator_motor_;

  bool manual_enabled_ = false, robot_disabled_ = false;
  bool share_button_, menu_button_, home_button_, x_button_;

  double left_joystick_x_, left_joystick_y_, right_joystick_y_;
  double d_pad_vertical_;
  double right_trigger_, left_trigger_;

  // Oscillation control variables
  //OscillationState oscillation_state_;
  //std::chrono::steady_clock::time_point last_oscillation_time_;
  //const std::chrono::milliseconds fast_duration_;
  //const std::chrono::milliseconds slow_duration_;
};

/**
 * @brief Main function
 * Initializes and spins the ControllerTeleop node.
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ControllerTeleop>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}