// File: src/cmd_vel_motor_driver/cmd_vel_motor_driver.cpp

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "SparkMax.hpp"

static constexpr double MAX_VELOCITY = 2500.0; // Max RPM

class CmdVelMotorDriver : public rclcpp::Node {
public:
  CmdVelMotorDriver()
  : Node("cmd_vel_motor_driver") {
    // Motor initialization
    left_motor_ = std::make_shared<SparkMax>("can0", 1);
    right_motor_ = std::make_shared<SparkMax>("can0", 2);

    left_motor_->SetIdleMode(IdleMode::kBrake);
    right_motor_->SetIdleMode(IdleMode::kBrake);
    left_motor_->SetMotorType(MotorType::kBrushless);
    right_motor_->SetMotorType(MotorType::kBrushless);
    left_motor_->SetSensorType(SensorType::kHallSensor);
    right_motor_->SetSensorType(SensorType::kHallSensor);
    left_motor_->SetInverted(false);
    right_motor_->SetInverted(true);

    left_motor_->BurnFlash();
    right_motor_->BurnFlash();

    // Subscriber to /cmd_vel
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&CmdVelMotorDriver::cmd_vel_callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(get_logger(), "cmd_vel_motor_driver node initialized");
  }

private:
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    double linear = msg->linear.x;    // m/s
    double angular = msg->angular.z;  // rad/s

    double wheel_separation = 0.5;  // Distance between wheels (meters) (adjust to your robot!)

    double left_speed = linear - (angular * wheel_separation / 2.0);
    double right_speed = linear + (angular * wheel_separation / 2.0);

    double meters_per_sec_to_rpm = 60.0 / (0.319 * M_PI); // Assuming wheel diameter 0.319m (~12.5") (adjust if needed)

    double left_rpm = left_speed * meters_per_sec_to_rpm;
    double right_rpm = right_speed * meters_per_sec_to_rpm;

    // Clamp RPM
    left_rpm = std::clamp(left_rpm, -MAX_VELOCITY, MAX_VELOCITY);
    right_rpm = std::clamp(right_rpm, -MAX_VELOCITY, MAX_VELOCITY);

    left_motor_->SetVelocity(left_rpm);
    right_motor_->SetVelocity(right_rpm);

    left_motor_->Heartbeat();
    right_motor_->Heartbeat();

    RCLCPP_INFO(get_logger(), "Set motors: left=%.1f RPM, right=%.1f RPM", left_rpm, right_rpm);
  }

  std::shared_ptr<SparkMax> left_motor_, right_motor_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CmdVelMotorDriver>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

