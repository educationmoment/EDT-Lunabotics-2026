#include <chrono>
#include <cstdlib>
#include <memory>
#include <limits>
#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "SparkMax.hpp"

using namespace std::chrono_literals;

static constexpr double TARGET_FORWARD_REVOLUTIONS = 4.0;
static constexpr double LEFT_TURN_REVOLUTIONS = 4.0;
static constexpr double RIGHT_TURN_REVOLUTIONS = -3.0;
static constexpr double MOTOR_RPM = 1000.0; // moderate speed

enum class State { DRIVE_FORWARD, TURN_RIGHT, FINISHED };

class BasicNavigationNode : public rclcpp::Node {
public:
  BasicNavigationNode()
  : Node("navigation_node"), state_(State::DRIVE_FORWARD)
  {
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

    start_left_position_ = left_motor_->GetPosition();
    start_right_position_ = right_motor_->GetPosition();

    timer_ = create_wall_timer(100ms, std::bind(&BasicNavigationNode::control_loop, this));
    RCLCPP_INFO(get_logger(), "Node initialized and running.");
  }

private:
  void control_loop() {
    double current_left_position = left_motor_->GetPosition();
    double current_right_position = right_motor_->GetPosition();

    double left_delta = current_left_position - start_left_position_;
    double right_delta = current_right_position - start_right_position_;

    switch (state_) {
      case State::DRIVE_FORWARD: {
        double average_revolutions = (left_delta + right_delta) / 2.0;
        RCLCPP_INFO(get_logger(), "State: DRIVE_FORWARD | Left Delta: %.2f | Right Delta: %.2f | Avg: %.2f", left_delta, right_delta, average_revolutions);

        if (average_revolutions < TARGET_FORWARD_REVOLUTIONS) {
          left_motor_->SetVelocity(MOTOR_RPM);
          right_motor_->SetVelocity(MOTOR_RPM);
        } else {
          left_motor_->SetVelocity(0);
          right_motor_->SetVelocity(0);

          start_left_position_ = current_left_position;
          start_right_position_ = current_right_position;

          state_ = State::TURN_RIGHT;
          RCLCPP_INFO(get_logger(), "Finished DRIVE_FORWARD, starting TURN_RIGHT");
        }
        break;
      }

      case State::TURN_RIGHT: {
        bool left_done = left_delta >= LEFT_TURN_REVOLUTIONS;
        bool right_done = right_delta <= RIGHT_TURN_REVOLUTIONS;
        RCLCPP_INFO(get_logger(), "State: TURN_RIGHT | Left Delta: %.2f | Right Delta: %.2f", left_delta, right_delta);

        if (!left_done)
          left_motor_->SetVelocity(MOTOR_RPM);
        else
          left_motor_->SetVelocity(0);

        if (!right_done)
          right_motor_->SetVelocity(-MOTOR_RPM);
        else
          right_motor_->SetVelocity(0);

        if (left_done && right_done) {
          left_motor_->SetVelocity(0);
          right_motor_->SetVelocity(0);
          state_ = State::FINISHED;
          RCLCPP_INFO(get_logger(), "Finished TURN_RIGHT, navigation complete.");
        }
        break;
      }

      case State::FINISHED:
      default:
        RCLCPP_INFO(get_logger(), "State: FINISHED");
        break;
    }

    left_motor_->Heartbeat();
    right_motor_->Heartbeat();
  }

  State state_;
  double start_left_position_, start_right_position_;
  std::shared_ptr<SparkMax> left_motor_, right_motor_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BasicNavigationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

