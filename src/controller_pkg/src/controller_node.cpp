// pilot_node.cpp
#include "SparkMax.hpp"  // Assumes SparkMax.hpp is available in your include path
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <cmath>
#include <string>

// Define CAN IDs for our devices (vibrator is omitted)
enum CAN_IDs {
  LEFT_MOTOR  = 1,
  RIGHT_MOTOR = 2,
  LEFT_LIFT   = 3,
  RIGHT_LIFT  = 4,
  TILT        = 5
};

class PilotNode : public rclcpp::Node
{
public:
  PilotNode(const std::string & can_interface)
  : Node("pilot_node"),
    leftMotor(can_interface, LEFT_MOTOR),
    rightMotor(can_interface, RIGHT_MOTOR),
    leftLift(can_interface, LEFT_LIFT),
    rightLift(can_interface, RIGHT_LIFT),
    tilt(can_interface, TILT)
  {
    // --- Configure Drive Motors (Brushless with Hall Sensor) ---
    leftMotor.SetIdleMode(IdleMode::kBrake);
    rightMotor.SetIdleMode(IdleMode::kBrake);
    leftMotor.SetMotorType(MotorType::kBrushless);
    rightMotor.SetMotorType(MotorType::kBrushless);
    leftMotor.SetSensorType(SensorType::kHallSensor);
    rightMotor.SetSensorType(SensorType::kHallSensor);

    // --- Configure Lift Motors (Brushed with Encoder) ---
    leftLift.SetIdleMode(IdleMode::kBrake);
    rightLift.SetIdleMode(IdleMode::kBrake);
    leftLift.SetMotorType(MotorType::kBrushed);
    rightLift.SetMotorType(MotorType::kBrushed);
    leftLift.SetSensorType(SensorType::kEncoder);
    rightLift.SetSensorType(SensorType::kEncoder);

    // --- Configure Tilt (Brushed with Encoder) ---
    tilt.SetIdleMode(IdleMode::kBrake);
    tilt.SetMotorType(MotorType::kBrushed);
    tilt.SetSensorType(SensorType::kEncoder);

    // --- Invert All Motors ---
    leftMotor.SetInverted(false);
    rightMotor.SetInverted(true);
    leftLift.SetInverted(true);
    rightLift.SetInverted(true);
    tilt.SetInverted(true);

    // Optionally, burn these parameters to flash.
    leftMotor.BurnFlash();
    rightMotor.BurnFlash();
    leftLift.BurnFlash();
    rightLift.BurnFlash();
    tilt.BurnFlash();

    // Create subscription to the /joy topic.
    joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10,
      std::bind(&PilotNode::joy_callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "PilotNode initialized on CAN interface: %s", can_interface.c_str());
  }

private:
  // Motor controller objects using SparkMax.
  SparkMax leftMotor;
  SparkMax rightMotor;
  SparkMax leftLift;
  SparkMax rightLift;
  SparkMax tilt;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;

  // Helper: returns +0.25 or -0.25 if the axis input exceeds a deadband; else 0.
  float computeDutyFromAxis(float axisValue, float deadband = 0.1f)
  {
    if (std::fabs(axisValue) < deadband) {
      return 0.0f;
    }
    return (axisValue > 0) ? 0.25f : -0.25f;
  }

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
  {
    if (joy_msg->axes.size() < 2 || joy_msg->buttons.size() < 8) {
      RCLCPP_WARN(this->get_logger(), "Insufficient axes/buttons in Joy message");
      return;
    }

    // --- Drive Motors ---
    // Use the left joystick vertical axis (axes[1]) for drive.
    // Invert so that pushing forward (normally negative) yields a positive command.
    float raw_drive = -joy_msg->axes[1];
    float drive_duty = computeDutyFromAxis(raw_drive);

    // --- Lift Motors (Actuators) ---
    // Instead of a single trigger, use both triggers:
    // Right trigger (button index 7) yields +0.5 duty cycle.
    // Left trigger (button index 6) yields -0.5 duty cycle.
    float lift_duty = 0.0f;
    if (joy_msg->buttons[7] > 0) {
      lift_duty = 0.5f;
    } else if (joy_msg->buttons[6] > 0) {
      lift_duty = -0.5f;
    }

    try {
      // Send heartbeat to maintain communication.
      leftMotor.Heartbeat();
      rightMotor.Heartbeat();
      leftLift.Heartbeat();
      rightLift.Heartbeat();
      //tilt.Heartbeat();

      // Command drive motors.
      leftMotor.SetDutyCycle(drive_duty);
      rightMotor.SetDutyCycle(drive_duty);

      // Command lift motors.
      leftLift.SetDutyCycle(lift_duty);
      rightLift.SetDutyCycle(lift_duty);

      // Command tilt (here, no command; adjust as needed).
      //tilt.SetDutyCycle(0.0f);
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(this->get_logger(), "Error sending CAN command: %s", ex.what());
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::string can_interface = "can0";
  auto temp_node = rclcpp::Node::make_shared("pilot_param_node");
  temp_node->declare_parameter<std::string>("can_interface", "can0");
  temp_node->get_parameter("can_interface", can_interface);

  auto node = std::make_shared<PilotNode>(can_interface);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
