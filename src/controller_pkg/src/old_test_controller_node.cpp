// pilot_node.cpp
#include "SparkMax.hpp"  
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <cmath>
#include <string>

enum CAN_IDs {
  LEFT_MOTOR  = 1,
  RIGHT_MOTOR = 2,
  LEFT_LIFT   = 3,
  RIGHT_LIFT  = 4,
  TILT        = 5,
  VIBRATOR    = 6
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
    tilt(can_interface, TILT),
    vibrator(can_interface, VIBRATOR)
  {
    leftMotor.SetIdleMode(IdleMode::kBrake);
    rightMotor.SetIdleMode(IdleMode::kBrake);
    leftMotor.SetMotorType(MotorType::kBrushless);
    rightMotor.SetMotorType(MotorType::kBrushless);
    leftMotor.SetSensorType(SensorType::kHallSensor);
    rightMotor.SetSensorType(SensorType::kHallSensor);

    leftLift.SetIdleMode(IdleMode::kBrake);
    rightLift.SetIdleMode(IdleMode::kBrake);
    leftLift.SetMotorType(MotorType::kBrushed);
    rightLift.SetMotorType(MotorType::kBrushed);
    leftLift.SetSensorType(SensorType::kEncoder);
    rightLift.SetSensorType(SensorType::kEncoder);

    tilt.SetIdleMode(IdleMode::kBrake);
    tilt.SetMotorType(MotorType::kBrushed);
    tilt.SetSensorType(SensorType::kEncoder);

    vibrator.SetIdleMode(IdleMode::kBrake);
    vibrator.SetMotorType(MotorType::kBrushed);
    vibrator.SetSensorType(SensorType::kEncoder); //im ean we probalby dotn need an encoder butttttttt hey man you all star get your game on hey hey

    leftMotor.SetInverted(false);
    rightMotor.SetInverted(true);
    leftLift.SetInverted(true);
    rightLift.SetInverted(true);
    tilt.SetInverted(true);
    vibrator.SetInverted(true);

    leftMotor.BurnFlash();
    rightMotor.BurnFlash();
    leftLift.BurnFlash();
    rightLift.BurnFlash();
    tilt.BurnFlash();
    vibrator.BurnFlash();

    joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10,
      std::bind(&PilotNode::joy_callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "PilotNode initialized on CAN interface: %s", can_interface.c_str());
  }

private:
  SparkMax leftMotor;
  SparkMax rightMotor;
  SparkMax leftLift;
  SparkMax rightLift;
  SparkMax tilt;
  SparkMax vibrator;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;

  float computeDutyFromAxis(float axisValue, float deadband = 0.25f) //increased from 0.1 to 0.25
  {
    if (std::fabs(axisValue) < deadband) {
      return 0.0f;
    }
    return (axisValue > 0) ? 0.80f : -0.80f;
  }

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
  {
    if (joy_msg->axes.size() < 2 || joy_msg->buttons.size() < 8) {
      RCLCPP_WARN(this->get_logger(), "Insufficient axes/buttons in Joy message");
      return;
    }

    // --- Drive Motors ---
    // move with left joystick, cool math shit that joy doez for me :3
    float left_drive = -joy_msg->axes[1];
    float left_drive_duty = computeDutyFromAxis(left_drive);

    float right_drive = -joy_msg->axes[3];
    float right_drive_duty = computeDutyFromAxis(right_drive);

    // --- Lift Motors ---
    // right triger make go up left trigger make go down
    float lift_duty = 0.0f;
    if (joy_msg->buttons[7] > 0) {
      lift_duty = 1.0f;
    } else if (joy_msg->buttons[6] > 0) {
      lift_duty = -1.0f;
    }

    // --- Tilt Actuator ---
    // right bumper moves tilt forward with +0.5.
    // left bumper moves tilt backward with -0.5.
    float tilt_duty = 0.0f;
    if (joy_msg->buttons[5] > 0) {
      tilt_duty = 1.0f;
    } else if (joy_msg->buttons[4] > 0) {
      tilt_duty = -1.0f;
    }

    // --- Vibrator ---
    // vibrator when "Y" is pressed with a duty cycle of 0.8.
    float vibrator_duty = (joy_msg->buttons[3] > 0) ? 0.8f : 0.0f;

    try {
      leftMotor.Heartbeat();
      rightMotor.Heartbeat();
      leftLift.Heartbeat();
      rightLift.Heartbeat();
      tilt.Heartbeat();
      vibrator.Heartbeat();

      leftMotor.SetDutyCycle(left_drive_duty);
      rightMotor.SetDutyCycle(right_drive_duty);

      leftLift.SetDutyCycle(lift_duty);
      rightLift.SetDutyCycle(lift_duty);

      tilt.SetDutyCycle(tilt_duty);

      vibrator.SetDutyCycle(vibrator_duty);
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

