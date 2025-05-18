//UCF ODOMETRY, ONLY TO BE USED AT UCF
#include "SparkMax.hpp"
#include "rclcpp/rclcpp.hpp"
#include "interfaces_pkg/msg/motor_health.hpp"

class OdometryNode : public rclcpp::Node{
public:
    OdometryNode() : Node("odometry_node"), leftMotor("can0", 1),
    rightMotor("can0", 2) {
      health_subscriber_ = this->create_subscription<interfaces_pkg::msg::MotorHealth>(
        "/health_topic", 10,
        std::bind(&OdometryNode::positional_test, this, std::placeholders::_1)
      );
}
private:
    SparkMax leftMotor;
    SparkMax rightMotor;
    //Motor controllers

    rclcpp::Subscription<interfaces_pkg::msg::MotorHealth>::SharedPtr health_subscriber_;

    float setpointMeters = 5.0f;
    float radius = 0.1524f;
    float setpointRotations = 108.0f * (setpointMeters / (6.28f * radius)); //conversion from distance in meters to rotations
    float initialPosition;
    bool initialized = false;

    void positional_test(const interfaces_pkg::msg::MotorHealth::SharedPtr health_msg){
      if (!initialized) {
        initialPosition = health_msg->left_motor_position;
        initialized = true;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Initalized at : %f", 6.28 * radius * ((initialPosition) / 108));
      }

      if (health_msg->left_motor_position - initialPosition <= setpointRotations){ //I might have to take an average of both motors, I will see with testing
        leftMotor.SetVelocity(2500.0f);
        rightMotor.SetVelocity(2500.0f);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Current positions: Left Motor = %f", 6.28 * radius * ((health_msg->left_motor_position - initialPosition) / 108));
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Current positions: Right Motor = %f", 6.28 * radius * ((health_msg->left_motor_position - initialPosition) / 108));
      }
      else {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Setpoint successfully reached!");
        leftMotor.SetDutyCycle(0.0f);
        rightMotor.SetDutyCycle(0.0f);
      }
    }
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdometryNode>());
  rclcpp::shutdown();
  return 0;
}
