#define UCF_CODE true
#ifndef UCF_CODE
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
          std::bind(&OdometryNode::positional_test, thcis, std::placeholders::_1)
        );
  }
  private:
      SparkMax leftMotor;
      SparkMax rightMotor;
      //Motor controllers

      rclcpp::Subscription<interfaces_pkg::msg::MotorHealth>::SharedPtr health_subscriber_;

      float setpointMeters = 4.0f;
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
#else
  //KSC ODOMETRY, ONLY TO BE USED AT KENNEDY SPACE CENTER ARENA
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


      float setpointMeters = 4.0f; //same for both forward and turn
      float radius = 0.1524f;
      float setpointRotations = 108.0f * (setpointMeters / (6.28f * radius)); //conversion from distance in meters to rotations
      float initialPosition;
      bool initialized = false;

      enum class TravelAutonomy {FORWARD, TURN_RIGHT, RIGHT, DONE};

      TravelAutonomy state = TravelAutonomy::FORWARD;

      void positional_test(const interfaces_pkg::msg::MotorHealth::SharedPtr health_msg){
        switch (state){
          case TravelAutonomy::FORWARD:
            if (!initialized) {
              initialPosition = health_msg->left_motor_position;
              initialized = true;
              RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Initalized at : %f", 6.28 * radius * ((initialPosition) / 108));
            }

            if (health_msg->left_motor_position - initialPosition <= setpointRotations){
              leftMotor.SetVelocity(2500.0f);
              rightMotor.SetVelocity(2500.0f);
              RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Current positions: Left Motor = %f", 6.28 * radius * ((health_msg->left_motor_position - initialPosition) / 108));
              RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Current positions: Right Motor = %f", 6.28 * radius * ((health_msg->left_motor_position - initialPosition) / 108));
            }
            else {
              RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Setpoint successfully reached!");
              leftMotor.SetDutyCycle(0.0f);
              rightMotor.SetDutyCycle(0.0f);
              state = TravelAutonomy::TURN_RIGHT;
              initialized = false;
            }
          break;

          case TravelAutonomy::TURN_RIGHT:
            if (!initialized) {
              initialPosition = health_msg->left_motor_position;
              initialized = true;
              RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Initalized at : %f", 6.28 * radius * ((initialPosition) / 108));
            }

            if (health_msg->left_motor_position - initialPosition <= 125){ //90 degree turn
              leftMotor.SetVelocity(2500.0f);
              rightMotor.SetVelocity(-2500.0f);
              RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Current positions: Left Motor = %f", 6.28 * radius * ((health_msg->left_motor_position - initialPosition) / 108));
              RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Current positions: Right Motor = %f", 6.28 * radius * ((health_msg->left_motor_position - initialPosition) / 108));
            }
            else {
              RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Setpoint successfully reached!");
              leftMotor.SetDutyCycle(0.0f);
              rightMotor.SetDutyCycle(0.0f);
              state = TravelAutonomy::RIGHT;
              initialized = false;
            }
          break;

          case TravelAutonomy::RIGHT:
            if (!initialized) {
              initialPosition = health_msg->left_motor_position;
              initialized = true;
              RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Initalized at : %f", 6.28 * radius * ((initialPosition) / 108));
            }

            if (health_msg->left_motor_position - initialPosition <= setpointRotations){
              leftMotor.SetVelocity(2500.0f);
              rightMotor.SetVelocity(2500.0f);
              RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Current positions: Left Motor = %f", 6.28 * radius * ((health_msg->left_motor_position - initialPosition) / 108));
              RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Current positions: Right Motor = %f", 6.28 * radius * ((health_msg->left_motor_position - initialPosition) / 108));
            }
            else {
              RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Setpoint successfully reached!");
              leftMotor.SetDutyCycle(0.0f);
              rightMotor.SetDutyCycle(0.0f);
              state = TravelAutonomy::DONE;
              initialized = false;
            }
          break;

          case TravelAutonomy::DONE:
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Autonomy successfully completed!");
            rclcpp::shutdown();
        }
      }
  };

  int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdometryNode>());
    rclcpp::shutdown();
    return 0;
  }
#endif
