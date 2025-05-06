//KSC ODOMETRY, ONLY TO BE USED AT KENNEDY SPACE CENTER ARENA
#include "SparkMax.hpp"
#include "rclcpp/rclcpp.hpp"
#include "interfaces_pkg/msg/motor_health.hpp"
#include "std_msgs/msg/bool.hpp"

class OdometryNode : public rclcpp::Node{
public:
    OdometryNode() : Node("odometry_node"), leftMotor("can0", 1),
    rightMotor("can0", 2) {
      health_subscriber_ = this->create_subscription<interfaces_pkg::msg::MotorHealth>(
        "/health_topic", 10,
        std::bind(&OdometryNode::positional_test, this, std::placeholders::_1)
      );

      left_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
        "/obstacle_detection/left", 10,
        std::bind(&OdometryNode::left_obstacle_detection, this, std::placeholders::_1));

      right_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
        "/obstacle_detection/right", 10,
        std::bind(&OdometryNode::right_obstacle_detection, this, std::placeholders::_1));
}

private:
    SparkMax leftMotor;
    SparkMax rightMotor;
    //Motor controllers

    rclcpp::Subscription<interfaces_pkg::msg::MotorHealth>::SharedPtr health_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr left_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr right_subscriber_;

    float setpointMeters = 1.0f; 
    float radius = 0.1524f;
    float setpointRotations = 108.0f * (setpointMeters / (6.28f * radius)); //conversion from distance in meters to rotations
    float initialPosition;
    bool initialized = false;
    
    int obstacle_stage = 0; // add this as a member variable

    float initialPositionAvoidance;
    bool initializedAvoidance = false;

    bool left_obstacle_detected = false;
    bool right_obstacle_detected = false;

    void left_obstacle_detection(const std_msgs::msg::Bool::SharedPtr left_msg) {
      left_obstacle_detected = left_msg->data;
    }
  
    void right_obstacle_detection(const std_msgs::msg::Bool::SharedPtr right_msg) {
      right_obstacle_detected = right_msg->data;
    }

    enum class TravelAutonomy {FORWARD, AVOID_OBSTACLE, DONE};

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
            if (left_obstacle_detected || right_obstacle_detected){
              state = TravelAutonomy::AVOID_OBSTACLE;
            }
          }
          else {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Setpoint successfully reached!");
            leftMotor.SetDutyCycle(0.0f);
            rightMotor.SetDutyCycle(0.0f);
            state = TravelAutonomy::DONE;
          }
        break;

        case TravelAutonomy::AVOID_OBSTACLE:
          if (!initializedAvoidance) {
            initialPositionAvoidance = health_msg->left_motor_position;
            initializedAvoidance = true;
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "OBSTACLE DETECTED");
          }
        
          switch (obstacle_stage) {
            case 0: // rotate
              if (health_msg->left_motor_position - initialPositionAvoidance <= (left_obstacle_detected ? -100 : 100)) {
                leftMotor.SetVelocity((left_obstacle_detected ? -2500.0 : 2500.0));
                rightMotor.SetVelocity((left_obstacle_detected ? 2500.0 : -2500.0));
              } else {
                leftMotor.SetDutyCycle(0.0f);
                rightMotor.SetDutyCycle(0.0f);
                obstacle_stage++;
                initializedAvoidance = false;
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "STAGE 1 COMPLETE");
              }
              break;
        
            case 1: // forward
              if (health_msg->left_motor_position - initialPositionAvoidance <= 200) {
                leftMotor.SetVelocity(2500.0f);
                rightMotor.SetVelocity(2500.0f);
              } else {
                leftMotor.SetDutyCycle(0.0f);
                rightMotor.SetDutyCycle(0.0f);
                obstacle_stage++;
                initializedAvoidance = false;
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "STAGE 2 COMPLETE");
              }
              break;
        
            case 2: // rotate back
              if (health_msg->left_motor_position - initialPositionAvoidance <= (left_obstacle_detected ? 150 : -150)) {
                leftMotor.SetVelocity((left_obstacle_detected ? 2500.0 : -2500.0));
                rightMotor.SetVelocity((left_obstacle_detected ? -2500.0 : 2500.0));
              } else {
                leftMotor.SetDutyCycle(0.0f);
                rightMotor.SetDutyCycle(0.0f);
                obstacle_stage++;
                initializedAvoidance = false;
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "STAGE 3 COMPLETE");
              }
              break;
        
            case 3: // forward
              if (health_msg->left_motor_position - initialPositionAvoidance <= 200) {
                leftMotor.SetVelocity(2500.0f);
                rightMotor.SetVelocity(2500.0f);
              } else {
                leftMotor.SetDutyCycle(0.0f);
                rightMotor.SetDutyCycle(0.0f);
                obstacle_stage++;
                initializedAvoidance = false;
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "STAGE 4 COMPLETE");
              }
              break;
        
            case 4: // rotate final
              if (health_msg->left_motor_position - initialPositionAvoidance <= (left_obstacle_detected ? 100 : -100)) {
                leftMotor.SetVelocity((left_obstacle_detected ? 2500.0 : -2500.0));
                rightMotor.SetVelocity((left_obstacle_detected ? -2500.0 : 2500.0));
              } else {
                leftMotor.SetDutyCycle(0.0f);
                rightMotor.SetDutyCycle(0.0f);
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "OBSTACLE SUCCESSFULLY AVOIDED");
                state = TravelAutonomy::FORWARD;
                obstacle_stage = 0;
                initializedAvoidance = false;
                left_obstacle_detected = false;
                right_obstacle_detected = false;
              }
              break;
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
