#include "SparkMax.hpp"
#include "rclcpp/rclcpp.hpp"
#include "interfaces_pkg/msg/motor_health.hpp"
#include <chrono>

using namespace std::chrono_literals;
/*const float DRIVETRAIN_HARD_CURRENT_LIMIT = 40.0f //HARD-CODED LIMIT
const float DRIVETRAIN_SOFT_CURRENT_LIMIT = 35.0f //SOFT-CODED LIMIT
const float DRIVETRAIN_TEMPERATURE_LIMIT = 100.0f //TO-DO, check if in celcius or farenheight
*/

class HealthNode : public rclcpp::Node{
public:
    HealthNode() : Node("health_node"), leftMotor("can0", 1),
    rightMotor("can0", 2),
    leftLift("can0", 3),
    rightLift("can0", 4),
    tilt("can0", 5),
    vibrator("can0", 6) {
        health_publisher_ = this->create_publisher<interfaces_pkg::msg::MotorHealth>("health_topic", 10);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&HealthNode::status_monitoring, this));
}
private:
    SparkMax leftMotor;
    SparkMax rightMotor;
    SparkMax leftLift;
    SparkMax rightLift;
    SparkMax tilt;
    SparkMax vibrator;
    //Motor controllers
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<interfaces_pkg::msg::MotorHealth>::SharedPtr health_publisher_;

    void status_monitoring(){
        auto msg = interfaces_pkg::msg::MotorHealth();

        //Left Motor Monitoring
        try{
            msg.left_motor_velocity = leftMotor.GetVelocity();
            msg.left_motor_current = leftMotor.GetCurrent();
            msg.left_motor_voltage = leftMotor.GetVoltage();
        } catch (const std::exception & ex) {
            RCLCPP_ERROR(this->get_logger(), "ERROR: Could not gain readings from left motor, %s", ex.what());
        }
        //Left Motor Monitoring

        //Right Motor Monitoring
        try{
            msg.right_motor_velocity = rightMotor.GetVelocity();
            msg.right_motor_current = rightMotor.GetCurrent();
            msg.right_motor_voltage = rightMotor.GetVoltage();
        } catch (const std::exception & ex) {
            RCLCPP_ERROR(this->get_logger(), "ERROR: Could not gain readings from right motor, %s", ex.what());
        }
        //Right motor monitoring

        //Left Lift Monitoring
        try{
            msg.left_lift_position = leftLift.GetPosition();
            msg.left_lift_current = leftLift.GetCurrent();
            msg.left_lift_voltage = leftLift.GetVoltage();
        } catch (const std::exception & ex) {
            RCLCPP_ERROR(this->get_logger(), "ERROR: Could not gain readings from left lift, %s", ex.what());
        }
        //Left lift monitoring

        //Right Lift Monitoring
        try{
            msg.right_lift_position = rightLift.GetPosition();
            msg.right_lift_current = rightLift.GetCurrent();
            msg.right_lift_voltage = rightLift.GetVoltage();
        } catch (const std::exception & ex) {
            RCLCPP_ERROR(this->get_logger(), "ERROR: Could not gain readings from right lift, %s", ex.what());
        }
        //Right Lift monitoring

        //Tilt Monitoring
        try{
            msg.tilt_position = tilt.GetAnalogPosition();
            msg.tilt_current = tilt.GetCurrent();
            msg.tilt_voltage = tilt.GetVoltage();
        } catch (const std::exception & ex) {
            RCLCPP_ERROR(this->get_logger(), "ERROR: Could not gain readings from tilt, %s", ex.what());
        }
        //Tilt monitoring

        //Vibrator Monitoring
        try{
            msg.vibrator_current = vibrator.GetCurrent();
            msg.vibrator_voltage = vibrator.GetVoltage();
        } catch (const std::exception & ex) {
            RCLCPP_ERROR(this->get_logger(), "ERROR: Could not gain readings from vibrator, %s", ex.what());
        }
        //Vibrator monitoring

        health_publisher_->publish(msg);
    }

};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HealthNode>());
  rclcpp::shutdown();
  return 0;
}
