#include "SparkMax.hpp"
#include "rclcpp/rclcpp.hpp"
#include "interfaces_pkg/msg/motor_health.hpp"
#include "std_msgs/msg/float32.hpp"

const float VIBRATOR_DUTY = 1.0f;
const float ERROR = 0.1f;
float buffer = 0.0;

class OdometryNode : public rclcpp::Node{
public:
    OdometryNode() : Node("odometry_node"), leftMotor("can0", 1),
    rightMotor("can0", 2), leftLift("can0", 3), rightLift("can0", 4), tilt("can0", 5), vibrator("can0", 6) {
      depth_detection_pub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/depth_detection", 5,
        std::bind(&OdometryNode::depth_callback, this, std::placeholders::_1)
      );   

      buffer = tilt.GetPosition();

      health_subscriber_ = this->create_subscription<interfaces_pkg::msg::MotorHealth>(
      "/health_topic", 10,
      std::bind(&OdometryNode::positional_test, this, std::placeholders::_1)
    );
}
private:
    SparkMax leftMotor;
    SparkMax rightMotor;
    SparkMax leftLift;
    SparkMax rightLift;
    SparkMax tilt;
    SparkMax vibrator;
    //Motor controllers

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr depth_detection_pub_;
    rclcpp::Subscription<interfaces_pkg::msg::MotorHealth>::SharedPtr health_subscriber_;

    bool begin_excavation = false;
    bool begin_depositing = false;
    float distance = 0.0;
    float initialPosition = 0.0;
    bool initialized = false;
    int cycleNumber = 0;
    float buffer;

    enum class EXDEP {BACK, EXCAVATE, FORWARD, DEPOSIT, DONE};

    EXDEP state = EXDEP::BACK;

    void MoveBucket (float lift_setpoint, float tilt_setpoint, bool activate_vibrator, float drive_speed) {
        auto timer_start = std::chrono::high_resolution_clock::now();
        bool leftLiftReached = (fabs(lift_setpoint - leftLift.GetPosition() ) <=  ERROR);
        bool rightLiftReached = (fabs(lift_setpoint - rightLift.GetPosition() ) <=  ERROR);
        bool tiltReached = (fabs(tilt_setpoint - tilt.GetPosition()) <=  ERROR);

        while (!((leftLiftReached && rightLiftReached) && tiltReached)){
            std::this_thread::sleep_for(std::chrono::milliseconds(5)); //prevents CAN buffer from overflowing
            if (fabs(leftLift.GetPosition() - rightLift.GetPosition()) >= 0.2){
                if (fabs(leftLift.GetPosition() - rightLift.GetPosition()) >= 0.75){
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "WARNING: ACTUATORS GREATELY MISALIGNED");
                }
                leftLift.SetPosition(rightLift.GetPosition());
                rightLift.SetPosition(rightLift.GetPosition());
            } //block for lift realignment
            else {
                leftLift.SetPosition(lift_setpoint);
                rightLift.SetPosition(lift_setpoint);
                tilt.SetPosition(tilt_setpoint);
            } //block for normal bucket movement

            if (activate_vibrator) vibrator.SetDutyCycle(VIBRATOR_DUTY);

            leftMotor.SetVelocity(drive_speed);
            rightMotor.SetVelocity(drive_speed);

            if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - timer_start).count() > 5) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Skipping stage...");
                break;
            } //Timer for when to quit a stage due to timeout
            
            leftLiftReached = (fabs(lift_setpoint - leftLift.GetPosition() ) <=  ERROR);
            rightLiftReached = (fabs(lift_setpoint - rightLift.GetPosition() ) <=  ERROR);
            tiltReached = (fabs(tilt_setpoint - tilt.GetPosition() ) <=  ERROR); //Updates statuses
        }
    }

    void depth_callback(const std_msgs::msg::Float32::SharedPtr depth_msg){
        distance = depth_msg->data;
    }

    void positional_test(const interfaces_pkg::msg::MotorHealth::SharedPtr health_msg){
    if (!initialized) {
        initialPosition = health_msg->left_motor_position;
        initialized = true;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Initalized at : %f", initialPosition);
    }

    switch (state){
        case EXDEP::BACK:
        if (health_msg->left_motor_position - initialPosition <= -200){
            state = EXDEP::EXCAVATE;
            leftMotor.SetDutyCycle(0.0f);
            rightMotor.SetDutyCycle(0.0f);
            std::this_thread::sleep_for(std::chrono::milliseconds(5)); //prevents CAN buffer from overflowing
        }
        else {
            leftMotor.SetVelocity(-1500.0f);
            rightMotor.SetVelocity(-1500.0f);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Current position %f", health_msg->left_motor_position - initialPosition);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        break;

        case EXDEP::EXCAVATE:
        MoveBucket(-2.0, -2.6 + buffer, false, false);
        MoveBucket(-3.2 + (-0.1 * cycleNumber), -3.0 + buffer, true, 500);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Current position %f", health_msg->left_motor_position - initialPosition);
        MoveBucket(0.0, 0.0, false, false);
        vibrator.SetDutyCycle(0.0);
        state = EXDEP::FORWARD;
        break;

        case EXDEP::FORWARD:
        if ((health_msg->left_motor_position - initialPosition) >= 0){
            state = EXDEP::DEPOSIT;
        } 
        else {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            leftMotor.SetVelocity(1500.0f);
            rightMotor.SetVelocity(1500.0f);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Current position %f", leftMotor.GetPosition() - initialPosition);
            
        }
        break;

        case EXDEP::DEPOSIT:
        MoveBucket(1.9, 0.0 + buffer, false, 0);  
        MoveBucket(2.9, 0.0 + buffer, true, 0);
        MoveBucket(0.0, 0.0 + buffer, false, 0);  
        vibrator.SetDutyCycle(0.0);
        leftMotor.SetDutyCycle(0.0);
        rightMotor.SetDutyCycle(0.0);
        cycleNumber++;
        if (cycleNumber == 3){
            state = EXDEP::DONE;
        } else {
            initialized = false;
            state = EXDEP::BACK;
        }
        break;

        case EXDEP::DONE:
        std::this_thread::sleep_for(std::chrono::milliseconds(5));

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "THE AUTO WORKED!!!");
        rclcpp::shutdown();
        break;
      }
    }
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdometryNode>());
  rclcpp::shutdown();
  return 0;
}
