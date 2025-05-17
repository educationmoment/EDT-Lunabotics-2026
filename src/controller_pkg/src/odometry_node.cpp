#include "SparkMax.hpp"
#include "rclcpp/rclcpp.hpp"
#include "interfaces_pkg/msg/motor_health.hpp"
#include "std_msgs/msg/float32.hpp"
#include "interfaces_pkg/srv/depositing_request.hpp"

const float VIBRATOR_DUTY = 0.1f;
const float ERROR = 0.1f;

class OdometryNode : public rclcpp::Node{
public:
    OdometryNode() : Node("odometry_node"), leftMotor("can0", 1),
    rightMotor("can0", 2), leftLift("can0", 3), rightLift("can0", 4), tilt("can0", 5), vibrator("can0", 6) {
      depth_detection_pub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/depth_detection", 5,
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
    rclcpp::Client<interfaces_pkg::srv::DepositingRequest>::SharedPtr depositing_client_;

    bool begin_excavation = false;
    bool begin_depositing = false;
    float distance = 0.0;
    float initialPosition = 0.0;
    bool initialized = false;

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

    void positional_test(const std_msgs::msg::Float32::SharedPtr depth_msg){
    if (!initialized){
        initialPosition = leftMotor.GetPosition();
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Initialized at: %f", initialPosition);
        initialized = true;
    }

    switch (state){
        case EXDEP::BACK:
        if (leftMotor.GetPosition() - initialPosition >= -225){
            state = EXDEP::EXCAVATE;
            leftMotor.SetDutyCycle(0.0f);
            rightMotor.SetDutyCycle(0.0f);
        }
        else {
            leftMotor.SetVelocity(-1500.0f);
            rightMotor.SetVelocity(-1500.0f);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Current position %f", leftMotor.GetPosition() - initialPosition);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        break;

        case EXDEP::EXCAVATE:
        MoveBucket(-2.0, -2.6, false, false);
        MoveBucket(-3.0,-3.0, true, 1500);
        MoveBucket(0.0, 0.0, false, false);
        state = EXDEP::FORWARD;
        break;

        case EXDEP::FORWARD:
        distance = depth_msg->data;
        if (distance < 1.0f || leftMotor.GetPosition() <= 2000){
            state = EXDEP::DEPOSIT;
        }
        else {
            leftMotor.SetVelocity(1500.0f);
            rightMotor.SetVelocity(1500.0f);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Current position %f", leftMotor.GetPosition() - initialPosition);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        break;

        case EXDEP::DEPOSIT:
        if (!begin_depositing){
        if (!depositing_client_ || !depositing_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_ERROR(this->get_logger(), "Service not available");
            return;
        }
        auto request = std::make_shared<interfaces_pkg::srv::DepositingRequest::Request>();
        request->start_depositing = true;
        RCLCPP_INFO(this->get_logger(), "Deposit request sent");
        begin_depositing = true;
        depositing_client_->async_send_request(request, [this](rclcpp::Client<interfaces_pkg::srv::DepositingRequest>::SharedFuture future) {
        auto response = future.get();
        if (response->depositing_successful) {
            RCLCPP_INFO(this->get_logger(), "Depositing successful, complete :3");
            state = EXDEP::DONE;
        } else {
            RCLCPP_WARN(this->get_logger(), "Depositing failed");
        }
        });
        }
        break;

        case EXDEP::DONE:
        leftMotor.SetDutyCycle(0.0);
        rightMotor.SetDutyCycle(0.0);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
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
