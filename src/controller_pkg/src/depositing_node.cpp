#include "SparkMax.hpp"
#include "rclcpp/rclcpp.hpp"
#include "interfaces_pkg/srv/depositing_request.hpp"
#include "std_msgs/msg/bool.hpp"

const float VIBRATOR_DUTY = 0.1f;
const float ERROR = 0.1;

SparkMax leftLift("can0", 3);
SparkMax rightLift("can0", 4);
SparkMax tilt("can0", 5);
SparkMax vibrator("can0", 6);
//Initalizes motor controllers

void MoveBucket (float lift_setpoint, float tilt_setpoint, bool activate_vibrator) {
    auto timer_start = std::chrono::high_resolution_clock::now();
    bool leftLiftReached = (fabs(lift_setpoint - leftLift.GetPosition() ) <=  ERROR);
    bool rightLiftReached = (fabs(lift_setpoint - rightLift.GetPosition() ) <=  ERROR);
    bool tiltReached = (fabs(tilt_setpoint - tilt.GetPosition() ) <=  ERROR);

    while (!((leftLiftReached && rightLiftReached) && tiltReached)){
        std::this_thread::sleep_for(std::chrono::milliseconds(5)); //prevents CAN buffer from overflowing
        if (fabs(leftLift.GetPosition() - rightLift.GetPosition()) >= 0.2){
            if (fabs(leftLift.GetPosition() - rightLift.GetPosition()) >= 0.75){
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "WARNING: ACTUATORS GREATELY MISALIGNEDT");
                rclcpp::shutdown();
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

        if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - timer_start).count() > 5) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "ERROR: Depositing Cancelled");
            break;
        } //Timer for when to quit a stage due to timeout
        
        leftLiftReached = (fabs(lift_setpoint - leftLift.GetPosition() ) <=  ERROR);
        rightLiftReached = (fabs(lift_setpoint - rightLift.GetPosition() ) <=  ERROR);
        tiltReached = (fabs(tilt_setpoint - tilt.GetPosition() ) <=  ERROR); //Updates statuses
    }
}
//Fix depositing to be under the bar
void Deposit(const std::shared_ptr<interfaces_pkg::srv::DepositingRequest::Request> request,
    std::shared_ptr<interfaces_pkg::srv::DepositingRequest::Response> response) {
        if (!request->start_depositing){
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Received request but start_depositing is false");
            response->depositing_successful = false;
            return;
        } //Checks to make sure start_depositing is set to true

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting depositing process");

        MoveBucket(3.2, 2.0, false); //Moves bucket up, tilts bucket 
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Stage 1 complete");

        auto jiggleout_start = std::chrono::high_resolution_clock::now();
        while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - jiggleout_start).count() < 20) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5)); //prevents CAN buffer from overflowing
            vibrator.SetDutyCycle(VIBRATOR_DUTY);
        } //Vibrates sand out of bucket

        vibrator.SetDutyCycle(0.0f); //ensures vibrator turns off
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Stage 2 complete, resetting bucket");

        MoveBucket(0.0, 0.0, false); //Resets bucket
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Bucket successfully reset");
        response->depositing_successful = true;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("depositing_node");

    rclcpp::Service<interfaces_pkg::srv::DepositingRequest>::SharedPtr service =
    node->create_service<interfaces_pkg::srv::DepositingRequest>("depositing_service", &Deposit);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Depositing Initalized");

    rclcpp::spin(node);
    rclcpp::shutdown();
}
