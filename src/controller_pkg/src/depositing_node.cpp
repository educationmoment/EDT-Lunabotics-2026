#include "SparkMax.hpp"
#include "rclcpp/rclcpp.hpp"
#include "interfaces_pkg/srv/depositing_request.hpp"

const float VIBRATOR_DUTY = 1.0;
const float ERROR = 0.1;

SparkMax leftLift("can0", 3);
SparkMax rightLift("can0", 4);
SparkMax tilt("can0", 5);
SparkMax vibrator("can0", 6); 
//Initalizes motor controllers

void MoveBucket (float lift_setpoint, float tilt_setpoint, bool activate_vibrator, std::shared_ptr<interfaces_pkg::srv::DepositingRequest::Response> response) {
    auto timer_start = std::chrono::high_resolution_clock::now();
    while (fabs(leftLift.GetPosition() - lift_setpoint) > ERROR ||
    fabs(rightLift.GetPosition() - lift_setpoint) > ERROR ||
    fabs(tilt.GetPosition() - tilt_setpoint) > ERROR){

        if (fabs(leftLift.GetPosition() - rightLift.GetPosition()) >= 0.125){
            leftLift.SetPosition(rightLift.GetPosition());
            rightLift.SetPosition(leftLift.GetPosition());
        } //lift realignment
        else {
            leftLift.SetPosition(lift_setpoint);
            rightLift.SetPosition(lift_setpoint);
            tilt.SetPosition(tilt_setpoint);
        }

        if (activate_vibrator){
            vibrator.SetDutyCycle(VIBRATOR_DUTY);
        }

        if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - timer_start).count() > 10) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "ERROR: Depositing Cancelled Midway");
            response->depositing_successful = false;
            break;
        }
    }
}    

void Deposit(const std::shared_ptr<interfaces_pkg::srv::DepositingRequest::Request> request,
    std::shared_ptr<interfaces_pkg::srv::DepositingRequest::Response> response) {
        if (!request->start_depositing){
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Received request but start_depositing is false");
            response->depositing_successful = false;
            return;
        } //Checks to make sure start_depositing is set to true

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting depositing process");

        MoveBucket(3.8, 2.0, false, response); //Moves bucket up, tilts bucket 
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Scoop is lifted, beginning to jiggle");

        auto jiggleout_start = std::chrono::high_resolution_clock::now();
        while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - jiggleout_start).count() < 5) {
            MoveBucket(3.8, 1.8, true, response);
        } //Vibrates sand out of bucket
        vibrator.SetDutyCycle(0.0f); //ensures vibrator turns off
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Done jiggling, returning to resting position");
    
        MoveBucket(0.0, 0.0, false, response); //Moves bucket up, tilts bucket 
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Bucket successuly reset");
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
