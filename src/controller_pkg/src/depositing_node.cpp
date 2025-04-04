#include "SparkMax.hpp"
#include "rclcpp/rclcpp.hpp"
#include "interfaces_pkg/srv/depositing_request.hpp"

void add(const std::shared_ptr<interfaces_pkg::srv::DepositingRequest::Request> request,
    std::shared_ptr<interfaces_pkg::srv::DepositingRequest::Response> response) {
        if (!request->start_depositing){
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Received request but start_depositing is false");
            response->depositing_successful = false;
            return;
        } //Checks to make sure start_depositing is set to true

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting depositing process");
        
        SparkMax leftDrive("can0", 1);
        SparkMax rightDrive("can0", 2);
        SparkMax leftLift("can0", 3);
        SparkMax rightLift("can0", 4);
        SparkMax tilt("can0", 5);
        SparkMax vibrator("can0", 6); //Initalizes motor controllers

        auto bucketliftup_start = std::chrono::high_resolution_clock::now();
        while (leftLift.GetPosition() <= 4.60f) {
            leftLift.Heartbeat();
            leftLift.SetPosition(5.0f);

            rightLift.Heartbeat();
            rightLift.SetPosition(5.0f);

            tilt.Heartbeat();
            tilt.SetPosition(4.0f);

            if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - bucketliftup_start).count() > 10) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "ERROR: Scoop failed to lift");
                response->depositing_successful = false;
                return;
            }
        } //Lifts the scoop up while tilting the bucket backwards

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Scoop is lifted, beginning to jiggle");

        auto jiggleout_start = std::chrono::high_resolution_clock::now();
        while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - jiggleout_start).count() < 5) {
            tilt.Heartbeat();
            tilt.SetPosition(3.0f);

            vibrator.Heartbeat();
            vibrator.SetDutyCycle(0.25f);
        } //Tilts the bucket and vibrates it to remove regolith

        vibrator.Heartbeat();
        vibrator.SetDutyCycle(0.0f);
        //Resets vibrator

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Done jiggling, returning to resting position");

        auto liftdown_start = std::chrono::high_resolution_clock::now();
        while (leftLift.GetPosition() >= 0.25f) {
            leftLift.Heartbeat();
            leftLift.SetPosition(0.0f);

            rightLift.Heartbeat();
            rightLift.SetPosition(0.0f);

            tilt.Heartbeat();
            tilt.SetPosition(3.0f);
            
            if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - liftdown_start).count() > 5) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "ERROR: Scoop failed to reset");
                response->depositing_successful = false;
                return;
            }
        } //Returns scoop to its default position 

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Depositing successful, returning to starting position");

        response->depositing_successful = true;
    }
    
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("depositing_node");

    rclcpp::Service<interfaces_pkg::srv::DepositingRequest>::SharedPtr service =
    node->create_service<interfaces_pkg::srv::DepositingRequest>("depositing_service", &add);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Depositing Initalized");

    rclcpp::spin(node);
    rclcpp::shutdown();
}
