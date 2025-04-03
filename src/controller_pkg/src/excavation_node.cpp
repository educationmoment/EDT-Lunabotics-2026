#include "SparkMax.hpp"
#include "rclcpp/rclcpp.hpp"
#include "interfaces_pkg/srv/excavation_request.hpp"

void add(const std::shared_ptr<interfaces_pkg::srv::ExcavationRequest::Request> request,
    std::shared_ptr<interfaces_pkg::srv::ExcavationRequest::Response> response) {
        if (!request->start_excavation){
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Received request but start_excavation is false");
            response->excavation_successful = false;
            return;
        } //Checks to make sure start_excavation is set to true

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting excavation process");
        
        SparkMax leftDrive("can0", 1);
        SparkMax rightDrive("can0", 2);
        SparkMax leftLift("can0", 3);
        SparkMax rightLift("can0", 4);
        SparkMax tilt("can0", 5);
        SparkMax vibrator("can0", 6); //Initalizes motor controllers

        auto tiltup_start = std::chrono::high_resolution_clock::now();
        while (leftLift.GetPosition() < 1.75f){
            leftLift.Heartbeat();
            leftLift.SetPosition(2.0f);
           
            rightLift.Heartbeat();
            rightLift.SetPosition(2.0f);
            
            tilt.Heartbeat();
            tilt.SetPosition(0.0f);
           
            if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - tiltup_start).count() >= 10){
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "ERROR: Initial tilt up failed to execute");
                return;
            }
           
        } //Brings actuators up and tilts bucket back
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Bucket above the sand, starting initial lowering");
    
    
        auto liftdown_start = std::chrono::high_resolution_clock::now();
        while (leftLift.GetPosition() > -1.75f){
            leftLift.Heartbeat();
            leftLift.SetPosition(-2.0f);
           
            rightLift.Heartbeat();
            rightLift.SetPosition(-2.0f);
            
            tilt.Heartbeat();
            tilt.SetPosition(1.5f);
           
            vibrator.Heartbeat();
            vibrator.SetDutyCycle(0.2f); //(not) MAXIMUM JIGGLE
           
            if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - liftdown_start).count() >= 10){
                response->excavation_successful = false;
                return;
            }
           
        } //Lowers lift actuators into the sand, tilts bucket forward, turns on jiggler
        
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Bucket lowered into sand, driving forward now");
    
        auto dig_start = std::chrono::high_resolution_clock::now();
        while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - dig_start).count() < 6) {
            //Timer will be replaced by bucket sensor system
            tilt.Heartbeat();
            tilt.SetPosition(1.5f);
            
            vibrator.Heartbeat();
            vibrator.SetDutyCycle(0.2f);
           
            leftDrive.Heartbeat();
            leftDrive.SetDutyCycle(0.3f);
           
            rightDrive.Heartbeat();
            rightDrive.SetDutyCycle(0.3f);
            //TO-DO: Convert to velocity and impliment current checking 
        }//Runs the scoop forward for a set amount of time, while tilting bucket up

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Bucket driven through sand, lifting up now");
       
        vibrator.Heartbeat();
        vibrator.SetDutyCycle(0.0f);
           
        leftDrive.Heartbeat();
        leftDrive.SetDutyCycle(0.0f);
           
        rightDrive.Heartbeat();
        rightDrive.SetDutyCycle(0.0f);
        //Resets duty cycle calls, turns off motors and vibrators
    
        auto liftup_start = std::chrono::high_resolution_clock::now();
        while (leftLift.GetPosition() < -0.25f){
            tilt.Heartbeat();
            tilt.SetPosition(3.0f);
            
            leftLift.Heartbeat();
            leftLift.SetPosition(0.0f);
           
            rightLift.Heartbeat();
            rightLift.SetPosition(0.0f);
           
            if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - liftup_start).count() >= 10){
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "ERROR: Bucket failed to reset");
                response->excavation_successful = false;
                return;
            } 
        }//Resets bucket to initial position

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Excavation Sequence successfully completed");
        response->excavation_successful = true;
        
    }
    
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("excavation_node");

    rclcpp::Service<interfaces_pkg::srv::ExcavationRequest>::SharedPtr service =
    node->create_service<interfaces_pkg::srv::ExcavationRequest>("excavation_service", &add);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Excavation Initalized");

    rclcpp::spin(node);
    rclcpp::shutdown();
}
