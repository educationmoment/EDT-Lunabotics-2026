#include "SparkMax.hpp"
#include "rclcpp/rclcpp.hpp"
#include "interfaces_pkg/srv/excavation_request.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"

const float VIBRATOR_DUTY = 0.1f;
const float ERROR = 0.1f;

SparkMax leftDrive("can0", 1);
SparkMax rightDrive("can0", 2);
SparkMax leftLift("can0", 3);
SparkMax rightLift("can0", 4);
SparkMax tilt("can0", 5);
SparkMax vibrator("can0", 6); //Initalizes motor controllers

std::atomic<bool> bucket_filled = false;

void BucketSensorCallback(const std_msgs::msg::Float32::SharedPtr sensor_msg) {
    bucket_filled = sensor_msg->data > 90 ? true : false;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Bucket filled");
}

void MoveBucket (float lift_setpoint, float tilt_setpoint, bool activate_vibrator, bool drive_forward) {
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

        if (drive_forward){
            leftDrive.SetVelocity(1500.0f);
            rightDrive.SetVelocity(1500.0f);
        }

        if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - timer_start).count() > 5) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Skipping stage...");
            break;
        } //Timer for when to quit a stage due to timeout
        
        leftLiftReached = (fabs(lift_setpoint - leftLift.GetPosition() ) <=  ERROR);
        rightLiftReached = (fabs(lift_setpoint - rightLift.GetPosition() ) <=  ERROR);
        tiltReached = (fabs(tilt_setpoint - tilt.GetPosition() ) <=  ERROR); //Updates statuses
    }
}

void Excavate(const std::shared_ptr<interfaces_pkg::srv::ExcavationRequest::Request> request,
    std::shared_ptr<interfaces_pkg::srv::ExcavationRequest::Response> response) {
        if (!request->start_excavation){
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Received request but start_excavation is false");
            response->excavation_successful = false;
            return;
        } //Checks to make sure start_excavation is set to true

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting excavation process");

        MoveBucket(-3.2,-5.3, false, false);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Stage 1 complete");
        //Stage 1 

        auto dig_timer1 = std::chrono::high_resolution_clock::now();
        while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - dig_timer1).count() < 8){
            MoveBucket(-3.6,-3.5, true, true);
            std::this_thread::sleep_for(std::chrono::milliseconds(5)); //prevents CAN buffer from overflowing
            leftDrive.SetVelocity(1500.0f);
            rightDrive.SetVelocity(1500.0f);
            vibrator.SetDutyCycle(VIBRATOR_DUTY);
            //Keeps the drivetrain and vibrator moving even when the while loop is being skipped
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Stage 2 complete");
        //Stage 2

        auto dig_timer2 = std::chrono::high_resolution_clock::now();
        while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - dig_timer2).count() < 8){
            MoveBucket(-3.8,-3.0, true, true);
            std::this_thread::sleep_for(std::chrono::milliseconds(5)); //prevents CAN buffer from overflowing
            leftDrive.SetVelocity(1500.0f);
            rightDrive.SetVelocity(1500.0f);
            vibrator.SetDutyCycle(VIBRATOR_DUTY);
            //Keeps the drivetrain and vibrator moving even when the while loop is being skipped
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Stage 3 excavation completed, resetting bucket");
        //Stage 3

        leftDrive.SetDutyCycle(0.0f);
        rightDrive.SetDutyCycle(0.0f);
        vibrator.SetDutyCycle(0.0f);
        //Turns off da motors

        MoveBucket(0.0, 0.0, false, false); //Resets bucket
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Excavation Sequence successfully completed");
        response->excavation_successful = true;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("excavation_node");

    rclcpp::Service<interfaces_pkg::srv::ExcavationRequest>::SharedPtr service =
    node->create_service<interfaces_pkg::srv::ExcavationRequest>("excavation_service", &Excavate);

    auto bucket_subscription = node->create_subscription<std_msgs::msg::Float32>("arduino_data", 10, BucketSensorCallback);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Excavation Initalized");

    rclcpp::spin(node);
    rclcpp::shutdown();
}
