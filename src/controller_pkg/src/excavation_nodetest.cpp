#include "SparkMax.hpp"
#include "rclcpp/rclcpp.hpp"
#include "interfaces_pkg/srv/excavation_request.hpp"
#include "interfaces_pkg/msg/motor_health.hpp"
const float VIBRATOR_DUTY = 1.0f;
const float ERROR = 0.1f;
float buffer = 0.0f;
SparkMax leftDrive("can0", 1);
SparkMax rightDrive("can0", 2);
SparkMax leftLift("can0", 3);
SparkMax rightLift("can0", 4);
SparkMax tilt("can0", 5);
SparkMax vibrator("can0", 6); //Initalizes motor controllers
rclcpp::Subscription<interfaces_pkg::msg::MotorHealth>::SharedPtr health_subscriber_;
std::shared_ptr<rclcpp::Node> node;
/**
 * @brief MoveBucket positions the bucket to a predefined position specified in the passed
 *        parameters. By choice, the vibrator can be enabled to assist in cleaving through regolith.
 *        Motor output during excavation cycle is also passed to function call.
 * @param lift_setpoint Desired lift actuator setpoint expressed as a float
 * @param tilt_setpoint Desired tilt actuator setpoint expressed as a float
 * @param activate_vibrator Should the vibrator be activated? (true == yes)
 * @param drive_speed Floating point value used to express desired motor speed during auto excavation
 * @returns None
 */
void MoveBucket (float lift_setpoint, float tilt_setpoint, bool activate_vibrator, float drive_speed) {
    const float HIGH_PASS_FILTER = 0.38; // 0.38 position units
    const float KP_LIFT = 8.0f; // Proportional gain
    
    auto timer_start = std::chrono::high_resolution_clock::now();
    bool leftLiftReached = (fabs(lift_setpoint - leftLift.GetPosition() ) <=  ERROR);
    bool rightLiftReached = (fabs(lift_setpoint - rightLift.GetPosition() ) <=  ERROR);
    bool tiltReached = (fabs(tilt_setpoint - tilt.GetPosition()) <=  ERROR);
    while (!((leftLiftReached && rightLiftReached) && tiltReached)){
        std::this_thread::sleep_for(std::chrono::milliseconds(5)); //prevents CAN buffer from overflowing
        
        // ---- LIFT POSITION SYNC (from controller_node) ---- //
        // Compute lift error (right - left)
        float lift_error = rightLift.GetPosition() - leftLift.GetPosition();
        
        // High-pass deadband
        if (fabs(lift_error) < HIGH_PASS_FILTER)
            lift_error = 0.0f;
        
        // Determine base direction for each actuator based on setpoint
        float left_position_error = lift_setpoint - leftLift.GetPosition();
        float right_position_error = lift_setpoint - rightLift.GetPosition();
        
        // Determine base duty cycle direction (towards setpoint)
        float left_duty = (left_position_error > ERROR) ? 1.0f : 
                         ((left_position_error < -ERROR) ? -1.0f : 0.0f);
        float right_duty = (right_position_error > ERROR) ? 1.0f : 
                          ((right_position_error < -ERROR) ? -1.0f : 0.0f);
        
        // Proportional correction (always slows the higher side)
        float correction = KP_LIFT * fabs(lift_error);
        
        // Right is higher → slow right
        if (lift_error > 0) {
            right_duty = right_duty - correction;
        }
        // Left is higher → slow left
        else if (lift_error < 0) {
            left_duty = left_duty - correction;
        }
        
        // Clamp duties
        left_duty = std::clamp(left_duty, -1.0f, 1.0f);
        right_duty = std::clamp(right_duty, -1.0f, 1.0f);
        
        // Set lift duties with alignment correction
        leftLift.SetDutyCycle(left_duty);
        rightLift.SetDutyCycle(right_duty);
        
        // Set tilt position
        tilt.SetPosition(tilt_setpoint);
        if (activate_vibrator) vibrator.SetDutyCycle(VIBRATOR_DUTY);
        
        leftDrive.SetVelocity(drive_speed);
        rightDrive.SetVelocity(drive_speed);
        if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - timer_start).count() > 5) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Skipping stage...");
            break;
        } //Timer for when to quit a stage due to timeout
        
        leftLiftReached = (fabs(lift_setpoint - leftLift.GetPosition() ) <=  ERROR);
        rightLiftReached = (fabs(lift_setpoint - rightLift.GetPosition() ) <=  ERROR);
        tiltReached = (fabs(tilt_setpoint - tilt.GetPosition() ) <=  ERROR); //Updates statuses
    }
}
/**
 * @param health_msg interfaces_pkg::msg::MotorHealth, tilt_position read from node and stored in buffer
 * @returns None
 */
void updateTiltPosition(const interfaces_pkg::msg::MotorHealth::SharedPtr health_msg){
    buffer = health_msg->tilt_position;
}
/**
 * @brief Callback for interfaces_pkg::srv::ExcavationRequest::Request interface. Handles autonomous excavation.
 * @param request std::shared_ptr<interfaces_pkg::srv::ExcavationRequest::Request>, client provided request
 * @param response std::shared_ptr<interfaces_pkg::srv::ExcavationRequest::Request>, server response
 * @returns None
 */
void Excavate(const std::shared_ptr<interfaces_pkg::srv::ExcavationRequest::Request> request,
    std::shared_ptr<interfaces_pkg::srv::ExcavationRequest::Response> response) {
        if (!request->start_excavation){
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Received request but start_excavation is false");
            response->excavation_successful = false;
            return;
        } //Checks to make sure start_excavation is set to true
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Excavation Sequence successfully completed, new buffer at %f", buffer);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting excavation process");
        MoveBucket(-2.5, -2.6 + buffer, false, 0.0f);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Stage 1 complete");
        //Stage 1 
        MoveBucket(-3.0,-2.6 + buffer, true, 1500.0f);
        auto dig_timer1 = std::chrono::high_resolution_clock::now();
        while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - dig_timer1).count() < 2){
            leftDrive.SetVelocity(1500.0f);
            rightDrive.SetVelocity(1500.0f);
            vibrator.SetDutyCycle(VIBRATOR_DUTY);
            MoveBucket(-2.5,-3.2 + buffer, true, 1500.0f); // old lift -3.3, tilt -3.2
            std::this_thread::sleep_for(std::chrono::milliseconds(5)); //prevents CAN buffer from overflowing
            //Keeps the drivetrain and vibrator moving even when the while loop is being skipped
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Stage 2 complete");
        //Stage 2
        auto dig_timer2 = std::chrono::high_resolution_clock::now();
        while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - dig_timer2).count() < 2){
            leftDrive.SetVelocity(1000.0f);
            rightDrive.SetVelocity(1000.0f);
            vibrator.SetDutyCycle(VIBRATOR_DUTY);
            MoveBucket(-2.7 ,-3.0 + buffer, true, 1000.0f); // old lift -3.5, tilt -3.0
            std::this_thread::sleep_for(std::chrono::milliseconds(5)); //prevents CAN buffer from overflowing
            //Keeps the drivetrain and vibrator moving even when the while loop is being skipped
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Stage 3 excavation completed, resetting bucket");
        //Stage 3
        auto dig_timer3 = std::chrono::high_resolution_clock::now();
        while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - dig_timer3).count() < 2){
            leftDrive.SetVelocity(1000.0f);
            rightDrive.SetVelocity(1000.0f); 
            vibrator.SetDutyCycle(VIBRATOR_DUTY);
            MoveBucket(-1.9,-2.5 + buffer, true, 1000.0f); // old lift -3.5, tilt -2.5
            std::this_thread::sleep_for(std::chrono::milliseconds(5)); //prevents CAN buffer from overflowing
            //Keeps the drivetrain and vibrator moving even when the while loop is being skipped
        }
        auto dig_timer4 = std::chrono::high_resolution_clock::now();
        while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - dig_timer4).count() < 4){
            leftDrive.SetVelocity(500.0f);
            rightDrive.SetVelocity(500.0f); //reduced RPM to 500
            vibrator.SetDutyCycle(VIBRATOR_DUTY);
            MoveBucket(-1.9,-2.5 + buffer, true, 500.0f); // old lift -3.5, tilt -2.5
            std::this_thread::sleep_for(std::chrono::milliseconds(5)); //prevents CAN buffer from overflowing
            //Keeps the drivetrain and vibrator moving even when the while loop is being skipped
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Stage 3 excavation completed, resetting bucket");
        leftDrive.SetDutyCycle(0.0f);
        rightDrive.SetDutyCycle(0.0f);
        vibrator.SetDutyCycle(0.0f);
        MoveBucket(0.0, 0.0 + buffer, false, 0.0f); //Resets bucket
        auto reset_tilt = std::chrono::high_resolution_clock::now();
        while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - reset_tilt).count() < 1){
            tilt.SetDutyCycle(1.0f);
            std::this_thread::sleep_for(std::chrono::milliseconds(5)); //prevents CAN buffer from overflowing
        }
        response->excavation_successful = true;
}
int main(int argc, char **argv) {
    rclcpp::init(argc, argv); 
    node = rclcpp::Node::make_shared("excavation_node");
    rclcpp::Service<interfaces_pkg::srv::ExcavationRequest>::SharedPtr service =
    node->create_service<interfaces_pkg::srv::ExcavationRequest>("excavation_service", &Excavate);
    health_subscriber_ = node->create_subscription<interfaces_pkg::msg::MotorHealth>(
    "/health_topic", 10, updateTiltPosition);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Excavation Initalized");
    rclcpp::spin(node);
    rclcpp::shutdown();
}