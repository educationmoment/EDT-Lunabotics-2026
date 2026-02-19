#include "SparkMax.hpp"
#include "rclcpp/rclcpp.hpp"
#include "interfaces_pkg/srv/depositing_request.hpp"
#include "std_msgs/msg/bool.hpp"

const float VIBRATOR_DUTY = 1.0f;
const float ERROR = 0.1;

SparkMax leftLift("can0", 3);
SparkMax rightLift("can0", 4);
SparkMax tilt("can0", 5);
SparkMax vibrator("can0", 6);
//Initalizes motor controllers


/**
 * @brief Move the bucket to a setpoint and activate as specified by the parameters. 
 * @param lift_setpoint Setpoint for the lift actuators expressed as a float
 * @param tilt_setpoint Setpoint for the tilt actuators expressed as a float
 * @param activate_vibrator Should the vibrator be set on? (true == yes)
 * @returns None
 */
void MoveBucket (float lift_setpoint, float tilt_setpoint, bool activate_vibrator) {
    const float HIGH_PASS_FILTER = 0.38; // 0.38 position units
    const float KP_LIFT = 8.0f; // Proportional gain
    
    auto timer_start = std::chrono::high_resolution_clock::now();
    bool leftLiftReached = (fabs(lift_setpoint - leftLift.GetPosition() ) <=  ERROR);
    bool rightLiftReached = (fabs(lift_setpoint - rightLift.GetPosition() ) <=  ERROR);
    bool tiltReached = (fabs(tilt_setpoint - tilt.GetPosition() ) <=  ERROR);

    while (!((leftLiftReached && rightLiftReached) && tiltReached)){
        std::this_thread::sleep_for(std::chrono::milliseconds(5)); //prevents CAN buffer from overflowing
        
        // ---- LIFT POSITION SYNC (from controller_node) ---- //
        // Compute lift error (right - left)
        float lift_error = rightLift.GetPosition() - leftLift.GetPosition();
        
        // High-pass deadband
        if (fabs(lift_error) < HIGH_PASS_FILTER)
            lift_error = 0.0f;
        
        // Determine direction to move towards setpoint
        float left_position_error = lift_setpoint - leftLift.GetPosition();
        float right_position_error = lift_setpoint - rightLift.GetPosition();
        
        // Base duty cycles (direction towards setpoint)
        float left_duty = (left_position_error > ERROR) ? 1.0f : 
                         ((left_position_error < -ERROR) ? -1.0f : 0.0f);
        float right_duty = (right_position_error > ERROR) ? 1.0f : 
                          ((right_position_error < -ERROR) ? -1.0f : 0.0f);
        
        // Proportional correction (slows the higher side)
        float correction = KP_LIFT * fabs(lift_error);
        
        // Apply correction: subtract from whichever side is higher
        if (lift_error > 0) {
            // Right is higher → slow right
            right_duty = right_duty - correction;
        }
        else if (lift_error < 0) {
            // Left is higher → slow left
            left_duty = left_duty - correction;
        }
        
        // Clamp duties to valid range
        left_duty = std::clamp(left_duty, -1.0f, 1.0f);
        right_duty = std::clamp(right_duty, -1.0f, 1.0f);
        
        // Set lift duties with alignment correction
        leftLift.SetDutyCycle(left_duty);
        rightLift.SetDutyCycle(right_duty);
        
        // Set tilt position
        tilt.SetPosition(tilt_setpoint);

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
/**
 * @brief Begins autonomous depositing cycle. If request is TRUE, a cycle is begun with a 20 second 
 *        duration used to shake material out of the bucket. Afterwards, resets bucket.
 * @param request interfaces_pkg::srv::DepositingRequest::Request is the request from the client node (controller_node)
 * @param response interfaces_pkg::srv::DepositingRequest::Response is the response to the client node
 * @returns None
 */
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
        while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - jiggleout_start).count() < 10) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5)); //prevents CAN buffer from overflowing
            vibrator.SetDutyCycle(VIBRATOR_DUTY);
        } //Vibrates sand out of bucket

        vibrator.SetDutyCycle(0.0f); //ensures vibrator turns off
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Stage 2 complete, resetting bucket");

        MoveBucket(0.0, 0.0, false); //Resets bucket
        auto reset_tilt = std::chrono::high_resolution_clock::now();
        while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - reset_tilt).count() < 1){
            tilt.SetDutyCycle(1.0f);
            std::this_thread::sleep_for(std::chrono::milliseconds(5)); //prevents CAN buffer from overflowing
        }
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