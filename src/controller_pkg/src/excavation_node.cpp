#include "SparkMax.hpp"
#include "rclcpp/rclcpp.hpp"
#include "interfaces_pkg/srv/excavation_request.hpp"
#include "std_msgs/msg/float32.hpp"

const float LIFT_OFFSET = 6.0f;
const float TILT_OFFSET = 3.0f;
const float VIBRATOR_DUTY = 0.2f;
const float ERROR = 0.1f;

std::atomic<bool> bucket_filled = false;

void BucketSensorCallback(const std_msgs::msg::Float32::SharedPtr msg) {
    bucket_filled = msg->data > 90 ? true : false;
}

void Excavate(const std::shared_ptr<interfaces_pkg::srv::ExcavationRequest::Request> request,
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

        vibrator.Heartbeat();
        vibrator.SetDutyCycle(0.0);

        auto tiltup_start = std::chrono::high_resolution_clock::now();
        while ((leftLift.GetPosition() < -0.7f - ERROR || leftLift.GetPosition() > -0.7f + ERROR) ||
        (tilt.GetPosition() < -1.41f - ERROR || tilt.GetPosition() -1.41f + ERROR)){
            leftLift.SetPosition(-0.7f);
            rightLift.SetPosition(-0.7f);
            tilt.SetPosition(-1.41f);

            while (fabs(leftLift.GetPosition() - rightLift.GetPosition()) >= 0.125){
                leftLift.SetPosition(rightLift.GetPosition());
                rightLift.SetPosition(leftLift.GetPosition());
            }

            if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - tiltup_start).count() >= 5){
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "ERROR: Initial tilt up failed to execute");
                break;
            }

        } //Brings actuators up and tilts bucket back

        auto dig1_start = std::chrono::high_resolution_clock::now();
        while ((leftLift.GetPosition() < -2.2f - ERROR || leftLift.GetPosition() > -2.2f + ERROR)){
            leftLift.SetPosition(-2.2f);
            rightLift.SetPosition(-2.2f);

            leftDrive.SetVelocity(1500.0f);
            rightDrive.SetVelocity(1500.0f);

            vibrator.SetDutyCycle(VIBRATOR_DUTY);

            while (fabs(leftLift.GetPosition() - rightLift.GetPosition()) >= 0.125){
                leftLift.SetPosition(rightLift.GetPosition());
                rightLift.SetPosition(leftLift.GetPosition());
            }

            if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - dig1_start).count() >= 5){
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "ERROR: Initial tilt up failed to execute");
                break;
            }

        } //Brings actuators up and tilts bucket back

      auto dig2_start = std::chrono::high_resolution_clock::now();
      while ((std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::high_resolution_clock::now() - dig2_start) <= std::chrono::seconds(10)) || !bucket_filled){
          leftLift.SetPosition(-2.0f);
          rightLift.SetPosition(-2.0f);

          leftDrive.SetVelocity(1500.0f);
          rightDrive.SetVelocity(1500.0f);

          tilt.SetPosition(-1.00f);
          vibrator.SetDutyCycle(VIBRATOR_DUTY);

          while (fabs(leftLift.GetPosition() - rightLift.GetPosition()) >= 0.125){
            leftLift.SetPosition(rightLift.GetPosition());
            rightLift.SetPosition(leftLift.GetPosition());
        }
      } //Brings actuators up and tilts bucket back

      leftDrive.SetDutyCycle(0.0f);
      rightDrive.SetDutyCycle(0.0f);
      vibrator.SetDutyCycle(0.0f);
      //Turns off da motors

      auto reset_start = std::chrono::high_resolution_clock::now();
      while (leftLift.GetPosition() < -0.25f || leftLift.GetPosition() > 0.25f){
          tilt.SetPosition(0.0f);
          leftLift.SetPosition(0.0f);
          rightLift.SetPosition(0.0f);

          if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - reset_start).count() >= 10){
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
    node->create_service<interfaces_pkg::srv::ExcavationRequest>("excavation_service", &Excavate);
    
    auto subscription = node->create_subscription<std_msgs::msg::Float32>("arduino_data", 10, BucketSensorCallback);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Excavation Initalized");

    rclcpp::spin(node);
    rclcpp::shutdown();
}
