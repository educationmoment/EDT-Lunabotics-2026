#include "SparkMax.hpp"
#include "rclcpp/rclcpp.hpp"
#include "interfaces_pkg/msg/motor_health.hpp"
#include "std_msgs/msg/float32.hpp"
#include "interfaces_pkg/srv/depositing_request.hpp"
#include "interfaces_pkg/srv/excavation_request.hpp"


class OdometryNode : public rclcpp::Node{
public:
    OdometryNode() : Node("odometry_node"), leftMotor("can0", 1),
    rightMotor("can0", 2) {
      depth_detection_pub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/depth_detection", 5,
        std::bind(&OdometryNode::positional_test, this, std::placeholders::_1)
      );

    depositing_client_ = (this->create_client<interfaces_pkg::srv::DepositingRequest>("depositing_service"));
    excavation_client_ = (this->create_client<interfaces_pkg::srv::ExcavationRequest>("excavation_service"));

      
}
private:
    SparkMax leftMotor;
    SparkMax rightMotor;
    //Motor controllers

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr depth_detection_pub_;
    rclcpp::Client<interfaces_pkg::srv::DepositingRequest>::SharedPtr depositing_client_;
    rclcpp::Client<interfaces_pkg::srv::ExcavationRequest>::SharedPtr excavation_client_;

    bool begin_excavation = false;
    bool begin_depositing = false;
    float distance = 0.0;

    enum class EXDEP {EXCAVATE, FORWARD, DEPOSIT, DONE};

    EXDEP state = EXDEP::EXCAVATE;

    void positional_test(const std_msgs::msg::Float32::SharedPtr depth_msg){
      switch (state){
        case EXDEP::EXCAVATE:
        if (!begin_excavation){
        if (!excavation_client_ || !excavation_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_ERROR(this->get_logger(), "Service not available");
            break;
        }
        auto request = std::make_shared<interfaces_pkg::srv::ExcavationRequest::Request>();
        request->start_excavation = true;
        begin_excavation = true;
        RCLCPP_INFO(this->get_logger(), "Excavation request sent");
        excavation_client_->async_send_request(request, [this](rclcpp::Client<interfaces_pkg::srv::ExcavationRequest>::SharedFuture future) {
            auto response = future.get();
            if (response->excavation_successful) {
            RCLCPP_INFO(this->get_logger(), "Excavation successful, moving forward to deposit");
            state = EXDEP::FORWARD;
            } else {
                RCLCPP_WARN(this->get_logger(), "Excavation failed");
            }
        });
        }
        break;

        case EXDEP::FORWARD:
        distance = depth_msg->data;
        if (distance < 1.0f){
            state = EXDEP::DEPOSIT;
        }
        else {
            leftMotor.SetVelocity(1500.0f);
            rightMotor.SetVelocity(1500.0f);
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
