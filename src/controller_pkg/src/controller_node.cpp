// pilot_node.cpp
#include "SparkMax.hpp" // Assumes SparkMax.hpp is available in your include path
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <cmath>
#include <string>
#include <vector>
#include <cassert>

#define DRIVE_MOTOR_SCALE 0.25
#define LIFT_MOTOR_SCALE 0.5
#define TILT_MOTOR_SCALE 0.0

#define LEFT_JOY_VERTICAL 1
#define LEFT_JOY_HORIZONTAL 0

#define RIGHT_JOY_VERTICAL 3
#define RIGHT_JOY_HORIZONTAL 2
#define RIGHT_JOY_TRIGGER 5
#define LEFT_JOY_TRIGGER 4

// Define CAN IDs for our devices (vibrator is omitted)
// #define LEFT_MOTOR  1
// #define RIGHT_MOTOR 2
// #define LEFT_LIFT   3
// #define RIGHT_LIFT  4
// #define TILT        5

enum CAN_IDs
{
    LEFT_MOTOR = 1,
    RIGHT_MOTOR = 2,
    LEFT_LIFT = 3,
    RIGHT_LIFT = 4,
    TILT = 5
};

class PilotNode : public rclcpp::Node
{
public:
    PilotNode(const std::string &can_interface)
        : Node("pilot_node"),
          leftMotor(can_interface, LEFT_MOTOR),
          rightMotor(can_interface, RIGHT_MOTOR),
          leftLift(can_interface, LEFT_LIFT),
          rightLift(can_interface, RIGHT_LIFT),
          tilt(can_interface, TILT)
    {

        // Burn flash for all controllers
        RCLCPP_INFO(this->get_logger(), "Burning flash for all controllers");
        (void)burn_flash();

        // Create subscription to the /joy topic.
        RCLCPP_INFO(this->get_logger(), "Creating subscription to /joy topic");
        joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10,
            std::bind(&PilotNode::joy_callback, this, std::placeholders::_1));

        // Create a timer to send heartbeat messages.
        // RCLCPP_INFO(this->get_logger(), "Creating timer to send heartbeat messages");
        // timer_ = this->create_wall_timer(
        //   std::chrono::milliseconds(100),
        //   // std::bind(&PilotNode::send_heartbeat, this, std::)
        //   std::bind(&PilotNode::send_heartbeat, this, std::vector<SparkMax>{leftMotor, rightMotor, leftLift, rightLift, tilt})
        // );
        RCLCPP_INFO(this->get_logger(), "PilotNode initialized on CAN interface: %s", can_interface.c_str());
    }

private:
    // Motor controller objects using SparkMax.
    SparkMax
        leftMotor,
        rightMotor,
        leftLift,
        rightLift,
        tilt;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Helper: Computes and Returns duty cycle from joystick axis value.
    // The axis value is expected to be in the range [-1.0, 1.0]. (Less depending on scale)
    float computeDutyFromAxis(float axisValue, float scale_factor = 0.0f, float deadband = 0.1f)
    {
        if (std::fabs(axisValue) < deadband)
        {
            RCLCPP_WARN( this->get_logger(), "Axos value is below deadband: %f", axisValue);
            return 0.0f;
        }
        RCLCPP_INFO(this->get_logger(), "Axis value: %f", axisValue);
        float scaled_value = axisValue * scale_factor;

        RCLCPP_INFO(this->get_logger(), "Scaled value: %f", scaled_value);

        // Test Assertions
        assert(axisValue >= -1.0f && axisValue <= 1.0f);
        assert(scale_factor >= 0.0f && scale_factor <= 1.0f);
        assert(scaled_value >= -1.0f && scaled_value <= 1.0f);
        return (axisValue > 0) ? axisValue * scale_factor : -axisValue * scale_factor;
    }

    // Helper: sends heartbeat to all controllers.
    // This is a no-op if the controller is not connected.
    void send_heartbeat(std::vector<SparkMax> controllers)
    {
        RCLCPP_INFO(this->get_logger(), "Sending heartbeat to all controllers");
        // Send heartbeat to all controllers.
        for (const auto &controller : controllers)
        {
            (void)controller.Heartbeat();
        }
        return;
    }

    // Burn flash for all Controllers
    void burn_flash()
    {
        // Configure all controllers to burn flash.
        // --- Configure Lift Motors (Brushed with Encoder) ---
        leftLift.SetIdleMode(IdleMode::kBrake);
        rightLift.SetIdleMode(IdleMode::kBrake);
        leftLift.SetMotorType(MotorType::kBrushed);
        rightLift.SetMotorType(MotorType::kBrushed);
        leftLift.SetSensorType(SensorType::kEncoder);
        rightLift.SetSensorType(SensorType::kEncoder);

        // --- Configure Tilt (Brushed with Encoder) ---
        tilt.SetIdleMode(IdleMode::kBrake);
        tilt.SetMotorType(MotorType::kBrushed);
        tilt.SetSensorType(SensorType::kEncoder);

        // --- Invert All Motors ---
        leftMotor.SetInverted(false);
        rightMotor.SetInverted(true);
        leftLift.SetInverted(true);
        rightLift.SetInverted(true);
        tilt.SetInverted(true);

        // Burn flash for all controllers.
        this->leftMotor.BurnFlash();
        this->rightMotor.BurnFlash();
        this->leftLift.BurnFlash();
        this->rightLift.BurnFlash();
        this->tilt.BurnFlash();
        return;
    }

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
    {
        if (joy_msg->axes.size() < 2 || joy_msg->buttons.size() < 8)
        {
            RCLCPP_WARN(this->get_logger(), "Insufficient axes/buttons in Joy message");
            return;
        }

        // Attempt to send commands to the motors.
        try
        {
            // Send heartbeat to maintain communication.
            // this->send_heartbeat({leftMotor, rightMotor, leftLift, rightLift, tilt});
            (void)leftMotor.Heartbeat();
            (void)rightMotor.Heartbeat();
            (void)leftLift.Heartbeat();
            (void)rightLift.Heartbeat();
            (void)tilt.Heartbeat();

            // --- Drive Motors ---
            // Use the left joystick vertical axis (axes[1]) for drive.
            // Invert so that pushing forward (normally negative) yields a positive command.

            assert( joy_msgs->axes.size() >= LEFT_JOY_VERTICAL && joy_msgs->axes.size() >= RIGHT_JOY_VERTICAL);


            /* Implement Tank Steering of the Robot */
            //---------------------------------------------//
            float 
                drive_duty_left = computeDutyFromAxis(joy_msg->axes[LEFT_JOY_VERTICAL], DRIVE_MOTOR_SCALE),
                drive_duty_right = computeDutyFromAxis(joy_msg->axes[RIGHT_JOY_VERTICAL], DRIVE_MOTOR_SCALE);
            //---------------------------------------------//
            

            (void)leftMotor.SetDutyCycle(drive_duty_left);
            (void)rightMotor.SetDutyCycle(drive_duty_right);

            // --- Lift Motors (Actuators) ---
            // Instead of a single trigger, use both triggers:
            // Right trigger (button index 7) yields +0.5 duty cycle.
            // Left trigger (button index 6) yields -0.5 duty cycle.
            // float lift_duty = 0.0f;
            float lift_duty = computeDutyFromAxis(joy_msg->axes[RIGHT_JOY_TRIGGER] - joy_msg->axes[LEFT_JOY_TRIGGER], LIFT_MOTOR_SCALE);

            assert(joy_msg->buttons.size() >= 8);
            if (joy_msg->buttons[7] > 0)
            {
                lift_duty = 0.5f;
            }
            else if (joy_msg->buttons[6] > 0)
            {
                lift_duty = -0.5f;
            }

            // Command lift motors.
            (void)leftLift.SetDutyCycle(lift_duty);
            (void)rightLift.SetDutyCycle(lift_duty);

            // Command tilt (here, no command; adjust as needed).
            (void)tilt.SetDutyCycle(0.0f);
        }
        catch (const std::exception &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Error sending CAN command: %s", ex.what());
        }
        return;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::string can_interface = "can0";
    auto temp_node = rclcpp::Node::make_shared("pilot_param_node");
    temp_node->declare_parameter<std::string>("can_interface", "can0");
    temp_node->get_parameter("can_interface", can_interface);

    auto node = std::make_shared<PilotNode>(can_interface);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
