#include "SparkMax.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/string.hpp"
#include "interfaces_pkg/msg/motor_health.hpp"
#include "interfaces_pkg/srv/depositing_request.hpp"
#include "interfaces_pkg/srv/excavation_request.hpp"
#include <cmath>
#include <string>
#include <cstdlib>
#include <algorithm>

const float VELOCITY_MAX = 2500.0; //rpm, after gearbox turns into 11.1 RPM
const float VIBRATOR_OUTPUT = 1.0f; //Constant value for vibrator output

enum CAN_IDs {
  LEFT_MOTOR  = 1,
  RIGHT_MOTOR = 2,
  LEFT_LIFT   = 3,
  RIGHT_LIFT  = 4,
  TILT        = 5,
  VIBRATOR    = 6
};

enum BUTTONS_CONTROLLER {
  RIGHT_BUMPER = 5,
  RIGHT_TRIGGER = 6,
  LEFT_TRIGGER  = 7,
  Y_BUTTON = 3
  A_BUTTON = 0
};

enum JOYSTICK_CONTROLLER {
  HORIZONTAL_LEFT_JOY = 0,
  VERTICAL_LEFT_JOY = 1
};

/**
 * @class Class::ControllerNode
 * @brief Converts controller input from /joy
 *        into commands. Used for both manual and
 *        automatic control.
 * 
 * ==================================================
 * @details Node burns flash for sparkmax motor controllers,
 *          handles sending heartbeats at 1 second intervals.
 *          The node also handles handoff of automatic control between
 *          depositing and excavation agents.
 * 
 *********************************************************************/
class ControllerNode : public rclcpp::Node
{
public:

  /**
   * @brief Node Constructor: flashes Sparkmax controllers, initiallizes timer, joy subscription, and depositing and excavation service calls.
   * @param can_interface Sets the OS interface used by Sparkcan
   *********************************************************************/
  ControllerNode(const std::string & can_interface)
  : Node("controller_node"),
    leftMotor(can_interface, LEFT_MOTOR),
    rightMotor(can_interface, RIGHT_MOTOR),
    leftLift(can_interface, LEFT_LIFT),
    rightLift(can_interface, RIGHT_LIFT),
    tilt(can_interface, TILT),
    vibrator(can_interface, VIBRATOR),
    vibrator_active_(false),
    prev_vibrator_button_(false),
    alternate_mode_active_(false),
    prev_alternate_button_(false)
  {
    RCLCPP_INFO(this->get_logger(), "Begin Initializing Node");

    RCLCPP_INFO(this->get_logger(), "Initializing Motor Controllers");

    leftMotor.SetIdleMode(IdleMode::kBrake);
    rightMotor.SetIdleMode(IdleMode::kBrake);
    leftMotor.SetMotorType(MotorType::kBrushless);
    rightMotor.SetMotorType(MotorType::kBrushless);
    leftMotor.SetSensorType(SensorType::kHallSensor);
    rightMotor.SetSensorType(SensorType::kHallSensor);
    //Initializes the settings for the drivetrain motors

    leftLift.SetIdleMode(IdleMode::kBrake);
    rightLift.SetIdleMode(IdleMode::kBrake);
    leftLift.SetMotorType(MotorType::kBrushed);
    rightLift.SetMotorType(MotorType::kBrushed);
    leftLift.SetSensorType(SensorType::kEncoder);
    rightLift.SetSensorType(SensorType::kEncoder);
    //Initializes the settings for the lift actuators

    tilt.SetIdleMode(IdleMode::kBrake);
    tilt.SetMotorType(MotorType::kBrushed);
    tilt.SetSensorType(SensorType::kEncoder);
    //Initializes the settings for the tilt actuator

    vibrator.SetIdleMode(IdleMode::kBrake);
    vibrator.SetMotorType(MotorType::kBrushed);
    vibrator.SetSensorType(SensorType::kEncoder);
    //Initializes the settings for the vibrator

    leftMotor.SetInverted(false);
    rightMotor.SetInverted(true);
    leftLift.SetInverted(true);
    rightLift.SetInverted(true);
    tilt.SetInverted(true);
    vibrator.SetInverted(true);
    //Initializes the inverting status

    leftMotor.SetP(0, 0.0002f);
    leftMotor.SetI(0, 0.0f);
    leftMotor.SetD(0, 0.0f);
    leftMotor.SetF(0, 0.00021f);
    //PID settings for left motor

    rightMotor.SetP(0, 0.0002f);
    rightMotor.SetI(0, 0.0f);
    rightMotor.SetD(0, 0.0f);
    rightMotor.SetF(0, 0.00021f);
    //PID settings for right motor

    leftLift.SetP(0, 1.51f);
    leftLift.SetI(0, 0.0f);
    leftLift.SetD(0, 0.0f);
    leftLift.SetF(0, 0.00021f);
    //PID settings for left lift

    rightLift.SetP(0, 1.51f);
    rightLift.SetI(0, 0.0f);
    rightLift.SetD(0, 0.0f);
    rightLift.SetF(0, 0.00021f);
    //PID settings for right lift

    //PID settings for tilt 
    tilt.SetP(0, 1.51f);
    tilt.SetI(0, 0.0f);
    tilt.SetD(0, 0.0f);
    tilt.SetF(0, 0.00021f);
    //PID settings for tilt

    leftMotor.BurnFlash();
    rightMotor.BurnFlash();
    leftLift.BurnFlash();
    rightLift.BurnFlash();
    tilt.BurnFlash();
    vibrator.BurnFlash();
    RCLCPP_INFO(this->get_logger(), "Motor Controllers Initialized");

    // ---ROS SUBSCRIPTIONS--- //
    RCLCPP_INFO(this->get_logger(), "Initializing Joy Subscription");
    joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10,
        std::bind(&ControllerNode::joy_callback, this, std::placeholders::_1)
    );
    RCLCPP_INFO(this->get_logger(), "Joy Subscription Initialized");

    health_subscriber_ = this->create_subscription<interfaces_pkg::msg::MotorHealth>(
      "/health_topic", 10,
      std::bind(&ControllerNode::position_callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "Initializing depositing and excavation client");
    depositing_client_ = (this->create_client<interfaces_pkg::srv::DepositingRequest>("depositing_service"));
    excavation_client_ = (this->create_client<interfaces_pkg::srv::ExcavationRequest>("excavation_service"));
    RCLCPP_INFO(this->get_logger(), "Excavation and depositing client initialized");

    RCLCPP_INFO(this->get_logger(), "Initializing Heartbeat Publisher");
    heartbeatPub = this->create_publisher<std_msgs::msg::String>("/heartbeat", 10);
    RCLCPP_INFO(this->get_logger(), "Heartbeat Publisher Initialized");

    RCLCPP_INFO(this->get_logger(), "Initializing Timer");
    timer = this->create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&ControllerNode::publish_heartbeat, this)
    );
    RCLCPP_INFO(this->get_logger(), "Timer Initialized");

    RCLCPP_INFO(this->get_logger(), "Node Initialization Complete");
  }

private:
  // Direct object members.
  SparkMax leftMotor;
  SparkMax rightMotor;
  SparkMax leftLift;
  SparkMax rightLift;
  SparkMax tilt;
  SparkMax vibrator;

  rclcpp::Client<interfaces_pkg::srv::DepositingRequest>::SharedPtr depositing_client_;
  rclcpp::Client<interfaces_pkg::srv::ExcavationRequest>::SharedPtr excavation_client_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
  rclcpp::Subscription<interfaces_pkg::msg::MotorHealth>::SharedPtr health_subscriber_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeatPub;
  rclcpp::TimerBase::SharedPtr timer;

  //Autonomy flag
  bool is_autonomy_active_ = false;

  // Vibrator toggle
  bool vibrator_active_;
  bool prev_vibrator_button_;


  /**
   * @brief Used to determine variable motor power during turning
   * @param value float
   * 
   *********************************************************************/
  float computeStepDuty(float value)
  {
    float absVal = std::fabs(value);
    if (absVal < 0.25f)
      return 0.0f;
    else if (absVal < 0.5f)
      return (value > 0 ? 0.25f : -0.25f);
    else if (absVal < 0.75f)
      return (value > 0 ? 0.5f : -0.5f);
    else if (absVal < 1.0f)
      return (value > 0 ? 0.75f : -0.75f);
    return (value > 0 ? 1.0f : -1.0f);
  }

  /**
   * @brief Sends request to depositing node and manages response
   * @param None
   *********************************************************************/
  void send_deposit_request() {
    // Determine of the Depositing Client is Available
    if (!depositing_client_ || !depositing_client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_ERROR(this->get_logger(), "Service not available");
      return;
    }
    auto request = std::make_shared<interfaces_pkg::srv::DepositingRequest::Request>();
    request->start_depositing = true;

    // Send Depositing Request
    depositing_client_->async_send_request(request, [this](rclcpp::Client<interfaces_pkg::srv::DepositingRequest>::SharedFuture future) {
      auto response = future.get();
      if (response->depositing_successful) {
        RCLCPP_INFO(this->get_logger(), "Depositing successful");
      } else {
        RCLCPP_WARN(this->get_logger(), "Depositing failed");
      }
    });
  }


  /**
   * @brief Sends request to excavation node and manages response
   * @param None
   *********************************************************************/
  void send_excavation_request() {
    // Determine if the Excavation Client is available
    if (!excavation_client_ || !excavation_client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_ERROR(this->get_logger(), "Service not available");
      return;
    }



    auto request = std::make_shared<interfaces_pkg::srv::ExcavationRequest::Request>();
    request->start_excavation = true;

    // Send Excavation Request
    excavation_client_->async_send_request(request, [this](rclcpp::Client<interfaces_pkg::srv::ExcavationRequest>::SharedFuture future) {
      auto response = future.get();
      if (response->excavation_successful) {
        RCLCPP_INFO(this->get_logger(), "Excavation successful");
      } else {
        RCLCPP_WARN(this->get_logger(), "Excavation failed");
      }
    });
  }


  /**
   * @brief Handler for received /joy subscription messages.
   *        Primary source of manual/semiautomatic control.
   * @param joy_msg sensor_msgs::msg::SharedPtr
   *********************************************************************/
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
  {
    if (joy_msg->axes.size() < 2 || joy_msg->buttons.size() < 17) {
      RCLCPP_WARN(this->get_logger(), "Insufficient axes/buttons in Joy message");
      return;
    }


    //SAFETY LOCK (Right or left trigger)//
    bool triggersPressed = (joy_msg->buttons[ BUTTONS_CONTROLLER::RIGHT_TRIGGER ] > 0 || joy_msg->buttons[ BUTTONS_CONTROLLER::LEFT_TRIGGER ] > 0);
    //SAFETY LOCK (Right or left trigger)//

    //VIBRATOR (Right bumper, triggers)//
    bool current_vibrator_button = (joy_msg->buttons[ BUTTONS_CONTROLLER::RIGHT_BUMPER ] > 0);

    if (current_vibrator_button && !prev_vibrator_button_) {
      vibrator_active_ = !vibrator_active_;
      RCLCPP_INFO(this->get_logger(), "Vibrator toggled %s", vibrator_active_ ? "ON" : "OFF");
    }
    prev_vibrator_button_ = current_vibrator_button;
    float vibrator_duty = vibrator_active_ ? VIBRATOR_OUTPUT : 0.0f;

    vibrator.SetDutyCycle(vibrator_duty);

    if (joy_msg->buttons[2] > 0){
      leftLift.SetPosition(0.0f);
      rightLift.SetPosition(0.0f);
      tilt.SetPosition(0.0f);
    }
    else { 
    // TILT ACTUATOR (D pad left and right)
      float tilt_duty = 0.0f;
      if (joy_msg->buttons[15] > 0 && joy_msg->buttons[13] == 0) {
        tilt_duty = 1.0f;
      } else if (joy_msg->buttons[14] > 0 && joy_msg->buttons[16] == 0) {
        tilt_duty = -1.0f;
      }
      tilt.SetDutyCycle(tilt_duty);

      // LIFT ACTUATOR (D pad up and down)
      
      if (joy_msg->buttons[12] > 0 && joy_msg->buttons[14] == 0) {
        lift_setpoint += 1.0f;
      } else if (joy_msg->buttons[13] > 0 && joy_msg->buttons[15] == 0) {
        lift_setpoint += -1.0f;
      }
      else {
        lift_setpoint = lift_position;
      }

      lift_setpoint = std::clamp(lift_setpoint, -7.0f, 5.0f);
      leftLift.SetPosition(lift_setpoint);
      rightLift.SetPosition(lift_setpoint);
    }
    // EXCAVATION RESET BUTTON (X button)
    
    //----------EXCAVATION SYSTEM----------//

    //----------DRIVETRAIN----------//
    // Toggling Alternate Control Mode using the left bumper (button index 4).
    bool current_alternate_button = (joy_msg->buttons[4] > 0);
    if (current_alternate_button && !prev_alternate_button_) {
      alternate_mode_active_ = !alternate_mode_active_;
      RCLCPP_INFO(this->get_logger(), "Alternate control mode %s", alternate_mode_active_ ? "activated" : "deactivated");
    }
    prev_alternate_button_ = current_alternate_button;


    //DRIVETRAIN (Left joystick)//
    float forward = joy_msg->axes[ JOYSTICK_CONTROLLER::HORIZONTAL_LEFT_JOY ];
    float turn = joy_msg->axes[ JOYSTICK_CONTROLLER::VERTICAL_LEFT_JOY ];

    float left_drive_raw = forward + turn;
    float right_drive_raw = forward - turn;

    left_drive_raw = std::max(-1.0f, std::min(1.0f, left_drive_raw));
    right_drive_raw = std::max(-1.0f, std::min(1.0f, right_drive_raw));

    float left_drive = computeStepDuty(left_drive_raw);
    float right_drive = computeStepDuty(right_drive_raw);
    //DRIVETRAIN (Left joystick)//

    //DEPOSIT AUTONOMY(Y button)//
    bool current_deposit_button = (joy_msg->buttons[ BUTTONS_CONTROLLER::Y_BUTTON ] > 0);

    static bool prev_deposit_button = false;
    if (current_deposit_button && !prev_deposit_button) {
       send_deposit_request();
    }
    prev_deposit_button = current_deposit_button;


    //EXCAVATION AUTONOMY(A button)//
    bool current_excavate_button = (joy_msg->buttons[ BUTTONS_CONTROLLER::A_BUTTON ] > 0);

    static bool prev_excavate_button = false;
    if (current_excavate_button && !prev_excavate_button) {
       send_excavation_request();
    }
    prev_excavate_button = current_excavate_button;
    //----------AUTONOMOUS FUNCTIONS----------//
  }


  /**
   * @brief Publish string message heartbeat
   * @param None
   *********************************************************************/
  void publish_heartbeat()
  {
    auto msg = std_msgs::msg::String();
    msg.data = "Heartbeat";
    heartbeatPub->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Heartbeat published");
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::string can_interface = "can0";
  auto temp_node = rclcpp::Node::make_shared("controller_param_node");
  temp_node->declare_parameter<std::string>("can_interface", "can0");
  temp_node->get_parameter("can_interface", can_interface);
  auto node = std::make_shared<ControllerNode>(can_interface);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
