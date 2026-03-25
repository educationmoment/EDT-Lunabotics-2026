#include "SparkMax.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "interfaces_pkg/msg/motor_health.hpp"
#include "msg_pkg/action/excavation.hpp"  // Excavation.action → excavation.hpp → msg_pkg::action::Excavation

#include <algorithm>
#include <chrono>
#include <memory>
#include <thread>

const float VIBRATOR_DUTY    = 1.0f;
const float ERROR            = 0.1f;
const float HIGH_PASS_FILTER = 0.38f;
const float KP_LIFT          = 8.0f;
const float KP_TILT          = 8.0f;

// ── MOTOR CONTROLLERS ────────────────────────────────────────────────────────
SparkMax leftDrive("can0", 1);
SparkMax rightDrive("can0", 2);
SparkMax leftLift("can0", 3);
SparkMax rightLift("can0", 4);
SparkMax leftTilt("can0", 5);
SparkMax vibrator("can0", 6);
SparkMax rightTilt("can0", 7);

// ── TYPE ALIASES ─────────────────────────────────────────────────────────────
// ROS 2 codegen rule:  <FileName>.action  →  msg_pkg::action::<FileName>
//   Excavation.action  →  msg_pkg::action::Excavation   (NOT ExcavationAction)
using ExcavationAction     = msg_pkg::action::Excavation;
using GoalHandleExcavation = rclcpp_action::ServerGoalHandle<ExcavationAction>;

// ── SYNC HELPERS ─────────────────────────────────────────────────────────────
void SyncedMoveTilt(float tilt_setpoint)
{
    float left_pos   = leftTilt.GetPosition();
    float right_pos  = rightTilt.GetPosition();
    float tilt_error = right_pos - left_pos;

    if (fabs(tilt_error) < HIGH_PASS_FILTER)
        tilt_error = 0.0f;

    float left_pos_err  = tilt_setpoint - left_pos;
    float right_pos_err = tilt_setpoint - right_pos;

    float left_duty  = (left_pos_err  >  ERROR) ? 1.0f : ((left_pos_err  < -ERROR) ? -1.0f : 0.0f);
    float right_duty = (right_pos_err >  ERROR) ? 1.0f : ((right_pos_err < -ERROR) ? -1.0f : 0.0f);

    float correction = KP_TILT * fabs(tilt_error);
    if (tilt_error > 0)      right_duty -= correction;
    else if (tilt_error < 0) left_duty  -= correction;

    leftTilt.SetDutyCycle(std::clamp(left_duty,  -1.0f, 1.0f));
    rightTilt.SetDutyCycle(std::clamp(right_duty, -1.0f, 1.0f));
}

void SetTiltDutyCycle(float duty)
{
    leftTilt.SetDutyCycle(duty);
    rightTilt.SetDutyCycle(duty);
}

// ── MOVE BUCKET ───────────────────────────────────────────────────────────────
bool MoveBucket(float lift_setpoint, float tilt_setpoint,
                bool activate_vibrator, float drive_speed,
                std::shared_ptr<GoalHandleExcavation> goal_handle)
{
    auto timer_start = std::chrono::high_resolution_clock::now();

    auto leftLiftDone  = [&]{ return fabs(lift_setpoint - leftLift.GetPosition())  <= ERROR; };
    auto rightLiftDone = [&]{ return fabs(lift_setpoint - rightLift.GetPosition()) <= ERROR; };
    auto tiltDone      = [&]{ return fabs(tilt_setpoint - leftTilt.GetPosition())  <= ERROR
                                  && fabs(tilt_setpoint - rightTilt.GetPosition()) <= ERROR; };

    while (!(leftLiftDone() && rightLiftDone() && tiltDone()))
    {
        if (goal_handle->is_canceling())
        {
            leftDrive.SetDutyCycle(0.0f);
            rightDrive.SetDutyCycle(0.0f);
            vibrator.SetDutyCycle(0.0f);
            return false;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(5));

        // ---- LIFT SYNC ---- //
        float lift_error = rightLift.GetPosition() - leftLift.GetPosition();
        if (fabs(lift_error) < HIGH_PASS_FILTER) lift_error = 0.0f;

        if (fabs(leftLift.GetPosition() - rightLift.GetPosition()) >= 0.7)
            RCLCPP_WARN(rclcpp::get_logger("excavation_node"), "WARNING: LIFT ACTUATORS GREATLY MISALIGNED");

        float l_lift_err  = lift_setpoint - leftLift.GetPosition();
        float r_lift_err  = lift_setpoint - rightLift.GetPosition();
        float l_lift_duty = (l_lift_err >  ERROR) ? 1.0f : ((l_lift_err < -ERROR) ? -1.0f : 0.0f);
        float r_lift_duty = (r_lift_err >  ERROR) ? 1.0f : ((r_lift_err < -ERROR) ? -1.0f : 0.0f);
        float lift_corr   = KP_LIFT * fabs(lift_error);
        if (lift_error > 0)      r_lift_duty -= lift_corr;
        else if (lift_error < 0) l_lift_duty -= lift_corr;
        leftLift.SetDutyCycle(std::clamp(l_lift_duty, -1.0f, 1.0f));
        rightLift.SetDutyCycle(std::clamp(r_lift_duty, -1.0f, 1.0f));
        // ---- LIFT SYNC ---- //

        SyncedMoveTilt(tilt_setpoint);

        if (activate_vibrator) vibrator.SetDutyCycle(VIBRATOR_DUTY);
        leftDrive.SetVelocity(drive_speed);
        rightDrive.SetVelocity(drive_speed);

        if (std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::high_resolution_clock::now() - timer_start).count() > 5)
        {
            RCLCPP_ERROR(rclcpp::get_logger("excavation_node"), "Stage timeout — skipping");
            break;
        }
    }
    return true;
}

// ── ACTION SERVER NODE ────────────────────────────────────────────────────────
class ExcavationNode : public rclcpp::Node
{
public:
    explicit ExcavationNode() : Node("excavation_node")
    {
        action_server_ = rclcpp_action::create_server<ExcavationAction>(
            this, "excavation_action",
            std::bind(&ExcavationNode::handle_goal,     this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&ExcavationNode::handle_cancel,   this, std::placeholders::_1),
            std::bind(&ExcavationNode::handle_accepted, this, std::placeholders::_1));

        health_subscriber_ = this->create_subscription<interfaces_pkg::msg::MotorHealth>(
            "/health_topic", 10,
            std::bind(&ExcavationNode::update_tilt_position, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "Excavation Action Server Initialized");
    }

private:
    rclcpp_action::Server<ExcavationAction>::SharedPtr action_server_;
    rclcpp::Subscription<interfaces_pkg::msg::MotorHealth>::SharedPtr health_subscriber_;
    float buffer_ = 0.0f;

    void update_tilt_position(const interfaces_pkg::msg::MotorHealth::SharedPtr msg)
    {
        buffer_ = msg->tilt_position;
    }

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const ExcavationAction::Goal>)
    {
        RCLCPP_INFO(get_logger(), "Excavation goal received");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleExcavation>)
    {
        RCLCPP_WARN(get_logger(), "Excavation cancel requested");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleExcavation> goal_handle)
    {
        std::thread([this, goal_handle]() { execute(goal_handle); }).detach();
    }

    void send_feedback(const std::shared_ptr<GoalHandleExcavation> & gh, const std::string & msg)
    {
        auto fb = std::make_shared<ExcavationAction::Feedback>();
        fb->feedback_message = msg;
        gh->publish_feedback(fb);
        RCLCPP_INFO(get_logger(), "[Feedback] %s", msg.c_str());
    }

    void execute(const std::shared_ptr<GoalHandleExcavation> goal_handle)
    {
        auto result = std::make_shared<ExcavationAction::Result>();

        send_feedback(goal_handle, "Excavation started, buffer=" + std::to_string(buffer_));

        send_feedback(goal_handle, "Stage 1: Approaching position");
        if (!MoveBucket(-2.5f, -2.6f + buffer_, false, 0.0f, goal_handle))
        { result->success = false; goal_handle->canceled(result); return; }
        send_feedback(goal_handle, "Stage 1 complete");

        send_feedback(goal_handle, "Stage 2: Initial dig at 1500 RPM");
        MoveBucket(-3.0f, -2.6f + buffer_, true, 1500.0f, goal_handle);
        auto dig_timer1 = std::chrono::high_resolution_clock::now();
        while (std::chrono::duration_cast<std::chrono::seconds>(
                   std::chrono::high_resolution_clock::now() - dig_timer1).count() < 2)
        {
            if (goal_handle->is_canceling()) { result->success = false; goal_handle->canceled(result); return; }
            leftDrive.SetVelocity(1500.0f); rightDrive.SetVelocity(1500.0f);
            vibrator.SetDutyCycle(VIBRATOR_DUTY);
            MoveBucket(-2.5f, -2.6f + buffer_, true, 1500.0f, goal_handle);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        send_feedback(goal_handle, "Stage 2 complete");

        send_feedback(goal_handle, "Stage 3: Deeper dig at 1000 RPM");
        auto dig_timer2 = std::chrono::high_resolution_clock::now();
        while (std::chrono::duration_cast<std::chrono::seconds>(
                   std::chrono::high_resolution_clock::now() - dig_timer2).count() < 2)
        {
            if (goal_handle->is_canceling()) { result->success = false; goal_handle->canceled(result); return; }
            leftDrive.SetVelocity(1000.0f); rightDrive.SetVelocity(1000.0f);
            vibrator.SetDutyCycle(VIBRATOR_DUTY);
            MoveBucket(-2.7f, -2.1f + buffer_, true, 1000.0f, goal_handle);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        send_feedback(goal_handle, "Stage 3 complete");

        send_feedback(goal_handle, "Stage 4: Scoop at 1000 RPM");
        auto dig_timer3 = std::chrono::high_resolution_clock::now();
        while (std::chrono::duration_cast<std::chrono::seconds>(
                   std::chrono::high_resolution_clock::now() - dig_timer3).count() < 2)
        {
            if (goal_handle->is_canceling()) { result->success = false; goal_handle->canceled(result); return; }
            leftDrive.SetVelocity(1000.0f); rightDrive.SetVelocity(1000.0f);
            vibrator.SetDutyCycle(VIBRATOR_DUTY);
            MoveBucket(-1.91f, -2.5f + buffer_, true, 1000.0f, goal_handle);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }

        send_feedback(goal_handle, "Stage 5: Slow scoop at 500 RPM");
        auto dig_timer4 = std::chrono::high_resolution_clock::now();
        while (std::chrono::duration_cast<std::chrono::seconds>(
                   std::chrono::high_resolution_clock::now() - dig_timer4).count() < 4)
        {
            if (goal_handle->is_canceling()) { result->success = false; goal_handle->canceled(result); return; }
            leftDrive.SetVelocity(500.0f); rightDrive.SetVelocity(500.0f);
            vibrator.SetDutyCycle(VIBRATOR_DUTY);
            MoveBucket(-1.91f, -2.5f + buffer_, true, 500.0f, goal_handle);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        send_feedback(goal_handle, "Stage 5 complete — stopping drivetrain");

        leftDrive.SetDutyCycle(0.0f);
        rightDrive.SetDutyCycle(0.0f);
        vibrator.SetDutyCycle(0.0f);

        send_feedback(goal_handle, "Resetting bucket to home");
        MoveBucket(0.0f, 0.0f + buffer_, false, 0.0f, goal_handle);

        auto reset_tilt = std::chrono::high_resolution_clock::now();
        while (std::chrono::duration_cast<std::chrono::seconds>(
                   std::chrono::high_resolution_clock::now() - reset_tilt).count() < 1)
        {
            SetTiltDutyCycle(1.0f);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }

        send_feedback(goal_handle, "Excavation complete");
        result->success = true;
        goal_handle->succeed(result);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ExcavationNode>());
    rclcpp::shutdown();
    return 0;
}
