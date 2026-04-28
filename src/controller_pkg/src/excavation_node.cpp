#include "SparkMax.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "interfaces_pkg/msg/motor_health.hpp"
#include "msg_pkg/action/excavation.hpp"

#include <algorithm>
#include <chrono>
#include <memory>
#include <thread>
#include <mutex>

// ═════════════════════════════════════════════════════════════════════════════
// TUNE THESE TONIGHT — all position setpoints in rotations
// ═════════════════════════════════════════════════════════════════════════════
static constexpr float LIFT_APPROACH    = -0.70f;
static constexpr float LIFT_DIG         = -0.77f;
static constexpr float TILT_APPROACH    = -1.08f;
static constexpr float TILT_DIG         = -1.04f;
static constexpr float LIFT_HOME        =  0.0f;
static constexpr float TILT_HOME        =  0.0f;
// ═════════════════════════════════════════════════════════════════════════════
// ═════════════════════════════════════════════════════════════════════════════

// ── TUNING CONSTANTS ──────────────────────────────────────────────────────────
const float VIBRATOR_DUTY  = 1.0f;
const float POS_TOLERANCE  = 0.12f;   // rotations — stop within this of target
const float SYNC_DEADBAND  = 0.08f;   // rotations — ignore sync error below this
const float KP_LIFT        = 1.3f;    // matches controller_node
const float KP_TILT        = 0.5f;    // matches controller_node

// ── MOTOR CONTROLLERS ─────────────────────────────────────────────────────────
SparkMax leftDrive ("can0", 1);
SparkMax rightDrive("can0", 2);
SparkMax leftLift  ("can0", 3);
SparkMax rightLift ("can0", 4);
SparkMax leftTilt  ("can0", 5);
SparkMax vibrator  ("can0", 6);
SparkMax rightTilt ("can0", 7);

using ExcavationAction     = msg_pkg::action::Excavation;
using GoalHandleExcavation = rclcpp_action::ServerGoalHandle<ExcavationAction>;

// ── SYNCED LIFT TO POSITION ───────────────────────────────────────────────────
bool SyncedLiftToPos(float target, float duty,
                     std::shared_ptr<GoalHandleExcavation> gh,
                     float timeout_s = 5.0f)
{
    RCLCPP_INFO(rclcpp::get_logger("excavation_node"),
        "LIFT start — target=%.3f  L=%.3f  R=%.3f  duty=%.2f",
        target, leftLift.GetPosition(), rightLift.GetPosition(), duty);

    auto t0       = std::chrono::high_resolution_clock::now();
    int  log_tick = 0;

    while (true)
    {
        if (gh->is_canceling())
        {
            leftLift.SetDutyCycle(0.0f);
            rightLift.SetDutyCycle(0.0f);
            RCLCPP_WARN(rclcpp::get_logger("excavation_node"), "LIFT cancelled");
            return false;
        }

        float elapsed = std::chrono::duration_cast<std::chrono::duration<float>>(
            std::chrono::high_resolution_clock::now() - t0).count();
        if (elapsed > timeout_s)
        {
            RCLCPP_ERROR(rclcpp::get_logger("excavation_node"),
                "LIFT timeout — target=%.3f  L=%.3f  R=%.3f",
                target, leftLift.GetPosition(), rightLift.GetPosition());
            break;
        }

        float l_pos  = leftLift.GetPosition();
        float r_pos  = rightLift.GetPosition();
        bool  l_done = fabs(l_pos - target) <= POS_TOLERANCE;
        bool  r_done = fabs(r_pos - target) <= POS_TOLERANCE;

        if (++log_tick % 40 == 0)
            RCLCPP_INFO(rclcpp::get_logger("excavation_node"),
                "LIFT moving — target=%.3f  L=%.3f(done=%d)  R=%.3f(done=%d)",
                target, l_pos, l_done, r_pos, r_done);

        if (l_done && r_done)
        {
            RCLCPP_INFO(rclcpp::get_logger("excavation_node"),
                "LIFT reached — target=%.3f  L=%.3f  R=%.3f", target, l_pos, r_pos);
            break;
        }

        float dir    = (target > l_pos) ? 1.0f : -1.0f;
        float l_duty = l_done ? 0.0f
            : std::clamp(fabsf(l_pos - target) * 3.0f, 0.12f, duty);
        float r_duty = r_done ? 0.0f
            : std::clamp(fabsf(r_pos - target) * 3.0f, 0.12f, duty);

        float sync_error = r_pos - l_pos;
        if (fabs(sync_error) > SYNC_DEADBAND)
        {
            float correction = KP_LIFT * fabs(sync_error);
            float factor     = std::max(0.0f, 1.0f - correction);
            if (sync_error > 0) r_duty = duty * factor;
            else                l_duty = duty * factor;

            RCLCPP_INFO(rclcpp::get_logger("excavation_node"),
                "LIFT sync — err=%.3f  factor=%.3f  L_duty=%.3f  R_duty=%.3f",
                sync_error, factor, l_duty * dir, r_duty * dir);
        }

        leftLift.SetDutyCycle(l_duty * dir);
        rightLift.SetDutyCycle(r_duty * dir);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    leftLift.SetDutyCycle(0.0f);
    rightLift.SetDutyCycle(0.0f);
    return true;
}

// ── SYNCED TILT TO POSITION ───────────────────────────────────────────────────
bool SyncedTiltToPos(float target, float duty,
                     std::shared_ptr<GoalHandleExcavation> gh,
                     float timeout_s = 5.0f)
{
    RCLCPP_INFO(rclcpp::get_logger("excavation_node"),
        "TILT start — target=%.3f  L=%.3f  R=%.3f  duty=%.2f",
        target, leftTilt.GetPosition(), rightTilt.GetPosition(), duty);

    auto t0       = std::chrono::high_resolution_clock::now();
    int  log_tick = 0;

    while (true)
    {
        if (gh->is_canceling())
        {
            leftTilt.SetDutyCycle(0.0f);
            rightTilt.SetDutyCycle(0.0f);
            RCLCPP_WARN(rclcpp::get_logger("excavation_node"), "TILT cancelled");
            return false;
        }

        float elapsed = std::chrono::duration_cast<std::chrono::duration<float>>(
            std::chrono::high_resolution_clock::now() - t0).count();
        if (elapsed > timeout_s)
        {
            RCLCPP_ERROR(rclcpp::get_logger("excavation_node"),
                "TILT timeout — target=%.3f  L=%.3f  R=%.3f",
                target, leftTilt.GetPosition(), rightTilt.GetPosition());
            break;
        }

        float l_pos  = leftTilt.GetPosition();
        float r_pos  = rightTilt.GetPosition();
        bool  l_done = fabs(l_pos - target) <= POS_TOLERANCE;
        bool  r_done = fabs(r_pos - target) <= POS_TOLERANCE;

        if (++log_tick % 40 == 0)
            RCLCPP_INFO(rclcpp::get_logger("excavation_node"),
                "TILT moving — target=%.3f  L=%.3f(done=%d)  R=%.3f(done=%d)",
                target, l_pos, l_done, r_pos, r_done);

        if (l_done && r_done)
        {
            RCLCPP_INFO(rclcpp::get_logger("excavation_node"),
                "TILT reached — target=%.3f  L=%.3f  R=%.3f", target, l_pos, r_pos);
            break;
        }

        float dir    = (target > l_pos) ? 1.0f : -1.0f;
        float l_duty = l_done ? 0.0f
            : std::clamp(fabsf(l_pos - target) * 3.1f, 0.13f, duty);
        float r_duty = r_done ? 0.0f
            : std::clamp(fabsf(r_pos - target) * 3.1f, 0.13f, duty);

        float sync_error = r_pos - l_pos;
        if (fabs(sync_error) > SYNC_DEADBAND)
        {
            float correction = KP_TILT * fabs(sync_error);
            float factor     = std::max(0.0f, 1.0f - correction);
            if (sync_error > 0) r_duty = duty * factor;
            else                l_duty = duty * factor;

            RCLCPP_INFO(rclcpp::get_logger("excavation_node"),
                "TILT sync — err=%.3f  factor=%.3f  L_duty=%.3f  R_duty=%.3f",
                sync_error, factor, l_duty * dir, r_duty * dir);
        }

        leftTilt.SetDutyCycle(l_duty * dir);
        rightTilt.SetDutyCycle(r_duty * dir);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    leftTilt.SetDutyCycle(0.0f);
    rightTilt.SetDutyCycle(0.0f);
    return true;
}

// ── CANCEL CHECK MACRO ────────────────────────────────────────────────────────
#define CHECK(call) \
    if (!(call)) { \
        leftDrive.SetDutyCycle(0.0f); \
        rightDrive.SetDutyCycle(0.0f); \
        vibrator.SetDutyCycle(0.0f); \
        result->success = false; \
        goal_handle->canceled(result); \
        return; \
    }

// ── ACTION SERVER NODE ────────────────────────────────────────────────────────
class ExcavationNode : public rclcpp::Node
{
public:
    explicit ExcavationNode() : Node("excavation_node")
    {
        action_server_ = rclcpp_action::create_server<ExcavationAction>(
            this, "excavation_action",
            std::bind(&ExcavationNode::handle_goal,     this,
                      std::placeholders::_1, std::placeholders::_2),
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

    std::mutex buffer_mutex_;
    float      buffer_ = 0.0f;

    void update_tilt_position(const interfaces_pkg::msg::MotorHealth::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
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

    void send_feedback(const std::shared_ptr<GoalHandleExcavation> & gh,
                       const std::string & msg)
    {
        auto fb = std::make_shared<ExcavationAction::Feedback>();
        fb->feedback_message = msg;
        gh->publish_feedback(fb);
        RCLCPP_INFO(get_logger(), "[Feedback] %s", msg.c_str());
    }

    void execute(const std::shared_ptr<GoalHandleExcavation> goal_handle)
    {
        auto result = std::make_shared<ExcavationAction::Result>();

        float buffer;
        {
            std::lock_guard<std::mutex> lock(buffer_mutex_);
            buffer = buffer_;
        }

        RCLCPP_INFO(get_logger(),
            "PRE-EXEC — LL=%.3f  RL=%.3f  LT=%.3f  RT=%.3f  buffer=%.3f",
            leftLift.GetPosition(), rightLift.GetPosition(),
            leftTilt.GetPosition(), rightTilt.GetPosition(), buffer);

        // ── STAGE 1A: Lift descends to approach height ────────────────────
        // Lift moves FIRST. Tilt does not move until lift confirms arrival.
        // Change LIFT_APPROACH at the top of the file.
// ── STAGE 1: Lift and Tilt move to approach position simultaneously ───────
        send_feedback(goal_handle, "Stage 1: Lift to approach height + Tilt to approach angle (parallel)");
        {
            bool lift_ok = true, tilt_ok = true;

            std::thread lift_thread([&]() {
                lift_ok = SyncedLiftToPos(LIFT_APPROACH, 0.9f, goal_handle);
            });
            std::thread tilt_thread([&]() {
                tilt_ok = SyncedTiltToPos(TILT_APPROACH + buffer, 0.8f, goal_handle);
            });

            lift_thread.join();
            tilt_thread.join();

            if (!lift_ok || !tilt_ok)
            {
                leftDrive.SetDutyCycle(0.0f);
                rightDrive.SetDutyCycle(0.0f);
                vibrator.SetDutyCycle(0.0f);
                result->success = false;
                goal_handle->canceled(result);
                return;
            }
        }
        send_feedback(goal_handle, "Stage 1 complete — at approach position");

        // ── STAGE 2: Drive forward, deepen tilt to full dig angle ─────────
        // Robot motion carries bucket into material.
        // Tilt deepens to TILT_DIG during forward travel — gradual entry.
        // Change TILT_DIG at the top of the file.
        send_feedback(goal_handle, "Stage 2: Entry drive + tilt to dig angle");
        auto t2 = std::chrono::high_resolution_clock::now();
        while (std::chrono::duration_cast<std::chrono::milliseconds>(
                   std::chrono::high_resolution_clock::now() - t2).count() < 7000)
        {
            if (goal_handle->is_canceling())
            {
                leftDrive.SetDutyCycle(0.0f);
                rightDrive.SetDutyCycle(0.0f);
                vibrator.SetDutyCycle(0.0f);
                result->success = false;
                goal_handle->canceled(result);
                return;
            }

            leftDrive.SetVelocity(1400.0f);
            rightDrive.SetVelocity(1400.0f);
            vibrator.SetDutyCycle(VIBRATOR_DUTY);

            // Inline tilt sync toward full dig angle
            float l_pos      = leftTilt.GetPosition();
            float r_pos      = rightTilt.GetPosition();
            float dig_target = TILT_DIG + buffer;
            float dir        = (dig_target > l_pos) ? 1.0f : -1.0f;
            float l_duty     = (fabs(l_pos - dig_target) > POS_TOLERANCE) ? 0.8f : 0.0f;
            float r_duty     = (fabs(r_pos - dig_target) > POS_TOLERANCE) ? 0.8f : 0.0f;
            float sync_err   = r_pos - l_pos;
            if (fabs(sync_err) > SYNC_DEADBAND)
            {
                float factor = std::max(0.0f, 1.0f - KP_TILT * fabsf(sync_err));
                if (sync_err > 0) r_duty = 0.8f * factor;
                else              l_duty = 0.8f * factor;
            }
            leftTilt.SetDutyCycle(l_duty * dir);
            rightTilt.SetDutyCycle(r_duty * dir);

            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        send_feedback(goal_handle, "Stage 2 complete");

		// ── STAGE 3: Slow scoop — lift drops to dig depth ────────────────────
		// Speed reduced so material loads into bucket.
		// Lift drops from LIFT_APPROACH to LIFT_DIG inline while driving.
		// Change LIFT_DIG at the top of the file.
		send_feedback(goal_handle, "Stage 3: Slow scoop at 800 RPM + lift to dig depth");
		auto t3 = std::chrono::high_resolution_clock::now();
		while (std::chrono::duration_cast<std::chrono::milliseconds>(
				   std::chrono::high_resolution_clock::now() - t3).count() < 8000)
		{
			if (goal_handle->is_canceling())
			{
				leftDrive.SetDutyCycle(0.0f);
				rightDrive.SetDutyCycle(0.0f);
				vibrator.SetDutyCycle(0.0f);
				result->success = false;
				goal_handle->canceled(result);
				return;
			}

			leftDrive.SetVelocity(1000.0f);
			rightDrive.SetVelocity(1000.0f);
			vibrator.SetDutyCycle(VIBRATOR_DUTY);

			// Inline lift sync toward dig depth — same factor pattern
			float l_pos  = leftLift.GetPosition();
			float r_pos  = rightLift.GetPosition();
			float dir    = (LIFT_DIG > l_pos) ? 1.0f : -1.0f;
			float l_duty = (fabs(l_pos - LIFT_DIG) > POS_TOLERANCE) ? 0.8f : 0.0f;
			float r_duty = (fabs(r_pos - LIFT_DIG) > POS_TOLERANCE) ? 0.8f : 0.0f;
			float sync_err = r_pos - l_pos;
			if (fabs(sync_err) > SYNC_DEADBAND)
			{
				float factor = std::max(0.0f, 1.0f - KP_LIFT * fabsf(sync_err));
				if (sync_err > 0) r_duty = 0.8f * factor;
				else              l_duty = 0.8f * factor;
			}
			leftLift.SetDutyCycle(l_duty * dir);
			rightLift.SetDutyCycle(r_duty * dir);

			std::this_thread::sleep_for(std::chrono::milliseconds(5));
		}
		leftLift.SetDutyCycle(0.0f);
		rightLift.SetDutyCycle(0.0f);
		send_feedback(goal_handle, "Stage 3 complete");

        // ── STAGE 4: Stop drive and vibrator ─────────────────────────────
        leftDrive.SetDutyCycle(0.0f);
        rightDrive.SetDutyCycle(0.0f);
        vibrator.SetDutyCycle(0.0f);

        // ── STAGE 5: Tilt returns home FIRST ─────────────────────────────
        // Tilt must return to TILT_HOME before lift rises.
        // Lift rising with tilt still at dig angle risks chassis contact.
        // Change TILT_HOME at the top of the file.
        //send_feedback(goal_handle, "Stage 5: Tilt returning to home (0.0)");
        //CHECK(SyncedTiltToPos(TILT_HOME + buffer, 0.5f, goal_handle))
        //send_feedback(goal_handle, "Stage 5 complete");

        // ── STAGE 6: Lift returns home ────────────────────────────────────
        // Tilt confirmed at home. Lift rises back to zero.
        // Change LIFT_HOME at the top of the file.
        send_feedback(goal_handle, "Stage 6: Lift returning to home (0.0)");
        CHECK(SyncedTiltToPos(TILT_HOME + buffer, 0.8f, goal_handle))

        CHECK(SyncedLiftToPos(LIFT_HOME, 0.9f, goal_handle))

        RCLCPP_INFO(get_logger(),
            "POST-EXEC — LL=%.3f  RL=%.3f  LT=%.3f  RT=%.3f",
            leftLift.GetPosition(), rightLift.GetPosition(),
            leftTilt.GetPosition(), rightTilt.GetPosition());

        send_feedback(goal_handle, "Excavation complete — reset to home");
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