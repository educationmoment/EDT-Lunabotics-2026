#include "SparkMax.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "msg_pkg/action/depositing.hpp"

#include <algorithm>
#include <chrono>
#include <memory>
#include <thread>

// ═════════════════════════════════════════════════════════════════════════════
// TUNE THESE — all position setpoints in rotations
// ═════════════════════════════════════════════════════════════════════════════
static constexpr float TILT_PARTIAL =  0.18f;  // Clearance tilt before lift rises
static constexpr float LIFT_DUMP    =  1.5f;
static constexpr float TILT_DUMP    =  0.23f;  // Final dump angle
static constexpr float LIFT_HOME  =  0.0f;
static constexpr float TILT_HOME  =  0.0f;
// ═════════════════════════════════════════════════════════════════════════════

// ── TUNING CONSTANTS ──────────────────────────────────────────────────────────
const float VIBRATOR_DUTY  = 1.0f;
const float POS_TOLERANCE  = 0.12f;   // rotations — stop within this of target
const float SYNC_DEADBAND  = 0.08f;  // rotations — ignore sync error below this
const float KP_LIFT        = 1.3f;
const float KP_TILT        = 0.5f;

// ── MOTOR CONTROLLERS ─────────────────────────────────────────────────────────
SparkMax leftLift ("can0", 3);
SparkMax rightLift("can0", 4);
SparkMax leftTilt ("can0", 5);
SparkMax vibrator ("can0", 6);
SparkMax rightTilt("can0", 7);

// ── TYPE ALIASES ──────────────────────────────────────────────────────────────
using DepositingAction     = msg_pkg::action::Depositing;
using GoalHandleDepositing = rclcpp_action::ServerGoalHandle<DepositingAction>;

// ── SYNCED LIFT TO POSITION ───────────────────────────────────────────────────
bool SyncedLiftToPos(float target, float duty,
                     std::shared_ptr<GoalHandleDepositing> gh,
                     float timeout_s = 10.0f)
{
    RCLCPP_INFO(rclcpp::get_logger("depositing_node"),
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
            RCLCPP_WARN(rclcpp::get_logger("depositing_node"), "LIFT cancelled");
            return false;
        }

        float elapsed = std::chrono::duration_cast<std::chrono::duration<float>>(
            std::chrono::high_resolution_clock::now() - t0).count();
        if (elapsed > timeout_s)
        {
            RCLCPP_ERROR(rclcpp::get_logger("depositing_node"),
                "LIFT timeout — target=%.3f  L=%.3f  R=%.3f",
                target, leftLift.GetPosition(), rightLift.GetPosition());
            break;
        }

        float l_pos  = leftLift.GetPosition();
        float r_pos  = rightLift.GetPosition();
        bool  l_done = fabs(l_pos - target) <= POS_TOLERANCE;
        bool  r_done = fabs(r_pos - target) <= POS_TOLERANCE;

        if (++log_tick % 40 == 0)
            RCLCPP_INFO(rclcpp::get_logger("depositing_node"),
                "LIFT moving — target=%.3f  L=%.3f(done=%d)  R=%.3f(done=%d)",
                target, l_pos, l_done, r_pos, r_done);

        if (l_done && r_done)
        {
            RCLCPP_INFO(rclcpp::get_logger("depositing_node"),
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

            RCLCPP_INFO(rclcpp::get_logger("depositing_node"),
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
                     std::shared_ptr<GoalHandleDepositing> gh,
                     float timeout_s = 10.0f)
{
    RCLCPP_INFO(rclcpp::get_logger("depositing_node"),
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
            RCLCPP_WARN(rclcpp::get_logger("depositing_node"), "TILT cancelled");
            return false;
        }

        float elapsed = std::chrono::duration_cast<std::chrono::duration<float>>(
            std::chrono::high_resolution_clock::now() - t0).count();
        if (elapsed > timeout_s)
        {
            RCLCPP_ERROR(rclcpp::get_logger("depositing_node"),
                "TILT timeout — target=%.3f  L=%.3f  R=%.3f",
                target, leftTilt.GetPosition(), rightTilt.GetPosition());
            break;
        }

        float l_pos  = leftTilt.GetPosition();
        float r_pos  = rightTilt.GetPosition();
        bool  l_done = fabs(l_pos - target) <= POS_TOLERANCE;
        bool  r_done = fabs(r_pos - target) <= POS_TOLERANCE;

        if (++log_tick % 40 == 0)
            RCLCPP_INFO(rclcpp::get_logger("depositing_node"),
                "TILT moving — target=%.3f  L=%.3f(done=%d)  R=%.3f(done=%d)",
                target, l_pos, l_done, r_pos, r_done);

        if (l_done && r_done)
        {
            RCLCPP_INFO(rclcpp::get_logger("depositing_node"),
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

            RCLCPP_INFO(rclcpp::get_logger("depositing_node"),
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
        vibrator.SetDutyCycle(0.0f); \
        result->success = false; \
        goal_handle->canceled(result); \
        return; \
    }

// ── ACTION SERVER NODE ────────────────────────────────────────────────────────
class DepositingNode : public rclcpp::Node
{
public:
    explicit DepositingNode() : Node("depositing_node")
    {
        action_server_ = rclcpp_action::create_server<DepositingAction>(
            this, "depositing_action",
            std::bind(&DepositingNode::handle_goal,     this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&DepositingNode::handle_cancel,   this, std::placeholders::_1),
            std::bind(&DepositingNode::handle_accepted, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "Depositing Action Server Initialized");
    }

private:
    rclcpp_action::Server<DepositingAction>::SharedPtr action_server_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const DepositingAction::Goal>)
    {
        RCLCPP_INFO(get_logger(), "Depositing goal received");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleDepositing>)
    {
        RCLCPP_WARN(get_logger(), "Depositing cancel requested");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleDepositing> goal_handle)
    {
        std::thread([this, goal_handle]() { execute(goal_handle); }).detach();
    }

    void send_feedback(const std::shared_ptr<GoalHandleDepositing> & gh, const std::string & msg)
    {
        auto fb = std::make_shared<DepositingAction::Feedback>();
        fb->feedback_message = msg;
        gh->publish_feedback(fb);
        RCLCPP_INFO(get_logger(), "[Feedback] %s", msg.c_str());
    }

    void execute(const std::shared_ptr<GoalHandleDepositing> goal_handle)
    {
        auto result = std::make_shared<DepositingAction::Result>();

        RCLCPP_INFO(get_logger(),
            "PRE-EXEC — LL=%.3f  RL=%.3f  LT=%.3f  RT=%.3f",
            leftLift.GetPosition(), rightLift.GetPosition(),
            leftTilt.GetPosition(), rightTilt.GetPosition());

        // ── STAGE 1: Lift and Tilt move to dump positions simultaneously ──────
        send_feedback(goal_handle, "Stage 1: Lift to dump height + Tilt to dump angle (parallel)");
        {
            bool lift_ok = true, tilt_ok = true;

            std::thread lift_thread([&]() {
                lift_ok = SyncedLiftToPos(LIFT_DUMP, 0.8f, goal_handle, 15.0f);
            });
            std::thread tilt_thread([&]() {
                tilt_ok = SyncedTiltToPos(TILT_DUMP, 0.8f, goal_handle);
            });

            lift_thread.join();
            tilt_thread.join();

            if (!lift_ok || !tilt_ok)
            {
                vibrator.SetDutyCycle(0.0f);
                result->success = false;
                goal_handle->canceled(result);
                return;
            }
        }
        send_feedback(goal_handle, "Stage 1 complete");

        // ── STAGE 2: Vibrate material out ─────────────────────────────────────
        send_feedback(goal_handle, "Stage 2: Vibrating (10s)");
        {
            auto jiggle_start = std::chrono::high_resolution_clock::now();
            while (std::chrono::duration_cast<std::chrono::seconds>(
                    std::chrono::high_resolution_clock::now() - jiggle_start).count() < 10)
            {
                if (goal_handle->is_canceling())
                {
                    vibrator.SetDutyCycle(0.0f);
                    result->success = false;
                    goal_handle->canceled(result);
                    return;
                }
                vibrator.SetDutyCycle(VIBRATOR_DUTY);
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
            vibrator.SetDutyCycle(0.0f);
        }
        send_feedback(goal_handle, "Stage 2 complete");

        // ── STAGE 3: Lift and Tilt return home simultaneously ─────────────────
        send_feedback(goal_handle, "Stage 3: Lift and Tilt returning home (parallel)");
        {
            bool lift_ok = true, tilt_ok = true;

            std::thread lift_thread([&]() {
                lift_ok = SyncedLiftToPos(LIFT_HOME, 0.8f, goal_handle);
            });
            std::thread tilt_thread([&]() {
                tilt_ok = SyncedTiltToPos(TILT_HOME, 0.8f, goal_handle);
            });

            lift_thread.join();
            tilt_thread.join();

            if (!lift_ok || !tilt_ok)
            {
                result->success = false;
                goal_handle->canceled(result);
                return;
            }
        }
        send_feedback(goal_handle, "Stage 3 complete");

        RCLCPP_INFO(get_logger(),
            "POST-EXEC — LL=%.3f  RL=%.3f  LT=%.3f  RT=%.3f",
            leftLift.GetPosition(), rightLift.GetPosition(),
            leftTilt.GetPosition(), rightTilt.GetPosition());

        send_feedback(goal_handle, "Depositing complete — reset to home");
        result->success = true;
        goal_handle->succeed(result);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DepositingNode>());
    rclcpp::shutdown();
    return 0;
}