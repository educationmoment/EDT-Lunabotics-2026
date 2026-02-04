#include <chrono>
#include <cmath>
#include <memory>
#include <thread>

#include "geometry_msgs/msg/twist.hpp"
#include "msg_pkg/action/localization.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

/**
  * @class LocalizationServer
  * @brief Handles localization by aligning the robot with tag 7 using transform lookups and publishes velocity commands.
  */
class LocalizationServer : public rclcpp::Node
{
    public:
        using Localization = msg_pkg::action::Localization;
        using GoalHandleLocalization = rclcpp_action::ServerGoalHandle<Localization>;

        /*
        * @brief Constructor for LocalizationServer.
        */
        LocalizationServer() : Node("localization_server")
        {
            // Initialize flags
            success_ = false;  // FIXED: Remove 'bool' - this should set the member variable
            
            // Create TF buffer and listener
            tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

            // Publisher for robot velocity commands
            cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

            // Create the action server
            action_server_ = rclcpp_action::create_server<Localization>(
                this, "localization_action",

                // Called when a goal is received
                std::bind(&LocalizationServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),

                // Called when a goal is canceled
                std::bind(&LocalizationServer::handle_cancel, this, std::placeholders::_1),

                // Called when a goal is accepted
                std::bind(&LocalizationServer::handle_accepted, this, std::placeholders::_1)
            );

            // Timer that runs localization logic every 100 ms
            localization_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(&LocalizationServer::localize, this)
            );

            // Record the time when the node starts
            start_time_ = this->now();
        }
        
    private:
        // Member variables - ALL declared in private section
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
        rclcpp_action::Server<Localization>::SharedPtr action_server_;
        rclcpp::TimerBase::SharedPtr localization_timer_;
        rclcpp::Time start_time_;
        
        bool success_;
        double depth_distance_;
        double lateral_distance_;
        
        // Action server callback handlers
        rclcpp_action::GoalResponse handle_goal(
            const rclcpp_action::GoalUUID & uuid,
            std::shared_ptr<const Localization::Goal> goal)
        {
            (void)uuid;
            (void)goal;
            RCLCPP_INFO(get_logger(), "Received localization goal");
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::CancelResponse handle_cancel(
            const std::shared_ptr<GoalHandleLocalization> goal_handle)
        {
            (void)goal_handle;
            RCLCPP_INFO(get_logger(), "Received request to cancel goal");
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        void handle_accepted(const std::shared_ptr<GoalHandleLocalization> goal_handle)
        {
            // Execute the goal in a separate thread
            std::thread{std::bind(&LocalizationServer::execute, this, std::placeholders::_1), goal_handle}.detach();
        }
        
        // This function runs when localization starts
        void execute(const std::shared_ptr<GoalHandleLocalization> goal_handle)
        {
            // This will contain the robots position which will be communicated via goal_handle
            auto result = std::make_shared<Localization::Result>();

            // wait until localization succeeds
            while (!success_ && rclcpp::ok()) 
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            // set the robots position in result which will be communicated back via goal_handle
            if (success_)
            {
                result->x = depth_distance_ + 0.1; // Center of robot is offset slightly
                result->y = lateral_distance_ + 1.0; // Tag is located 1m away from corner
                result->success = true;
                goal_handle->succeed(result);
                rclcpp::shutdown();
            }
            else
            {
                rclcpp::shutdown();
            }
        }

        void localize()
        {
            geometry_msgs::msg::Twist twist;

            try
            {
                // Check if D455 sees tag 7 immediately
                try
                {
                    // FIXED: Use -> operator for shared_ptr
                    auto tag7_to_d455 = tf_buffer_->lookupTransform("d455_link", "tag36h11:7", tf2::TimePointZero);
                    auto d455_to_base = tf_buffer_->lookupTransform("base_link", "d455_link", tf2::TimePointZero);
                
                    tf2::Transform tag7_to_d455_tf;
                    tf2::fromMsg(tag7_to_d455.transform, tag7_to_d455_tf);

                    tf2::Transform d455_to_base_tf;
                    tf2::fromMsg(d455_to_base.transform, d455_to_base_tf);

                    // Combine the camera-to-robot-base transform with the tag-to-camera transform to get tag relative to robot base
                    tf2::Transform tag7_to_base_tf = d455_to_base_tf * tag7_to_d455_tf;

                    depth_distance_ = -tag7_to_base_tf.getOrigin().x();
                    lateral_distance_ = -tag7_to_base_tf.getOrigin().y();
                    double yaw = tf2::getYaw(tag7_to_base_tf.getRotation());

                    // Check if the yaw is within the desired range (e.g., facing east)
                    if (std::abs(yaw - 1.57) < 0.05)  // 1.57 radians = 90 degrees (east)
                    {
                        RCLCPP_INFO(get_logger(), "TAG 7 VISIBLE BY D455 AND YAW IS CORRECT: Localization successful.");
                        success_ = true;
                        return; // Exit early since localization is successful
                    } 
                    else 
                    {
                        RCLCPP_WARN(get_logger(), "TAG 7 VISIBLE BY D455 BUT YAW IS INCORRECT: Adjusting orientation.");
                    }
                } 
                catch (tf2::TransformException & ex) 
                {
                    RCLCPP_WARN_THROTTLE(
                    get_logger(),
                    *get_clock(), 2000, "TAG 7 NOT VISIBLE BY D455: %s", ex.what());
                }
            } 
            catch (tf2::TransformException & ex) 
            {
                RCLCPP_WARN(get_logger(), "TRANSFORM UNAVAILABLE: %s", ex.what());
            }
        }
};

/**
 * @brief Main function.
 * Initializes and runs the LocalizationServer node.
 */
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalizationServer>());
  rclcpp::shutdown();
  return 0;
}
