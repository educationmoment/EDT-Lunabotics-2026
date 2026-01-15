/**
 * @file navigation_client.cpp
 * @author Grayson Arendt
 * @date 12/19/2024
 */

#include <array>
#include <chrono>
#include <thread>

#include "geometry_msgs/msg/pose.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "msg_pkg/action/excavation.hpp"

class NavigationClient : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateToPose>;
  using Excavation = msg_pkg::action::Excavation;
  using GoalHandleExcavation = rclcpp_action::ClientGoalHandle<Excavation>;

  NavigationClient()
  : Node("navigation_client"),
    start_navigation_(true),
    start_excavation_(false)
  {
    navigation_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
    excavation_client_ = rclcpp_action::create_client<Excavation>(this, "excavation_action");

    execution_timer_ =
      create_wall_timer(std::chrono::seconds(1), std::bind(&NavigationClient::execute, this));
  }

private:
  struct GoalXY { double x; double y; };
  const std::array<GoalXY, 3> goals_{{{5.0, 1.7}, {5.0, 1.3}, {5.0, 1.2}}};

  void execute()
  {
    if (start_navigation_) {
      request_navigation();
    } else if (start_excavation_) {
      request_excavation();
    }
  }

  void request_navigation()
  {
    if (!navigation_client_->wait_for_action_server(std::chrono::seconds(1))) {return;}

    static bool first_call = true;
    if (first_call) {std::this_thread::sleep_for(std::chrono::seconds(7)); first_call = false;}

    auto goal_msg = NavigateToPose::Goal();
    geometry_msgs::msg::Pose goal_pose;
    goal_pose.position.x = goals_[current_goal_index_].x;
    goal_pose.position.y = goals_[current_goal_index_].y;
    goal_pose.orientation.z = 0.707;
    goal_pose.orientation.w = 0.707;

    goal_msg.pose.pose = goal_pose;
    goal_msg.pose.header.stamp = now();
    goal_msg.pose.header.frame_id = "map";

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback =
      std::bind(&NavigationClient::handle_navigation_result, this, std::placeholders::_1);

    navigation_client_->async_send_goal(goal_msg, send_goal_options);
    start_navigation_ = false;
  }

  void handle_navigation_result(const GoalHandleNavigate::WrappedResult & result)
  {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      start_excavation_ = true;
    } else {
      RCLCPP_ERROR(get_logger(), "\033[1;31mNAVIGATION FAILED\033[0m");
      rclcpp::shutdown();
    }
  }

  void request_excavation()
  {
    if (!excavation_client_->wait_for_action_server(std::chrono::seconds(1))) {return;}

    auto goal_msg = Excavation::Goal();
    auto send_goal_options = rclcpp_action::Client<Excavation>::SendGoalOptions();
    send_goal_options.result_callback =
      std::bind(&NavigationClient::handle_excavation_result, this, std::placeholders::_1);

    excavation_client_->async_send_goal(goal_msg, send_goal_options);
    start_excavation_ = false;
  }

  void handle_excavation_result(const GoalHandleExcavation::WrappedResult & result)
  {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      ++current_goal_index_;
      if (current_goal_index_ < goals_.size()) {
        start_navigation_ = true;
      } else {
        RCLCPP_INFO(get_logger(), "\033[1;32mCOMPLETED THREE CYCLES\033[0m");
        rclcpp::shutdown();
      }
    } else {
      RCLCPP_ERROR(get_logger(), "\033[1;31mEXCAVATION FAILED\033[0m");
      rclcpp::shutdown();
    }
  }

  rclcpp_action::Client<NavigateToPose>::SharedPtr navigation_client_;
  rclcpp_action::Client<Excavation>::SharedPtr excavation_client_;
  rclcpp::TimerBase::SharedPtr execution_timer_;

  bool start_navigation_;
  bool start_excavation_;
  std::size_t current_goal_index_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavigationClient>());
  rclcpp::shutdown();
  return 0;
}
