#include <chrono>
#include <cstdlib>
#include <functional>
#include <future>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <tf2/LinearMath/Quaternion.h> 
#include "nav2_msgs/action/navigate_to_pose.hpp"

using namespace std::chrono_literals;

class PoseClient : public rclcpp::Node
{
public:

  PoseClient()
  : Node("pose_client")
  {
    action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");
  }

  void send_goal(double target_x, double target_y, double target_theta)
  {
    nav2_msgs::action::NavigateToPose::Goal goal_msg;
    tf2::Quaternion q;
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = this->now();

    goal_msg.pose.pose.position.x = target_x;
    goal_msg.pose.pose.position.y = target_y;

    q.setRPY(0.0, 0.0, target_theta);
    goal_msg.pose.pose.orientation.x = q.x();
    goal_msg.pose.pose.orientation.y = q.y();
    goal_msg.pose.pose.orientation.z = q.z();
    goal_msg.pose.pose.orientation.w = q.w();

    
    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();

    send_goal_options.goal_response_callback =
      std::bind(&PoseClient::goal_response_callback, this, std::placeholders::_1);

    send_goal_options.feedback_callback =
      std::bind(&PoseClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);

    send_goal_options.result_callback =
      std::bind(&PoseClient::result_callback, this, std::placeholders::_1);

    action_client_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  void goal_response_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(get_logger(), "Goal was rejected by the server.");
      return; 
    }
    RCLCPP_INFO(get_logger(), "Goal accepted by the server. Waiting for result...");
  }

  void feedback_callback(
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle,
    const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
  {
    (void)goal_handle;
    RCLCPP_INFO(
      get_logger(),
      "Feedback: distanza rimanente: %f m",
      feedback->distance_remaining);
  }

  void result_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result)
  {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
    {
        RCLCPP_INFO(this->get_logger(), "Operation completed successfully! Il robot e' arrivato.");
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Operation failed or canceled.");
    }
  }

  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  double target_x = 17.0;
  double target_y = 6.0;
  double target_theta = 0.0;

  auto node = std::make_shared<PoseClient>();
  node->send_goal(target_x, target_y, target_theta);
  rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
}