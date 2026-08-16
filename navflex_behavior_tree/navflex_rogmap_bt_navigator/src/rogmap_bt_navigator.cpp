// Copyright 2026 Navflex contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_behavior_tree/bt_action_server.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/time.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace navflex_rogmap_bt_navigator
{

class RogMapBtNavigator : public nav2_util::LifecycleNode
{
public:
  using Action = nav2_msgs::action::NavigateToPose;
  using BtActionServer = nav2_behavior_tree::BtActionServer<Action>;

  explicit RogMapBtNavigator(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : nav2_util::LifecycleNode("rogmap_bt_navigator", "", [](rclcpp::NodeOptions node_options) {
      return node_options.enable_rosout(false);
    }(options))
  {
    const auto share_dir = ament_index_cpp::get_package_share_directory(
      "navflex_rogmap_bt_navigator");
    declare_parameter("global_frame", rclcpp::ParameterValue("map"));
    declare_parameter("robot_base_frame", rclcpp::ParameterValue("base_link"));
    declare_parameter("transform_tolerance", rclcpp::ParameterValue(0.3));
    declare_parameter("action_name", rclcpp::ParameterValue("navigate_to_pose"));
    declare_parameter("goal_topic", rclcpp::ParameterValue("goal_pose"));
    declare_parameter("rviz_point_topic", rclcpp::ParameterValue("clicked_point"));
    declare_parameter("accept_goal_topic", rclcpp::ParameterValue(true));
    declare_parameter("accept_rviz_point", rclcpp::ParameterValue(true));
    declare_parameter("clicked_point_goal_yaw", rclcpp::ParameterValue(0.0));
    declare_parameter(
      "default_nav_to_pose_bt_xml",
      rclcpp::ParameterValue(
        share_dir + "/behavior_trees/navigate_to_pose_rogmap.xml"));
    declare_parameter(
      "plugin_lib_names",
      rclcpp::ParameterValue(
        std::vector<std::string>{
        "nav2_rate_controller_bt_node",
        "nav2_pipeline_sequence_bt_node",
        "navflex_get_path_action",
        "navflex_exe_path_action"}));
  }

  ~RogMapBtNavigator() override = default;

protected:
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    global_frame_ = get_parameter("global_frame").as_string();
    robot_base_frame_ = get_parameter("robot_base_frame").as_string();
    transform_tolerance_ = get_parameter("transform_tolerance").as_double();
    action_name_ = get_parameter("action_name").as_string();
    const auto goal_topic = get_parameter("goal_topic").as_string();
    const auto rviz_point_topic = get_parameter("rviz_point_topic").as_string();
    accept_goal_topic_ = get_parameter("accept_goal_topic").as_bool();
    accept_rviz_point_ = get_parameter("accept_rviz_point").as_bool();
    clicked_point_goal_yaw_ = get_parameter("clicked_point_goal_yaw").as_double();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    self_client_ = rclcpp_action::create_client<Action>(shared_from_this(), action_name_);

    if (accept_goal_topic_) {
      goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        goal_topic, rclcpp::SystemDefaultsQoS(),
        std::bind(&RogMapBtNavigator::onGoalPose, this, std::placeholders::_1));
    }
    if (accept_rviz_point_) {
      point_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
        rviz_point_topic, rclcpp::SystemDefaultsQoS(),
        std::bind(&RogMapBtNavigator::onClickedPoint, this, std::placeholders::_1));
    }

    const auto default_tree = get_parameter("default_nav_to_pose_bt_xml").as_string();
    const auto plugin_libraries = get_parameter("plugin_lib_names").as_string_array();
    auto parent = std::dynamic_pointer_cast<nav2_util::LifecycleNode>(shared_from_this());
    bt_action_server_ = std::make_unique<BtActionServer>(
      parent, action_name_, plugin_libraries, default_tree,
      std::bind(&RogMapBtNavigator::onGoalReceived, this, std::placeholders::_1),
      std::bind(&RogMapBtNavigator::onLoop, this),
      std::bind(&RogMapBtNavigator::onPreempt, this, std::placeholders::_1),
      std::bind(
        &RogMapBtNavigator::onCompletion, this, std::placeholders::_1,
        std::placeholders::_2));
    if (!bt_action_server_->on_configure()) {
      RCLCPP_ERROR(get_logger(), "Failed to configure ROG-Map BT action server");
      return nav2_util::CallbackReturn::FAILURE;
    }

    auto blackboard = bt_action_server_->getBlackboard();
    blackboard->set<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer", tf_buffer_);
    blackboard->set<int>("number_recoveries", 0);
    active_ = false;
    RCLCPP_INFO(
      get_logger(),
      "Configured independent 3D navigator: action=%s goal_topic=%s point_topic=%s",
      action_name_.c_str(), goal_topic.c_str(), rviz_point_topic.c_str());
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    if (!bt_action_server_ || !bt_action_server_->on_activate()) {
      return nav2_util::CallbackReturn::FAILURE;
    }
    active_ = true;
    createBond();
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    active_ = false;
    if (bt_action_server_) {
      bt_action_server_->on_deactivate();
    }
    destroyBond();
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    active_ = false;
    if (bt_action_server_) {
      bt_action_server_->on_cleanup();
    }
    bt_action_server_.reset();
    point_sub_.reset();
    goal_sub_.reset();
    self_client_.reset();
    tf_listener_.reset();
    tf_buffer_.reset();
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override
  {
    active_ = false;
    return nav2_util::CallbackReturn::SUCCESS;
  }

private:
  bool transformGoal(
    const geometry_msgs::msg::PoseStamped & input,
    geometry_msgs::msg::PoseStamped & output)
  {
    auto goal = input;
    if (goal.header.frame_id.empty()) {
      goal.header.frame_id = global_frame_;
    }
    if (goal.header.stamp.sec == 0 && goal.header.stamp.nanosec == 0) {
      goal.header.stamp = now();
    }
    try {
      output = goal.header.frame_id == global_frame_ ?
        goal : tf_buffer_->transform(
        goal, global_frame_, tf2::durationFromSec(transform_tolerance_));
    } catch (const tf2::TransformException & exception) {
      RCLCPP_WARN(
        get_logger(), "Cannot transform 3D goal from '%s' to '%s': %s",
        goal.header.frame_id.c_str(), global_frame_.c_str(), exception.what());
      return false;
    }
    const auto & point = output.pose.position;
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      RCLCPP_WARN(get_logger(), "Rejected non-finite 3D navigation goal");
      return false;
    }
    return true;
  }

  bool onGoalReceived(Action::Goal::ConstSharedPtr goal)
  {
    if (!bt_action_server_->loadBehaviorTree(goal->behavior_tree)) {
      RCLCPP_ERROR(get_logger(), "Failed to load requested ROG-Map behavior tree");
      return false;
    }
    geometry_msgs::msg::PoseStamped transformed;
    if (!transformGoal(goal->pose, transformed)) {
      return false;
    }
    initializeGoal(transformed);
    return true;
  }

  void onPreempt(Action::Goal::ConstSharedPtr goal)
  {
    const auto requested_tree = goal->behavior_tree.empty() ?
      bt_action_server_->getDefaultBTFilename() : goal->behavior_tree;
    if (requested_tree != bt_action_server_->getCurrentBTFilename()) {
      RCLCPP_WARN(get_logger(), "Rejecting preemption with a different behavior tree");
      bt_action_server_->terminatePendingGoal();
      return;
    }
    geometry_msgs::msg::PoseStamped transformed;
    if (!transformGoal(goal->pose, transformed)) {
      bt_action_server_->terminatePendingGoal();
      return;
    }
    bt_action_server_->acceptPendingGoal();
    initializeGoal(transformed);
  }

  void initializeGoal(const geometry_msgs::msg::PoseStamped & goal)
  {
    start_time_ = now();
    auto blackboard = bt_action_server_->getBlackboard();
    blackboard->set<int>("number_recoveries", 0);
    blackboard->set<geometry_msgs::msg::PoseStamped>("goal", goal);
    RCLCPP_INFO(
      get_logger(), "Navigating to 3D goal (%.3f, %.3f, %.3f) in frame %s",
      goal.pose.position.x, goal.pose.position.y, goal.pose.position.z,
      goal.header.frame_id.c_str());
  }

  void onLoop()
  {
    auto feedback = std::make_shared<Action::Feedback>();
    feedback->navigation_time = now() - start_time_;
    auto blackboard = bt_action_server_->getBlackboard();
    int number_of_recoveries = 0;
    blackboard->get<int>("number_recoveries", number_of_recoveries);
    feedback->number_of_recoveries = static_cast<int16_t>(number_of_recoveries);
    float distance_remaining = 0.0F;
    try {
      if (blackboard->get<float>("controller_feedback_distance_to_goal", distance_remaining)) {
        feedback->distance_remaining = distance_remaining;
      }
    } catch (const std::exception &) {
      // The controller output port exists but has no value until execution starts.
    }
    try {
      geometry_msgs::msg::PoseStamped robot_pose;
      robot_pose.header.frame_id = robot_base_frame_;
      robot_pose.header.stamp = now();
      robot_pose.pose.orientation.w = 1.0;
      feedback->current_pose = tf_buffer_->transform(
        robot_pose, global_frame_, tf2::durationFromSec(0.0));
    } catch (const tf2::TransformException &) {
      // Feedback is best effort; planning and control perform their own TF checks.
    }
    bt_action_server_->publishFeedback(feedback);
  }

  void onCompletion(
    Action::Result::SharedPtr,
    nav2_behavior_tree::BtStatus status)
  {
    const char * result = status == nav2_behavior_tree::BtStatus::SUCCEEDED ?
      "succeeded" : status == nav2_behavior_tree::BtStatus::CANCELED ? "canceled" : "failed";
    RCLCPP_INFO(get_logger(), "3D navigation behavior tree %s", result);
  }

  void sendGoal(const geometry_msgs::msg::PoseStamped & pose)
  {
    if (!active_) {
      RCLCPP_WARN(get_logger(), "Ignoring 3D goal while navigator is inactive");
      return;
    }
    if (!self_client_->action_server_is_ready()) {
      RCLCPP_WARN(get_logger(), "NavigateToPose action server is not ready");
      return;
    }
    Action::Goal goal;
    goal.pose = pose;
    self_client_->async_send_goal(goal);
  }

  void onGoalPose(const geometry_msgs::msg::PoseStamped::SharedPtr goal)
  {
    sendGoal(*goal);
  }

  void onClickedPoint(const geometry_msgs::msg::PointStamped::SharedPtr point)
  {
    geometry_msgs::msg::PointStamped transformed;
    try {
      transformed = point->header.frame_id == global_frame_ ?
        *point : tf_buffer_->transform(
        *point, global_frame_, tf2::durationFromSec(transform_tolerance_));
    } catch (const tf2::TransformException & exception) {
      RCLCPP_WARN(
        get_logger(), "Cannot transform RViz point from '%s' to '%s': %s",
        point->header.frame_id.c_str(), global_frame_.c_str(), exception.what());
      return;
    }
    geometry_msgs::msg::PoseStamped goal;
    goal.header = transformed.header;
    goal.header.frame_id = global_frame_;
    goal.header.stamp = now();
    goal.pose.position = transformed.point;
    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, clicked_point_goal_yaw_);
    goal.pose.orientation = tf2::toMsg(orientation);
    sendGoal(goal);
  }

  std::string global_frame_{"map"};
  std::string robot_base_frame_{"base_link"};
  std::string action_name_{"navigate_to_pose"};
  double transform_tolerance_{0.3};
  double clicked_point_goal_yaw_{0.0};
  bool accept_goal_topic_{true};
  bool accept_rviz_point_{true};
  std::atomic_bool active_{false};
  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<BtActionServer> bt_action_server_;
  rclcpp_action::Client<Action>::SharedPtr self_client_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_sub_;
};

}  // namespace navflex_rogmap_bt_navigator

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<navflex_rogmap_bt_navigator::RogMapBtNavigator>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
