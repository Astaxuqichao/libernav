// Copyright 2026 Navflex contributors
// SPDX-License-Identifier: BSD-3-Clause

#include "scan_planner/scan_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"

namespace scan_planner
{

double ScanController::clamp(double value, double minimum, double maximum)
{
  return std::max(minimum, std::min(value, maximum));
}

double ScanController::distance(const Point & first, const Point & second)
{
  return std::hypot(
    std::hypot(second.x - first.x, second.y - first.y),
    second.z - first.z);
}

geometry_msgs::msg::Point ScanController::toMessage(const Point & point)
{
  geometry_msgs::msg::Point message;
  message.x = point.x;
  message.y = point.y;
  message.z = point.z;
  return message;
}

void ScanController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  navflex_rog_map::RogMap::ConstPtr map)
{
  node_ = parent.lock();
  if (!node_) {
    throw std::runtime_error("ScanController cannot lock lifecycle node");
  }
  if (!map) {
    throw std::invalid_argument("ScanController requires a ROG map");
  }
  name_ = name;
  tf_ = std::move(tf);
  map_ = std::move(map);
  global_frame_ = map_->frameId();

  auto declare = [this](const std::string & key, const auto & value) {
      nav2_util::declare_parameter_if_not_declared(
        node_, name_ + "." + key, rclcpp::ParameterValue(value));
    };
  declare("footprint_type", std::string("sphere"));
  declare("footprint_offset", std::vector<double>{0.0, 0.0, 0.0});
  declare("footprint_radius", 0.35);
  declare("footprint_height", 0.7);
  declare("footprint_size", std::vector<double>{0.7, 0.5, 0.4});
  declare("front_sphere_offset", std::vector<double>{0.25, 0.0, 0.0});
  declare("front_sphere_radius", 0.3);
  declare("rear_sphere_offset", std::vector<double>{-0.25, 0.0, 0.0});
  declare("rear_sphere_radius", 0.3);
  declare("footprint_safety_margin", 0.0);
  declare("robot_frame", std::string("base_link"));
  declare("trajectory_topic", std::string("scan_planner/optimized_path"));
  declare("sample_distance", 0.1);
  declare("control_point_distance", 0.2);
  declare("astar_resolution", 0.15);
  declare("astar_search_margin", 2.0);
  declare("astar_max_expansions", 30000);
  declare("optimization_iterations", 60);
  declare("optimization_step_size", 0.08);
  declare("lambda_smooth", 1.0);
  declare("lambda_collision", 1.0);
  declare("lambda_reference", 1.0);
  declare("lambda_feasibility", 0.1);
  declare("collision_distance", 0.2);
  declare("max_reference_deviation", 1.5);
  declare("lookahead_time", 0.8);
  declare("control_lookahead_time", 0.15);
  declare("max_velocity", 0.8);
  declare("max_vertical_velocity", 0.4);
  declare("max_acceleration", 1.5);
  declare("max_yaw_rate", 1.0);
  declare("kp_position", 1.5);
  declare("kp_yaw", 2.0);
  declare("heading_error_threshold", 0.8);
  declare("min_yaw_control_speed", 0.05);

  footprint_.type = navflex_rog_map::footprintTypeFromString(
    node_->get_parameter(name_ + ".footprint_type").as_string());
  const auto offset = node_->get_parameter(name_ + ".footprint_offset").as_double_array();
  const auto size = node_->get_parameter(name_ + ".footprint_size").as_double_array();
  const auto front_offset =
    node_->get_parameter(name_ + ".front_sphere_offset").as_double_array();
  const auto rear_offset =
    node_->get_parameter(name_ + ".rear_sphere_offset").as_double_array();
  if (offset.size() != 3 || size.size() != 3 || front_offset.size() != 3 ||
    rear_offset.size() != 3)
  {
    throw std::invalid_argument("3D footprint offset and size parameters require three values");
  }
  footprint_.offset.x = offset[0];
  footprint_.offset.y = offset[1];
  footprint_.offset.z = offset[2];
  footprint_.radius = node_->get_parameter(name_ + ".footprint_radius").as_double();
  footprint_.height = node_->get_parameter(name_ + ".footprint_height").as_double();
  footprint_.size.x = size[0];
  footprint_.size.y = size[1];
  footprint_.size.z = size[2];
  footprint_.front_sphere.offset.x = front_offset[0];
  footprint_.front_sphere.offset.y = front_offset[1];
  footprint_.front_sphere.offset.z = front_offset[2];
  footprint_.front_sphere.radius =
    node_->get_parameter(name_ + ".front_sphere_radius").as_double();
  footprint_.rear_sphere.offset.x = rear_offset[0];
  footprint_.rear_sphere.offset.y = rear_offset[1];
  footprint_.rear_sphere.offset.z = rear_offset[2];
  footprint_.rear_sphere.radius =
    node_->get_parameter(name_ + ".rear_sphere_radius").as_double();
  footprint_.safety_margin =
    node_->get_parameter(name_ + ".footprint_safety_margin").as_double();
  navflex_rog_map::validateFootprint(footprint_);
  node_->get_parameter(name_ + ".sample_distance", planner_config_.sample_distance);
  node_->get_parameter(
    name_ + ".control_point_distance", planner_config_.control_point_distance);
  node_->get_parameter(name_ + ".astar_resolution", planner_config_.astar_resolution);
  node_->get_parameter(name_ + ".astar_search_margin", planner_config_.astar_search_margin);
  node_->get_parameter(name_ + ".astar_max_expansions", planner_config_.astar_max_expansions);
  node_->get_parameter(
    name_ + ".optimization_iterations", planner_config_.optimization_iterations);
  node_->get_parameter(
    name_ + ".optimization_step_size", planner_config_.optimization_step_size);
  node_->get_parameter(name_ + ".lambda_smooth", planner_config_.smooth_weight);
  node_->get_parameter(name_ + ".lambda_collision", planner_config_.collision_weight);
  node_->get_parameter(name_ + ".lambda_reference", planner_config_.reference_weight);
  node_->get_parameter(name_ + ".lambda_feasibility", planner_config_.feasibility_weight);
  node_->get_parameter(name_ + ".collision_distance", planner_config_.collision_distance);
  node_->get_parameter(
    name_ + ".max_reference_deviation", planner_config_.max_reference_deviation);
  node_->get_parameter(name_ + ".lookahead_time", lookahead_time_);
  node_->get_parameter(name_ + ".control_lookahead_time", control_lookahead_time_);
  node_->get_parameter(name_ + ".max_velocity", max_velocity_);
  node_->get_parameter(name_ + ".max_vertical_velocity", max_vertical_velocity_);
  node_->get_parameter(name_ + ".max_acceleration", max_acceleration_);
  node_->get_parameter(name_ + ".max_yaw_rate", max_yaw_rate_);
  node_->get_parameter(name_ + ".kp_position", position_gain_);
  node_->get_parameter(name_ + ".kp_yaw", yaw_gain_);
  node_->get_parameter(name_ + ".heading_error_threshold", heading_error_threshold_);
  node_->get_parameter(name_ + ".robot_frame", robot_frame_);
  node_->get_parameter(name_ + ".trajectory_topic", trajectory_topic_);
  node_->get_parameter(name_ + ".min_yaw_control_speed", min_yaw_control_speed_);
  planner_config_.max_velocity = max_velocity_;
  planner_config_.max_acceleration = max_acceleration_;
  if (planner_config_.sample_distance <= 0.0 ||
    planner_config_.control_point_distance <= 0.0 ||
    planner_config_.astar_resolution <= 0.0 || planner_config_.astar_search_margin < 0.0 ||
    planner_config_.astar_max_expansions <= 0 || planner_config_.optimization_iterations < 0 ||
    planner_config_.optimization_step_size <= 0.0 ||
    planner_config_.smooth_weight < 0.0 || planner_config_.collision_weight < 0.0 ||
    planner_config_.reference_weight < 0.0 || planner_config_.feasibility_weight < 0.0 ||
    planner_config_.collision_distance < 0.0 ||
    planner_config_.max_reference_deviation < 0.0 || max_velocity_ <= 0.0 ||
    max_vertical_velocity_ < 0.0 || max_acceleration_ <= 0.0 || max_yaw_rate_ < 0.0 ||
    lookahead_time_ < 0.0 || control_lookahead_time_ < 0.0 ||
    control_lookahead_time_ > lookahead_time_ || heading_error_threshold_ < 0.0 ||
    min_yaw_control_speed_ < 0.0 ||
    robot_frame_.empty() || trajectory_topic_.empty())
  {
    throw std::invalid_argument("Invalid ScanController motion or frame parameters");
  }
  planner_ = std::make_unique<SingleScanPlanner>(planner_config_, map_, footprint_);
  trajectory_publisher_ = node_->create_publisher<nav_msgs::msg::Path>(
    trajectory_topic_, rclcpp::QoS(1).transient_local());
  last_time_ = node_->now();
}

void ScanController::cleanup()
{
  std::lock_guard<std::mutex> lock(mutex_);
  active_ = false;
  planned_trajectory_ = TimedScanTrajectory();
  input_trajectory_ = navflex_rogmap_core::Trajectory3D();
  map_.reset();
  planner_.reset();
  trajectory_publisher_.reset();
  tf_.reset();
  node_.reset();
}

void ScanController::activate()
{
  std::lock_guard<std::mutex> lock(mutex_);
  active_ = true;
  canceled_ = false;
  if (trajectory_publisher_) {
    trajectory_publisher_->on_activate();
  }
  last_time_ = node_->now();
}

void ScanController::deactivate()
{
  std::lock_guard<std::mutex> lock(mutex_);
  active_ = false;
  if (trajectory_publisher_) {
    trajectory_publisher_->on_deactivate();
  }
}

void ScanController::setTrajectory(const navflex_rogmap_core::Trajectory3D & trajectory)
{
  std::lock_guard<std::mutex> lock(mutex_);
  input_trajectory_ = trajectory;
  planned_trajectory_ = TimedScanTrajectory();
  trajectory_time_ = 0.0;
  trajectory_planned_ = false;
  canceled_ = false;
}

bool ScanController::cancel()
{
  std::lock_guard<std::mutex> lock(mutex_);
  canceled_ = true;
  return true;
}

bool ScanController::occupied(const geometry_msgs::msg::Pose & pose) const
{
  return !map_->isCollisionFree(pose, footprint_);
}

bool ScanController::segmentFree(const Point & start, const Point & end) const
{
  geometry_msgs::msg::Pose start_pose;
  geometry_msgs::msg::Pose end_pose;
  start_pose.position = toMessage(start);
  end_pose.position = toMessage(end);
  const double yaw = std::atan2(end.y - start.y, end.x - start.x);
  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, yaw);
  start_pose.orientation.x = orientation.x();
  start_pose.orientation.y = orientation.y();
  start_pose.orientation.z = orientation.z();
  start_pose.orientation.w = orientation.w();
  end_pose.orientation = start_pose.orientation;
  return map_->raycastFree(start_pose, end_pose, footprint_);
}

ScanController::Point ScanController::evaluate(double time) const
{
  if (planned_trajectory_.points.empty()) {
    return {};
  }
  const double scaled_time = std::max(0.0, time * speed_scale_);
  if (scaled_time >= planned_trajectory_.duration) {
    return planned_trajectory_.points.back();
  }
  const auto upper = std::upper_bound(
    planned_trajectory_.times.begin(), planned_trajectory_.times.end(), scaled_time);
  const size_t next = static_cast<size_t>(
    std::distance(planned_trajectory_.times.begin(), upper));
  if (next == 0 || next >= planned_trajectory_.points.size()) {
    return planned_trajectory_.points[std::min(next, planned_trajectory_.points.size() - 1)];
  }
  const size_t previous = next - 1;
  const double interval = planned_trajectory_.times[next] - planned_trajectory_.times[previous];
  const double ratio = interval > 1e-9 ?
    (scaled_time - planned_trajectory_.times[previous]) / interval : 0.0;
  return {
    planned_trajectory_.points[previous].x + ratio *
    (planned_trajectory_.points[next].x - planned_trajectory_.points[previous].x),
    planned_trajectory_.points[previous].y + ratio *
    (planned_trajectory_.points[next].y - planned_trajectory_.points[previous].y),
    planned_trajectory_.points[previous].z + ratio *
    (planned_trajectory_.points[next].z - planned_trajectory_.points[previous].z)};
}

ScanController::Point ScanController::evaluateVelocity(double time) const
{
  constexpr double kDerivativeStep = 0.05;
  const Point first = evaluate(time);
  const Point second = evaluate(time + kDerivativeStep);
  return {(second.x - first.x) / kDerivativeStep,
    (second.y - first.y) / kDerivativeStep,
    (second.z - first.z) / kDerivativeStep};
}

void ScanController::publishPlannedTrajectory() const
{
  if (!trajectory_publisher_ || !trajectory_publisher_->is_activated()) {
    return;
  }
  nav_msgs::msg::Path path;
  path.header.stamp = node_->now();
  path.header.frame_id = global_frame_;
  path.poses.reserve(planned_trajectory_.points.size());
  for (size_t i = 0; i < planned_trajectory_.points.size(); ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position = toMessage(planned_trajectory_.points[i]);
    const size_t next = std::min(i + 1, planned_trajectory_.points.size() - 1);
    const size_t previous = i > 0 ? i - 1 : i;
    const double yaw = std::atan2(
      planned_trajectory_.points[next].y - planned_trajectory_.points[previous].y,
      planned_trajectory_.points[next].x - planned_trajectory_.points[previous].x);
    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, yaw);
    pose.pose.orientation.x = orientation.x();
    pose.pose.orientation.y = orientation.y();
    pose.pose.orientation.z = orientation.z();
    pose.pose.orientation.w = orientation.w();
    path.poses.push_back(std::move(pose));
  }
  trajectory_publisher_->publish(path);
}

uint32_t ScanController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  geometry_msgs::msg::TwistStamped & command,
  nav2_core::GoalChecker *, std::string & message)
{
  using Result = nav2_msgs::action::FollowPath::Result;
  command = geometry_msgs::msg::TwistStamped();
  command.header = pose.header;
  command.header.frame_id = robot_frame_;
  std::lock_guard<std::mutex> lock(mutex_);
  if (!active_ || !map_) {
    message = "ScanController is not active or has no ROG map";
    return Result::NOT_INITIALIZED;
  }
  if (canceled_) {
    message = "ScanController execution canceled";
    return Result::CANCELED;
  }
  if (input_trajectory_.path.poses.empty()) {
    message = "ScanController has no input 3D trajectory";
    return Result::INVALID_PATH;
  }
  if (!input_trajectory_.path.header.frame_id.empty() &&
    input_trajectory_.path.header.frame_id != global_frame_)
  {
    message = "ScanController trajectory frame does not match ROG map frame";
    return Result::INVALID_PATH;
  }

  const Point current{
    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z};
  if (occupied(pose.pose)) {
    message = "Current robot position collides with the ROG map";
    return Result::BLOCKED_PATH;
  }
  if (!trajectory_planned_) {
    if (!planner_) {
      message = "Single-shot SCAN planner is not configured";
      return Result::NOT_INITIALIZED;
    }
    if (!planner_->plan(input_trajectory_.path, pose.pose, planned_trajectory_, message)) {
      return Result::BLOCKED_PATH;
    }
    trajectory_time_ = 0.0;
    trajectory_planned_ = true;
    publishPlannedTrajectory();
  }
  if (!segmentFree(current, evaluate(trajectory_time_ + lookahead_time_))) {
    message = "Single-shot SCAN trajectory became blocked; a new action goal is required";
    return Result::BLOCKED_PATH;
  }

  const rclcpp::Time now = node_->now();
  const double time_step = clamp((now - last_time_).seconds(), 0.001, 0.1);
  last_time_ = now;
  const Point desired = evaluate(trajectory_time_ + control_lookahead_time_);
  const Point feed_forward = evaluateVelocity(trajectory_time_);
  double velocity_x = feed_forward.x + position_gain_ * (desired.x - current.x);
  double velocity_y = feed_forward.y + position_gain_ * (desired.y - current.y);

  const double horizontal_limit = max_velocity_ * speed_scale_;
  const double horizontal_norm = std::hypot(velocity_x, velocity_y);
  if (horizontal_norm > horizontal_limit) {
    velocity_x *= horizontal_limit / horizontal_norm;
    velocity_y *= horizontal_limit / horizontal_norm;
  }

  // The trajectory and feedback terms above are expressed in the map frame,
  // while the measured Twist is expressed in the robot frame.
  const double yaw = tf2::getYaw(pose.pose.orientation);
  const double measured_velocity_x =
    std::cos(yaw) * velocity.linear.x - std::sin(yaw) * velocity.linear.y;
  const double measured_velocity_y =
    std::sin(yaw) * velocity.linear.x + std::cos(yaw) * velocity.linear.y;
  velocity_x = clamp(
    velocity_x, measured_velocity_x - max_acceleration_ * time_step,
    measured_velocity_x + max_acceleration_ * time_step);
  velocity_y = clamp(
    velocity_y, measured_velocity_y - max_acceleration_ * time_step,
    measured_velocity_y + max_acceleration_ * time_step);

  const double horizontal_speed = std::hypot(velocity_x, velocity_y);
  const double desired_yaw = horizontal_speed >= min_yaw_control_speed_ ?
    std::atan2(velocity_y, velocity_x) : yaw;
  const double yaw_error = std::atan2(
    std::sin(desired_yaw - yaw), std::cos(desired_yaw - yaw));
  if (std::abs(yaw_error) > heading_error_threshold_) {
    command.twist.angular.z = clamp(
      yaw_gain_ * yaw_error, -max_yaw_rate_, max_yaw_rate_);
    message = "Rotating toward the single-shot SCAN trajectory";
    return Result::SUCCESS;
  }
  trajectory_time_ = std::min(
    planned_trajectory_.duration / std::max(speed_scale_, 1e-6),
    trajectory_time_ + time_step);
  command.twist.linear.x = std::cos(yaw) * velocity_x + std::sin(yaw) * velocity_y;
  command.twist.linear.y = -std::sin(yaw) * velocity_x + std::cos(yaw) * velocity_y;
  // A quadruped follows body-height changes through contact with the terrain;
  // cmd_vel.z is not a flight command. Z remains part of planning and collision checks.
  command.twist.linear.z = 0.0;
  command.twist.angular.z = clamp(
    yaw_gain_ * yaw_error, -max_yaw_rate_, max_yaw_rate_);
  message = "Tracking single-shot SCAN trajectory using ROG map and ESDF";
  return Result::SUCCESS;
}

void ScanController::setSpeedLimit(double speed_limit, bool percentage)
{
  std::lock_guard<std::mutex> lock(mutex_);
  speed_scale_ = percentage ? clamp(speed_limit / 100.0, 0.0, 1.0) :
    clamp(speed_limit / max_velocity_, 0.0, 1.0);
}

}  // namespace scan_planner

PLUGINLIB_EXPORT_CLASS(
  scan_planner::ScanController,
  navflex_rogmap_core::Controller)
