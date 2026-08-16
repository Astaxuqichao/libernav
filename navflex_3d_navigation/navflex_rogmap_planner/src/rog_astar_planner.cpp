// Copyright 2024 Yunfan REN, MaRS Lab, University of Hong Kong
// Copyright 2026 Navflex contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "navflex_rogmap_planner/rog_astar_planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <queue>
#include <utility>

#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace navflex_rogmap_planner
{

namespace
{
constexpr uint32_t kSuccess = 0;
constexpr uint32_t kNotConfigured = 100;
constexpr uint32_t kInvalidStart = 101;
constexpr uint32_t kInvalidGoal = 102;
constexpr uint32_t kNoPath = 103;
constexpr uint32_t kCanceled = 104;
constexpr uint32_t kTimeout = 105;
constexpr double kTieBreaker = 1.0 + 1e-5;

}  // namespace

bool RogAStarPlanner::NodeComparator::operator()(
  const GridNode * first, const GridNode * second) const
{
  return first->total_score > second->total_score;
}

void RogAStarPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  navflex_rog_map::RogMap::ConstPtr map)
{
  node_ = parent.lock();
  if (!node_) {
    throw std::runtime_error("RogAStarPlanner lifecycle node expired");
  }
  name_ = name;
  tf_ = std::move(tf);
  map_ = std::move(map);

  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".map_voxel_num",
    rclcpp::ParameterValue(std::vector<int64_t>{200, 200, 100}));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".allow_diagonal", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".heuristic_type", rclcpp::ParameterValue(2));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".use_inflated_map", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".unknown_as_obstacle", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".max_planning_time", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".goal_region_xy_radius", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".goal_region_z_radius", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".footprint_type", rclcpp::ParameterValue(std::string("sphere")));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".footprint_offset",
    rclcpp::ParameterValue(std::vector<double>{0.0, 0.0, 0.0}));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".footprint_radius", rclcpp::ParameterValue(0.35));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".footprint_height", rclcpp::ParameterValue(0.7));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".footprint_size",
    rclcpp::ParameterValue(std::vector<double>{0.7, 0.5, 0.4}));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".front_sphere_offset",
    rclcpp::ParameterValue(std::vector<double>{0.25, 0.0, 0.0}));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".front_sphere_radius", rclcpp::ParameterValue(0.3));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".rear_sphere_offset",
    rclcpp::ParameterValue(std::vector<double>{-0.25, 0.0, 0.0}));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".rear_sphere_radius", rclcpp::ParameterValue(0.3));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".footprint_safety_margin", rclcpp::ParameterValue(0.0));

  const auto voxel_count = node_->get_parameter(name_ + ".map_voxel_num").as_integer_array();
  if (voxel_count.size() != 3 || voxel_count[0] < 3 || voxel_count[1] < 3 ||
    voxel_count[2] < 3)
  {
    throw std::invalid_argument(name_ + ".map_voxel_num must contain three values >= 3");
  }
  size_x_ = static_cast<int>(voxel_count[0] / 2);
  size_y_ = static_cast<int>(voxel_count[1] / 2);
  size_z_ = static_cast<int>(voxel_count[2] / 2);
  allow_diagonal_ = node_->get_parameter(name_ + ".allow_diagonal").as_bool();
  use_inflated_map_ = node_->get_parameter(name_ + ".use_inflated_map").as_bool();
  unknown_as_obstacle_ = node_->get_parameter(name_ + ".unknown_as_obstacle").as_bool();
  max_planning_time_ = node_->get_parameter(name_ + ".max_planning_time").as_double();
  goal_region_xy_radius_ = node_->get_parameter(name_ + ".goal_region_xy_radius").as_double();
  goal_region_z_radius_ = node_->get_parameter(name_ + ".goal_region_z_radius").as_double();
  if (goal_region_xy_radius_ < 0.0 || goal_region_z_radius_ < 0.0) {
    throw std::invalid_argument("ROG A* goal-region radii must be non-negative");
  }
  footprint_.type = navflex_rog_map::footprintTypeFromString(
    node_->get_parameter(name_ + ".footprint_type").as_string());
  const auto footprint_offset =
    node_->get_parameter(name_ + ".footprint_offset").as_double_array();
  const auto footprint_size =
    node_->get_parameter(name_ + ".footprint_size").as_double_array();
  const auto front_offset =
    node_->get_parameter(name_ + ".front_sphere_offset").as_double_array();
  const auto rear_offset =
    node_->get_parameter(name_ + ".rear_sphere_offset").as_double_array();
  if (footprint_offset.size() != 3 || footprint_size.size() != 3 ||
    front_offset.size() != 3 || rear_offset.size() != 3)
  {
    throw std::invalid_argument("3D footprint offset and size parameters require three values");
  }
  footprint_.offset.x = footprint_offset[0];
  footprint_.offset.y = footprint_offset[1];
  footprint_.offset.z = footprint_offset[2];
  footprint_.radius = node_->get_parameter(name_ + ".footprint_radius").as_double();
  footprint_.height = node_->get_parameter(name_ + ".footprint_height").as_double();
  footprint_.size.x = footprint_size[0];
  footprint_.size.y = footprint_size[1];
  footprint_.size.z = footprint_size[2];
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
  const int heuristic_value = node_->get_parameter(name_ + ".heuristic_type").as_int();
  if (heuristic_value < 0 || heuristic_value > 2) {
    throw std::invalid_argument(name_ + ".heuristic_type must be 0, 1, or 2");
  }
  heuristic_type_ = static_cast<Heuristic>(heuristic_value);
  // This global planner always searches the persistent global grid.
  resolution_ = map_->resolution();
  inverse_resolution_ = 1.0 / resolution_;

  const size_t count = static_cast<size_t>(size_x_ * 2 + 1) *
    static_cast<size_t>(size_y_ * 2 + 1) * static_cast<size_t>(size_z_ * 2 + 1);
  nodes_.resize(count);
  RCLCPP_INFO(
    node_->get_logger(),
    "Configured %s: voxels=%zu resolution=%.3f diagonal=%s inflated=%s unknown_occ=%s "
    "goal_region=(%.2f, %.2f)",
    name_.c_str(), count, resolution_, allow_diagonal_ ? "true" : "false",
    use_inflated_map_ ? "true" : "false", unknown_as_obstacle_ ? "true" : "false",
    goal_region_xy_radius_, goal_region_z_radius_);
}

void RogAStarPlanner::cleanup()
{
  active_ = false;
  nodes_.clear();
  map_.reset();
  tf_.reset();
  node_.reset();
}

void RogAStarPlanner::activate()
{
  active_ = true;
}

void RogAStarPlanner::deactivate()
{
  active_ = false;
  cancel_requested_.store(true);
}

bool RogAStarPlanner::cancel()
{
  cancel_requested_.store(true);
  return true;
}

uint32_t RogAStarPlanner::makePlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  navflex_rogmap_core::Trajectory3D & trajectory,
  std::string & message)
{
  trajectory = navflex_rogmap_core::Trajectory3D();
  if (!active_ || !map_ || !node_) {
    message = "ROG A* planner is not active or configured";
    return kNotConfigured;
  }
  cancel_requested_.store(false);
  if (!isStateValid(start.pose.position)) {
    message = "Start is occupied, unknown, or outside the ROG map";
    return kInvalidStart;
  }
  geometry_msgs::msg::PoseStamped planning_goal;
  if (!resolveGoal(goal, planning_goal)) {
    message = "Goal and its configured planning region are occupied, unknown, or outside the ROG map";
    return kInvalidGoal;
  }

  const GridIndex start_index = positionToIndex(start.pose.position);
  const GridIndex goal_index = positionToIndex(planning_goal.pose.position);
  search_center_.x = (start_index.x + goal_index.x) / 2;
  search_center_.y = (start_index.y + goal_index.y) / 2;
  search_center_.z = (start_index.z + goal_index.z) / 2;
  if (!insideSearchMap(start_index) || !insideSearchMap(goal_index)) {
    message = "Start and goal exceed the configured ROG A* search window";
    return kNoPath;
  }

  ++round_;
  if (round_ == 0) {
    for (auto & node : nodes_) {
      node.round = 0;
    }
    ++round_;
  }
  auto * start_node = &nodes_[localAddress(start_index)];
  start_node->index = start_index;
  start_node->distance_score = 0.0;
  start_node->total_score = heuristic(start_index, goal_index);
  start_node->parent = nullptr;
  start_node->round = round_;
  start_node->state = NodeState::OPEN;

  std::priority_queue<GridNode *, std::vector<GridNode *>, NodeComparator> open_set;
  open_set.push(start_node);
  const auto begin = std::chrono::steady_clock::now();
  size_t iterations = 0;

  while (!open_set.empty()) {
    if (cancel_requested_.load()) {
      message = "ROG A* planning canceled";
      return kCanceled;
    }
    if (max_planning_time_ > 0.0 &&
      std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count() >
      max_planning_time_)
    {
      message = "ROG A* planning time limit exceeded";
      return kTimeout;
    }

    GridNode * current = open_set.top();
    open_set.pop();
    if (current->state == NodeState::CLOSED) {
      continue;
    }
    ++iterations;
    if (current->index.x == goal_index.x && current->index.y == goal_index.y &&
      current->index.z == goal_index.z)
    {
      buildTrajectory(current, start, planning_goal, trajectory);
      trajectory.duration =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count();
      message = "ROG A* found a 3D path with " +
        std::to_string(trajectory.path.poses.size()) + " poses";
      if (planning_goal.pose.position.x != goal.pose.position.x ||
        planning_goal.pose.position.y != goal.pose.position.y ||
        planning_goal.pose.position.z != goal.pose.position.z)
      {
        message += "; adjusted requested goal to a valid point in the planning region";
      }
      return kSuccess;
    }
    current->state = NodeState::CLOSED;

    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dz = -1; dz <= 1; ++dz) {
          if (dx == 0 && dy == 0 && dz == 0) {
            continue;
          }
          if (!allow_diagonal_ && std::abs(dx) + std::abs(dy) + std::abs(dz) > 1) {
            continue;
          }
          const GridIndex next{current->index.x + dx, current->index.y + dy,
            current->index.z + dz};
          if (!insideSearchMap(next) || !isTransitionValid(current->index, next)) {
            continue;
          }
          GridNode * neighbor = &nodes_[localAddress(next)];
          const bool explored = neighbor->round == round_;
          if (explored && neighbor->state == NodeState::CLOSED) {
            continue;
          }
          const double candidate = current->distance_score +
            std::sqrt(static_cast<double>(dx * dx + dy * dy + dz * dz));
          if (!explored || candidate < neighbor->distance_score) {
            neighbor->index = next;
            neighbor->distance_score = candidate;
            neighbor->total_score = candidate + heuristic(next, goal_index);
            neighbor->parent = current;
            neighbor->round = round_;
            neighbor->state = NodeState::OPEN;
            open_set.push(neighbor);
          }
        }
      }
    }
  }

  message = "ROG A* could not find a path after " + std::to_string(iterations) + " expansions";
  return kNoPath;
}

bool RogAStarPlanner::isTransitionValid(const GridIndex & from, const GridIndex & to) const
{
  const int dx = to.x - from.x;
  const int dy = to.y - from.y;
  const int dz = to.z - from.z;
  if (!isStateValid(indexToPosition(to))) {
    return false;
  }
  // Prevent diagonal corner cutting through a single occupied voxel. Every
  // partial move must be collision free for a 3D footprint.
  for (int sx = 0; sx <= 1; ++sx) {
    for (int sy = 0; sy <= 1; ++sy) {
      for (int sz = 0; sz <= 1; ++sz) {
        if (sx == 0 && sy == 0 && sz == 0) {
          continue;
        }
        GridIndex intermediate{from.x + (sx ? dx : 0), from.y + (sy ? dy : 0),
          from.z + (sz ? dz : 0)};
        if (intermediate.x == from.x && intermediate.y == from.y &&
          intermediate.z == from.z)
        {
          continue;
        }
        if (intermediate.x == to.x && intermediate.y == to.y &&
          intermediate.z == to.z)
        {
          continue;
        }
        if (!insideSearchMap(intermediate) || !isStateValid(indexToPosition(intermediate))) {
          return false;
        }
      }
    }
  }
  return true;
}

bool RogAStarPlanner::isStateValid(const geometry_msgs::msg::Point & point) const
{
  navflex_rog_map::Index3D index;
  if (!map_->worldToGrid(point, index)) {
    return false;
  }
  const auto state = use_inflated_map_ ?
    map_->globalInflatedState(point) : map_->globalState(point);
  if (state == navflex_rog_map::OccupancyState::OCCUPIED) {
    return false;
  }
  if (unknown_as_obstacle_ && state == navflex_rog_map::OccupancyState::UNKNOWN) {
    return false;
  }
  geometry_msgs::msg::Pose pose;
  pose.position = point;
  pose.orientation.w = 1.0;
  return map_->isGlobalCollisionFree(pose, footprint_, unknown_as_obstacle_);
}

bool RogAStarPlanner::resolveGoal(
  const geometry_msgs::msg::PoseStamped & requested,
  geometry_msgs::msg::PoseStamped & resolved) const
{
  if (isStateValid(requested.pose.position)) {
    resolved = requested;
    return true;
  }
  if (goal_region_xy_radius_ <= 0.0 && goal_region_z_radius_ <= 0.0) {
    return false;
  }

  const GridIndex center = positionToIndex(requested.pose.position);
  const int xy_steps = static_cast<int>(std::ceil(goal_region_xy_radius_ * inverse_resolution_));
  const int z_steps = static_cast<int>(std::ceil(goal_region_z_radius_ * inverse_resolution_));
  const double xy_radius_squared = goal_region_xy_radius_ * goal_region_xy_radius_;
  double best_distance_squared = std::numeric_limits<double>::infinity();
  geometry_msgs::msg::Point best_point;
  bool found = false;

  for (int z = -z_steps; z <= z_steps; ++z) {
    for (int y = -xy_steps; y <= xy_steps; ++y) {
      for (int x = -xy_steps; x <= xy_steps; ++x) {
        const double dx = x * resolution_;
        const double dy = y * resolution_;
        if (dx * dx + dy * dy > xy_radius_squared) {
          continue;
        }
        const GridIndex candidate_index{center.x + x, center.y + y, center.z + z};
        const auto candidate = indexToPosition(candidate_index);
        const double dz = candidate.z - requested.pose.position.z;
        if (std::abs(dz) > goal_region_z_radius_ || !isStateValid(candidate)) {
          continue;
        }
        const double distance_squared =
          (candidate.x - requested.pose.position.x) *
          (candidate.x - requested.pose.position.x) +
          (candidate.y - requested.pose.position.y) *
          (candidate.y - requested.pose.position.y) + dz * dz;
        if (distance_squared < best_distance_squared) {
          best_distance_squared = distance_squared;
          best_point = candidate;
          found = true;
        }
      }
    }
  }
  if (!found) {
    return false;
  }
  resolved = requested;
  resolved.pose.position = best_point;
  return true;
}

bool RogAStarPlanner::insideSearchMap(const GridIndex & index) const
{
  return std::abs(index.x - search_center_.x) <= size_x_ &&
         std::abs(index.y - search_center_.y) <= size_y_ &&
         std::abs(index.z - search_center_.z) <= size_z_;
}

RogAStarPlanner::GridIndex RogAStarPlanner::positionToIndex(
  const geometry_msgs::msg::Point & point) const
{
  const auto bounds = map_->bounds();
  return {static_cast<int>(std::floor((point.x - bounds.min_x) * inverse_resolution_)),
    static_cast<int>(std::floor((point.y - bounds.min_y) * inverse_resolution_)),
    static_cast<int>(std::floor((point.z - bounds.min_z) * inverse_resolution_))};
}

geometry_msgs::msg::Point RogAStarPlanner::indexToPosition(const GridIndex & index) const
{
  const auto bounds = map_->bounds();
  geometry_msgs::msg::Point point;
  point.x = bounds.min_x + (static_cast<double>(index.x) + 0.5) * resolution_;
  point.y = bounds.min_y + (static_cast<double>(index.y) + 0.5) * resolution_;
  point.z = bounds.min_z + (static_cast<double>(index.z) + 0.5) * resolution_;
  return point;
}

size_t RogAStarPlanner::localAddress(const GridIndex & index) const
{
  const size_t x = static_cast<size_t>(index.x - search_center_.x + size_x_);
  const size_t y = static_cast<size_t>(index.y - search_center_.y + size_y_);
  const size_t z = static_cast<size_t>(index.z - search_center_.z + size_z_);
  return (x * static_cast<size_t>(size_y_ * 2 + 1) + y) *
         static_cast<size_t>(size_z_ * 2 + 1) + z;
}

double RogAStarPlanner::heuristic(const GridIndex & first, const GridIndex & second) const
{
  double dx = std::abs(first.x - second.x);
  double dy = std::abs(first.y - second.y);
  double dz = std::abs(first.z - second.z);
  if (heuristic_type_ == Heuristic::MANHATTAN) {
    return kTieBreaker * (dx + dy + dz);
  }
  if (heuristic_type_ == Heuristic::EUCLIDEAN) {
    return kTieBreaker * std::sqrt(dx * dx + dy * dy + dz * dz);
  }
  const double diagonal = std::min({dx, dy, dz});
  dx -= diagonal;
  dy -= diagonal;
  dz -= diagonal;
  double value = std::sqrt(3.0) * diagonal;
  if (dx == 0.0) {
    value += std::sqrt(2.0) * std::min(dy, dz) + std::abs(dy - dz);
  } else if (dy == 0.0) {
    value += std::sqrt(2.0) * std::min(dx, dz) + std::abs(dx - dz);
  } else {
    value += std::sqrt(2.0) * std::min(dx, dy) + std::abs(dx - dy);
  }
  return kTieBreaker * value;
}

void RogAStarPlanner::buildTrajectory(
  GridNode * goal_node,
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  navflex_rogmap_core::Trajectory3D & trajectory) const
{
  std::vector<GridNode *> nodes;
  for (GridNode * current = goal_node; current != nullptr; current = current->parent) {
    nodes.push_back(current);
  }
  std::reverse(nodes.begin(), nodes.end());
  trajectory.path.header = start.header;
  trajectory.path.poses.reserve(nodes.size() + 2);
  trajectory.path.poses.push_back(start);
  for (GridNode * node : nodes) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = start.header;
    pose.pose.position = indexToPosition(node->index);
    pose.pose.orientation.w = 1.0;
    trajectory.path.poses.push_back(pose);
  }
  trajectory.path.poses.push_back(goal);
}

}  // namespace navflex_rogmap_planner

PLUGINLIB_EXPORT_CLASS(
  navflex_rogmap_planner::RogAStarPlanner,
  navflex_rogmap_core::GlobalPlanner)
