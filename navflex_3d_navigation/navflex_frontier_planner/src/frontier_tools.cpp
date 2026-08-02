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

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <queue>
#include <unordered_map>
#include <utility>

#include "navflex_frontier_planner/fael_frontier_core.hpp"
#include "nav2_util/node_utils.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace navflex_frontier_planner
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

geometry_msgs::msg::Point toMsg(const Point3 & point)
{
  geometry_msgs::msg::Point result;
  result.x = point.x;
  result.y = point.y;
  result.z = point.z;
  return result;
}

geometry_msgs::msg::Quaternion yawQuaternion(double yaw)
{
  geometry_msgs::msg::Quaternion result;
  result.z = std::sin(0.5 * yaw);
  result.w = std::cos(0.5 * yaw);
  return result;
}

void orientPath(nav_msgs::msg::Path & path)
{
  if (path.poses.empty()) {
    return;
  }
  for (std::size_t index = 0; index < path.poses.size(); ++index) {
    const std::size_t other = index + 1 < path.poses.size() ? index + 1 : index - (index > 0);
    const auto & here = path.poses[index].pose.position;
    const auto & there = path.poses[other].pose.position;
    if (index + 1 < path.poses.size()) {
      path.poses[index].pose.orientation = yawQuaternion(
        std::atan2(there.y - here.y, there.x - here.x));
    } else if (index > 0) {
      path.poses[index].pose.orientation = path.poses[index - 1].pose.orientation;
    } else {
      path.poses[index].pose.orientation.w = 1.0;
    }
  }
}

struct GridKey
{
  int x;
  int y;
  bool operator==(const GridKey & other) const {return x == other.x && y == other.y;}
};

struct GridKeyHash
{
  std::size_t operator()(const GridKey & key) const
  {
    return std::hash<int>()(key.x) ^ (std::hash<int>()(key.y) << 1U);
  }
};

struct SearchNode
{
  GridKey key;
  double f;
};

struct SearchNodeGreater
{
  bool operator()(const SearchNode & first, const SearchNode & second) const
  {
    return first.f > second.f;
  }
};
}  // namespace

double Point3::distanceXY(const Point3 & other) const
{
  return std::hypot(x - other.x, y - other.y);
}

double Point3::distance(const Point3 & other) const
{
  return std::sqrt(
    (x - other.x) * (x - other.x) +
    (y - other.y) * (y - other.y) +
    (z - other.z) * (z - other.z));
}

void FaelFrontierCore::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  navflex_rog_map::RogMap::ConstPtr map)
{
  node_ = parent.lock();
  if (!node_) {
    throw std::runtime_error("Frontier planner lifecycle node expired");
  }
  if (!map) {
    throw std::invalid_argument("Frontier planner requires a ROGMap instance");
  }
  name_ = name;
  tf_ = std::move(tf);
  map_ = std::move(map);
  frame_id_ = map_->frameId();

  const std::string prefix = "frontier_shared_config.";
  auto declare = [this, &prefix](const std::string & key, const auto & value) {
      nav2_util::declare_parameter_if_not_declared(
        node_, prefix + key, rclcpp::ParameterValue(value));
    };
  declare("visualization_topic_prefix", visualization_topic_prefix_);
  declare("local_range", local_range_);
  declare("scan_resolution", scan_resolution_);
  declare("scan_z_min", scan_z_min_);
  declare("scan_z_max", scan_z_max_);
  declare("sample_dist", sample_dist_);
  declare("candidate_separation", candidate_separation_);
  declare("robot_clear_radius", robot_clear_radius_);
  declare("min_candidate_dist", min_candidate_dist_);
  declare("frontier_gain", frontier_gain_);
  declare("distance_weight", distance_weight_);
  declare("goal_distance_weight", goal_distance_weight_);
  declare("min_frontier_cells", min_frontier_cells_);
  declare("max_candidate_count", max_candidate_count_);
  declare("planning_resolution", planning_resolution_);
  declare("max_astar_expansions", max_astar_expansions_);
  declare("nominal_velocity", nominal_velocity_);

  node_->get_parameter(prefix + "visualization_topic_prefix", visualization_topic_prefix_);
  node_->get_parameter(prefix + "local_range", local_range_);
  node_->get_parameter(prefix + "scan_resolution", scan_resolution_);
  node_->get_parameter(prefix + "scan_z_min", scan_z_min_);
  node_->get_parameter(prefix + "scan_z_max", scan_z_max_);
  node_->get_parameter(prefix + "sample_dist", sample_dist_);
  node_->get_parameter(prefix + "candidate_separation", candidate_separation_);
  node_->get_parameter(prefix + "robot_clear_radius", robot_clear_radius_);
  node_->get_parameter(prefix + "min_candidate_dist", min_candidate_dist_);
  node_->get_parameter(prefix + "frontier_gain", frontier_gain_);
  node_->get_parameter(prefix + "distance_weight", distance_weight_);
  node_->get_parameter(prefix + "goal_distance_weight", goal_distance_weight_);
  node_->get_parameter(prefix + "min_frontier_cells", min_frontier_cells_);
  node_->get_parameter(prefix + "max_candidate_count", max_candidate_count_);
  node_->get_parameter(prefix + "planning_resolution", planning_resolution_);
  node_->get_parameter(prefix + "max_astar_expansions", max_astar_expansions_);
  node_->get_parameter(prefix + "nominal_velocity", nominal_velocity_);

  scan_resolution_ = std::max(scan_resolution_, map_->resolution());
  planning_resolution_ = std::max(planning_resolution_, map_->resolution());
  if (local_range_ <= 0.0 || scan_z_max_ < scan_z_min_ || sample_dist_ <= 0.0 ||
    candidate_separation_ <= 0.0 || robot_clear_radius_ < 0.0 ||
    max_candidate_count_ <= 0 || max_astar_expansions_ <= 0 || nominal_velocity_ <= 0.0)
  {
    throw std::invalid_argument("Invalid frontier_shared_config parameter value");
  }

  const auto qos = rclcpp::QoS(1).transient_local().reliable();
  candidate_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    visualization_topic_prefix_ + "/candidates", qos);
  selected_candidate_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
    visualization_topic_prefix_ + "/selected_candidate", qos);
  cancel_requested_.store(false);

  RCLCPP_INFO(
    node_->get_logger(),
    "[%s] configured ROGMap frontier planner: frame=%s map_resolution=%.3f scan=%.3f",
    name_.c_str(), frame_id_.c_str(), map_->resolution(), scan_resolution_);
}

void FaelFrontierCore::activate()
{
  candidate_pub_->on_activate();
  selected_candidate_pub_->on_activate();
}

void FaelFrontierCore::deactivate()
{
  candidate_pub_->on_deactivate();
  selected_candidate_pub_->on_deactivate();
}

void FaelFrontierCore::cleanup()
{
  candidate_pub_.reset();
  selected_candidate_pub_.reset();
  map_.reset();
  tf_.reset();
  node_.reset();
}

bool FaelFrontierCore::cancel()
{
  cancel_requested_.store(true);
  return true;
}

bool FaelFrontierCore::isFree(const Point3 & point, double radius) const
{
  const auto position = toMsg(point);
  return map_->globalInflatedState(position) == navflex_rog_map::OccupancyState::FREE &&
         map_->isCollisionFree(position, radius);
}

bool FaelFrontierCore::isVisible(const Point3 & from, const Point3 & to, double radius) const
{
  return map_->raycastFree(toMsg(from), toMsg(to), radius);
}

std::vector<Point3> FaelFrontierCore::findFrontiers(const Point3 & current) const
{
  std::vector<Point3> frontiers;
  const auto bounds = map_->localBounds();
  const double min_x = std::max(bounds.min_x, current.x - local_range_);
  const double max_x = std::min(bounds.max_x, current.x + local_range_);
  const double min_y = std::max(bounds.min_y, current.y - local_range_);
  const double max_y = std::min(bounds.max_y, current.y + local_range_);
  const double min_z = std::max(bounds.min_z, current.z + scan_z_min_);
  const double max_z = std::min(bounds.max_z, current.z + scan_z_max_);

  for (double z = min_z; z <= max_z; z += scan_resolution_) {
    for (double y = min_y; y <= max_y; y += scan_resolution_) {
      for (double x = min_x; x <= max_x; x += scan_resolution_) {
        if (cancel_requested_.load()) {
          return {};
        }
        Point3 point{x, y, z};
        if (point.distanceXY(current) <= local_range_ && map_->isFrontier(toMsg(point))) {
          frontiers.push_back(point);
        }
      }
    }
  }
  return frontiers;
}

std::vector<Candidate> FaelFrontierCore::buildCandidates(
  const Point3 & current, const Point3 & requested_goal,
  const std::vector<Point3> & frontiers) const
{
  std::unordered_map<GridKey, Candidate, GridKeyHash> grouped;
  constexpr int direction_count = 16;
  for (const auto & frontier : frontiers) {
    if (cancel_requested_.load()) {
      return {};
    }
    std::optional<Point3> best;
    double best_distance = std::numeric_limits<double>::max();
    for (int index = 0; index < direction_count; ++index) {
      const double angle = 2.0 * kPi * index / direction_count;
      Point3 viewpoint{
        frontier.x + sample_dist_ * std::cos(angle),
        frontier.y + sample_dist_ * std::sin(angle), current.z};
      const double travel_distance = viewpoint.distanceXY(current);
      if (travel_distance < min_candidate_dist_ ||
        !isFree(viewpoint, robot_clear_radius_) ||
        !isVisible(viewpoint, frontier, 0.0))
      {
        continue;
      }
      if (travel_distance < best_distance) {
        best = viewpoint;
        best_distance = travel_distance;
      }
    }
    if (!best) {
      continue;
    }
    const GridKey key{
      static_cast<int>(std::floor(best->x / candidate_separation_)),
      static_cast<int>(std::floor(best->y / candidate_separation_))};
    auto & candidate = grouped[key];
    if (candidate.frontiers.empty()) {
      candidate.point = *best;
    }
    candidate.frontiers.push_back(frontier);
  }

  std::vector<Candidate> candidates;
  for (auto & item : grouped) {
    auto & candidate = item.second;
    if (static_cast<int>(candidate.frontiers.size()) < min_frontier_cells_) {
      continue;
    }
    candidate.score = frontier_gain_ * candidate.frontiers.size() -
      distance_weight_ * candidate.point.distanceXY(current) -
      goal_distance_weight_ * candidate.point.distanceXY(requested_goal);
    candidates.push_back(std::move(candidate));
  }
  std::sort(
    candidates.begin(), candidates.end(),
    [](const Candidate & first, const Candidate & second) {return first.score > second.score;});
  if (candidates.size() > static_cast<std::size_t>(max_candidate_count_)) {
    candidates.resize(max_candidate_count_);
  }
  return candidates;
}

std::vector<Candidate> FaelFrontierCore::selectCandidates(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & requested_goal)
{
  cancel_requested_.store(false);
  const Point3 current{
    start.pose.position.x, start.pose.position.y, start.pose.position.z};
  const Point3 goal{
    requested_goal.pose.position.x, requested_goal.pose.position.y,
    requested_goal.pose.position.z};
  auto frontiers = findFrontiers(current);
  auto candidates = buildCandidates(current, goal, frontiers);
  publishCandidates(candidates, candidates.empty() ? nullptr : &candidates.front());
  RCLCPP_INFO(
    node_->get_logger(), "[%s] ROGMap frontier scan: cells=%zu candidates=%zu revision=%lu",
    name_.c_str(), frontiers.size(), candidates.size(), map_->revision());
  return candidates;
}

nav_msgs::msg::Path FaelFrontierCore::makeCandidatePath(
  const geometry_msgs::msg::PoseStamped & start,
  const std::vector<Candidate> & candidates) const
{
  nav_msgs::msg::Path path;
  path.header = start.header;
  path.header.frame_id = frame_id_;
  for (const auto & candidate : candidates) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position = toMsg(candidate.point);
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  orientPath(path);
  return path;
}

nav_msgs::msg::Path FaelFrontierCore::makeAStarPath(
  const geometry_msgs::msg::PoseStamped & start, const Candidate & candidate)
{
  nav_msgs::msg::Path path;
  path.header = start.header;
  path.header.frame_id = frame_id_;
  cancel_requested_.store(false);
  const Point3 origin{start.pose.position.x, start.pose.position.y, start.pose.position.z};
  const Point3 target = candidate.point;

  if (isVisible(origin, target, robot_clear_radius_)) {
    path.poses = {start};
    path.poses.front().header = path.header;
    geometry_msgs::msg::PoseStamped goal_pose;
    goal_pose.header = path.header;
    goal_pose.pose.position = toMsg(target);
    path.poses.push_back(goal_pose);
    orientPath(path);
    return path;
  }

  const auto to_key = [this, &origin](const Point3 & point) {
      return GridKey{
      static_cast<int>(std::lround((point.x - origin.x) / planning_resolution_)),
      static_cast<int>(std::lround((point.y - origin.y) / planning_resolution_))};
    };
  const auto to_point = [this, &origin](const GridKey & key) {
      return Point3{
      origin.x + key.x * planning_resolution_,
      origin.y + key.y * planning_resolution_, origin.z};
    };
  const GridKey start_key{0, 0};
  const GridKey goal_key = to_key(target);
  std::priority_queue<SearchNode, std::vector<SearchNode>, SearchNodeGreater> open;
  std::unordered_map<GridKey, double, GridKeyHash> costs;
  std::unordered_map<GridKey, GridKey, GridKeyHash> parents;
  open.push({start_key, 0.0});
  costs[start_key] = 0.0;
  const std::array<GridKey, 8> neighbors{{
    {-1, -1}, {-1, 0}, {-1, 1}, {0, -1},
    {0, 1}, {1, -1}, {1, 0}, {1, 1}}};
  GridKey reached = start_key;
  bool found = false;
  int expansions = 0;

  while (!open.empty() && expansions++ < max_astar_expansions_) {
    if (cancel_requested_.load()) {
      return nav_msgs::msg::Path();
    }
    const auto current = open.top().key;
    open.pop();
    if (std::abs(current.x - goal_key.x) <= 1 && std::abs(current.y - goal_key.y) <= 1) {
      reached = current;
      found = true;
      break;
    }
    for (const auto & offset : neighbors) {
      const GridKey next{current.x + offset.x, current.y + offset.y};
      const Point3 point = to_point(next);
      if (point.distanceXY(origin) > local_range_ || !isFree(point, robot_clear_radius_)) {
        continue;
      }
      const double step = planning_resolution_ *
        ((offset.x != 0 && offset.y != 0) ? std::sqrt(2.0) : 1.0);
      const double cost = costs[current] + step;
      const auto existing = costs.find(next);
      if (existing != costs.end() && existing->second <= cost) {
        continue;
      }
      costs[next] = cost;
      parents[next] = current;
      const double heuristic = std::hypot(next.x - goal_key.x, next.y - goal_key.y) *
        planning_resolution_;
      open.push({next, cost + heuristic});
    }
  }
  if (!found) {
    return nav_msgs::msg::Path();
  }

  std::vector<Point3> points{target};
  for (GridKey key = reached; !(key == start_key); key = parents.at(key)) {
    points.push_back(to_point(key));
  }
  points.push_back(origin);
  std::reverse(points.begin(), points.end());
  for (const auto & point : points) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position = toMsg(point);
    path.poses.push_back(pose);
  }
  orientPath(path);
  return path;
}

void FaelFrontierCore::publishSelection(
  const geometry_msgs::msg::PoseStamped & start,
  const std::vector<Candidate> & candidates, const Candidate & selected) const
{
  publishCandidates(candidates, &selected);
  geometry_msgs::msg::PoseStamped pose;
  pose.header = start.header;
  pose.header.frame_id = frame_id_;
  pose.pose.position = toMsg(selected.point);
  pose.pose.orientation.w = 1.0;
  selected_candidate_pub_->publish(pose);
}

void FaelFrontierCore::publishCandidates(
  const std::vector<Candidate> & candidates, const Candidate * selected) const
{
  if (!candidate_pub_ || !candidate_pub_->is_activated()) {
    return;
  }
  visualization_msgs::msg::MarkerArray array;
  visualization_msgs::msg::Marker clear;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  array.markers.push_back(clear);
  int id = 0;
  for (const auto & candidate : candidates) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = node_->now();
    marker.ns = "rogmap_frontier_candidates";
    marker.id = id++;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position = toMsg(candidate.point);
    marker.pose.orientation.w = 1.0;
    marker.scale.x = marker.scale.y = marker.scale.z = 0.35;
    const bool is_selected = selected && candidate.point.distance(selected->point) < 1e-6;
    marker.color.r = is_selected ? 0.1F : 0.1F;
    marker.color.g = is_selected ? 1.0F : 0.5F;
    marker.color.b = is_selected ? 0.1F : 1.0F;
    marker.color.a = 0.9F;
    array.markers.push_back(marker);
  }
  candidate_pub_->publish(array);
}

}  // namespace navflex_frontier_planner
